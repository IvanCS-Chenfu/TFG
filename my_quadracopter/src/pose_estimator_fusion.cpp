#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <cmath>
#include <array>
#include <string>
#include <algorithm>
#include <optional>

class PoseEstimatorFusion : public rclcpp::Node
{
public:
  PoseEstimatorFusion() : rclcpp::Node("pose_estimator_fusion")
  {
    // ===== Parámetros =====
    // Baro / física
    p0_    = declare_parameter<double>("baro.p0",           101325.0); // Pa
    T_     = declare_parameter<double>("baro.T",            288.15);   // K
    z_ref_ = declare_parameter<double>("baro.ref_altitude", 0.0);      // m
    M_     = declare_parameter<double>("phys.M",            0.0289644);// kg/mol
    g_     = declare_parameter<double>("phys.g",            9.80665);  // m/s^2
    Rgas_  = declare_parameter<double>("phys.R",            8.3144598);// J/(mol*K)

    // Kalman roll/pitch (x=[angle; bias]) — sencillo 1D por eje
    q_angle_ = declare_parameter<double>("kf.q_angle", 1e-3);
    q_bias_  = declare_parameter<double>("kf.q_bias",  1e-5);
    r_meas_  = declare_parameter<double>("kf.r_meas",  1e-2);  // var medida (ang de accel)
    acc_gravity_tol_ = declare_parameter<double>("kf.gravity_tol", 0.5); // m/s^2

    // Complementarios
    comp_alpha_ = declare_parameter<double>("alt.comp_alpha", 0.98); // baro + a_z
    alpha_yaw_  = declare_parameter<double>("yaw.comp_alpha", 0.98); // gyro + mag

    // Declinación magnética (rad)
    decl_rad_ = declare_parameter<double>("mag.declination_rad", 0.0);

    // Integrador por timer
    freq_hz_ = declare_parameter<double>("frequency", 400.0); // Hz
    gyro_bias_tau_s_ = declare_parameter<double>("gyro_bias_time_constant", 0.0); // 0=off

    // Tópicos
    topic_imu_  = declare_parameter<std::string>("topics.imu",  "sensor/imu");
    topic_baro_ = declare_parameter<std::string>("topics.baro", "sensor/baro");
    topic_mag_  = declare_parameter<std::string>("topics.mag",  "sensor/mag");
    topic_R_    = declare_parameter<std::string>("topics.R",    "R");
    topic_out_  = declare_parameter<std::string>("topics.out",  "est/pose");

    // ===== Pub/Sub (QoS compatible con Gazebo plugins) =====
    rclcpp::SensorDataQoS sub_qos;  // BestEffort para sensores
    pub_pose_ = create_publisher<geometry_msgs::msg::Pose>(topic_out_, rclcpp::QoS(10));

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      topic_imu_, sub_qos, std::bind(&PoseEstimatorFusion::onImu, this, std::placeholders::_1));
    sub_baro_ = create_subscription<sensor_msgs::msg::FluidPressure>(
      topic_baro_, sub_qos, std::bind(&PoseEstimatorFusion::onBaro, this, std::placeholders::_1));
    sub_mag_ = create_subscription<sensor_msgs::msg::MagneticField>(
      topic_mag_, sub_qos, std::bind(&PoseEstimatorFusion::onMag, this, std::placeholders::_1));
    sub_R_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      topic_R_, rclcpp::QoS(10),
      std::bind(&PoseEstimatorFusion::onR, this, std::placeholders::_1));

    // ===== Inicialización estado =====
    // Roll
    angle_roll_ = 0.0; bias_roll_ = 0.0;
    P_roll_[0][0] = 1; P_roll_[0][1] = 0; P_roll_[1][0] = 0; P_roll_[1][1] = 1;
    // Pitch
    angle_pitch_ = 0.0; bias_pitch_ = 0.0;
    P_pitch_[0][0] = 1; P_pitch_[0][1] = 0; P_pitch_[1][0] = 0; P_pitch_[1][1] = 1;

    yaw_ = 0.0;
    have_R_ = false;
    R_ = {1,0,0, 0,1,0, 0,0,1};

    z_est_ = 0.0;
    vz_est_ = 0.0;

    // Timer de integración (estilo nodo de test)
    const auto dt_ms = static_cast<int>(std::round(1000.0 / std::max(1.0, freq_hz_)));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(dt_ms),
      std::bind(&PoseEstimatorFusion::onTimer, this));

    RCLCPP_INFO(get_logger(), "PoseEstimatorFusion ready. Subscribing: imu=%s, baro=%s, mag=%s, R=%s | f=%.1f Hz",
                topic_imu_.c_str(), topic_baro_.c_str(), topic_mag_.c_str(), topic_R_.c_str(), freq_hz_);
  }

private:
  // ========== Callbacks ==========
  void onR(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 9) {
      RCLCPP_WARN(get_logger(), "R must have 9 elements (row-major). Got %zu.", msg->data.size());
      return;
    }
    for (size_t i = 0; i < 9; ++i) R_[i] = msg->data[i];
    have_R_ = true;
  }

  void onBaro(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
  {
    const double P = msg->fluid_pressure;
    if (P <= 1.0) return;
    z_baro_ = z_ref_ + (Rgas_ * T_) / (M_ * g_) * std::log(p0_ / P);
    have_baro_ = true;
  }

  void onMag(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    // Cacheamos el magnetómetro; la corrección (tilt compensation) se hace en el timer con r/p actuales
    mx_ = msg->magnetic_field.x;
    my_ = msg->magnetic_field.y;
    mz_ = msg->magnetic_field.z;
    have_mag_ = true;
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Guardamos muestras más recientes (se integran en el timer)
    // Acelerómetros
    ax_ = msg->linear_acceleration.x;
    ay_ = msg->linear_acceleration.y;
    az_ = msg->linear_acceleration.z;
    have_acc_ = true;

    // Gyros
    double gx = msg->angular_velocity.x;
    double gy = msg->angular_velocity.y;
    double gz = msg->angular_velocity.z;

    // Si está activo el estimador de sesgo, guardamos "raw" para filtrarlo en el timer
    if (gyro_bias_tau_s_ > 0.0) {
      gx_raw_ = gx; gy_raw_ = gy; gz_raw_ = gz;
      have_raw_gyro_ = true;
    } else {
      gx_ = gx; gy_ = gy; gz_ = gz;
      have_raw_gyro_ = false;
      have_gyro_ = true;
    }

    // Inicialización desde el primer cuaternión de la IMU (world->imu)
    if (!initialized_) {
      const auto &q = msg->orientation;
      double qw=q.w, qx=q.x, qy=q.y, qz=q.z;
      const double n = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
      if (n > 1e-12) { qw/=n; qx/=n; qy/=n; qz/=n; }

      const double sinr_cosp = 2*(qw*qx + qy*qz);
      const double cosr_cosp = 1 - 2*(qx*qx + qy*qy);
      angle_roll_ = std::atan2(sinr_cosp, cosr_cosp);

      const double sinp = 2*(qw*qy - qz*qx);
      angle_pitch_ = (std::abs(sinp)>=1) ? std::copysign(M_PI/2, sinp) : std::asin(sinp);

      const double siny_cosp = 2*(qw*qz + qx*qy);
      const double cosy_cosp = 1 - 2*(qy*qy + qz*qz);
      yaw_ = std::atan2(siny_cosp, cosy_cosp);

      initialized_ = true;
      RCLCPP_INFO(get_logger(), "Initialized from IMU quaternion: roll=%.3f, pitch=%.3f, yaw=%.3f (rad)",
                  angle_roll_, angle_pitch_, yaw_);
    }

    last_imu_stamp_ = msg->header.stamp;
    have_imu_msg_ = true;
  }

  // ========== Timer de integración ==========
  void onTimer()
  {
    if (!have_imu_msg_) return; // espera tener algo

    // dt del timer (constante ideal si el reloj no avanza)
    const rclcpp::Time now = this->get_clock()->now();
    if (!last_timer_time_) last_timer_time_ = now;
    double dt = (now - *last_timer_time_).seconds();
    if (dt <= 0.0) dt = 1.0 / std::max(1.0, freq_hz_);
    last_timer_time_ = now;

    // ===== Gyro: actualizar sesgo si procede y fijar muestras para integrador =====
    if (gyro_bias_tau_s_ > 0.0 && have_raw_gyro_) {
      const double alpha = std::clamp(dt / gyro_bias_tau_s_, 0.0, 1.0);
      bias_gx_ = (1.0 - alpha) * bias_gx_ + alpha * gx_raw_;
      bias_gy_ = (1.0 - alpha) * bias_gy_ + alpha * gy_raw_;
      bias_gz_ = (1.0 - alpha) * bias_gz_ + alpha * gz_raw_;
      gx_ = gx_raw_ - bias_gx_;
      gy_ = gy_raw_ - bias_gy_;
      gz_ = gz_raw_ - bias_gz_;
      have_gyro_ = true;
    }

    // ===== KF roll/pitch: predicción con gyro =====
    if (have_gyro_) {
      kf_predict(angle_roll_,  bias_roll_,  P_roll_,  gx_, dt);
      kf_predict(angle_pitch_, bias_pitch_, P_pitch_, gy_, dt);
    }

    // ===== KF roll/pitch: corrección con acelerómetros si ||a|| ~ g =====
    if (have_acc_) {
      const double anorm = std::sqrt(ax_*ax_ + ay_*ay_ + az_*az_);
      if (std::abs(anorm - g_) < acc_gravity_tol_) {
        const double roll_acc  = std::atan2(ay_, az_);
        const double pitch_acc = std::atan2(-ax_, std::sqrt(ay_*ay_ + az_*az_));
        kf_update(angle_roll_,  bias_roll_,  P_roll_,  roll_acc);
        kf_update(angle_pitch_, bias_pitch_, P_pitch_, pitch_acc);
      }
    }

    // ===== Yaw: integra con gz y corrige con magnetómetro (tilt compensation) =====
    if (have_gyro_) {
      yaw_ += gz_ * dt;
      yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));
    }
    if (have_mag_) {
      const double cr = std::cos(angle_roll_);
      const double sr = std::sin(angle_roll_);
      const double cp = std::cos(angle_pitch_);
      const double sp = std::sin(angle_pitch_);
      const double Xh = mx_*cp + mz_*sp;
      const double Yh = mx_*sr*sp + my_*cr - mz_*sr*cp;

      if (std::abs(Xh) > 1e-12 || std::abs(Yh) > 1e-12) {
        double yaw_mag = std::atan2(Yh, Xh) + decl_rad_;
        yaw_mag = std::atan2(std::sin(yaw_mag), std::cos(yaw_mag));
        yaw_ = alpha_yaw_ * yaw_ + (1.0 - alpha_yaw_) * yaw_mag;
        yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));
      }
    }

    // ===== Altitud: dinámica con a_z específica (en mundo) + baro complementario =====
    double a_z_spec = 0.0;
    if (have_acc_) {
      if (have_R_) {
        const double axw = R_[0]*ax_ + R_[1]*ay_ + R_[2]*az_;
        const double ayw = R_[3]*ax_ + R_[4]*ay_ + R_[5]*az_;
        const double azw = R_[6]*ax_ + R_[7]*ay_ + R_[8]*az_;
        a_z_spec = azw - (-g_);  // ENU: g_world=(0,0,-g)
      } else {
        a_z_spec = az_ - (-g_);
      }
      vz_est_ += a_z_spec * dt;
      z_est_  += vz_est_ * dt;
    }

    if (have_baro_) {
      z_est_ = comp_alpha_ * z_est_ + (1.0 - comp_alpha_) * z_baro_;
      have_baro_ = false; // usa cada medición una vez
    }

    // ===== Publicar Pose (quat de rpy y z) =====
    geometry_msgs::msg::Pose pose;
    const double cr = std::cos(angle_roll_ * 0.5);
    const double sr = std::sin(angle_roll_ * 0.5);
    const double cp = std::cos(angle_pitch_ * 0.5);
    const double sp = std::sin(angle_pitch_ * 0.5);
    const double cy = std::cos(yaw_ * 0.5);
    const double sy = std::sin(yaw_ * 0.5);

    pose.orientation.w = cy*cp*cr + sy*sp*sr;
    pose.orientation.x = cy*cp*sr - sy*sp*cr;
    pose.orientation.y = cy*sp*cr + sy*cp*sr;
    pose.orientation.z = sy*cp*cr - cy*sp*sr;

    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = z_est_;

    pub_pose_->publish(pose);
  }

  // ========== KF helpers (1D angle+bias) ==========
  void kf_predict(double &angle, double &bias, double P[2][2], double gyro, double dt)
  {
    angle += (gyro - bias) * dt;
    P[0][0] += dt*(dt*P[1][1] - P[0][1] - P[1][0] + q_angle_);
    P[0][1] += dt*(-P[1][1] + q_bias_);
    P[1][0] += dt*(-P[1][1] + q_bias_);
    P[1][1] += q_bias_ * dt;
  }

  void kf_update(double &angle, double &bias, double P[2][2], double z_meas)
  {
    const double S = P[0][0] + r_meas_;
    const double K0 = P[0][0] / S;
    const double K1 = P[1][0] / S;
    const double y  = z_meas - angle; // innovación

    angle += K0 * y;
    bias  += K1 * y;

    const double P00 = P[0][0];
    const double P01 = P[0][1];
    const double P10 = P[1][0];
    const double P11 = P[1][1];

    P[0][0] = (1 - K0) * P00;
    P[0][1] = (1 - K0) * P01;
    P[1][0] = -K1 * P00 + P10;
    P[1][1] = -K1 * P01 + P11;
  }

  // ===== Members =====
  // parámetros
  double p0_, T_, z_ref_, M_, g_, Rgas_;
  double q_angle_, q_bias_, r_meas_, acc_gravity_tol_;
  double comp_alpha_, alpha_yaw_, decl_rad_;
  double freq_hz_, gyro_bias_tau_s_;
  std::string topic_imu_, topic_baro_, topic_mag_, topic_R_, topic_out_;

  // pubs / subs
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr sub_baro_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sub_mag_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_R_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose_;
  rclcpp::TimerBase::SharedPtr timer_;

  // estado filtros roll/pitch
  double angle_roll_, bias_roll_, P_roll_[2][2];
  double angle_pitch_, bias_pitch_, P_pitch_[2][2];

  // yaw
  double yaw_;

  // altitud
  double z_est_, vz_est_;
  double z_baro_{0.0};
  bool have_baro_{false};

  // matriz R (fila-mayor)
  std::array<double,9> R_;
  bool have_R_{false};

  // caché IMU / MAG
  // accel
  double ax_{0.0}, ay_{0.0}, az_{0.0};
  bool have_acc_{false};
  // gyro (con posible filtro de sesgo)
  double gx_{0.0}, gy_{0.0}, gz_{0.0};
  bool have_gyro_{false};
  double gx_raw_{0.0}, gy_raw_{0.0}, gz_raw_{0.0};
  double bias_gx_{0.0}, bias_gy_{0.0}, bias_gz_{0.0};
  bool have_raw_gyro_{false};

  // magnetómetro
  double mx_{0.0}, my_{0.0}, mz_{0.0};
  bool have_mag_{false};

  // inicialización / timing
  bool initialized_{false};
  bool have_imu_msg_{false};
  std::optional<rclcpp::Time> last_imu_stamp_;
  std::optional<rclcpp::Time> last_timer_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseEstimatorFusion>());
  rclcpp::shutdown();
  return 0;
}
