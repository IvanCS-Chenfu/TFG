#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <cmath>
#include <string>
#include <optional>
#include <algorithm>
#include <array>

class PoseEstimatorFusion : public rclcpp::Node
{
public:
  PoseEstimatorFusion() : rclcpp::Node("pose_estimator_fusion")
  {
    // ===== Físicas / baro =====
    p0_    = declare_parameter<double>("baro.p0",           101325.0);
    T_     = declare_parameter<double>("baro.T",            288.15);
    z_ref_ = declare_parameter<double>("baro.ref_altitude", 0.0);
    M_     = declare_parameter<double>("phys.M",            0.0289644);
    g_     = declare_parameter<double>("phys.g",            9.80665);
    Rgas_  = declare_parameter<double>("phys.R",            8.3144598);

    // ===== Fusión actitud =====
    freq_hz_      = declare_parameter<double>("frequency", 400.0);     // Hz integración
    alpha_rp_     = declare_parameter<double>("rp.comp_alpha", 0.98);  // roll/pitch
    alpha_yaw_    = declare_parameter<double>("yaw.comp_alpha", 0.98); // yaw
    acc_tol_      = declare_parameter<double>("rp.gravity_tol", 0.5);  // m/s^2
    decl_rad_     = declare_parameter<double>("mag.declination_rad", 0.0);
    gyro_tau_s_   = declare_parameter<double>("gyro_bias_time_constant", 0.0); // 0=off

    // ===== Altitud =====
    comp_alpha_z_ = declare_parameter<double>("alt.comp_alpha", 0.98); // baro + dinámica
    vz_leak_hz_   = declare_parameter<double>("alt.vz_leak_hz", 0.20); // fuga de vel z (anti-deriva)
    use_external_R_ = declare_parameter<bool>("alt.use_external_R", false);

    // ===== Tópicos =====
    topic_imu_  = declare_parameter<std::string>("topics.imu",  "sensor/imu");
    topic_baro_ = declare_parameter<std::string>("topics.baro", "sensor/baro");
    topic_mag_  = declare_parameter<std::string>("topics.mag",  "sensor/mag");
    topic_R_    = declare_parameter<std::string>("topics.R",    "R"); // opcional
    topic_out_  = declare_parameter<std::string>("topics.out",  "est/pose");

    // ===== Pub/Sub =====
    pub_pose_ = create_publisher<geometry_msgs::msg::Pose>(topic_out_, rclcpp::QoS(10));

    auto sensor_qos = rclcpp::SensorDataQoS(); // BestEffort para hacer match con Gazebo
    sub_imu_  = create_subscription<sensor_msgs::msg::Imu>(
      topic_imu_, sensor_qos, std::bind(&PoseEstimatorFusion::onImu, this, std::placeholders::_1));
    sub_baro_ = create_subscription<sensor_msgs::msg::FluidPressure>(
      topic_baro_, sensor_qos, std::bind(&PoseEstimatorFusion::onBaro, this, std::placeholders::_1));
    sub_mag_  = create_subscription<sensor_msgs::msg::MagneticField>(
      topic_mag_, sensor_qos, std::bind(&PoseEstimatorFusion::onMag, this, std::placeholders::_1));

    if (use_external_R_) {
      sub_R_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        topic_R_, rclcpp::QoS(10),
        std::bind(&PoseEstimatorFusion::onR, this, std::placeholders::_1));
    }

    // ===== Estado inicial =====
    // Cuaternión estimado (world->imu). Empezamos identidad.
    qw_ = 1.0; qx_ = qy_ = qz_ = 0.0;

    // Timer de integración (como el test)
    const int dt_ms = std::max(1, (int)std::round(1000.0 / std::max(1.0, freq_hz_)));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(dt_ms),
      std::bind(&PoseEstimatorFusion::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "PoseEstimatorFusion (quat complementario) f=%.1f Hz, use_external_R=%s",
      freq_hz_, use_external_R_ ? "true" : "false");
  }

private:
  // ====== Subs ======
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Cache acelerómetros
    ax_ = msg->linear_acceleration.x;
    ay_ = msg->linear_acceleration.y;
    az_ = msg->linear_acceleration.z;
    have_acc_ = true;

    // Cache giros (con optional sesgo LPF)
    if (gyro_tau_s_ > 0.0) {
      gx_raw_ = msg->angular_velocity.x;
      gy_raw_ = msg->angular_velocity.y;
      gz_raw_ = msg->angular_velocity.z;
      have_raw_gyro_ = true;
    } else {
      gx_ = msg->angular_velocity.x;
      gy_ = msg->angular_velocity.y;
      gz_ = msg->angular_velocity.z;
      have_gyro_ = true;
      have_raw_gyro_ = false;
    }

    // Inicializa con el quaternion de la IMU si viene bien definido
    if (!initialized_) {
      double w = msg->orientation.w;
      double x = msg->orientation.x;
      double y = msg->orientation.y;
      double z = msg->orientation.z;
      const double n = std::sqrt(w*w + x*x + y*y + z*z);
      if (n > 1e-9) {
        w/=n; x/=n; y/=n; z/=n;
        // Usamos esa orientación inicial
        qw_ = w; qx_ = x; qy_ = y; qz_ = z;
        initialized_ = true;
        RCLCPP_INFO(get_logger(), "Initialized attitude from IMU quaternion.");
      }
    }

    last_stamp_ = msg->header.stamp;
    have_imu_ = true;
  }

  void onMag(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    mx_ = msg->magnetic_field.x;
    my_ = msg->magnetic_field.y;
    mz_ = msg->magnetic_field.z;
    have_mag_ = true;
  }

  void onBaro(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
  {
    const double P = msg->fluid_pressure;
    if (P > 1.0) {
      z_baro_ = z_ref_ + (Rgas_ * T_) / (M_ * g_) * std::log(p0_ / P);
      have_baro_ = true;
      // (opcional) inicializa z al primer baro si no hay nada aún
      if (!z_inited_) { z_est_ = z_baro_; z_inited_ = true; }
    }
  }

  void onR(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!use_external_R_) return;
    if (msg->data.size() != 9) return;
    for (int i=0;i<9;++i) R_ext_[i] = msg->data[i];
    have_R_ext_ = true;
  }

  // ====== Timer (integra + fusiona) ======
  void onTimer()
  {
    if (!have_imu_) return;

    // dt del timer
    const rclcpp::Time now = this->get_clock()->now();
    if (!last_timer_time_) last_timer_time_ = now;
    double dt = (now - *last_timer_time_).seconds();
    if (dt <= 0.0) dt = 1.0 / std::max(1.0, freq_hz_);
    last_timer_time_ = now;

    // === Estimar sesgo giro (LPF exp muy simple) ===
    if (gyro_tau_s_ > 0.0 && have_raw_gyro_) {
      const double alpha = std::clamp(dt / gyro_tau_s_, 0.0, 1.0);
      bias_gx_ = (1.0 - alpha) * bias_gx_ + alpha * gx_raw_;
      bias_gy_ = (1.0 - alpha) * bias_gy_ + alpha * gy_raw_;
      bias_gz_ = (1.0 - alpha) * bias_gz_ + alpha * gz_raw_;
      gx_ = gx_raw_ - bias_gx_;
      gy_ = gy_raw_ - bias_gy_;
      gz_ = gz_raw_ - bias_gz_;
      have_gyro_ = true;
    }

    // === 1) Predicción: integrar giros en CUATERNIÓN ===
    if (have_gyro_) {
      // dq = [cos(|w|dt/2), u*sin(|w|dt/2)]
      const double omega = std::sqrt(gx_*gx_ + gy_*gy_ + gz_*gz_);
      double dw=1, dx=0, dy=0, dz=0;
      if (omega > 1e-12) {
        const double th2 = 0.5 * omega * dt;
        const double s = std::sin(th2), c = std::cos(th2);
        const double ux = gx_ / omega, uy = gy_ / omega, uz = gz_ / omega;
        dw = c; dx = ux*s; dy = uy*s; dz = uz*s;
      }
      // q = q ⊗ dq   (gyro en marco cuerpo)
      quatMul(qw_, qx_, qy_, qz_, dw, dx, dy, dz, qw_, qx_, qy_, qz_);
      quatNorm(qw_, qx_, qy_, qz_);
    }

    // === 2) Corrección roll/pitch con acelerómetros (complementario en cuaterniones) ===
    if (have_acc_) {
      const double an = std::sqrt(ax_*ax_ + ay_*ay_ + az_*az_);
      if (std::abs(an - g_) < acc_tol_) {
        // roll/pitch a partir de accel (yaw del gyro actual)
        const double roll_acc  = std::atan2(ay_, az_);
        const double pitch_acc = std::atan2(-ax_, std::sqrt(ay_*ay_ + az_*az_));
        const double yaw_gyro  = yawFromQuat(qw_, qx_, qy_, qz_);
        double wa, xa, ya, za;
        rpyToQuat(yaw_gyro, pitch_acc, roll_acc, wa, xa, ya, za); // ZYX

        // slerp: q_corrected = slerp(q_current, q_acc, beta) con beta = (1-alpha_rp_)
        const double beta = std::clamp(1.0 - alpha_rp_, 0.0, 1.0);
        slerp(qw_, qx_, qy_, qz_, wa, xa, ya, za, beta, qw_, qx_, qy_, qz_);
      }
    }

    // === 3) Corrección yaw con magnetómetro (tilt compensation) ===
    if (have_mag_) {
      // roll/pitch actuales desde q
      double roll, pitch, yaw_cur;
      quatToRPY(qw_, qx_, qy_, qz_, roll, pitch, yaw_cur);

      const double cr = std::cos(roll),  sr = std::sin(roll);
      const double cp = std::cos(pitch), sp = std::sin(pitch);
      const double Xh = mx_*cp + mz_*sp;
      const double Yh = mx_*sr*sp + my_*cr - mz_*sr*cp;

      if (std::abs(Xh) > 1e-12 || std::abs(Yh) > 1e-12) {
        double yaw_mag = std::atan2(Yh, Xh) + decl_rad_;
        yaw_mag = std::atan2(std::sin(yaw_mag), std::cos(yaw_mag));

        // Construye cuat. de “medida” manteniendo r/p actuales y yaw=yaw_mag
        double wm, xm, ym, zm;
        rpyToQuat(yaw_mag, pitch, roll, wm, xm, ym, zm);

        // slerp yaw (peso 1-alpha_yaw_)
        const double beta_y = std::clamp(1.0 - alpha_yaw_, 0.0, 1.0);
        slerp(qw_, qx_, qy_, qz_, wm, xm, ym, zm, beta_y, qw_, qx_, qy_, qz_);
      }
    }

    // === 4) Altitud: dinámica con fuerza específica + baro ===
    // R (body->world) desde nuestro cuaternión
    double Rbw[9]; quatToR(qw_, qx_, qy_, qz_, Rbw);

    // a_world (incluye gravedad porque IMU publica linear_acc con gravedad si remove_gravity=false)
    const double axw = Rbw[0]*ax_ + Rbw[1]*ay_ + Rbw[2]*az_;
    const double ayw = Rbw[3]*ax_ + Rbw[4]*ay_ + Rbw[5]*az_;
    const double azw = Rbw[6]*ax_ + Rbw[7]*ay_ + Rbw[8]*az_;

    // fuerza específica en mundo: a_spec = a_world - g_world ; ENU: g_world=(0,0,-g)
    const double a_z_spec = azw - (-g_);

    // Integrar con un pequeño “leak” en vz para evitar deriva por sesgo
    const double leak = std::clamp(vz_leak_hz_, 0.0, 5.0);  // Hz
    vz_est_ += a_z_spec * dt;
    if (leak > 0.0) vz_est_ *= std::max(0.0, 1.0 - leak * dt); // exp ~ 1 - leak*dt

    z_est_  += vz_est_ * dt;

    if (have_baro_) {
      z_est_ = comp_alpha_z_ * z_est_ + (1.0 - comp_alpha_z_) * z_baro_;
      have_baro_ = false;
    }

    // === 5) Publicar Pose ===
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = qw_;
    pose.orientation.x = qx_;
    pose.orientation.y = qy_;
    pose.orientation.z = qz_;

    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = z_est_;

    pub_pose_->publish(pose);
  }

  // ===== Utilidades cuaternión/matriz =====
  static void quatMul(double aw,double ax,double ay,double az,
                      double bw,double bx,double by,double bz,
                      double &ow,double &ox,double &oy,double &oz)
  {
    ow = aw*bw - ax*bx - ay*by - az*bz;
    ox = aw*bx + ax*bw + ay*bz - az*by;
    oy = aw*by - ax*bz + ay*bw + az*bx;
    oz = aw*bz + ax*by - ay*bx + az*bw;
  }

  static void quatNorm(double &w,double &x,double &y,double &z)
  {
    const double n = std::sqrt(w*w + x*x + y*y + z*z);
    if (n > 1e-12) { w/=n; x/=n; y/=n; z/=n; }
    else { w=1; x=y=z=0; }
  }

  static void rpyToQuat(double yaw,double pitch,double roll,
                        double &w,double &x,double &y,double &z)
  {
    const double cy = std::cos(yaw*0.5), sy = std::sin(yaw*0.5);
    const double cp = std::cos(pitch*0.5), sp = std::sin(pitch*0.5);
    const double cr = std::cos(roll*0.5),  sr = std::sin(roll*0.5);
    w = cy*cp*cr + sy*sp*sr;
    x = cy*cp*sr - sy*sp*cr;
    y = cy*sp*cr + sy*cp*sr;
    z = sy*cp*cr - cy*sp*sr;
    quatNorm(w,x,y,z);
  }

  static void quatToRPY(double w,double x,double y,double z,
                        double &roll,double &pitch,double &yaw)
  {
    // ZYX
    const double sinr_cosp = 2*(w*x + y*z);
    const double cosr_cosp = 1 - 2*(x*x + y*y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2*(w*y - z*x);
    pitch = (std::abs(sinp) >= 1) ? std::copysign(M_PI/2, sinp) : std::asin(sinp);

    const double siny_cosp = 2*(w*z + x*y);
    const double cosy_cosp = 1 - 2*(y*y + z*z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
  }

  static double yawFromQuat(double w,double x,double y,double z)
  {
    const double siny_cosp = 2*(w*z + x*y);
    const double cosy_cosp = 1 - 2*(y*y + z*z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static void slerp(double w1,double x1,double y1,double z1,
                    double w2,double x2,double y2,double z2,
                    double t,
                    double &wo,double &xo,double &yo,double &zo)
  {
    // SLERP clásico
    double cos_omega = w1*w2 + x1*x2 + y1*y2 + z1*z2;
    double wB=w2, xB=x2, yB=y2, zB=z2;
    if (cos_omega < 0.0) { cos_omega = -cos_omega; wB=-wB; xB=-xB; yB=-yB; zB=-zB; }

    const double EPS = 1e-6;
    double k0, k1;
    if (1.0 - cos_omega > EPS) {
      const double omega = std::acos(cos_omega);
      const double sin_omega = std::sin(omega);
      k0 = std::sin((1.0 - t)*omega)/sin_omega;
      k1 = std::sin(t*omega)/sin_omega;
    } else {
      // casi lineal
      k0 = 1.0 - t;
      k1 = t;
    }
    wo = k0*w1 + k1*wB;
    xo = k0*x1 + k1*xB;
    yo = k0*y1 + k1*yB;
    zo = k0*z1 + k1*zB;
    quatNorm(wo,xo,yo,zo);
  }

  static void quatToR(double w,double x,double y,double z, double R[9])
  {
    const double xx=x*x, yy=y*y, zz=z*z;
    const double xy=x*y, xz=x*z, yz=y*z;
    const double wx=w*x, wy=w*y, wz=w*z;

    // body->world (mundo = R * cuerpo)
    R[0] = 1 - 2*(yy + zz); R[1] = 2*(xy - wz);     R[2] = 2*(xz + wy);
    R[3] = 2*(xy + wz);     R[4] = 1 - 2*(xx + zz); R[5] = 2*(yz - wx);
    R[6] = 2*(xz - wy);     R[7] = 2*(yz + wx);     R[8] = 1 - 2*(xx + yy);
  }

  // ===== Miembros =====
  // parámetros
  double p0_, T_, z_ref_, M_, g_, Rgas_;
  double freq_hz_, alpha_rp_, alpha_yaw_, acc_tol_, decl_rad_, gyro_tau_s_;
  double comp_alpha_z_, vz_leak_hz_;
  bool use_external_R_;

  // tópicos
  std::string topic_imu_, topic_baro_, topic_mag_, topic_R_, topic_out_;

  // pubs/subs
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr sub_baro_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sub_mag_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_R_;
  rclcpp::TimerBase::SharedPtr timer_;

  // actitud (cuaternión world->imu)
  double qw_, qx_, qy_, qz_;
  bool initialized_{false};

  // cachés sensores
  // imu
  double ax_{0}, ay_{0}, az_{0}; bool have_acc_{false};
  double gx_{0}, gy_{0}, gz_{0}; bool have_gyro_{false};
  double gx_raw_{0}, gy_raw_{0}, gz_raw_{0}; bool have_raw_gyro_{false};
  double bias_gx_{0}, bias_gy_{0}, bias_gz_{0};
  bool have_imu_{false};
  // mag
  double mx_{0}, my_{0}, mz_{0}; bool have_mag_{false};
  // baro
  double z_baro_{0}; bool have_baro_{false}; bool z_inited_{false};

  // altitud
  double z_est_{0.0}, vz_est_{0.0};

  // R externa opcional
  bool have_R_ext_{false};
  std::array<double,9> R_ext_{ {1,0,0, 0,1,0, 0,0,1} };

  // timing
  std::optional<rclcpp::Time> last_stamp_;
  std::optional<rclcpp::Time> last_timer_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseEstimatorFusion>());
  rclcpp::shutdown();
  return 0;
}
