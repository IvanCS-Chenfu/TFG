#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <cmath>
#include <string>
#include <optional>

class ImuQuatSanityTimerNode : public rclcpp::Node
{
public:
  ImuQuatSanityTimerNode() : rclcpp::Node("imu_quat_sanity_timer_node")
  {
    // Parámetros
    topic_imu_  = declare_parameter<std::string>("topic_imu", "/imu/data");
    topic_acc_  = declare_parameter<std::string>("topic_quat_from_acc",  "/test/quat_from_acc");
    topic_gyro_ = declare_parameter<std::string>("topic_quat_from_gyro", "/test/quat_from_gyro");
    g_          = declare_parameter<double>("gravity", 9.80665);
    acc_tol_    = declare_parameter<double>("acc_norm_tolerance", 3.0); // m/s^2
    freq_hz_    = declare_parameter<double>("frequency", 400.0);        // Hz de integración
    // (opcional) estimación de sesgo gyro por media móvil simple (0 = desactivado)
    gyro_bias_tau_s_ = declare_parameter<double>("gyro_bias_time_constant", 0.0);

    // Publishers (fiables por defecto)
    pub_acc_  = create_publisher<geometry_msgs::msg::QuaternionStamped>(topic_acc_,  rclcpp::QoS(10));
    pub_gyro_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(topic_gyro_, rclcpp::QoS(10));

    // Subscriber IMU (QoS de sensores)
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      topic_imu_,
      rclcpp::SensorDataQoS(),
      std::bind(&ImuQuatSanityTimerNode::onImu, this, std::placeholders::_1));

    // Estado integración giros (world->imu), arrancamos identidad
    qg_w_ = 1.0; qg_x_ = qg_y_ = qg_z_ = 0.0;

    // Timer de integración a frecuencia fija (como en tu ejemplo)
    const auto dt_ms = static_cast<int>(std::round(1000.0 / std::max(1.0, freq_hz_)));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(dt_ms),
      std::bind(&ImuQuatSanityTimerNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "IMU sanity (timer). Sub: %s | Pub(acc): %s | Pub(gyro): %s | f=%.1f Hz",
      topic_imu_.c_str(), topic_acc_.c_str(), topic_gyro_.c_str(), freq_hz_);
  }

private:
  // ======= Callbacks =======
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // cache IMU para el timer
    last_stamp_ = msg->header.stamp;

    last_ax_ = msg->linear_acceleration.x;
    last_ay_ = msg->linear_acceleration.y;
    last_az_ = msg->linear_acceleration.z;

    // estimación (opcional) de sesgo por LPF exponencial (asumiendo que estará quieto “a veces”)
    const double gx = msg->angular_velocity.x;
    const double gy = msg->angular_velocity.y;
    const double gz = msg->angular_velocity.z;

    if (gyro_bias_tau_s_ > 0.0) {
      // factor alpha ~ dt/tau lo aplicamos en el timer con dt exacto; aquí guardamos muestra
      last_gx_raw_ = gx;
      last_gy_raw_ = gy;
      last_gz_raw_ = gz;
      have_raw_gyro_ = true;
    } else {
      last_gx_ = gx;
      last_gy_ = gy;
      last_gz_ = gz;
      have_raw_gyro_ = false;
    }

    have_imu_ = true;
  }

  void onTimer()
  {
    if (!have_imu_) return;

    // ======= dt del timer (constante) =======
    const rclcpp::Time now = this->get_clock()->now();
    if (!last_timer_time_) last_timer_time_ = now;
    double dt = (now - *last_timer_time_).seconds();
    // si el reloj salta o es 0, usa periodo ideal de la frecuencia
    if (dt <= 0.0) dt = 1.0 / std::max(1.0, freq_hz_);
    last_timer_time_ = now;

    // ======= 1) Publicación "solo acelerómetros" =======
    publishAccQuat();

    // ======= 2) Integración de giros a frecuencia fija =======
    // a) actualiza sesgo si está habilitado (LPF exponencial)
    if (gyro_bias_tau_s_ > 0.0 && have_raw_gyro_) {
      const double alpha = std::clamp(dt / gyro_bias_tau_s_, 0.0, 1.0);
      // si asumimos tramos en reposo, el sesgo converge lento; puedes mejorar con detector de reposo
      bias_gx_ = (1.0 - alpha) * bias_gx_ + alpha * last_gx_raw_;
      bias_gy_ = (1.0 - alpha) * bias_gy_ + alpha * last_gy_raw_;
      bias_gz_ = (1.0 - alpha) * bias_gz_ + alpha * last_gz_raw_;
      last_gx_ = last_gx_raw_ - bias_gx_;
      last_gy_ = last_gy_raw_ - bias_gy_;
      last_gz_ = last_gz_raw_ - bias_gz_;
    }

    integrateGyro(dt, last_gx_, last_gy_, last_gz_);

    geometry_msgs::msg::QuaternionStamped qg;
    qg.header.stamp = last_stamp_.has_value() ? *last_stamp_ : now;
    qg.header.frame_id = "imu"; // opcional
    qg.quaternion.w = qg_w_;
    qg.quaternion.x = qg_x_;
    qg.quaternion.y = qg_y_;
    qg.quaternion.z = qg_z_;
    pub_gyro_->publish(qg);
  }

  // ======= Lógica: acc-only =======
  void publishAccQuat()
  {
    const double ax = last_ax_;
    const double ay = last_ay_;
    const double az = last_az_;

    const double an = std::sqrt(ax*ax + ay*ay + az*az);
    if (std::abs(an - g_) > acc_tol_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Accel norm far from g (|a|=%.3f). Acc-only attitude may be off.", an);
    }

    // roll_acc = atan2(ay, az) ; pitch_acc = atan2(-ax, sqrt(ay^2+az^2)); yaw=0
    const double roll_acc  = std::atan2(ay, az);
    const double pitch_acc = std::atan2(-ax, std::sqrt(ay*ay + az*az));
    const double yaw_acc = quatToYawZYX(qg_w_, qg_x_, qg_y_, qg_z_);

    geometry_msgs::msg::QuaternionStamped qa;
    qa.header.stamp = last_stamp_.has_value() ? *last_stamp_ : this->get_clock()->now();
    qa.header.frame_id = "imu"; // opcional
    rpyToQuat(yaw_acc, pitch_acc, roll_acc, qa.quaternion); // ZYX
    pub_acc_->publish(qa);
  }


  static double quatToYawZYX(double w, double x, double y, double z)
  {
    // yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
    const double siny_cosp = 2.0 * (w*z + x*y);
    const double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  // ======= Lógica: integración de giros =======
  void integrateGyro(double dt, double wx, double wy, double wz)
  {
    // Exponencial de rotación: dq = [cos(|w|dt/2), (w/|w|) sin(|w|dt/2)]
    const double omega = std::sqrt(wx*wx + wy*wy + wz*wz);
    double dq_w = 1.0, dq_x = 0.0, dq_y = 0.0, dq_z = 0.0;
    if (omega > 1e-12) {
      const double th2 = 0.5 * omega * dt; // theta/2
      const double s = std::sin(th2);
      const double c = std::cos(th2);
      const double ux = wx / omega;
      const double uy = wy / omega;
      const double uz = wz / omega;
      dq_w = c; dq_x = ux * s; dq_y = uy * s; dq_z = uz * s;
    }
    // q_{k+1} = q_k ⊗ dq   (w en marco cuerpo)
    multQuat(qg_w_, qg_x_, qg_y_, qg_z_, dq_w, dq_x, dq_y, dq_z,
             qg_w_, qg_x_, qg_y_, qg_z_);
    normalize(qg_w_, qg_x_, qg_y_, qg_z_);
  }

  // ======= Utilidades cuaterniones =======
  static void rpyToQuat(double yaw, double pitch, double roll,
                        geometry_msgs::msg::Quaternion &q)
  {
    const double cy = std::cos(yaw   * 0.5);
    const double sy = std::sin(yaw   * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll  * 0.5);
    const double sr = std::sin(roll  * 0.5);

    q.w = cy*cp*cr + sy*sp*sr;
    q.x = cy*cp*sr - sy*sp*cr;
    q.y = cy*sp*cr + sy*cp*sr;
    q.z = sy*cp*cr - cy*sp*sr;

    const double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n > 1e-12) { q.w/=n; q.x/=n; q.y/=n; q.z/=n; }
  }

  static void multQuat(double aw,double ax,double ay,double az,
                       double bw,double bx,double by,double bz,
                       double &ow,double &ox,double &oy,double &oz)
  {
    ow = aw*bw - ax*bx - ay*by - az*bz;
    ox = aw*bx + ax*bw + ay*bz - az*by;
    oy = aw*by - ax*bz + ay*bw + az*bx;
    oz = aw*bz + ax*by - ay*bx + az*bw;
  }

  static void normalize(double &w,double &x,double &y,double &z)
  {
    const double n = std::sqrt(w*w + x*x + y*y + z*z);
    if (n > 1e-12) { w/=n; x/=n; y/=n; z/=n; }
  }

  // ======= Miembros =======
  // parámetros
  std::string topic_imu_, topic_acc_, topic_gyro_;
  double g_, acc_tol_, freq_hz_;
  double gyro_bias_tau_s_;

  // pub/sub
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_acc_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_gyro_;
  rclcpp::TimerBase::SharedPtr timer_;

  // cachés IMU
  bool have_imu_{false};
  std::optional<rclcpp::Time> last_stamp_;
  double last_ax_{0.0}, last_ay_{0.0}, last_az_{0.0};
  double last_gx_{0.0}, last_gy_{0.0}, last_gz_{0.0};
  // crude bias estimator
  bool   have_raw_gyro_{false};
  double last_gx_raw_{0.0}, last_gy_raw_{0.0}, last_gz_raw_{0.0};
  double bias_gx_{0.0}, bias_gy_{0.0}, bias_gz_{0.0};

  // estado integración giros
  double qg_w_, qg_x_, qg_y_, qg_z_;

  // timing del timer
  std::optional<rclcpp::Time> last_timer_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuQuatSanityTimerNode>());
  rclcpp::shutdown();
  return 0;
}
