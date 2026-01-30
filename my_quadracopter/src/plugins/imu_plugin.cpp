#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <string>
#include <memory>
#include <random>
#include <deque>

namespace gazebo
{
class ImuPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_  = gazebo_ros::Node::Get(sdf);

    // === Parámetros SDF (por compatibilidad) ===
    link_name_      = sdf->Get<std::string>("link_name_imu",   "imu_link").first;
    frame_id_       = sdf->Get<std::string>("frame_id_imu",    link_name_).first;
    topic_          = sdf->Get<std::string>("topic_imu",       "sensor/imu").first;
    update_rate_    = sdf->Get<double>("update_rate_imu",      200.0).first;
    remove_gravity_ = sdf->Get<bool>("remove_gravity_imu",     false).first; // ignorado (publicamos f=a-g)
    param_ns_       = sdf->Get<std::string>("param_ns", "imu").first;

    auto trim = [](std::string& s){
      auto a = s.find_first_not_of(" \t\r\n");
      auto b = s.find_last_not_of(" \t\r\n");
      s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
    };
    trim(link_name_); trim(frame_id_); trim(topic_); trim(param_ns_);
    //if (!topic_.empty() && topic_.front() != '/') topic_ = "/" + topic_;

    // === Parámetros ROS 2 (sobrescriben SDF si existen) ===
    getParam("link_name",      link_name_);
    getParam("frame_id",       frame_id_);
    getParam("topic",          topic_);
    getParam("update_rate",    update_rate_);

    // Ruido/bias opcional (0 => ideal)
    getParam("gyro_noise_stddev",  gyro_noise_stddev_);    // rad/s
    getParam("accel_noise_stddev", accel_noise_stddev_);   // m/s^2
    getParam("gyro_bias",          gyro_bias_);
    getParam("accel_bias",         accel_bias_);

    // NUEVO: ventana de media para aceleración
    getParam("accel_avg_window", accel_avg_window_);       // p.ej. 50

    RCLCPP_INFO(node_->get_logger(),
      "[IMU] model='%s' link='%s' frame='%s' topic='%s' rate=%.1f Hz | accel_avg_window=%d",
      model_->GetName().c_str(), link_name_.c_str(), frame_id_.c_str(), topic_.c_str(),
      update_rate_, accel_avg_window_);

    // Resolver link
    link_ = model_->GetLink(link_name_);
    if (!link_) {
      const std::string scoped = model_->GetName() + "::" + link_name_;
      link_ = model_->GetLink(scoped);
    }
    if (!link_) {
      RCLCPP_ERROR(node_->get_logger(), "[IMU] link '%s' not found.", link_name_.c_str());
      return;
    }

    // Publisher
    rclcpp::QoS qos{rclcpp::SensorDataQoS()};
    pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_, qos);

    // Ruido
    rng_.seed(std::random_device{}());
    norm_gyro_  = std::normal_distribution<double>(0.0, gyro_noise_stddev_);
    norm_accel_ = std::normal_distribution<double>(0.0, accel_noise_stddev_);

    // Timing
    last_update_ = model_->GetWorld()->SimTime();
    period_ = (update_rate_ > 0.0) ? common::Time(1.0 / update_rate_) : common::Time(0, 0);

    // Buffer media
    acc_sum_.Set(0,0,0);
    acc_buf_.clear();

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ImuPlugin::OnUpdate, this));
  }

private:
  template<typename T>
  void getParam(const std::string& key, T& value)
  {
    const std::string full = param_ns_.empty() ? key : (param_ns_ + "." + key);
    if (!node_->has_parameter(full)) node_->declare_parameter<T>(full, value);
    (void)node_->get_parameter(full, value);
  }

  void OnUpdate()
  {
    const auto now = model_->GetWorld()->SimTime();
    if (update_rate_ > 0.0 && (now - last_update_) < period_) return;
    last_update_ = now;

    // Orientación
    const ignition::math::Quaterniond q_lw = link_->WorldPose().Rot(); // link->world
    const ignition::math::Quaterniond q_wl = q_lw.Inverse();           // world->link (IMU)

    // Vel. angular (en IMU)
    const ignition::math::Vector3d w_world = link_->WorldAngularVel();
    ignition::math::Vector3d w_imu = q_wl * w_world;

    // Aceleración específica: f = a - g
    const ignition::math::Vector3d a_world = link_->WorldLinearAccel();
    const ignition::math::Vector3d g_world = model_->GetWorld()->Gravity(); // ~ (0,0,-9.81)
    const ignition::math::Vector3d f_world = a_world - g_world;
    ignition::math::Vector3d f_imu = q_wl * f_world;

    // Añadir ruido/bias (si stddev=0 => ideal)
    w_imu.X() += gyro_bias_  + norm_gyro_(rng_);
    w_imu.Y() += gyro_bias_  + norm_gyro_(rng_);
    w_imu.Z() += gyro_bias_  + norm_gyro_(rng_);
    f_imu.X() += accel_bias_ + norm_accel_(rng_);
    f_imu.Y() += accel_bias_ + norm_accel_(rng_);
    f_imu.Z() += accel_bias_ + norm_accel_(rng_);

    // === MEDIA MÓVIL SIMPLE SOBRE ACELERACIÓN (N muestras) ===
    ignition::math::Vector3d accel_out = f_imu;
    if (accel_avg_window_ > 1) {
      acc_buf_.push_back(f_imu);
      acc_sum_ += f_imu;
      if ((int)acc_buf_.size() > accel_avg_window_) {
        acc_sum_ -= acc_buf_.front();
        acc_buf_.pop_front();
      }
      const double invN = 1.0 / std::max(1, (int)acc_buf_.size());
      accel_out = acc_sum_ * invN;
    }

    // Mensaje
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = frame_id_;

    // Orientación world->imu (REP-145)
    msg.orientation.x = q_wl.X();
    msg.orientation.y = q_wl.Y();
    msg.orientation.z = q_wl.Z();
    msg.orientation.w = q_wl.W();

    msg.angular_velocity.x    = w_imu.X();
    msg.angular_velocity.y    = w_imu.Y();
    msg.angular_velocity.z    = w_imu.Z();

    msg.linear_acceleration.x = accel_out.X();
    msg.linear_acceleration.y = accel_out.Y();
    msg.linear_acceleration.z = accel_out.Z();

    for (int i = 0; i < 9; ++i) {
      msg.orientation_covariance[i]         = 0.0;
      msg.angular_velocity_covariance[i]    = 0.0;
      msg.linear_acceleration_covariance[i] = 0.0;
    }

    pub_->publish(msg);
  }

  // Members
  physics::ModelPtr model_;
  physics::LinkPtr  link_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  event::ConnectionPtr update_conn_;

  // Params (SDF/YAML)
  std::string link_name_, frame_id_, topic_, param_ns_;
  double update_rate_{200.0};
  bool   remove_gravity_{false}; // ignorado (publicamos f=a-g)

  // Ruido IMU
  double gyro_noise_stddev_{0.0}, accel_noise_stddev_{0.0};
  double gyro_bias_{0.0}, accel_bias_{0.0};
  std::mt19937 rng_;
  std::normal_distribution<double> norm_gyro_{0.0, 0.0};
  std::normal_distribution<double> norm_accel_{0.0, 0.0};

  // Media móvil
  int accel_avg_window_{50};
  std::deque<ignition::math::Vector3d> acc_buf_;
  ignition::math::Vector3d acc_sum_{0,0,0};

  // Timing
  common::Time last_update_;
  common::Time period_;
};

GZ_REGISTER_MODEL_PLUGIN(ImuPlugin)
} // namespace gazebo
