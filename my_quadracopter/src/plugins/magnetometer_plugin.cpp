#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <string>
#include <memory>
#include <random>

namespace gazebo
{
class MagnetometerPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_  = gazebo_ros::Node::Get(sdf);

    // SDF defaults
    link_name_   = sdf->Get<std::string>("link_name_mag", "base_link").first;
    frame_id_    = sdf->Get<std::string>("frame_id_mag",  link_name_).first;
    topic_       = sdf->Get<std::string>("topic_mag",     "sensor/mag").first;
    update_rate_ = sdf->Get<double>("update_rate_mag",    100.0).first;

    // Campo en WORLD
    B_world_.X() = sdf->Get<double>("Bx_world",  2.0e-5).first;
    B_world_.Y() = sdf->Get<double>("By_world",  0.0   ).first;
    B_world_.Z() = sdf->Get<double>("Bz_world", -4.0e-5).first;

    noise_stddev_ = sdf->Get<double>("noise_stddev_mag", 0.0).first;
    bias_T_       = sdf->Get<double>("bias_mag",         0.0).first;

    param_ns_     = sdf->Get<std::string>("param_ns", "magnetometer").first;

    auto trim = [](std::string& s){
      auto a = s.find_first_not_of(" \t\r\n");
      auto b = s.find_last_not_of(" \t\r\n");
      s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
    };
    trim(link_name_); trim(frame_id_); trim(topic_); trim(param_ns_);
    //if (!topic_.empty() && topic_.front() != '/') topic_ = "/" + topic_;

    // YAML overrides
    getParam("link_name",   link_name_);
    getParam("frame_id",    frame_id_);
    getParam("topic",       topic_);
    getParam("update_rate", update_rate_);

    // B_world: se puede dar como vector [Bx,By,Bz] o como tres dobles
    std::vector<double> Bvec{B_world_.X(), B_world_.Y(), B_world_.Z()};
    getParam("B_world", Bvec);
    if (Bvec.size() == 3) {
      B_world_.X() = Bvec[0]; B_world_.Y() = Bvec[1]; B_world_.Z() = Bvec[2];
    } else {
      // fallback por claves sueltas si no hay vector
      getParam("Bx_world", B_world_.X());
      getParam("By_world", B_world_.Y());
      getParam("Bz_world", B_world_.Z());
    }

    getParam("noise_stddev", noise_stddev_);
    getParam("bias",         bias_T_);

    // Resolver link
    link_ = model_->GetLink(link_name_);
    if (!link_) {
      const std::string scoped = model_->GetName() + "::" + link_name_;
      link_ = model_->GetLink(scoped);
    }
    if (!link_) {
      RCLCPP_ERROR(node_->get_logger(), "[MAG] link '%s' not found.", link_name_.c_str());
      for (auto &lnk : model_->GetLinks())
        RCLCPP_ERROR(node_->get_logger(), "  - '%s'", lnk->GetName().c_str());
      return;
    }

    rclcpp::QoS qos{rclcpp::SensorDataQoS()};
    pub_ = node_->create_publisher<sensor_msgs::msg::MagneticField>(topic_, qos);
    RCLCPP_INFO(node_->get_logger(), "[MAG] publishing on: %s", pub_->get_topic_name());

    rng_.seed(std::random_device{}());
    noise_norm_ = std::normal_distribution<double>(0.0, noise_stddev_);

    last_update_ = model_->GetWorld()->SimTime();
    period_ = (update_rate_ > 0.0) ? common::Time(1.0 / update_rate_) : common::Time(0, 0);

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MagnetometerPlugin::OnUpdate, this));
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

    const ignition::math::Quaterniond q_lw = link_->WorldPose().Rot();
    ignition::math::Vector3d B_sensor = q_lw.Inverse() * B_world_;

    // ruido/bias (0 => ideal)
    B_sensor.X() += bias_T_ + noise_norm_(rng_);
    B_sensor.Y() += bias_T_ + noise_norm_(rng_);
    B_sensor.Z() += bias_T_ + noise_norm_(rng_);

    sensor_msgs::msg::MagneticField msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.magnetic_field.x = B_sensor.X();
    msg.magnetic_field.y = B_sensor.Y();
    msg.magnetic_field.z = B_sensor.Z();

    const double var = noise_stddev_ * noise_stddev_;
    msg.magnetic_field_covariance[0] = var;
    msg.magnetic_field_covariance[4] = var;
    msg.magnetic_field_covariance[8] = var;

    pub_->publish(msg);
  }

  // Members
  physics::ModelPtr model_;
  physics::LinkPtr  link_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_;
  event::ConnectionPtr update_conn_;

  // Params
  std::string link_name_, frame_id_, topic_, param_ns_;
  double update_rate_{100.0};
  ignition::math::Vector3d B_world_{2.0e-5, 0.0, -4.0e-5};
  double noise_stddev_{0.0}, bias_T_{0.0};

  // Timing
  common::Time last_update_;
  common::Time period_;

  // Noise
  std::mt19937 rng_;
  std::normal_distribution<double> noise_norm_{0.0, 0.0};
};

GZ_REGISTER_MODEL_PLUGIN(MagnetometerPlugin)
} // namespace gazebo
