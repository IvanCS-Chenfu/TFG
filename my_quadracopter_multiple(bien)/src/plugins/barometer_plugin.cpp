#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

#include <string>
#include <memory>
#include <random>
#include <cmath>

namespace gazebo
{
class BarometerPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_  = gazebo_ros::Node::Get(sdf);

    // SDF defaults
    link_name_   = sdf->Get<std::string>("link_name_baro", "base_link").first;
    frame_id_    = sdf->Get<std::string>("frame_id_baro",  link_name_).first;
    topic_       = sdf->Get<std::string>("topic_baro",     "sensor/baro").first;
    update_rate_ = sdf->Get<double>("update_rate_baro",    50.0).first;

    ref_alt_     = sdf->Get<double>("ref_altitude_baro",   0.0).first;
    P0_          = sdf->Get<double>("p0_baro",             101325.0).first;
    T_           = sdf->Get<double>("T_baro",              288.15).first;

    noise_stddev_ = sdf->Get<double>("noise_stddev_baro",  0.0).first; // Pa
    bias_         = sdf->Get<double>("bias_baro",          0.0).first; // Pa

    param_ns_     = sdf->Get<std::string>("param_ns", "barometer").first;

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

    getParam("ref_altitude", ref_alt_);
    getParam("p0",           P0_);
    getParam("T",            T_);

    getParam("noise_stddev", noise_stddev_);
    getParam("bias",         bias_);

    // Resolver link
    link_ = model_->GetLink(link_name_);
    if (!link_) {
      const std::string scoped = model_->GetName() + "::" + link_name_;
      link_ = model_->GetLink(scoped);
    }
    if (!link_) {
      RCLCPP_ERROR(node_->get_logger(), "[BARO] link '%s' not found.", link_name_.c_str());
      for (auto &lnk : model_->GetLinks())
        RCLCPP_ERROR(node_->get_logger(), "  - '%s'", lnk->GetName().c_str());
      return;
    }

    rclcpp::QoS qos{rclcpp::SensorDataQoS()};
    pub_ = node_->create_publisher<sensor_msgs::msg::FluidPressure>(topic_, qos);
    RCLCPP_INFO(node_->get_logger(), "[BARO] publishing on: %s", pub_->get_topic_name());

    // Constantes físicas
    g_ = 9.80665;
    M_ = 0.0289644;
    R_ = 8.3144598;

    rng_.seed(std::random_device{}());
    noise_norm_ = std::normal_distribution<double>(0.0, noise_stddev_);

    last_update_ = model_->GetWorld()->SimTime();
    period_ = (update_rate_ > 0.0) ? common::Time(1.0 / update_rate_) : common::Time(0, 0);

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&BarometerPlugin::OnUpdate, this));
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

    const double z = link_->WorldPose().Pos().Z();
    const double exponent = -(M_ * g_ * (z - ref_alt_)) / (R_ * T_);
    double pressure = P0_ * std::exp(exponent);

    // Ruido/bias (si stddev=0 => ideal)
    pressure += bias_ + noise_norm_(rng_);

    sensor_msgs::msg::FluidPressure msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.fluid_pressure = pressure;
    msg.variance = noise_stddev_ * noise_stddev_; // 0 si ideal

    pub_->publish(msg);
  }

  // Members
  physics::ModelPtr model_;
  physics::LinkPtr  link_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_;
  event::ConnectionPtr update_conn_;

  // Params
  std::string link_name_, frame_id_, topic_, param_ns_;
  double update_rate_{50.0};
  double ref_alt_{0.0}, P0_{101325.0}, T_{288.15};
  double noise_stddev_{0.0}, bias_{0.0};

  // Física
  double g_{9.80665}, M_{0.0289644}, R_{8.3144598};

  // Timing
  common::Time last_update_;
  common::Time period_;

  // Noise
  std::mt19937 rng_;
  std::normal_distribution<double> noise_norm_{0.0, 0.0};
};

GZ_REGISTER_MODEL_PLUGIN(BarometerPlugin)
} // namespace gazebo
