#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>

#include <string>
#include <memory>
#include <deque>   // <-- para la media móvil

namespace gazebo
{
class GroundTruthPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    node_  = gazebo_ros::Node::Get(sdf);

    // Defaults SDF (retro-compat)
    link_name_    = sdf->Get<std::string>("link_name_gt", "base_link").first;
    frame_id_     = sdf->Get<std::string>("frame_id_gt",  "world").first;
    topic_pose_   = sdf->Get<std::string>("topic_gt",     "ground_truth/pose").first;
    topic_vel_    = sdf->Get<std::string>("topic_vel_gt", "ground_truth/vel").first;
    topic_acc_    = sdf->Get<std::string>("topic_acc_gt", "ground_truth/acc").first;
    update_rate_  = sdf->Get<double>("update_rate_gt",    100.0).first;

    // Namespace YAML
    param_ns_     = sdf->Get<std::string>("param_ns", "ground_truth").first;

    auto trim = [](std::string& s){
      auto a = s.find_first_not_of(" \t\r\n");
      auto b = s.find_last_not_of(" \t\r\n");
      s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
    };
    trim(link_name_); trim(frame_id_); trim(topic_pose_); trim(topic_vel_); trim(topic_acc_); trim(param_ns_);
    if (!topic_pose_.empty() && topic_pose_.front() != '/') topic_pose_ = "/" + topic_pose_;
    //if (!topic_vel_.empty()  && topic_vel_.front()  != '/') topic_vel_  = "/" + topic_vel_;
    //if (!topic_acc_.empty()  && topic_acc_.front()  != '/') topic_acc_  = "/" + topic_acc_;

    // Parámetros YAML (sobrescriben si existen)
    getParam("link_name",       link_name_);
    getParam("frame_id",        frame_id_);
    getParam("topic",           topic_pose_);   // compat: "topic" => pose
    getParam("topic_vel",       topic_vel_);
    getParam("topic_acc",       topic_acc_);
    getParam("update_rate",     update_rate_);
    getParam("accel_avg_window", accel_avg_window_); // NUEVO

    // Resolver link
    link_ = model_->GetLink(link_name_);
    if (!link_) {
      const std::string scoped = model_->GetName() + "::" + link_name_;
      link_ = model_->GetLink(scoped);
    }
    if (!link_) {
      RCLCPP_ERROR(node_->get_logger(), "[GT] link '%s' not found.", link_name_.c_str());
      for (auto &lnk : model_->GetLinks())
        RCLCPP_ERROR(node_->get_logger(), "  - '%s'", lnk->GetName().c_str());
      return;
    }

    // Publishers
    rclcpp::QoS qos{rclcpp::SensorDataQoS()};
    pub_pose_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_pose_, qos);
    pub_vel_  = node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic_vel_, qos);
    pub_acc_  = node_->create_publisher<geometry_msgs::msg::AccelStamped>(topic_acc_, qos);

    RCLCPP_INFO(node_->get_logger(),
      "[GT] pose='%s' vel='%s' acc='%s' | frame='%s' | rate=%.1f Hz | accel_avg_window=%d",
      pub_pose_->get_topic_name(), pub_vel_->get_topic_name(), pub_acc_->get_topic_name(),
      frame_id_.c_str(), update_rate_, accel_avg_window_);

    // Timing
    last_update_ = model_->GetWorld()->SimTime();
    period_ = (update_rate_ > 0.0) ? common::Time(1.0 / update_rate_) : common::Time(0, 0);

    // Buffers media móvil aceleración
    acc_lin_sum_.Set(0,0,0);
    acc_ang_sum_.Set(0,0,0);
    acc_lin_buf_.clear();
    acc_ang_buf_.clear();

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GroundTruthPlugin::OnUpdate, this));
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

    // Pose en WORLD
    const ignition::math::Pose3d pose = link_->WorldPose();
    const ignition::math::Vector3d p = pose.Pos();
    const ignition::math::Quaterniond q = pose.Rot();

    // Velocidades en WORLD
    const ignition::math::Vector3d v_lin = link_->WorldLinearVel();
    const ignition::math::Vector3d v_ang = link_->WorldAngularVel();

    // Aceleraciones en WORLD (sin filtrar)
    ignition::math::Vector3d a_lin_raw = link_->WorldLinearAccel();
    ignition::math::Vector3d a_ang_raw = link_->WorldAngularAccel();

    // === MEDIA MÓVIL SOBRE ACELERACIÓN (lineal y angular) ===
    ignition::math::Vector3d a_lin_out = a_lin_raw;
    ignition::math::Vector3d a_ang_out = a_ang_raw;

    if (accel_avg_window_ > 1) {
      // Lineal
      acc_lin_buf_.push_back(a_lin_raw);
      acc_lin_sum_ += a_lin_raw;
      if ((int)acc_lin_buf_.size() > accel_avg_window_) {
        acc_lin_sum_ -= acc_lin_buf_.front();
        acc_lin_buf_.pop_front();
      }
      const double invN_lin = 1.0 / std::max(1, (int)acc_lin_buf_.size());
      a_lin_out = acc_lin_sum_ * invN_lin;

      // Angular
      acc_ang_buf_.push_back(a_ang_raw);
      acc_ang_sum_ += a_ang_raw;
      if ((int)acc_ang_buf_.size() > accel_avg_window_) {
        acc_ang_sum_ -= acc_ang_buf_.front();
        acc_ang_buf_.pop_front();
      }
      const double invN_ang = 1.0 / std::max(1, (int)acc_ang_buf_.size());
      a_ang_out = acc_ang_sum_ * invN_ang;
    }

    // PoseStamped
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = node_->get_clock()->now();
    pose_msg.header.frame_id = frame_id_;
    pose_msg.pose.position.x = p.X();
    pose_msg.pose.position.y = p.Y();
    pose_msg.pose.position.z = p.Z();
    pose_msg.pose.orientation.x = q.X();
    pose_msg.pose.orientation.y = q.Y();
    pose_msg.pose.orientation.z = q.Z();
    pose_msg.pose.orientation.w = q.W();
    pub_pose_->publish(pose_msg);

    // TwistStamped
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header = pose_msg.header;
    twist_msg.twist.linear.x  = v_lin.X();
    twist_msg.twist.linear.y  = v_lin.Y();
    twist_msg.twist.linear.z  = v_lin.Z();
    twist_msg.twist.angular.x = v_ang.X();
    twist_msg.twist.angular.y = v_ang.Y();
    twist_msg.twist.angular.z = v_ang.Z();
    pub_vel_->publish(twist_msg);

    // AccelStamped (AQUÍ ya con media)
    geometry_msgs::msg::AccelStamped accel_msg;
    accel_msg.header = pose_msg.header;
    accel_msg.accel.linear.x  = a_lin_out.X();
    accel_msg.accel.linear.y  = a_lin_out.Y();
    accel_msg.accel.linear.z  = a_lin_out.Z();
    accel_msg.accel.angular.x = a_ang_out.X();
    accel_msg.accel.angular.y = a_ang_out.Y();
    accel_msg.accel.angular.z = a_ang_out.Z();
    pub_acc_->publish(accel_msg);
  }

  // Members
  physics::ModelPtr model_;
  physics::LinkPtr  link_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_vel_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr pub_acc_;
  event::ConnectionPtr update_conn_;

  // Params
  std::string link_name_, frame_id_, topic_pose_, topic_vel_, topic_acc_, param_ns_;
  double update_rate_{100.0};
  int    accel_avg_window_{50}; // NUEVO

  // Buffers media móvil (aceleración)
  std::deque<ignition::math::Vector3d> acc_lin_buf_;
  std::deque<ignition::math::Vector3d> acc_ang_buf_;
  ignition::math::Vector3d acc_lin_sum_{0,0,0};
  ignition::math::Vector3d acc_ang_sum_{0,0,0};

  // Timing
  common::Time last_update_;
  common::Time period_;
};

GZ_REGISTER_MODEL_PLUGIN(GroundTruthPlugin)
} // namespace gazebo
