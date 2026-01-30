#include <chrono>
#include <memory>
#include <tuple>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "my_quadracopter/action/poly3_traj.hpp"

class ForceFromFeedback : public rclcpp::Node {
public:
  using Poly3Traj = my_quadracopter::action::Poly3Traj;
  using FeedbackMsg = Poly3Traj::Impl::FeedbackMessage; // <- suscripción "raw" al tópico de feedback

  ForceFromFeedback() : Node("force_from_feedback") {
    // Ganancias y masa
    kp_ = this->declare_parameter<double>("kp", 13.0);
    kv_ = this->declare_parameter<double>("kv", 12.0);
    m_  = this->declare_parameter<double>("mass", 1.4);
    // Gravedad en I/world
    gI_.x = this->declare_parameter<double>("g_x", 0.0);
    gI_.y = this->declare_parameter<double>("g_y", 0.0);
    gI_.z = this->declare_parameter<double>("g_z", 9.81);

    // Nombre de la acción para localizar el topic de feedback
    action_name_ = this->declare_parameter<std::string>("action_name", "poly3_traj");
    feedback_topic_ = action_name_ + "/_action/feedback";

    pub_F_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("tray_avpt/Fuerzas", 10);

    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "ground_truth/pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
        p_.x = msg->pose.position.x; p_.y = msg->pose.position.y; p_.z = msg->pose.position.z;
        have_p_ = true; compute_and_publish();
      });

    sub_vel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "ground_truth/vel", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg){
        v_.x = msg->twist.linear.x; v_.y = msg->twist.linear.y; v_.z = msg->twist.linear.z;
        have_v_ = true; compute_and_publish();
      });

    sub_acc_ = this->create_subscription<geometry_msgs::msg::AccelStamped>(
      "ground_truth/acc", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::AccelStamped::ConstSharedPtr msg){
        a_gt_.x = msg->accel.linear.x; a_gt_.y = msg->accel.linear.y; a_gt_.z = msg->accel.linear.z;
        have_a_gt_ = true; compute_and_publish();
      });

    // Suscripción directa al feedback del action server
    sub_feedback_ = this->create_subscription<FeedbackMsg>(
      feedback_topic_, rclcpp::QoS(50),
      [this](FeedbackMsg::ConstSharedPtr fb){
        auto get3 = [](const std_msgs::msg::Float64MultiArray &arr){
          struct { double p, v, a; } out{0,0,0};
          if (arr.data.size() >= 3) { out.p = arr.data[0]; out.v = arr.data[1]; out.a = arr.data[2]; }
          return out;
        };
        auto X = get3(fb->feedback.x);
        auto Y = get3(fb->feedback.y);
        auto Z = get3(fb->feedback.z);
        pd_.x = X.p; vd_.x = X.v; ad_.x = X.a;
        pd_.y = Y.p; vd_.y = Y.v; ad_.y = Y.a;
        pd_.z = Z.p; vd_.z = Z.v; ad_.z = Z.a;
        have_ref_ = true;
        compute_and_publish();
      });

    RCLCPP_INFO(get_logger(), "Subscribiéndose a feedback: %s", feedback_topic_.c_str());
  }

private:
  // Tipos simples
  struct Vec3 { double x{0}, y{0}, z{0}; };

  // Estado GT
  Vec3 p_{}, v_{}, a_gt_{};
  bool have_p_{false}, have_v_{false}, have_a_gt_{false};

  // Referencias d(t)
  Vec3 pd_{}, vd_{}, ad_{};
  bool have_ref_{false};

  // Parámetros
  double kp_{3.0}, kv_{2.0}, m_{1.0};
  Vec3 gI_{0.0, 0.0, 9.81};

  // ROS
  std::string action_name_, feedback_topic_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_F_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr sub_acc_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr sub_feedback_;

  void compute_and_publish()
  {
    if (!(have_p_ && have_v_ && have_a_gt_ && have_ref_)) return;

    // e_p = p - p_d ; e_v = v - v_d
    //Vec3 ep{ 0, 0, p_.z - pd_.z };
    //Vec3 ev{ 0, 0, v_.z - vd_.z };

    Vec3 ep{ p_.x - pd_.x, p_.y - pd_.y, p_.z - pd_.z };
    Vec3 ev{ v_.x - vd_.x, v_.y - vd_.y, v_.z - vd_.z };

    //RCLCPP_INFO(this->get_logger(), "ep = [%.3f, %.3f, %.3f],  ev = [%.3f, %.3f, %.3f]", ep.x, ep.y, ep.z, ev.x, ev.y, ev.z);

    // F = -Kp*ep - Kv*ev + m*(ad + gI)
    Vec3 F;
    F.x = -kp_*ep.x - kv_*ev.x + m_*(ad_.x + gI_.x);
    F.y = -kp_*ep.y - kv_*ev.y + m_*(ad_.y + gI_.y);
    F.z = -kp_*ep.z - kv_*ev.z + m_*(ad_.z + gI_.z);
    
    
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "world";
    msg.vector.x = F.x;
    msg.vector.y = F.y;
    msg.vector.z = F.z;
    pub_F_->publish(msg);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForceFromFeedback>());
  rclcpp::shutdown();
  return 0;
}
