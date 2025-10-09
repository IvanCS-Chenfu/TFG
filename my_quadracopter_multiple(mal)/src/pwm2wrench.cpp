#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/wrench.hpp"

class MotorWrenchNode : public rclcpp::Node
{
public:
  MotorWrenchNode() : Node("motor_wrench_node")
  {
    // Declare configurable parameters with defaults
    kf_ = this->declare_parameter<double>("kf", 1.0);  // force gain
    kt_ = this->declare_parameter<double>("kt", 1.0);  // torque gain

    // Create publishers for each motor topic
    pub_front_ = this->create_publisher<geometry_msgs::msg::Wrench>("/drone/front_right_motor", 10);
    pub_back_  = this->create_publisher<geometry_msgs::msg::Wrench>("/drone/back_left_motor", 10);
    pub_left_  = this->create_publisher<geometry_msgs::msg::Wrench>("/drone/front_left_motor", 10);
    pub_right_ = this->create_publisher<geometry_msgs::msg::Wrench>("/drone/back_right_motor", 10);

    // Subscribe to pwm_data (expects 4 elements: [front, back, left, right])
    sub_pwm_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "pwm_data", 10,
      std::bind(&MotorWrenchNode::pwmCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "motor_wrench_node started. Waiting for pwm_data… (kf=%.3f, kt=%.3f)", kf_, kt_);
  }

private:
  void pwmCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 4) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "pwm_data size is %zu (expected 4). Ignoring.", msg->data.size());
      return;
    }

    // Order: [front, back, left, right]
    const float pwm_front_left  = msg->data[0];
    const float pwm_front_right = msg->data[1];
    const float pwm_back_left  = msg->data[2];
    const float pwm_back_right = msg->data[3];

    publishWrench(pub_left_,  pwm_front_left, true);
    publishWrench(pub_front_, pwm_front_right, false);
    publishWrench(pub_back_,  pwm_back_left, false);
    publishWrench(pub_right_, pwm_back_right, true);
  }

  void publishWrench(const rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr& pub,
                     float pwm_value, bool positive_turn)
  {
    geometry_msgs::msg::Wrench w;
    // Only z-components are set per your spec
    w.force.z  = static_cast<double>(pwm_value) * kf_;
    w.torque.z = static_cast<double>(pwm_value) * kt_;

    if (!positive_turn)
    {
        w.torque.z = -w.torque.z;
    }

    pub->publish(w);
  }

  // Members
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_pwm_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_front_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_back_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_left_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_right_;
  double kf_;
  double kt_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorWrenchNode>());
  rclcpp::shutdown();
  return 0;
}
