#ifndef QUADCOPTER_PLUGIN_HPP
#define QUADCOPTER_PLUGIN_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  class QuadcopterPlugin : public ModelPlugin
  {
  public:
    QuadcopterPlugin();
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

  private:
    void OnRosMsg(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void ApplyForces();

    physics::ModelPtr model_;
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;

    std::vector<int> pwm_values_;
    double coef_pwm_w_;
    double coef_force_;
    double coef_torque_;
    double radius_;
    std::vector<std::string> motor_links_;

    bool links_initialized_;  // Nuevo: indica si los links fueron encontrados
    std::vector<gazebo::physics::LinkPtr> links_;  // Nuevo: punteros a los links motores
  };
}

#endif
