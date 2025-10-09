#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "my_quadracopter/action/poly3_traj.hpp"

using Poly3Traj = my_quadracopter::action::Poly3Traj;
using GoalHandle = rclcpp_action::ClientGoalHandle<Poly3Traj>;

class Poly3TrajLauncher : public rclcpp::Node {
public:
  Poly3TrajLauncher() : Node("poly3_traj_launcher") {
    // Parámetros (idénticos al CLI que usabas)
    action_name_ = this->declare_parameter<std::string>("action_name", "/poly3_traj");
    goal_x_ = this->declare_parameter<double>("goal_x", 3.0);
    goal_y_ = this->declare_parameter<double>("goal_y", 2.0);
    goal_z_ = this->declare_parameter<double>("goal_z", 1.0);
    // orientación como quat (por defecto z=1, w=0 como en tu ejemplo)
    yaw_qz_ = this->declare_parameter<double>("goal_yaw_quat_z", 1.0);
    yaw_qw_ = this->declare_parameter<double>("goal_yaw_quat_w", 0.0);
    tx_ = this->declare_parameter<double>("tx",   5.0);
    ty_ = this->declare_parameter<double>("ty",   5.0);
    tz_ = this->declare_parameter<double>("tz",   15.0);
    tyaw_ = this->declare_parameter<double>("tyaw", 5.0);

    client_ = rclcpp_action::create_client<Poly3Traj>(this, action_name_);

    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Servidor de acción no disponible: %s", action_name_.c_str());
      return;
    }
    send_goal();
  }

private:
  rclcpp_action::Client<Poly3Traj>::SharedPtr client_;
  std::string action_name_;
  double goal_x_, goal_y_, goal_z_, yaw_qz_, yaw_qw_, tx_, ty_, tz_, tyaw_;

  void send_goal() {
    Poly3Traj::Goal goal;
    goal.target_pose.position.x = goal_x_;
    goal.target_pose.position.y = goal_y_;
    goal.target_pose.position.z = goal_z_;
    goal.target_pose.orientation.x = 0.0;
    goal.target_pose.orientation.y = 0.0;
    goal.target_pose.orientation.z = yaw_qz_;
    goal.target_pose.orientation.w = yaw_qw_;
    goal.tx = static_cast<float>(tx_);
    goal.ty = static_cast<float>(ty_);
    goal.tz = static_cast<float>(tz_);
    goal.tyaw = static_cast<float>(tyaw_);

    rclcpp_action::Client<Poly3Traj>::SendGoalOptions opt;
    opt.goal_response_callback = [this](std::shared_ptr<GoalHandle> gh){
      if (!gh) RCLCPP_ERROR(this->get_logger(), "Goal rechazado");
      else     RCLCPP_INFO(this->get_logger(), "Goal aceptado");
    };
    opt.feedback_callback = [this](std::shared_ptr<GoalHandle>, const std::shared_ptr<const Poly3Traj::Feedback> fb){
      (void)fb; // opcional: imprimir progreso si quieres
    };
    opt.result_callback = [this](const GoalHandle::WrappedResult &res){
      if (res.code == rclcpp_action::ResultCode::SUCCEEDED)
        RCLCPP_INFO(this->get_logger(), "Acción SUCCEEDED, t_total=%.3f", res.result->t_total);
      else
        RCLCPP_WARN(this->get_logger(), "Acción terminó con código %d", (int)res.code);
    };

    client_->async_send_goal(goal, opt);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Poly3TrajLauncher>());
  rclcpp::shutdown();
  return 0;
}
