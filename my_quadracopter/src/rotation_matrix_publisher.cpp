#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <string>
#include <array>
#include <cmath>

class RotationMatrixPublisher : public rclcpp::Node {
public:
  RotationMatrixPublisher() : rclcpp::Node("rotation_matrix_publisher") {
    topic_pose_in_  = declare_parameter<std::string>("topics.pose_in", "/ground_truth/pose");
    topic_R_out_    = declare_parameter<std::string>("topics.R_out",   "/R");
    fallback_pose_  = declare_parameter<bool>("fallback_pose", false); // <- escucha Pose si true

    // Publisher
    pub_R_ = create_publisher<std_msgs::msg::Float64MultiArray>(topic_R_out_, rclcpp::QoS(10));

    // Suscriptor principal: PoseStamped con QoS de sensor (BestEffort, volatile)
    sub_pose_st_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      topic_pose_in_, rclcpp::SensorDataQoS(),
      std::bind(&RotationMatrixPublisher::onPoseStamped, this, std::placeholders::_1));

    // Fallback opcional: si el tópico realmente es geometry_msgs/Pose
    if (fallback_pose_) {
      sub_pose_ = create_subscription<geometry_msgs::msg::Pose>(
        topic_pose_in_, rclcpp::SensorDataQoS(),
        std::bind(&RotationMatrixPublisher::onPose, this, std::placeholders::_1));
    }

    RCLCPP_INFO(get_logger(),
      "RotationMatrixPublisher: pose_in=%s (%s) -> R_out=%s | fallback_pose=%s",
      topic_pose_in_.c_str(),
      "PoseStamped",
      topic_R_out_.c_str(),
      fallback_pose_ ? "true" : "false");
  }

private:
  // === Callbacks ===
  void onPoseStamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    publishFromQuat(msg->pose.orientation);
  }

  void onPose(const geometry_msgs::msg::Pose::SharedPtr msg) { // fallback
    publishFromQuat(msg->orientation);
  }

  // Convierte cuaternión a R (row-major) y publica
  void publishFromQuat(const geometry_msgs::msg::Quaternion &qmsg) {
    double qw = qmsg.w, qx = qmsg.x, qy = qmsg.y, qz = qmsg.z;
    
    RCLCPP_INFO(this->get_logger(), "q = [%.3f, %.3f, %.3f, %.3f]", qx, qy, qz, qw);
    
    // Normaliza por seguridad
    const double n = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (n < 1e-12) {
      publishIdentity();
      return;
    }
    qw /= n; qx /= n; qy /= n; qz /= n;

    // Matriz de rotación (mundo = R * cuerpo) en fila mayor
    const double xx = qx*qx, yy = qy*qy, zz = qz*qz;
    const double xy = qx*qy, xz = qx*qz, yz = qy*qz;
    const double wx = qw*qx, wy = qw*qy, wz = qw*qz;

    std_msgs::msg::Float64MultiArray out;
    out.data = {
      1.0 - 2.0*(yy + zz),  2.0*(xy - wz),        2.0*(xz + wy),
      2.0*(xy + wz),        1.0 - 2.0*(xx + zz),  2.0*(yz - wx),
      2.0*(xz - wy),        2.0*(yz + wx),        1.0 - 2.0*(xx + yy)
    };

    // Layout opcional (3x3, row-major)
    out.layout.dim.resize(2);
    out.layout.dim[0].label = "rows"; out.layout.dim[0].size = 3; out.layout.dim[0].stride = 9;
    out.layout.dim[1].label = "cols"; out.layout.dim[1].size = 3; out.layout.dim[1].stride = 3;
    out.layout.data_offset = 0;

    pub_R_->publish(out);
  }

  void publishIdentity() {
    std_msgs::msg::Float64MultiArray out;
    out.data = {1,0,0, 0,1,0, 0,0,1};
    out.layout.dim.resize(2);
    out.layout.dim[0].label = "rows"; out.layout.dim[0].size = 3; out.layout.dim[0].stride = 9;
    out.layout.dim[1].label = "cols"; out.layout.dim[1].size = 3; out.layout.dim[1].stride = 3;
    out.layout.data_offset = 0;
    pub_R_->publish(out);
  }

  // Params / topics
  std::string topic_pose_in_, topic_R_out_;
  bool fallback_pose_;

  // Pub/Sub
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_R_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_st_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose_; // opcional
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RotationMatrixPublisher>());
  rclcpp::shutdown();
  return 0;
}
