#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <string>
#include <vector>
#include <cmath>

class RotationMatrixPublisher : public rclcpp::Node {
public:
  RotationMatrixPublisher() : rclcpp::Node("rotation_matrix_publisher") {
    // Parámetros
    num_drones_   = declare_parameter<int>("num_drones", 1);
    ns_prefix_    = declare_parameter<std::string>("ns_prefix", "/drone_");
    pose_suffix_  = declare_parameter<std::string>("pose_suffix", "/ground_truth/pose");
    R_suffix_     = declare_parameter<std::string>("R_suffix", "/R");
    fallback_pose_= declare_parameter<bool>("fallback_pose", false);

    if (num_drones_ < 1) {
      RCLCPP_WARN(get_logger(), "num_drones < 1, ajustando a 1");
      num_drones_ = 1;
    }

    pubs_.resize(num_drones_);
    subs_pose_st_.resize(num_drones_);
    if (fallback_pose_) subs_pose_.resize(num_drones_);

    // Crear canales por dron
    for (int i = 0; i < num_drones_; ++i) {
      const std::string in_topic  = buildAbs(ns_prefix_, i, pose_suffix_);
      const std::string out_topic = buildAbs(ns_prefix_, i, R_suffix_);

      // Publisher por dron
      pubs_[i] = create_publisher<std_msgs::msg::Float64MultiArray>(out_topic, rclcpp::QoS(10));

      // Subscriber principal PoseStamped
      subs_pose_st_[i] = create_subscription<geometry_msgs::msg::PoseStamped>(
        in_topic, rclcpp::SensorDataQoS(),
        [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
          publishFromQuat(i, msg->pose.orientation);
        });

      // (Opcional) Fallback a geometry_msgs/Pose en el MISMO tópico
      if (fallback_pose_) {
        subs_pose_[i] = create_subscription<geometry_msgs::msg::Pose>(
          in_topic, rclcpp::SensorDataQoS(),
          [this, i](const geometry_msgs::msg::Pose::SharedPtr msg){
            publishFromQuat(i, msg->orientation);
          });
      }

      RCLCPP_INFO(get_logger(), "Canal %d: sub=%s -> pub=%s", i, in_topic.c_str(), out_topic.c_str());
    }
  }

private:
  static std::string buildAbs(const std::string &prefix, int idx, const std::string &suffix) {
    // Ensambla "/drone_" + i + "/ground_truth/pose" cuidando barras
    std::string p = prefix;
    if (p.empty() || p[0] != '/') p = "/" + p;
    if (!p.empty() && p.back() == '/') p.pop_back();

    std::string s = suffix;
    if (s.empty() || s[0] != '/') s = "/" + s;

    return p + std::to_string(idx) + s;
  }

  void publishFromQuat(int i, const geometry_msgs::msg::Quaternion &qmsg) {
    double qw = qmsg.w, qx = qmsg.x, qy = qmsg.y, qz = qmsg.z;

    // Normaliza por seguridad
    const double n = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (n < 1e-12) {
      publishIdentity(i);
      return;
    }
    qw /= n; qx /= n; qy /= n; qz /= n;

    const double xx = qx*qx, yy = qy*qy, zz = qz*qz;
    const double xy = qx*qy, xz = qx*qz, yz = qy*qz;
    const double wx = qw*qx, wy = qw*qy, wz = qw*qz;

    std_msgs::msg::Float64MultiArray out;
    out.data = {
      1.0 - 2.0*(yy + zz),  2.0*(xy - wz),        2.0*(xz + wy),
      2.0*(xy + wz),        1.0 - 2.0*(xx + zz),  2.0*(yz - wx),
      2.0*(xz - wy),        2.0*(yz + wx),        1.0 - 2.0*(xx + yy)
    };

    out.layout.dim.resize(2);
    out.layout.dim[0].label = "rows"; out.layout.dim[0].size = 3; out.layout.dim[0].stride = 9;
    out.layout.dim[1].label = "cols"; out.layout.dim[1].size = 3; out.layout.dim[1].stride = 3;
    out.layout.data_offset = 0;

    pubs_[i]->publish(out);
  }

  void publishIdentity(int i) {
    std_msgs::msg::Float64MultiArray out;
    out.data = {1,0,0, 0,1,0, 0,0,1};
    out.layout.dim.resize(2);
    out.layout.dim[0].label = "rows"; out.layout.dim[0].size = 3; out.layout.dim[0].stride = 9;
    out.layout.dim[1].label = "cols"; out.layout.dim[1].size = 3; out.layout.dim[1].stride = 3;
    out.layout.data_offset = 0;
    pubs_[i]->publish(out);
  }

  // Params
  int num_drones_;
  std::string ns_prefix_, pose_suffix_, R_suffix_;
  bool fallback_pose_;

  // Pub/Sub por dron
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> pubs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> subs_pose_st_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> subs_pose_; // opcional
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RotationMatrixPublisher>());
  rclcpp::shutdown();
  return 0;
}
