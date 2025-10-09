#include <chrono>
#include <memory>
#include <tuple>
#include <algorithm>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "my_quadracopter/action/poly3_traj.hpp"

class ForceFromFeedback : public rclcpp::Node {
public:
  using Poly3Traj   = my_quadracopter::action::Poly3Traj;
  using FeedbackMsg = Poly3Traj::Impl::FeedbackMessage; // mensaje crudo del /_action/feedback

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
    action_name_    = this->declare_parameter<std::string>("action_name", "/poly3_traj");
    tray_topic_suf_ = this->declare_parameter<std::string>("tray_suffix", "/tray_avpt/Fuerzas");
    feedback_topic_ = action_name_ + "/_action/feedback";

    // Suscripción al feedback del action server
    sub_feedback_ = this->create_subscription<FeedbackMsg>(
      feedback_topic_, rclcpp::QoS(50),
      [this](FeedbackMsg::ConstSharedPtr fb){
        // 1) Tomar el namespace/id del dron desde el header del feedback
        const std::string ns = normalize_ns(fb->feedback.header.frame_id);

        // 2) Si cambiamos de dron, reconfigurar suscriptores y publicador
        if (ns != current_ns_) {
          configure_for_namespace(ns);
        }

        // 3) Extraer referencias d(t) del feedback (x,y,z)
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

        compute_and_publish(); // intenta publicar si ya tenemos GT
      });

    RCLCPP_INFO(get_logger(), "Subscribiéndose a feedback: %s", feedback_topic_.c_str());
  }

private:
  // Tipos simples
  struct Vec3 { double x{0}, y{0}, z{0}; };

  // === Estado GT (del dron ACTUAL) ===
  Vec3 p_{}, v_{}, a_gt_{};
  bool have_p_{false}, have_v_{false}, have_a_gt_{false};

  // === Referencias d(t) (del dron ACTUAL) ===
  Vec3 pd_{}, vd_{}, ad_{};
  bool have_ref_{false};

  // Parámetros control
  double kp_{3.0}, kv_{2.0}, m_{1.0};
  Vec3 gI_{0.0, 0.0, 9.81};

  // ROS
  std::string action_name_, feedback_topic_;
  std::string tray_topic_suf_;      // p.ej. "/tray_avpt/Fuerzas"
  std::string current_ns_;          // p.ej. "/drone_2"

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_F_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr sub_acc_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr                      sub_feedback_;

  // Normaliza namespace: empieza por '/', sin barra final sobrante
  static std::string normalize_ns(std::string ns) {
    auto trim = [](std::string &s){
      auto a = s.find_first_not_of(" \t\r\n");
      auto b = s.find_last_not_of(" \t\r\n");
      s = (a==std::string::npos) ? "" : s.substr(a, b-a+1);
    };
    trim(ns);
    if (ns.empty()) ns = "/drone_0";
    if (ns.front() != '/') ns.insert(ns.begin(), '/');
    while (ns.size() > 1 && ns.back() == '/') ns.pop_back();
    return ns;
  }

  // (Re)configura suscriptores y publicador para un namespace dado
  void configure_for_namespace(const std::string &ns) {
    current_ns_ = ns;

    // Reset de estado GT y flags
    have_p_ = have_v_ = have_a_gt_ = false;

    // Construir nombres de tópicos
    const std::string pose_topic = current_ns_ + "/ground_truth/pose";
    const std::string vel_topic  = current_ns_ + "/ground_truth/vel";
    const std::string acc_topic  = current_ns_ + "/ground_truth/acc";
    const std::string force_topic = current_ns_ + tray_topic_suf_; // p.ej. "/drone_i/tray_avpt/Fuerzas"

    // (Re)crear publisher
    pub_F_.reset();
    pub_F_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(force_topic, 10);

    // (Re)crear suscriptores GT
    sub_pose_.reset();
    sub_vel_.reset();
    sub_acc_.reset();

    sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
        // Acepta solo mensajes del namespace actual (por seguridad)
        if (normalize_ns(msg->header.frame_id) != current_ns_) return;
        p_.x = msg->pose.position.x; p_.y = msg->pose.position.y; p_.z = msg->pose.position.z;
        have_p_ = true; compute_and_publish();
      });

    sub_vel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      vel_topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg){
        if (normalize_ns(msg->header.frame_id) != current_ns_) return;
        v_.x = msg->twist.linear.x; v_.y = msg->twist.linear.y; v_.z = msg->twist.linear.z;
        have_v_ = true; compute_and_publish();
      });

    sub_acc_ = this->create_subscription<geometry_msgs::msg::AccelStamped>(
      acc_topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::AccelStamped::ConstSharedPtr msg){
        if (normalize_ns(msg->header.frame_id) != current_ns_) return;
        a_gt_.x = msg->accel.linear.x; a_gt_.y = msg->accel.linear.y; a_gt_.z = msg->accel.linear.z;
        have_a_gt_ = true; compute_and_publish();
      });

    RCLCPP_INFO(this->get_logger(),
      "Configurado para ns=%s | sub: [%s, %s, %s] | pub: [%s]",
      current_ns_.c_str(), pose_topic.c_str(), vel_topic.c_str(), acc_topic.c_str(), force_topic.c_str());
  }

  void compute_and_publish()
  {
    if (!(have_p_ && have_v_ && have_a_gt_ && have_ref_)) return;
    if (!pub_F_) return;

    // e_p = p - p_d ; e_v = v - v_d
    Vec3 ep{ p_.x - pd_.x, p_.y - pd_.y, p_.z - pd_.z };
    Vec3 ev{ v_.x - vd_.x, v_.y - vd_.y, v_.z - vd_.z };

    // F = -Kp*ep - Kv*ev + m*(ad + gI)
    Vec3 F;
    F.x = -kp_*ep.x - kv_*ev.x + m_*(ad_.x + gI_.x);
    F.y = -kp_*ep.y - kv_*ev.y + m_*(ad_.y + gI_.y);
    F.z = -kp_*ep.z - kv_*ev.z + m_*(ad_.z + gI_.z);

    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = current_ns_;  // ← publica con el MISMO ID/NS del dron
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
