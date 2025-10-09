// Son 4 trayectorias: X, Y, Z, Yaw
//
// Goal: X_fin, Y_fin, Z_fin, Yaw_fin y Tx, Ty, Tz, Tyaw
// Pose inicial: de /ground_truth/pose
//
// Feedback: para cada eje publicamos [pos, vel, acc, jerk, ratio] en un Float64MultiArray
//           x, y, z, yaw (cada uno 5 elementos)
// Result: success + t_total
//
// Cuando llega un nuevo goal, se cancela el anterior.

// Son 4 trayectorias: X, Y, Z, Yaw
//
// Goal: X_fin, Y_fin, Z_fin, Yaw_fin y Tx, Ty, Tz, Tyaw
// Pose inicial: de /ground_truth/pose  (PoseStamped)
//
// Feedback: para cada eje publicamos [pos, vel, acc, jerk, ratio] en Float64MultiArray
// Result: success + t_total

#include <chrono>
#include <cmath>
#include <thread>
#include <tuple>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "my_quadracopter/action/poly3_traj.hpp"

using namespace std::chrono_literals;

static inline double yaw_from_quat(const geometry_msgs::msg::Quaternion &q) {
  // yaw (Z) con convención ZYX
  return std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
}

struct Cubic {
  double a0{}, a1{}, a2{}, a3{};
  std::tuple<double,double,double> eval(double t, double tf) const {
    if (t < 0.0) t = 0.0;
    if (tf > 0.0 && t > tf) t = tf;
    const double p = a0 + a1*t + a2*t*t + a3*t*t*t;
    const double v = a1 + 2.0*a2*t + 3.0*a3*t*t;
    const double a = 2.0*a2 + 6.0*a3*t;
    return {p, v, a};
  }
};

static inline Cubic cubic_from_boundary(double q0, double qf, double tf) {
  Cubic c;
  const double d = qf - q0;
  c.a0 = q0;
  c.a1 = 0.0;
  if (tf > 0.0) {
    c.a2 = 3.0 * d / (tf * tf);
    c.a3 = -2.0 * d / (tf * tf * tf);
  } else {
    c.a2 = 0.0; c.a3 = 0.0;
  }
  return c;
}

class Poly3ActionServer : public rclcpp::Node {
public:
  using Poly3Traj = my_quadracopter::action::Poly3Traj;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Poly3Traj>;

  Poly3ActionServer()
  : Node("poly3_traj_server")
  {
    // Suscripción correcta a PoseStamped
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ground_truth/pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
        last_pose_ = msg->pose;  // guardamos solo el pose
        have_pose_ = true;
      });

    server_ = rclcpp_action::create_server<Poly3Traj>(
      this, "poly3_traj",
      std::bind(&Poly3ActionServer::on_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Poly3ActionServer::on_cancel, this, std::placeholders::_1),
      std::bind(&Poly3ActionServer::on_accept, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp_action::Server<Poly3Traj>::SharedPtr server_;
  geometry_msgs::msg::Pose last_pose_{};
  bool have_pose_ = false;

  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID&,
                                      std::shared_ptr<const Poly3Traj::Goal> goal)
  {
    if (goal->tx < 0.f || goal->ty < 0.f || goal->tz < 0.f || goal->tyaw < 0.f) {
      RCLCPP_WARN(get_logger(), "Duraciones negativas no válidas");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandle>) {
    RCLCPP_INFO(this->get_logger(), "Cancelación aceptada");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_accept(const std::shared_ptr<GoalHandle> gh) {
    std::thread{std::bind(&Poly3ActionServer::execute, this, std::placeholders::_1), gh}.detach();
  }

  static inline float clamp01(double x) {
    if (x < 0.0) return 0.0f;
    if (x > 1.0) return 1.0f;
    return static_cast<float>(x);
  }

  static std_msgs::msg::Float64MultiArray make_axis_array(
      double pos, double vel, double acc, double jerk, double ratio, const char* label)
  {
    std_msgs::msg::Float64MultiArray arr;
    arr.data = {pos, vel, acc, jerk, ratio};
    arr.layout.dim.resize(1);
    arr.layout.dim[0].label = label;
    arr.layout.dim[0].size = 5;
    arr.layout.dim[0].stride = 5;
    return arr;
  }

  void execute(const std::shared_ptr<GoalHandle> gh) {
    const auto goal = gh->get_goal();

    // 1) Pose inicial
    if (!have_pose_) {
      RCLCPP_WARN(this->get_logger(), "Esperando /ground_truth/pose (PoseStamped)...");
      rclcpp::Rate wait_r(50.0);
      for (int i=0; i<200 && rclcpp::ok() && !have_pose_; ++i) wait_r.sleep();
    }
    const geometry_msgs::msg::Pose p0 = last_pose_;

    // 2) yaw inicial y final
    const double yaw0 = yaw_from_quat(p0.orientation);
    const double yawf = yaw_from_quat(goal->target_pose.orientation);

    // 3) cúbicos por eje
    const Cubic cx   = cubic_from_boundary(p0.position.x, goal->target_pose.position.x, goal->tx);
    const Cubic cy   = cubic_from_boundary(p0.position.y, goal->target_pose.position.y, goal->ty);
    const Cubic cz   = cubic_from_boundary(p0.position.z, goal->target_pose.position.z, goal->tz);
    const Cubic cyaw = cubic_from_boundary(yaw0,           yawf,                              goal->tyaw);

    const double jx   = 6.0 * cx.a3;
    const double jy   = 6.0 * cy.a3;
    const double jz   = 6.0 * cz.a3;
    const double jyaw = 6.0 * cyaw.a3;

    const double Tf = std::max({(double)goal->tx, (double)goal->ty, (double)goal->tz, (double)goal->tyaw});

    auto t0 = this->get_clock()->now();
    rclcpp::Rate rate(50.0); // 50 Hz

    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        auto res = std::make_shared<Poly3Traj::Result>();
        res->success = false;
        res->t_total = (this->get_clock()->now() - t0).seconds();
        gh->canceled(res);
        return;
      }

      const double t = (this->get_clock()->now() - t0).seconds();
      if (t > Tf) break;

      auto [px, vx, ax] = cx.eval(t, goal->tx);
      auto [py, vy, ay] = cy.eval(t, goal->ty);
      auto [pz, vz, az] = cz.eval(t, goal->tz);
      auto [yw, ywd, ywdd] = cyaw.eval(t, goal->tyaw);

      const double rx   = (goal->tx   > 0.f) ? clamp01(t/goal->tx)   : 1.0;
      const double ry   = (goal->ty   > 0.f) ? clamp01(t/goal->ty)   : 1.0;
      const double rz   = (goal->tz   > 0.f) ? clamp01(t/goal->tz)   : 1.0;
      const double ryaw = (goal->tyaw > 0.f) ? clamp01(t/goal->tyaw) : 1.0;

      Poly3Traj::Feedback fb;
      fb.t_act = static_cast<float>(t);

      fb.x   = make_axis_array(px,  vx,  ax,  jx,   rx,   "x");
      fb.y   = make_axis_array(py,  vy,  ay,  jy,   ry,   "y");
      fb.z   = make_axis_array(pz,  vz,  az,  jz,   rz,   "z");
      fb.yaw = make_axis_array(yw,  ywd, ywdd, jyaw, ryaw, "yaw");

      gh->publish_feedback(std::make_shared<Poly3Traj::Feedback>(fb));
      rate.sleep();
    }

    auto res = std::make_shared<Poly3Traj::Result>();
    res->success = true;
    res->t_total = (this->get_clock()->now() - t0).seconds();
    gh->succeed(res);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Poly3ActionServer>());
  rclcpp::shutdown();
  return 0;
}
