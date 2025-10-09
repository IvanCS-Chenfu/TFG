/**/
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <string>

class AllocationSolver : public rclcpp::Node {
public:
  AllocationSolver() : rclcpp::Node("allocation_solver")
  {
    // --- Parámetros geométricos ---
    L_[0] = declare_parameter<double>("L1", 0.2);
    L_[1] = declare_parameter<double>("L2", 0.2);
    L_[2] = declare_parameter<double>("L3", 0.2);
    L_[3] = declare_parameter<double>("L4", 0.2);

    // Ángulos (deg). Quad en X típico: 45, 135, 225, 315
    alpha_deg_[0] = declare_parameter<double>("alpha1_deg", 45.0);
    alpha_deg_[1] = declare_parameter<double>("alpha2_deg", 45.0);
    alpha_deg_[2] = declare_parameter<double>("alpha3_deg", 45.0);
    alpha_deg_[3] = declare_parameter<double>("alpha4_deg", 45.0);

    // Constante de par (yaw)
    k_ = declare_parameter<double>("k_yaw", 0.01);
    clamp_nonneg_ = declare_parameter<bool>("clamp_nonnegative", false);

    // Tópicos de entrada
    topic_tau_ = declare_parameter<std::string>("topics.tau", "/Torques");
    topic_F_   = declare_parameter<std::string>("topics.F",   "/Fuerzas");

    // Tópicos de salida (Wrench por motor)
    topic_m1_ = declare_parameter<std::string>("topic_motor1", "/drone/front_left_motor");  // /drone/
    topic_m2_ = declare_parameter<std::string>("topic_motor2", "/drone/front_right_motor");
    topic_m3_ = declare_parameter<std::string>("topic_motor3", "/drone/back_left_motor");
    topic_m4_ = declare_parameter<std::string>("topic_motor4", "/drone/back_right_motor");

    pub_m1_ = create_publisher<geometry_msgs::msg::Wrench>(topic_m1_, 10);
    pub_m2_ = create_publisher<geometry_msgs::msg::Wrench>(topic_m2_, 10);
    pub_m3_ = create_publisher<geometry_msgs::msg::Wrench>(topic_m3_, 10);
    pub_m4_ = create_publisher<geometry_msgs::msg::Wrench>(topic_m4_, 10);

    sub_tau_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      topic_tau_, rclcpp::QoS(50),
      [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg){
        taux_ = msg->vector.x; tauy_ = msg->vector.y; tauz_ = msg->vector.z;
        have_tau_ = true; compute_and_publish();
      });

    sub_F_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      topic_F_, rclcpp::QoS(50),
      [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg){
        Fmag_ = std::sqrt(msg->vector.x*msg->vector.x +
                          msg->vector.y*msg->vector.y +
                          msg->vector.z*msg->vector.z);
        have_F_ = true; compute_and_publish();
      });

    build_A();  // precomputar A y su inversa

    RCLCPP_INFO(get_logger(),
      "AllocationSolver publicando Wrench: m1=%s m2=%s m3=%s m4=%s",
      topic_m1_.c_str(), topic_m2_.c_str(), topic_m3_.c_str(), topic_m4_.c_str());
  }

private:
  // Parámetros
  double L_[4];
  double alpha_deg_[4];
  double k_;
  bool clamp_nonneg_;
  std::string topic_tau_, topic_F_;
  std::string topic_m1_, topic_m2_, topic_m3_, topic_m4_;

  // Entradas
  bool have_tau_{false}, have_F_{false};
  double taux_{0}, tauy_{0}, tauz_{0}, Fmag_{0};

  // Matriz de asignación y su inversa
  Eigen::Matrix4d A_{Eigen::Matrix4d::Zero()};
  Eigen::Matrix4d Ainv_{Eigen::Matrix4d::Identity()};

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_tau_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_F_;
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_m1_, pub_m2_, pub_m3_, pub_m4_;

  void build_A()
  {
    // Filas: [FT, Mx, My, Mz]  Columnas: [F1 F2 F3 F4]
    A_.row(0) << 1, 1, 1, 1; // FT

    auto s = [&](int i){ return std::sin(alpha_deg_[i]*M_PI/180.0); };
    auto c = [&](int i){ return std::cos(alpha_deg_[i]*M_PI/180.0); };

    // Según tus ecuaciones (imagen):
    // Mx =  +F1 L1 sin(a1) - F2 L2 sin(a2) + F3 L3 sin(a3) - F4 L4 sin(a4)
    A_(1,0) =  L_[0]*s(0);
    A_(1,1) = -L_[1]*s(1);
    A_(1,2) =  L_[2]*s(2);
    A_(1,3) = -L_[3]*s(3);

    // My =  +F1 L1 cos(a1) + F2 L2 cos(a2) - F3 L3 cos(a3) - F4 L4 cos(a4)
    A_(2,0) = -L_[0]*c(0);
    A_(2,1) = -L_[1]*c(1);
    A_(2,2) = L_[2]*c(2);
    A_(2,3) = L_[3]*c(3);

    // Mz =  +k F1 - k F2 - k F3 + k F4  (signos [+ - - +])
    A_(3,0) = -k_;
    A_(3,1) = k_;
    A_(3,2) = k_;
    A_(3,3) = -k_;

    const double eps = 1e-10;
    if (std::abs(A_.determinant()) < eps) {
      RCLCPP_WARN(get_logger(), "A casi singular; usando pseudoinversa.");
      Ainv_ = A_.completeOrthogonalDecomposition().pseudoInverse();
    } else {
      Ainv_ = A_.inverse();
    }
  }

  void compute_and_publish()
  {
    if (!(have_tau_ && have_F_)) return;

    //taux_ = 0; tauy_ = 0; tauz_ = 0;
    int redux = 10;
    taux_ = taux_/redux; tauy_ = tauy_/redux; tauz_ = tauz_/redux;

    // U = [|F|, tau_x, tau_y, tau_z]
    Eigen::Vector4d U; U << Fmag_, taux_, tauy_, tauz_;

    // f = A^{-1} U
    Eigen::Vector4d f = Ainv_ * U;
    if (clamp_nonneg_) for (int i=0;i<4;i++) if (f[i] < 0.0) f[i] = 0.0;

    // Publicar Wrench por motor:
    // force = (0,0,Fi), torque = (0,0, s_i*k*Fi) con s = [+1, -1, -1, +1]
    const double s[4] = {-1.0, +1.0, +1.0, -1.0};
    geometry_msgs::msg::Wrench w;

    w.force.x = 0; w.force.y = 0; w.torque.x = 0; w.torque.y = 0;
    //RCLCPP_INFO(this->get_logger(), "ef = [%.3f, %.3f, %.3f, %.3f]", (Fmag_/4-f[0]),(Fmag_/4-f[1]),(Fmag_/4-f[2]),(Fmag_/4-f[3]));
    //RCLCPP_INFO(this->get_logger(), "F = [%.3f, %.3f, %.3f, %.3f], tau = [%.3f, %.3f, %.3f, %.3f]" , f[0],f[1],f[2],f[3],s[0]*k_*f[0],s[1]*k_*f[1],s[2]*k_*f[2],s[3]*k_*f[3]);

    
    w.force.z = f[0]; w.torque.z = s[0]*k_*f[0]; 
    pub_m1_->publish(w);
    w.force.z = f[1]; w.torque.z = s[1]*k_*f[1]; 
    pub_m2_->publish(w);
    w.force.z = f[2]; w.torque.z = s[2]*k_*f[2]; 
    pub_m3_->publish(w);
    w.force.z = f[3]; w.torque.z = s[3]*k_*f[3]; 
    pub_m4_->publish(w);
    
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AllocationSolver>());
  rclcpp::shutdown();
  return 0;
}
