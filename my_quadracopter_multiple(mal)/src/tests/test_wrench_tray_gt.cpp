#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <unordered_map>
#include <memory>
#include <array>

// Normaliza namespace: empieza por '/', sin barra final
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

class AllocationSolverMulti : public rclcpp::Node {
public:
  AllocationSolverMulti() : rclcpp::Node("allocation_solver_multi")
  {
    // --- Prefijos / configuración multi-dron ---
    ns_prefix_   = declare_parameter<std::string>("ns_prefix",   "/drone_");
    tray_prefix_ = declare_parameter<std::string>("tray_prefix", "/tray_avpt");
    num_drones_  = declare_parameter<int>("num_drones", 1);

    // Sufijos de entrada
    suffix_tau_ = declare_parameter<std::string>("suffix.tau", "/Torques");
    suffix_F_   = declare_parameter<std::string>("suffix.F",   "/Fuerzas");
    suffix_R_   = declare_parameter<std::string>("suffix.R",   "/R");

    // Sufijos de salida (motores abreviados: fl, fr, bl, br)
    motor_fl_ = declare_parameter<std::string>("motor.fl", "/fl_motor");
    motor_fr_ = declare_parameter<std::string>("motor.fr", "/fr_motor");
    motor_bl_ = declare_parameter<std::string>("motor.bl", "/bl_motor");
    motor_br_ = declare_parameter<std::string>("motor.br", "/br_motor");

    // --- Parámetros geométricos (iguales para todos los drones) ---
    L_[0] = declare_parameter<double>("L1", 0.2);
    L_[1] = declare_parameter<double>("L2", 0.2);
    L_[2] = declare_parameter<double>("L3", 0.2);
    L_[3] = declare_parameter<double>("L4", 0.2);

    // Ángulos (deg) — orden motores: 0:fl, 1:fr, 2:bl, 3:br
    alpha_deg_[0] = declare_parameter<double>("alpha1_deg", 45.0);
    alpha_deg_[1] = declare_parameter<double>("alpha2_deg", 45.0);
    alpha_deg_[2] = declare_parameter<double>("alpha3_deg", 45.0);
    alpha_deg_[3] = declare_parameter<double>("alpha4_deg", 45.0);

    // Constante de par (yaw) y opcional clamp a no negativo
    k_ = declare_parameter<double>("k_yaw", 0.01);
    clamp_nonneg_ = declare_parameter<bool>("clamp_nonnegative", false);

    build_A(); // precomputar A y Ainv con esta geometría

    // Crear contexto por dron
    for (int i = 0; i < std::max(1, num_drones_); ++i) {
      const std::string ns = normalize_ns(ns_prefix_ + std::to_string(i));
      make_drone_ctx(ns);
    }

    RCLCPP_INFO(get_logger(),
      "AllocationSolverMulti listo: drones=%d, ns_prefix='%s', tray_prefix='%s'",
      num_drones_, ns_prefix_.c_str(), tray_prefix_.c_str());
  }

private:
  struct DroneCtx {
    std::string ns;

    // Entradas
    bool have_tau{false}, have_F{false}, have_R{false};
    double taux{0}, tauy{0}, tauz{0};
    double Fx{0}, Fy{0}, Fz{0};
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    // Subs y pubs
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_tau;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_F;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   sub_R;

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_fl;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_fr;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_bl;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_br;
  };

  // ---------- parámetros globales ----------
  int num_drones_{1};
  std::string ns_prefix_{"/drone_"}, tray_prefix_{"/tray_avpt"};
  std::string suffix_tau_{"/Torques"}, suffix_F_{"/Fuerzas"}, suffix_R_{"/R"};
  std::string motor_fl_{"/fl_motor"}, motor_fr_{"/fr_motor"}, motor_bl_{"/bl_motor"}, motor_br_{"/br_motor"};

  // Geometría
  double L_[4];
  double alpha_deg_[4];
  double k_;
  bool clamp_nonneg_{false};

  // Matriz de asignación
  Eigen::Matrix4d A_{Eigen::Matrix4d::Zero()};
  Eigen::Matrix4d Ainv_{Eigen::Matrix4d::Identity()};

  // Contextos por dron
  std::unordered_map<std::string, std::unique_ptr<DroneCtx>> ctx_;

  void build_A()
  {
    // Filas U = [FT, tau_x, tau_y, tau_z] ; Columnas f = [f1 fl, f2 fr, f3 bl, f4 br]
    A_.setZero();
    A_.row(0) << 1, 1, 1, 1; // FT

    auto s = [&](int i){ return std::sin(alpha_deg_[i]*M_PI/180.0); };
    auto c = [&](int i){ return std::cos(alpha_deg_[i]*M_PI/180.0); };

    // Mx = +F1 L1 sin(a1) - F2 L2 sin(a2) + F3 L3 sin(a3) - F4 L4 sin(a4)
    A_(1,0) =  L_[0]*s(0);
    A_(1,1) = -L_[1]*s(1);
    A_(1,2) =  L_[2]*s(2);
    A_(1,3) = -L_[3]*s(3);

    // My = -F1 L1 cos(a1) - F2 L2 cos(a2) + F3 L3 cos(a3) + F4 L4 cos(a4)
    A_(2,0) = -L_[0]*c(0);
    A_(2,1) = -L_[1]*c(1);
    A_(2,2) =  L_[2]*c(2);
    A_(2,3) =  L_[3]*c(3);

    // Mz = -k F1 + k F2 + k F3 - k F4   (signos [- + + -] coherentes con s[] abajo)
    A_(3,0) = -k_;
    A_(3,1) =  k_;
    A_(3,2) =  k_;
    A_(3,3) = -k_;

    const double eps = 1e-10;
    if (std::abs(A_.determinant()) < eps) {
      RCLCPP_WARN(get_logger(), "A casi singular; usando pseudoinversa.");
      Ainv_ = A_.completeOrthogonalDecomposition().pseudoInverse();
    } else {
      Ainv_ = A_.inverse();
    }
  }

  void make_drone_ctx(const std::string &ns)
  {
    auto c = std::make_unique<DroneCtx>();
    c->ns = ns;

    // Tópicos con prefijos
    const std::string t_tau = ns + tray_prefix_ + suffix_tau_; // /drone_i/tray_avpt/Torques
    const std::string t_F   = ns + tray_prefix_ + suffix_F_;   // /drone_i/tray_avpt/Fuerzas
    const std::string t_R   = ns + suffix_R_;                  // /drone_i/R

    const std::string t_fl = ns + motor_fl_; // /drone_i/fl_motor
    const std::string t_fr = ns + motor_fr_; // /drone_i/fr_motor
    const std::string t_bl = ns + motor_bl_; // /drone_i/bl_motor
    const std::string t_br = ns + motor_br_; // /drone_i/br_motor

    // Publishers
    c->pub_fl = create_publisher<geometry_msgs::msg::Wrench>(t_fl, 10);
    c->pub_fr = create_publisher<geometry_msgs::msg::Wrench>(t_fr, 10);
    c->pub_bl = create_publisher<geometry_msgs::msg::Wrench>(t_bl, 10);
    c->pub_br = create_publisher<geometry_msgs::msg::Wrench>(t_br, 10);

    // Suscriptores
    c->sub_tau = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      t_tau, rclcpp::QoS(50),
      [this, ns](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg){
        auto it = ctx_.find(ns); if (it == ctx_.end()) return;
        auto &d = *it->second;
        d.taux = msg->vector.x; d.tauy = msg->vector.y; d.tauz = msg->vector.z;
        d.have_tau = true; compute_and_publish(d);
      });

    c->sub_F = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      t_F, rclcpp::QoS(50),
      [this, ns](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg){
        auto it = ctx_.find(ns); if (it == ctx_.end()) return;
        auto &d = *it->second;
        d.Fx = msg->vector.x; d.Fy = msg->vector.y; d.Fz = msg->vector.z;
        d.have_F = true; compute_and_publish(d);
      });

    c->sub_R = create_subscription<std_msgs::msg::Float64MultiArray>(
      t_R, rclcpp::QoS(10),
      [this, ns](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg){
        auto it = ctx_.find(ns); if (it == ctx_.end()) return;
        auto &d = *it->second;
        if (msg->data.size() >= 9) {
          d.R(0,0)=msg->data[0]; d.R(0,1)=msg->data[1]; d.R(0,2)=msg->data[2];
          d.R(1,0)=msg->data[3]; d.R(1,1)=msg->data[4]; d.R(1,2)=msg->data[5];
          d.R(2,0)=msg->data[6]; d.R(2,1)=msg->data[7]; d.R(2,2)=msg->data[8];
          d.have_R = true; compute_and_publish(d);
        }
      });

    RCLCPP_INFO(get_logger(),
      "[%s] subs: tau=%s, F=%s, R=%s | pubs: %s, %s, %s, %s",
      ns.c_str(), t_tau.c_str(), t_F.c_str(), t_R.c_str(),
      t_fl.c_str(), t_fr.c_str(), t_bl.c_str(), t_br.c_str());

    ctx_.emplace(ns, std::move(c));
  }

  void compute_and_publish(DroneCtx &d)
  {
    if (!(d.have_tau && d.have_F && d.have_R)) return;

    // b3 en mundo = tercera columna de R_WB
    Eigen::Vector3d b3(d.R(0,2), d.R(1,2), d.R(2,2));
    Eigen::Vector3d Fw(d.Fx, d.Fy, d.Fz);

    // Empuje total (proyección de F deseada en z cuerpo)
    const double An = b3.norm();
    if (An < 1e-12) return;
    const double FT = Fw.dot(b3 / An);

    // Vector de entrada U = [FT, tau_x, tau_y, tau_z]
    Eigen::Vector4d U; U << FT, d.taux, d.tauy, d.tauz;

    // Solución fuerzas por motor
    Eigen::Vector4d f = Ainv_ * U;
    if (clamp_nonneg_) for (int i=0;i<4;i++) if (f[i] < 0.0) f[i] = 0.0;

    // Signos de par por motor coherentes con la fila Mz de A ([- + + -])
    const double s[4] = {-1.0, +1.0, +1.0, -1.0};

    // Publicar Wrench por motor (eje z)
    geometry_msgs::msg::Wrench w;
    w.force.x = 0; w.force.y = 0; w.torque.x = 0; w.torque.y = 0;

    // fl (idx 0)
    w.force.z  = f[0];
    w.torque.z = s[0]*k_*f[0];
    d.pub_fl->publish(w);

    // fr (idx 1)
    w.force.z  = f[1];
    w.torque.z = s[1]*k_*f[1];
    d.pub_fr->publish(w);

    // bl (idx 2)
    w.force.z  = f[2];
    w.torque.z = s[2]*k_*f[2];
    d.pub_bl->publish(w);

    // br (idx 3)
    w.force.z  = f[3];
    w.torque.z = s[3]*k_*f[3];
    d.pub_br->publish(w);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AllocationSolverMulti>());
  rclcpp::shutdown();
  return 0;
}
