#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <cmath>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <memory>

// ----------------- Utilidades -----------------
struct Vec3 {
  double x{0}, y{0}, z{0};
  Vec3() = default;
  Vec3(double X,double Y,double Z):x(X),y(Y),z(Z) {}
  Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
  Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
};
static inline Vec3 cross(const Vec3&a, const Vec3&b){
  return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}

struct Mat3 {
  // row-major m[r][c]
  double m[3][3]{ {1,0,0},{0,1,0},{0,0,1} };

  static Mat3 Zero(){ Mat3 A; for(auto& r:A.m){ r[0]=r[1]=r[2]=0; } return A; }
  static Mat3 Identity(){ return Mat3(); }

  Mat3 T() const {
    Mat3 R;
    for(int r=0;r<3;++r)
      for(int c=0;c<3;++c)
        R.m[r][c] = m[c][r];
    return R;
  }

  Vec3 operator*(const Vec3& v) const {
    return {
      m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
      m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
      m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
    };
  }

  Mat3 operator*(const Mat3& B) const {
    Mat3 C = Mat3::Zero();
    for(int r=0;r<3;++r)
      for(int c=0;c<3;++c)
        for(int k=0;k<3;++k)
          C.m[r][c] += m[r][k]*B.m[k][c];
    return C;
  }

  Mat3 operator-(const Mat3& B) const {
    Mat3 C = Mat3::Zero();
    for(int r=0;r<3;++r)
      for(int c=0;c<3;++c)
        C.m[r][c] = m[r][c] - B.m[r][c];
    return C;
  }

  Mat3 operator+(const Mat3& B) const {
    Mat3 C = Mat3::Zero();
    for(int r=0;r<3;++r)
      for(int c=0;c<3;++c)
        C.m[r][c] = m[r][c] + B.m[r][c];
    return C;
  }
};

static inline Vec3 vee(const Mat3& M){
  return {
    0.5*(M.m[2][1]-M.m[1][2]),
    0.5*(M.m[0][2]-M.m[2][0]),
    0.5*(M.m[1][0]-M.m[0][1])
  };
}

static inline Mat3 mat_from_row_major(const std::array<double,9>& a){
  Mat3 R;
  for(int r=0;r<3;++r) for(int c=0;c<3;++c) R.m[r][c] = a[r*3+c];
  return R;
}

// Normaliza namespace: empieza por '/', sin barra final sobrante.
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

// ----------------- Nodo -----------------
class SO3TorqueControllerMulti : public rclcpp::Node {
public:
  SO3TorqueControllerMulti() : rclcpp::Node("so3_torque_controller_multi")
  {
    // --- Parámetros globales ---
    ns_prefix_   = declare_parameter<std::string>("ns_prefix", "/drone_");
    tray_prefix_ = declare_parameter<std::string>("tray_prefix", "/tray_avpt");
    num_drones_  = declare_parameter<int>("num_drones", 1);

    // sufijos de tópicos
    suffix_Rd_   = declare_parameter<std::string>("suffix.Rd",  "/R_desired");
    suffix_R_    = declare_parameter<std::string>("suffix.R",   "/R");
    suffix_vel_  = declare_parameter<std::string>("suffix.vel", "/ground_truth/vel");
    suffix_tau_  = declare_parameter<std::string>("suffix.tau", "/Torques"); // bajo tray_prefix

    // Ganancias
    kR_  = declare_parameter<double>("gains.kR", 1.0);
    kW_  = declare_parameter<double>("gains.kW", 1.0);

    // Inercia
    double Jxx = declare_parameter<double>("Jxx", 0.01);
    double Jxy = declare_parameter<double>("Jxy", 0.0);
    double Jxz = declare_parameter<double>("Jxz", 0.0);
    double Jyx = declare_parameter<double>("Jyx", 0.0);
    double Jyy = declare_parameter<double>("Jyy", 0.01);
    double Jyz = declare_parameter<double>("Jyz", 0.0);
    double Jzx = declare_parameter<double>("Jzx", 0.0);
    double Jzy = declare_parameter<double>("Jzy", 0.0);
    double Jzz = declare_parameter<double>("Jzz", 0.01);

    J_.m[0][0]=Jxx; J_.m[0][1]=Jxy; J_.m[0][2]=Jxz;
    J_.m[1][0]=Jyx; J_.m[1][1]=Jyy; J_.m[1][2]=Jyz;
    J_.m[2][0]=Jzx; J_.m[2][1]=Jzy; J_.m[2][2]=Jzz;

    if (std::fabs(Jxy - Jyx) > 1e-9 || std::fabs(Jxz - Jzx) > 1e-9 || std::fabs(Jyz - Jzy) > 1e-9) {
      RCLCPP_WARN(get_logger(), "J no es simétrica; revisa parámetros.");
    }

    // Crear contexto por dron
    for (int i = 0; i < std::max(1, num_drones_); ++i) {
      const std::string ns = normalize_ns(ns_prefix_ + std::to_string(i));
      make_drone_ctx(ns);
    }

    RCLCPP_INFO(get_logger(), "SO3TorqueControllerMulti listo para %d drones. ns_prefix='%s', tray_prefix='%s'",
                num_drones_, ns_prefix_.c_str(), tray_prefix_.c_str());
  }

private:
  struct DroneCtx {
    std::string ns;  // "/drone_i"

    // Estado
    bool have_Rd{false}, have_R{false}, have_w{false};
    Mat3 Rd, dRd, ddRd;
    Mat3 R;
    Vec3 w_world{0,0,0};

    // ROS
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_Rd;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_R;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vel;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr  pub_tau;
  };

  // Parámetros globales
  int    num_drones_{1};
  std::string ns_prefix_{"/drone_"}, tray_prefix_{"/tray_avpt"};
  std::string suffix_Rd_{"/R_desired"}, suffix_R_{"/R"}, suffix_vel_{"/ground_truth/vel"}, suffix_tau_{"/Torques"};
  double kR_{1.0}, kW_{1.0};
  Mat3 J_;

  // Contextos
  std::unordered_map<std::string, std::unique_ptr<DroneCtx>> ctx_;

  void make_drone_ctx(const std::string &ns)
  {
    auto ctx = std::make_unique<DroneCtx>();
    ctx->ns = ns;

    // Tópicos concretos
    const std::string topic_Rd  = ns + tray_prefix_ + suffix_Rd_;   // /drone_i/tray_avpt/R_desired
    const std::string topic_R   = ns + suffix_R_;                   // /drone_i/R
    const std::string topic_vel = ns + suffix_vel_;                 // /drone_i/ground_truth/vel
    const std::string topic_tau = ns + tray_prefix_ + suffix_tau_;  // /drone_i/tray_avpt/Torques

    // Publisher
    ctx->pub_tau = create_publisher<geometry_msgs::msg::Vector3Stamped>(topic_tau, 10);

    // Subscriptor de Rd (27 valores: R,Rdot,Rddot)
    ctx->sub_Rd = create_subscription<std_msgs::msg::Float64MultiArray>(
      topic_Rd, rclcpp::QoS(10),
      [this, ns](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg){
        auto it = ctx_.find(ns); if (it == ctx_.end()) return;
        auto &c = *it->second;
        if (msg->data.size() < 27) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[%s] R_desired tamaño inválido (%zu<27)",
                               ns.c_str(), msg->data.size());
          return;
        }
        std::array<double,9> aR, aRd, aRdd;
        for(int i=0;i<9;i++){ aR[i]=msg->data[i]; aRd[i]=msg->data[9+i]; aRdd[i]=msg->data[18+i]; }
        c.Rd   = mat_from_row_major(aR);
        c.dRd  = mat_from_row_major(aRd);
        c.ddRd = mat_from_row_major(aRdd);
        c.have_Rd = true;
        compute_and_publish(c);
      });

    // Subscriptor de R actual (9 valores)
    ctx->sub_R = create_subscription<std_msgs::msg::Float64MultiArray>(
      topic_R, rclcpp::QoS(10),
      [this, ns](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg){
        auto it = ctx_.find(ns); if (it == ctx_.end()) return;
        auto &c = *it->second;
        if (msg->data.size() < 9) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[%s] R tamaño inválido (%zu<9)",
                               ns.c_str(), msg->data.size());
          return;
        }
        std::array<double,9> a{};
        for(int i=0;i<9;i++) a[i]=msg->data[i];
        c.R = mat_from_row_major(a);
        c.have_R = true;
        compute_and_publish(c);
      });

    // Subscriptor de velocidad angular (TwistStamped en world)
    ctx->sub_vel = create_subscription<geometry_msgs::msg::TwistStamped>(
      topic_vel, rclcpp::SensorDataQoS(),
      [this, ns](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg){
        auto it = ctx_.find(ns); if (it == ctx_.end()) return;
        auto &c = *it->second;
        c.w_world = {msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z};
        c.have_w = true;
        compute_and_publish(c);
      });

    RCLCPP_INFO(get_logger(), "[%s] sub: Rd=%s, R=%s, vel=%s | pub: tau=%s",
                ns.c_str(), topic_Rd.c_str(), topic_R.c_str(), topic_vel.c_str(), topic_tau.c_str());

    ctx_.emplace(ns, std::move(ctx));
  }

  // J * v
  Vec3 J_mul(const Vec3& v) const {
    return {
      J_.m[0][0]*v.x + J_.m[0][1]*v.y + J_.m[0][2]*v.z,
      J_.m[1][0]*v.x + J_.m[1][1]*v.y + J_.m[1][2]*v.z,
      J_.m[2][0]*v.x + J_.m[2][1]*v.y + J_.m[2][2]*v.z
    };
  }

  void compute_and_publish(DroneCtx &c)
  {
    if (!(c.have_Rd && c.have_R && c.have_w)) return;

    // Omega_d^Bd = vee(Rd^T dRd)
    Mat3 RdT = c.Rd.T();
    Mat3 Wd_mat = RdT * c.dRd;
    Vec3 Omega_d_Bd = vee(Wd_mat); // en marco deseado

    // dot{Omega}_d^Bd = vee( Rd^T ddRd - (Rd^T dRd)^2 )
    Mat3 term = RdT * c.ddRd - (Wd_mat * Wd_mat);
    Vec3 dOmega_d_Bd = vee(term);

    // w_B = R^T * w_world
    Vec3 wB = c.R.T() * c.w_world;

    // e_R = 1/2 vee( Rd^T R - R^T Rd )
    Mat3 ERm = (RdT * c.R) - (c.R.T() * c.Rd);
    Vec3 eR{ 0.5*(ERm.m[2][1]-ERm.m[1][2]),
             0.5*(ERm.m[0][2]-ERm.m[2][0]),
             0.5*(ERm.m[1][0]-ERm.m[0][1]) };

    // e_w = wB - R^T Rd Omega_d^Bd
    Vec3 wd_in_B = (c.R.T() * (c.Rd * Omega_d_Bd)); // R^T Rd ΩdBd
    Vec3 ew = wB - wd_in_B;

    // tau = -KReR - Kw ew + J R^T Rd dOmega_d^Bd + wB × (J wB)
    Vec3 JRTRd_dO = J_mul( c.R.T() * (c.Rd * dOmega_d_Bd) );
    Vec3 JwB = J_mul(wB);
    Vec3 tau = (eR * (-kR_)) + (ew * (-kW_)) + JRTRd_dO + cross(wB, JwB);

    geometry_msgs::msg::Vector3Stamped out;
    out.header.stamp = this->now();
    out.header.frame_id = c.ns; // etiqueta con el dron
    out.vector.x = tau.x;
    out.vector.y = tau.y;
    out.vector.z = tau.z;

    c.pub_tau->publish(out);
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SO3TorqueControllerMulti>());
  rclcpp::shutdown();
  return 0;
}
