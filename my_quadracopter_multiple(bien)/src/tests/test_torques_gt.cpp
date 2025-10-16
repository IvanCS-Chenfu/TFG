#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <cmath>
#include <algorithm>
#include <string>

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
static inline double dot(const Vec3&a, const Vec3&b){
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

struct Mat3 {
  // row-major m[r][c]
  double m[3][3]{ {1,0,0},{0,1,0},{0,0,1} };
  static Mat3 Zero(){ Mat3 A; for(auto& r:A.m) r[0]=r[1]=r[2]=0; return A; }
  static Mat3 Identity(){ return Mat3(); }
  Mat3 T() const { Mat3 R; for(int r=0;r<3;++r) for(int c=0;c<3;++c) R.m[r][c] = m[c][r]; return R; }
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
    Mat3 C;
    for(int r=0;r<3;++r) for(int c=0;c<3;++c) C.m[r][c]=m[r][c]-B.m[r][c];
    return C;
  }
  Mat3 operator+(const Mat3& B) const {
    Mat3 C;
    for(int r=0;r<3;++r) for(int c=0;c<3;++c) C.m[r][c]=m[r][c]+B.m[r][c];
    return C;
  }
};

// vee( (skew) ) ; si la matriz no es 100% skew, coge la parte skew: (M - M^T)/2
static inline Vec3 vee(const Mat3& M){
  return {
    0.5*(M.m[2][1]-M.m[1][2]),
    0.5*(M.m[0][2]-M.m[2][0]),
    0.5*(M.m[1][0]-M.m[0][1])
  };
}
static inline Mat3 skew(const Vec3& v){
  Mat3 S = Mat3::Zero();
  S.m[0][1] = -v.z; S.m[0][2] =  v.y;
  S.m[1][0] =  v.z; S.m[1][2] = -v.x;
  S.m[2][0] = -v.y; S.m[2][1] =  v.x;
  return S;
}

static inline Mat3 mat_from_row_major(const std::array<double,9>& a){
  Mat3 R;
  for(int r=0;r<3;++r) for(int c=0;c<3;++c) R.m[r][c] = a[r*3+c];
  return R;
}

class SO3TorqueController : public rclcpp::Node {
public:
  SO3TorqueController() : rclcpp::Node("so3_torque_controller")
  {
    // Topics (configurables)
    topic_Rd_ = declare_parameter<std::string>("topics.Rd", "tray_avpt/R_desired");
    topic_R_  = declare_parameter<std::string>("topics.R",  "R");
    topic_vel_= declare_parameter<std::string>("topics.vel","ground_truth/vel");
    topic_tau_= declare_parameter<std::string>("topics.tau","tray_avpt/Torques");

    // Ganancias (escalares o por-eje)
    kR_  = declare_parameter<double>("gains.kR", 1.0);
    kW_  = declare_parameter<double>("gains.kW", 1.0);

    // Inercia J (diagonal por defecto) o completa si proporcionas los 9 elementos
    double Jxx = declare_parameter<double>("Jxx", 0.01);
    double Jxy = declare_parameter<double>("Jxy", 0.0);
    double Jxz = declare_parameter<double>("Jxz", 0.0);

    double Jyx = declare_parameter<double>("Jyx", 0.0);
    double Jyy = declare_parameter<double>("Jyy", 0.01);
    double Jyz = declare_parameter<double>("Jyz", 0.0);

    double Jzx = declare_parameter<double>("Jzx", 0.0);
    double Jzy = declare_parameter<double>("Jzy", 0.0);
    double Jzz = declare_parameter<double>("Jzz", 0.01);

    // Construir J (fila mayor)
    J_ = Mat3::Zero();
    J_.m[0][0] = Jxx; J_.m[0][1] = Jxy; J_.m[0][2] = Jxz;
    J_.m[1][0] = Jyx; J_.m[1][1] = Jyy; J_.m[1][2] = Jyz;
    J_.m[2][0] = Jzx; J_.m[2][1] = Jzy; J_.m[2][2] = Jzz;

    // (Opcional) avisar si no es simétrica: inercia física debería ser simétrica SPD
    if (std::fabs(Jxy - Jyx) > 1e-9 || std::fabs(Jxz - Jzx) > 1e-9 || std::fabs(Jyz - Jzy) > 1e-9) {
    RCLCPP_WARN(get_logger(), "J no es simétrica (Jxy!=Jyx, etc). Verifica tus parámetros.");
    }

    pub_tau_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(topic_tau_, 10);

    sub_Rd_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      topic_Rd_, rclcpp::QoS(10),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg){ onRd(msg); });

    sub_R_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      topic_R_, rclcpp::QoS(10),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg){ onR(msg); });

    sub_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      topic_vel_, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg){ onVel(msg); });

    RCLCPP_INFO(get_logger(), "SO3TorqueController listo: Rd=%s, R=%s, vel=%s -> tau=%s",
                topic_Rd_.c_str(), topic_R_.c_str(), topic_vel_.c_str(), topic_tau_.c_str());
  }

private:
  // Params
  std::string topic_Rd_, topic_R_, topic_vel_, topic_tau_;
  double kR_{3.0}, kW_{0.8};
  Mat3 J_;

  // State
  bool have_Rd_{false}, have_R_{false}, have_w_{false};
  Mat3 Rd_, dRd_, ddRd_;
  Mat3 R_;
  Vec3 w_world_{0,0,0}; // angular en world

  // ROS entities
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_tau_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_Rd_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_R_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vel_;

  void onRd(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg)
  {
    if (msg->data.size() < 27) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "R_desired tamaño inválido (%zu<27)", msg->data.size());
      return;
    }
    std::array<double,9> aR, aRd, aRdd;
    for(int i=0;i<9;i++){ aR[i]=msg->data[i]; aRd[i]=msg->data[9+i]; aRdd[i]=msg->data[18+i]; }
    Rd_   = mat_from_row_major(aR);
    dRd_  = mat_from_row_major(aRd);
    ddRd_ = mat_from_row_major(aRdd);
    have_Rd_ = true;
    compute_and_publish();
  }

  void onR(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg)
  {
    if (msg->data.size() < 9) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "R tamaño inválido (%zu<9)", msg->data.size());
      return;
    }
    std::array<double,9> a{};
    for(int i=0;i<9;i++) a[i]=msg->data[i];
    R_ = mat_from_row_major(a);
    //R_ = R_.T();
    have_R_ = true;
    compute_and_publish();
  }

  void onVel(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg)
  {
    w_world_.x = msg->twist.angular.x;
    w_world_.y = msg->twist.angular.y;
    w_world_.z = msg->twist.angular.z;
    have_w_ = true;
    compute_and_publish();
  }

  void compute_and_publish()
  {
    if (!(have_Rd_ && have_R_ && have_w_)) return;

    // Omega_d^Bd = vee(Rd^T dRd)
    Mat3 RdT = Rd_.T();
    Mat3 Wd_mat = RdT * dRd_;
    Vec3 Omega_d_Bd = vee(Wd_mat); // en marco deseado

    // dot{Omega}_d^Bd = vee( Rd^T ddRd - (Rd^T dRd)^2 )
    Mat3 term = RdT * ddRd_ - (Wd_mat * Wd_mat);
    Vec3 dOmega_d_Bd = vee(term);

    // w_B = R^T * w_world
    Vec3 wB = R_.T() * w_world_;

    //RCLCPP_INFO(this->get_logger(), "RdT = \n [%.3f, %.3f, %.3f] \n [%.3f, %.3f, %.3f] \n [%.3f, %.3f, %.3f] \n", RdT.m[0][0], RdT.m[0][1], RdT.m[0][2], RdT.m[1][0], RdT.m[1][1], RdT.m[1][2], RdT.m[2][0], RdT.m[2][1], RdT.m[2][2]);
    //RCLCPP_INFO(this->get_logger(), "R_ = \n [%.3f, %.3f, %.3f] \n [%.3f, %.3f, %.3f] \n [%.3f, %.3f, %.3f] \n", R_.m[0][0], R_.m[0][1], R_.m[0][2], R_.m[1][0], R_.m[1][1], R_.m[1][2], R_.m[2][0], R_.m[2][1], R_.m[2][2]);

    //RCLCPP_INFO(this->get_logger(), "(RdT*R_) = \n [%.3f, %.3f, %.3f] \n [%.3f, %.3f, %.3f] \n [%.3f, %.3f, %.3f] \n", (RdT*R_).m[0][0], (RdT*R_).m[0][1], (RdT*R_).m[0][2], (RdT*R_).m[1][0], (RdT*R_).m[1][1], (RdT*R_).m[1][2], (RdT*R_).m[2][0], (RdT*R_).m[2][1], (RdT*R_).m[2][2]);

    // e_R = 1/2 vee( Rd^T R - R^T Rd )
    Mat3 ERm = (RdT * R_) - (R_.T() * Rd_);
    Vec3 eR{ 0.5*(ERm.m[2][1]-ERm.m[1][2]),
             0.5*(ERm.m[0][2]-ERm.m[2][0]),
             0.5*(ERm.m[1][0]-ERm.m[0][1]) };

    //eR = 0.5 * eR;
    
    //RCLCPP_INFO(this->get_logger(), "ERm[0] = [%.3f, %.3f, %.3f], ERm[1] = [%.3f, %.3f, %.3f], ERm[2] = [%.3f, %.3f, %.3f]", ERm.m[0][0], ERm.m[0][1], ERm.m[0][2], ERm.m[1][0], ERm.m[1][1], ERm.m[1][2], ERm.m[2][0], ERm.m[2][1], ERm.m[2][2]);

    // e_w = wB - R^T Rd Omega_d^Bd
    Vec3 wd_in_B = (R_.T() * (Rd_ * Omega_d_Bd)); // R^T Rd ΩdBd
    Vec3 ew = wB - wd_in_B;

    // tau = -KReR - Kw ew + J R^T Rd dOmega_d^Bd + wB × (J wB)
    Vec3 JRTRd_dO = J_mul( R_.T() * (Rd_ * dOmega_d_Bd) );
    Vec3 JwB = J_mul(wB);
    Vec3 tau = (eR * (-kR_)) + (ew * (-kW_)) + JRTRd_dO + cross(wB, JwB);

    //RCLCPP_INFO(this->get_logger(), "eR = [%.3f, %.3f, %.3f],  ew = [%.3f, %.3f, %.3f]", eR.x, eR.y, eR.z, ew.x, ew.y, ew.z);

    geometry_msgs::msg::Vector3Stamped out;
    out.header.stamp = this->now();
    out.header.frame_id = "body"; // torques en {B}
    out.vector.x = tau.x;
    out.vector.y = tau.y;
    out.vector.z = tau.z;
    pub_tau_->publish(out);
  }

  // J * v (J general 3x3)
  Vec3 J_mul(const Vec3& v) const {
    return {
      J_.m[0][0]*v.x + J_.m[0][1]*v.y + J_.m[0][2]*v.z,
      J_.m[1][0]*v.x + J_.m[1][1]*v.y + J_.m[1][2]*v.z,
      J_.m[2][0]*v.x + J_.m[2][1]*v.y + J_.m[2][2]*v.z
    };
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SO3TorqueController>());
  rclcpp::shutdown();
  return 0;
}
