#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <my_quadracopter/action/poly3_traj.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <string>

using Poly3 = my_quadracopter::action::Poly3Traj;
using FeedbackMsg = Poly3::Impl::FeedbackMessage;

struct V3 {
  double x{0}, y{0}, z{0};
  V3() = default;
  V3(double X,double Y,double Z):x(X),y(Y),z(Z) {}
  V3 operator+(const V3&o)const{return {x+o.x,y+o.y,z+o.z};}
  V3 operator-(const V3&o)const{return {x-o.x,y-o.y,z-o.z};}
  V3 operator*(double s)const{return {x*s,y*s,z*s};}
};
static inline double dot(const V3&a,const V3&b){return a.x*b.x+a.y*b.y+a.z*b.z;}
static inline V3 cross(const V3&a,const V3&b){
  return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
static inline double norm(const V3&a){return std::sqrt(std::max(0.0,dot(a,a)));}
static inline V3 normalized(const V3&a){
  double n=norm(a); if(n<1e-9) return {0,0,1}; return a*(1.0/n);
}
static inline V3 proj_tangent(const V3&u, const V3&v){
  // (I - u u^T) v ; u assumed unit
  return v - u*(dot(u,v));
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

class RdFromForceNode : public rclcpp::Node {
public:
  RdFromForceNode(): Node("rd_from_force")
  {
    snap_zero_ = this->declare_parameter<bool>("snap_zero", true);
    action_name_    = this->declare_parameter<std::string>("action_name", "/poly3_traj");
    tray_prefix_    = this->declare_parameter<std::string>("tray_prefix", "/tray_avpt"); // base de trayectorias
    feedback_topic_ = action_name_ + "/_action/feedback";

    // Suscripción global al feedback: decide el dron y actualiza referencias
    sub_fb_ = this->create_subscription<FeedbackMsg>(
      feedback_topic_, rclcpp::QoS(100),
      [this](FeedbackMsg::ConstSharedPtr fb){
        const std::string ns = normalize_ns(fb->feedback.header.frame_id);

        // Cambió el dron → reconfigurar conexiones y resetear flags
        if (ns != current_ns_) {
          configure_for_namespace(ns);
        }

        // Extraer referencias d(t) del feedback (x,y,z; [pos, vel, acc, jerk, ratio])
        auto get3 = [](const std_msgs::msg::Float64MultiArray &a){
          V3 v{0,0,0};
          if ((int)a.data.size()>=3){ v={a.data[0],a.data[1],a.data[2]}; }
          return v;
        };
        // deseadas
        ad_ = { fb->feedback.x.data.size()>2 ? fb->feedback.x.data[2] : 0.0,
                fb->feedback.y.data.size()>2 ? fb->feedback.y.data[2] : 0.0,
                fb->feedback.z.data.size()>2 ? fb->feedback.z.data[2] : 0.0 };
        jd_ = { fb->feedback.x.data.size()>3 ? fb->feedback.x.data[3] : 0.0,
                fb->feedback.y.data.size()>3 ? fb->feedback.y.data[3] : 0.0,
                fb->feedback.z.data.size()>3 ? fb->feedback.z.data[3] : 0.0 };
        if (!std::isfinite(jd_.x)) jd_.x = 0.0;
        if (!std::isfinite(jd_.y)) jd_.y = 0.0;
        if (!std::isfinite(jd_.z)) jd_.z = 0.0;

        psi_   = fb->feedback.yaw.data.size()>0 ? fb->feedback.yaw.data[0] : 0.0;
        dpsi_  = fb->feedback.yaw.data.size()>1 ? fb->feedback.yaw.data[1] : 0.0;
        ddpsi_ = fb->feedback.yaw.data.size()>2 ? fb->feedback.yaw.data[2] : 0.0;

        // snap
        sd_ = snap_zero_ ? V3{0,0,0} :
              V3{ fb->feedback.x.data.size()>4 ? fb->feedback.x.data[4] : 0.0,
                  fb->feedback.y.data.size()>4 ? fb->feedback.y.data[4] : 0.0,
                  fb->feedback.z.data.size()>4 ? fb->feedback.z.data[4] : 0.0 };

        have_ref_ = true;
        compute_and_publish(this->now());
      });

    RCLCPP_INFO(get_logger(), "Escuchando feedback: %s", feedback_topic_.c_str());
  }

private:
  // Estado de entrada
  V3 A_{0,0,1}; bool have_A_{false};      // fuerza medida del tópico del dron actual
  V3 ad_{0,0,0}, jd_{0,0,0}, sd_{0,0,0};  // refs deseadas del feedback
  double psi_{0.0}, dpsi_{0.0}, ddpsi_{0.0}; bool have_ref_{false};
  bool snap_zero_{true};

  // Infra ROS
  std::string action_name_, feedback_topic_, tray_prefix_;
  std::string current_ns_; // "/drone_i"

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_R_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_F_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr sub_fb_;

  // Reconfigura suscriptor y publisher para el dron actual
  void configure_for_namespace(const std::string &ns) {
    current_ns_ = ns;

    // Reinicia flags de datos
    have_A_ = false;
    have_ref_ = false;

    // Construye nombres de tópicos
    const std::string topic_F = current_ns_ + tray_prefix_ + "/Fuerzas";     // /drone_i/tray_avpt/Fuerzas
    const std::string topic_R = current_ns_ + tray_prefix_ + "/R_desired";   // /drone_i/tray_avpt/R_desired

    // (Re)crear publisher y suscriptor
    pub_R_.reset();
    sub_F_.reset();

    pub_R_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_R, 10);

    sub_F_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      topic_F, rclcpp::QoS(50),
      [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg){
        // seguridad: aceptamos solo si el header.frame_id coincide con el dron activo
        if (normalize_ns(msg->header.frame_id) != current_ns_) return;
        A_ = {msg->vector.x, msg->vector.y, msg->vector.z};
        have_A_ = true;
        compute_and_publish(msg->header.stamp);
      });

    RCLCPP_INFO(this->get_logger(), "NS=%s | sub: %s | pub: %s",
      current_ns_.c_str(), topic_F.c_str(), topic_R.c_str());
  }

  void compute_and_publish(const rclcpp::Time& stamp)
  {
    if(!(have_A_ && have_ref_)) return;
    if(!pub_R_) return;

    // 1) b3d (unit) y derivadas
    const double An = std::max(1e-9, norm(A_));
    const V3 b3d = A_ * (1.0/An);

    const V3 db3d = proj_tangent(b3d, jd_ * (1.0/An));
    const double Aj = dot(A_, jd_);
    const V3 term = jd_ * (Aj / (An*An*An));  // j_d * (A^T j_d)/||A||^3
    V3 dd_b3d = proj_tangent(b3d, (sd_ * (1.0/An)) - term);
    dd_b3d = dd_b3d - db3d * (dot(b3d, db3d)); // estabilización

    // 2) yaw helpers
    const V3 cpsi{ std::cos(psi_), std::sin(psi_), 0.0 };
    const V3 dcpsi{ -dpsi_*std::sin(psi_),  dpsi_*std::cos(psi_), 0.0 };
    const V3 ddcpsi{ -ddpsi_*std::cos(psi_) - dpsi_*dpsi_*std::cos(psi_),
                     -ddpsi_*std::sin(psi_) - dpsi_*dpsi_*std::sin(psi_), 0.0 };

    // 3) h y derivadas
    const V3 h    = cross(b3d, cpsi);
    const V3 dh   = cross(b3d, dcpsi) + cross(db3d, cpsi);
    const V3 ddh  = cross(b3d, ddcpsi) + cross(db3d, dcpsi)*2.0 + cross(dd_b3d, cpsi);

    const double hn = std::max(1e-9, norm(h));
    const V3 b2d    = h * (1.0/hn);
    const V3 db2d   = proj_tangent(b2d, dh * (1.0/hn));
    const V3 dd_b2d = proj_tangent(b2d, (ddh * (1.0/hn)) - (h * (dot(h,dh)/(hn*hn*hn)))) - db2d * (dot(b2d, db2d));

    // 4) b1d y derivadas
    const V3 b1d    = cross(b2d, b3d);
    const V3 db1d   = cross(db2d, b3d) + cross(b2d, db3d);
    const V3 dd_b1d = cross(dd_b2d, b3d) + cross(db2d, db3d)*2.0 + cross(b2d, dd_b3d);

    // 5) empaquetar R, Rdot, Rddot en row-major
    std::array<double,27> arr{};
    auto put = [&](int which, const V3& c1, const V3& c2, const V3& c3){
      int base = which*9;
      arr[base+0]=c1.x; arr[base+1]=c2.x; arr[base+2]=c3.x;
      arr[base+3]=c1.y; arr[base+4]=c2.y; arr[base+5]=c3.y;
      arr[base+6]=c1.z; arr[base+7]=c2.z; arr[base+8]=c3.z;
    };
    put(0, b1d,    b2d,    b3d);
    put(1, db1d,   db2d,   db3d);
    put(2, dd_b1d, dd_b2d, dd_b3d);

    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(arr.begin(), arr.end());
    msg.layout.dim.resize(3);
    msg.layout.dim[0].label="which(R,Rdot,Rddot)"; msg.layout.dim[0].size=3; msg.layout.dim[0].stride=9;
    msg.layout.dim[1].label="row";                 msg.layout.dim[1].size=3; msg.layout.dim[1].stride=3;
    msg.layout.dim[2].label="col";                 msg.layout.dim[2].size=3; msg.layout.dim[2].stride=1;

    pub_R_->publish(msg);
  }
};

int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<RdFromForceNode>());
  rclcpp::shutdown();
  return 0;
}
