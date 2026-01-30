#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sstream>
#include <cstdio>
#include <random>
#include <string>

// utilidades
static void replace_all(std::string &s, const std::string &from, const std::string &to) {
  if (from.empty()) return;
  size_t pos = 0;
  while ((pos = s.find(from, pos)) != std::string::npos) {
    s.replace(pos, from.length(), to);
    pos += to.length();
  }
}
static std::string trim_xml(const std::string &raw) {
  auto b = raw.find('<'); if (b == std::string::npos) return "";
  auto e = raw.rfind('>'); if (e == std::string::npos || e < b) return "";
  return raw.substr(b, e-b+1);
}
static std::string run_cmd_stdout(const std::string &cmd) {
  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) return "";
  char buf[2048];
  std::stringstream out;
  while (fgets(buf, sizeof(buf), pipe) != nullptr) out << buf;
  pclose(pipe);
  return out.str();
}

class QuadcopterSpawner : public rclcpp::Node {
public:
  QuadcopterSpawner() : Node("quadcopter_spawner") {
    // parámetros
    name_   = this->declare_parameter<std::string>("robot_name", "my_quadcopter_0");
    ns_     = this->declare_parameter<std::string>("drone_namespace", "drone_0"); // sin barra
    area_   = this->declare_parameter<std::vector<double>>("area_size", {10.0,10.0});
    z_      = this->declare_parameter<double>("z", 0.5);
    random_ = this->declare_parameter<bool>("use_random_spawn", true);
    seed_   = this->declare_parameter<int>("random_seed", 0);

    // servicio /spawn_entity
    client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_logger(), "Esperando el servicio /spawn_entity...");
    }

    // pose inicial
    double x=0.0, y=0.0;
    if (random_ && area_.size()>=2) {
      std::mt19937 rng(static_cast<unsigned>(seed_) ^
                       static_cast<unsigned>(this->now().nanoseconds()) ^
                       static_cast<unsigned>(std::hash<std::string>{}(name_)));
      std::uniform_real_distribution<double> dx(-area_[0]/2.0, area_[0]/2.0);
      std::uniform_real_distribution<double> dy(-area_[1]/2.0, area_[1]/2.0);
      x = dx(rng); y = dy(rng);
    }

    // generar URDF desde xacro (sin args) y sustituir placeholders
    const std::string xacro_path =
      ament_index_cpp::get_package_share_directory("my_quadracopter") + "/urdf/quadcopter3.xacro";
    std::string xml = trim_xml(run_cmd_stdout(std::string("xacro \"") + xacro_path + "\""));
    if (xml.empty() || (xml.find("<robot") == std::string::npos && xml.find("<sdf")==std::string::npos)) {
      RCLCPP_ERROR(get_logger(), "URDF vacío o inválido. Revisa que xacro genere XML.");
      return;
    }

    // Derivar ns del contexto (PushRosNamespace) si existe
    std::string ns_from_ctx = this->get_namespace(); // p.ej. "/drone_1"
    if (!ns_from_ctx.empty() && ns_from_ctx != "/") {
      // quita la barra inicial para tus sustituciones __FRAME_PREFIX__
      if (ns_from_ctx.front() == '/') ns_from_ctx.erase(0,1);
      ns_ = ns_from_ctx;  // pisa el parámetro si venía mal/repetido
    }

    // __NS__ => "/drone_i", __FRAME_PREFIX__ => "drone_i_"
    std::string ns_with = ns_.empty() ? std::string("/") : (ns_.front()=='/' ? ns_ : "/"+ns_);
    std::string frame_prefix = ns_;
    if (!frame_prefix.empty() && frame_prefix.back()!='_') frame_prefix += "_";
    replace_all(xml, "__NS__", ns_with);
    replace_all(xml, "__FRAME_PREFIX__", frame_prefix);

    // petición spawn
    auto req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    req->name = name_;
    req->xml  = xml;
    req->robot_namespace = ns_with; // opcional; los plugins usan <ros><namespace>__NS__</namespace>
    req->initial_pose.position.x = x;
    req->initial_pose.position.y = y;
    req->initial_pose.position.z = z_;

    RCLCPP_INFO(get_logger(), "Spawning '%s' ns=%s (x=%.2f,y=%.2f,z=%.2f) XML=%zu bytes",
                name_.c_str(), ns_with.c_str(), x, y, z_, xml.size());

    auto fut = client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), fut)
          == rclcpp::FutureReturnCode::SUCCESS) {
      auto resp = fut.get();
      RCLCPP_INFO(get_logger(), "SpawnEntity success=%s msg=\"%s\"",
                  resp->success ? "true":"false", resp->status_message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Fallo en /spawn_entity");
    }
  }

private:
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client_;
  std::string name_, ns_;
  std::vector<double> area_;
  double z_;
  bool random_;
  int seed_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadcopterSpawner>());
  rclcpp::shutdown();
  return 0;
}
