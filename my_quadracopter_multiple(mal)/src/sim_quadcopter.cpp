#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <random>
#include <string>
#include <algorithm>

static void replace_all(std::string &s, const std::string &from, const std::string &to) {
    if (from.empty()) return;
    size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) {
        s.replace(pos, from.length(), to);
        pos += to.length();
    }
}

static std::string trim_xml_to_outer_tags(const std::string &raw) {
    // Quita lo que esté antes del primer '<' y después del último '>'
    auto first_lt = raw.find('<');
    if (first_lt == std::string::npos) return "";
    auto last_gt  = raw.rfind('>');
    if (last_gt == std::string::npos || last_gt < first_lt) return "";
    std::string s = raw.substr(first_lt, last_gt - first_lt + 1);
    // Recorta espacios iniciales/finales por si acaso
    while (!s.empty() && (s.front()=='\n' || s.front()=='\r' || s.front()==' ' || s.front()=='\t')) s.erase(s.begin());
    while (!s.empty() && (s.back()=='\n' || s.back()=='\r'  || s.back()==' '  || s.back()=='\t')) s.pop_back();
    return s;
}

class QuadcopterSpawner : public rclcpp::Node {
public:
    QuadcopterSpawner() : Node("quadcopter_spawner") {
        // Parámetros
        this->declare_parameter<int>("num_drones", 1);
        this->declare_parameter<double>("area_size", 10.0);
        this->declare_parameter<int>("seed", 0);  // 0 => aleatorio no determinista

        int num_drones = this->get_parameter("num_drones").as_int();
        double area_size = this->get_parameter("area_size").as_double();
        int seed = this->get_parameter("seed").as_int();

        if (num_drones < 1) { RCLCPP_WARN(this->get_logger(), "num_drones < 1, ajustando a 1."); num_drones = 1; }
        if (area_size <= 0.0) { RCLCPP_WARN(this->get_logger(), "area_size <= 0.0, ajustando a 10.0."); area_size = 10.0; }

        auto client = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Esperando el servicio /spawn_entity...");
        }

        // RNG
        std::mt19937 rng;
        if (seed == 0) { std::random_device rd; rng.seed(rd()); }
        else { rng.seed(static_cast<uint32_t>(seed)); }
        const double half = area_size * 0.5;
        std::uniform_real_distribution<double> dist(-half, half);

        for (int i = 0; i < num_drones; ++i) {
            const std::string ns = "drone_" + std::to_string(i); // sin barra
            const std::string ns_with_slash = "/" + ns;          // con barra

            // 1) Ejecuta xacro, 2) sanea XML, 3) reemplaza marcadores, 4) valida
            std::string urdf_xml = read_and_prepare_urdf(ns, ns_with_slash);
            if (urdf_xml.empty()) {
                RCLCPP_ERROR(this->get_logger(), "URDF vacío/invalidado para %s. No se spawnea.", ns.c_str());
                continue;
            }

            auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
            request->name = "my_quadcopter_" + std::to_string(i);
            request->xml  = urdf_xml;
            request->robot_namespace = ns_with_slash;

            request->initial_pose.position.x = dist(rng);
            request->initial_pose.position.y = dist(rng);
            request->initial_pose.position.z = 0.5;
            request->initial_pose.orientation.x = 0.0;
            request->initial_pose.orientation.y = 0.0;
            request->initial_pose.orientation.z = 0.0;
            request->initial_pose.orientation.w = 1.0;

            RCLCPP_INFO(this->get_logger(),
                "Spawning [%s] en (x=%.2f, y=%.2f, z=%.2f) con ns=%s",
                request->name.c_str(),
                request->initial_pose.position.x,
                request->initial_pose.position.y,
                request->initial_pose.position.z,
                ns_with_slash.c_str()
            );

            auto future = client->async_send_request(request);
            auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

            if (ret == rclcpp::FutureReturnCode::SUCCESS) {
                auto resp = future.get();
                if (!resp->success) {
                    RCLCPP_ERROR(this->get_logger(), "SpawnEntity respondió false: %s", resp->status_message.c_str());
                } else {
                    RCLCPP_INFO(this->get_logger(), "Spawn de %s OK.", request->name.c_str());
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Fallo al llamar al servicio SpawnEntity para %s.", request->name.c_str());
            }
        }

        RCLCPP_INFO(this->get_logger(), "Proceso de spawn completado.");
    }

private:
    std::string read_and_prepare_urdf(const std::string &ns_no_slash, const std::string &ns_with_slash) {
        std::string xacro_path =
            ament_index_cpp::get_package_share_directory("my_quadracopter") + "/urdf/quadcopter3.xacro";

        // IMPORTANTE: No mezcles stderr con stdout; queremos sólo stdout
        std::string command = "xacro --inorder \"" + xacro_path + "\""; // stdout = XML
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) {
            RCLCPP_ERROR(this->get_logger(), "Error al ejecutar xacro.");
            return "";
        }

        std::stringstream out;
        char buffer[4096];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            out << buffer;
        }
        int rc = pclose(pipe);
        if (rc != 0) {
            RCLCPP_ERROR(this->get_logger(), "xacro devolvió código %d (mira tu terminal para stderr).", rc);
            return "";
        }

        // Quitar basura antes/después del XML
        std::string xml = trim_xml_to_outer_tags(out.str());

        // Validar forma básica
        if (xml.find("<robot") == std::string::npos && xml.find("<sdf") == std::string::npos) {
            RCLCPP_ERROR(this->get_logger(),
                "La salida de xacro no parece XML de URDF/SDF. Primeros 200 chars:\n%s",
                xml.substr(0, 200).c_str());
            return "";
        }

        // Reemplazar marcadores por namespace/frames
        replace_all(xml, "__NS__", ns_with_slash);
        replace_all(xml, "__FRAME_PREFIX__", ns_no_slash);

        return xml;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuadcopterSpawner>());
    rclcpp::shutdown();
    return 0;
}
