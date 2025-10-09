#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>                // Para usar stringstream
#include <cstdlib>               // Para usar popen y pclose

class QuadcopterSpawner : public rclcpp::Node {
public:
    QuadcopterSpawner() : Node("quadcopter_spawner") {
        auto client = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Esperando el servicio /spawn_entity...");
        }

        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = "my_quadcopter";
        request->xml = read_urdf();                       // ← Ahora lee el archivo procesado por xacro
        request->robot_namespace = "/my_quadcopter";
        request->initial_pose.position.x = 0.0;
        request->initial_pose.position.y = 0.0;
        request->initial_pose.position.z = 0.5;

        std::cout << "[spawn_quadcopter_node] URDF enviado:\n" << request->xml << std::endl;

        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Quadcopter insertado exitosamente.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Fallo al insertar el quadcopter.");
        }
    }

private:
    std::string read_urdf() {
        std::string xacro_path = ament_index_cpp::get_package_share_directory("my_quadracopter") + "/urdf/quadcopter3.xacro";

        std::string command = "xacro " + xacro_path;      // ← Ejecutamos el procesador xacro por terminal
        FILE* pipe = popen(command.c_str(), "r");         // ← Abrimos el comando como un stream
        if (!pipe) {
            RCLCPP_ERROR(this->get_logger(), "Error al ejecutar xacro.");
            return "";
        }

        char buffer[128];
        std::stringstream result;
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result << buffer;                             // ← Leemos la salida del comando xacro
        }

        pclose(pipe);                                     // ← Cerramos el proceso
        return result.str();                              // ← Retornamos el contenido generado por xacro
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuadcopterSpawner>());
    rclcpp::shutdown();
    return 0;
}
