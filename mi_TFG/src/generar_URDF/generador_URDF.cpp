#include "rclcpp/rclcpp.hpp"

#include <gazebo_msgs/srv/spawn_entity.hpp>                 // Añadir interfaz usada en el servicio.
#include <ament_index_cpp/get_package_share_directory.hpp>  // Obtener la dirección del paquete y del Xacro

#include <fstream>
#include <sstream>                // Para usar stringstream
#include <cstdlib>               // Para usar popen y pclose

#include <random>               // Para la aparición del dron
#include <algorithm>

#include <chrono>
using namespace std::chrono_literals;

class Clase_Cliente : public rclcpp::Node 
{
    public:
        // Cambiamos la interfaz del servicio
        using SpawnEntity = gazebo_msgs::srv::SpawnEntity;

        Clase_Cliente() : rclcpp::Node("generador_URDF")  
        {   
            // Declarar parámetro (tipo, nombre, valor por defecto)
            this->declare_parameter<std::vector<double>>("fisico.cuerpo.dim", {0.2, 0.2, 0.05});
            this->declare_parameter<std::string>( "fisico.cuerpo.color", "White");
            this->declare_parameter<double>("fisico.cuerpo.masa", 1.0);
            this->declare_parameter<std::vector<double>>("fisico.cuerpo.matriz_inercia", {1e-4, 1e-4, 1e-4, 0.0, 0.0, 0.0});

            this->declare_parameter<int>("fisico.brazos.numero", 4);
            this->declare_parameter<double>("fisico.brazos.grados", 45.0);
            this->declare_parameter<std::vector<double>>("fisico.brazos.dim", {0.2, 0.01});
            this->declare_parameter<std::string>( "fisico.brazos.color", "Gray");
            this->declare_parameter<double>("fisico.brazos.masa", 0.05);
            this->declare_parameter<std::vector<double>>("fisico.brazos.matriz_inercia", {1e-5, 1e-5, 1e-5, 0.0, 0.0, 0.0});

            this->declare_parameter<std::vector<double>>("fisico.motores.dim", {0.02, 0.02});
            this->declare_parameter<std::string>( "fisico.motores.color", "Red");
            this->declare_parameter<double>("fisico.motores.masa", 0.05);
            this->declare_parameter<std::vector<double>>("fisico.motores.matriz_inercia", {1e-5, 1e-5, 1e-5, 0.0, 0.0, 0.0});

            this->declare_parameter<std::vector<double>>("fisico.camara.dim", {0.005, 0.01, 0.01});
            this->declare_parameter<std::string>( "fisico.camara.color", "Yellow");
            this->declare_parameter<double>("fisico.camara.masa", 0.01);
            this->declare_parameter<std::vector<double>>("fisico.camara.matriz_inercia", {1e-6, 1e-6, 1e-6, 0.0, 0.0, 0.0});

            this->declare_parameter<double>("actuadores.conversor.fuerza2torque", 0.02);

            this->declare_parameter<double>("sensores.ground_truth.publish_rate", 50.0);
            this->declare_parameter<double>("sensores.camara.porcentaje_estereo", 0.30);
            this->declare_parameter<double>("sensores.camara.publish_rate", 30.0);
            this->declare_parameter<std::string>("sensores.camara.mostrar_gazebo", "false");

            this->declare_parameter<int>("dron.numero", 1);
            this->declare_parameter<std::vector<double>>("dron.spawn_box", {-10.0, 10.0, -10.0, 10.0});

            // Obtener parámetro (decir tipo)
            fisico_cuerpo_dim = this->get_parameter("fisico.cuerpo.dim").as_double_array();
            fisico_cuerpo_color = this->get_parameter("fisico.cuerpo.color").as_string();
            fisico_cuerpo_masa = this->get_parameter("fisico.cuerpo.masa").as_double();
            fisico_cuerpo_matriz_inercia = this->get_parameter("fisico.cuerpo.matriz_inercia").as_double_array();

            fisico_brazos_numero = this->get_parameter("fisico.brazos.numero").as_int();
            fisico_brazos_grados = this->get_parameter("fisico.brazos.grados").as_double();
            fisico_brazos_dim = this->get_parameter("fisico.brazos.dim").as_double_array();
            fisico_brazos_color = this->get_parameter("fisico.brazos.color").as_string();
            fisico_brazos_masa = this->get_parameter("fisico.brazos.masa").as_double();
            fisico_brazos_matriz_inercia = this->get_parameter("fisico.brazos.matriz_inercia").as_double_array();

            fisico_motores_dim = this->get_parameter("fisico.motores.dim").as_double_array();
            fisico_motores_color = this->get_parameter("fisico.motores.color").as_string();
            fisico_motores_masa = this->get_parameter("fisico.motores.masa").as_double();
            fisico_motores_matriz_inercia = this->get_parameter("fisico.motores.matriz_inercia").as_double_array();

            fisico_camara_dim = this->get_parameter("fisico.camara.dim").as_double_array();
            fisico_camara_color = this->get_parameter("fisico.camara.color").as_string();
            fisico_camara_masa = this->get_parameter("fisico.camara.masa").as_double();
            fisico_camara_matriz_inercia = this->get_parameter("fisico.camara.matriz_inercia").as_double_array();

            actuadores_conversor_fuerza2torque = this->get_parameter("actuadores.conversor.fuerza2torque").as_double();

            sensores_ground_truth_publish_rate = this->get_parameter("sensores.ground_truth.publish_rate").as_double();
            sensores_camara_porcentaje_estereo = this->get_parameter("sensores.camara.porcentaje_estereo").as_double();
            sensores_camara_publish_rate = this->get_parameter("sensores.camara.publish_rate").as_double();
            sensores_camara_mostrar_gazebo = this->get_parameter("sensores.camara.mostrar_gazebo").as_string();

            dron_numero = this->get_parameter("dron.numero").as_int();
            dron_spawn_box = this->get_parameter("dron.spawn_box").as_double_array();

            // Namespace dado en multi_dron
            name_space = this->get_namespace();


            // Llamamos al servicio "spawn_entity"
            objeto_cliente = this->create_client<SpawnEntity>("/spawn_entity");

            while (!objeto_cliente->wait_for_service(1s)) 
            {
                RCLCPP_INFO(this->get_logger(), "Servicio no disponible");
            }

            request = std::make_shared<SpawnEntity::Request>();
        }

        void enviar_datos()
        {
            std::string xml = read_xacro();

            if (xml.empty()) 
            {
                RCLCPP_FATAL(this->get_logger(), "URDF vacío, abortando spawn");
            }
            else
            {
                // Declaramos los valores a enviar al servicio
                request->xml = xml;
                
                request->name = name_space.substr(1);
                request->robot_namespace = name_space;

                request->reference_frame = "world";
                
                double x = 0;
                double y = 0;

                if (dron_numero > 1) 
                {
                    std::random_device rd;
                    std::mt19937 rng_ = std::mt19937(rd());

                    double x1 = dron_spawn_box[0];
                    double x2 = dron_spawn_box[1];
                    double y1 = dron_spawn_box[2];
                    double y2 = dron_spawn_box[3];

                    std::uniform_real_distribution<double> dist_x(x1, x2);
                    std::uniform_real_distribution<double> dist_y(y1, y2);

                    x = dist_x(rng_);
                    y = dist_y(rng_);
                }

                request->initial_pose.position.x = x;
                request->initial_pose.position.y = y;
                request->initial_pose.position.z = 0.15;


                auto future_result = objeto_cliente->async_send_request(request);   // Enviamos los valores al servicio

                // Nos quedamos en bucle esperando hasta recibir la respuesta.
                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) == rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_INFO(this->get_logger(), "Robot model insertado exitosamente.");   //aqui
                } 
                else 
                {
                    RCLCPP_ERROR(this->get_logger(), "Fallo al insertar el robot.");
                }
            }
        }

    private:
        // Función que pasa el archivo ".xacro" a una variable tipo "string".
        std::string read_xacro()
        {   
            std::string nombre_paquete = "mi_tfg";         // Nombre del paquete
            std::string ruta_xacro = "/urdf/dron_plugins.xacro";       // Path del .xacro

            std::string xacro_path = ament_index_cpp::get_package_share_directory(nombre_paquete) + ruta_xacro;

            // Añadimos argumentos al archvo .xacro
            std::ostringstream args;
            args << " fisico_cuerpo_dim:=" << "\"" << fisico_cuerpo_dim[0] << " " << fisico_cuerpo_dim[1] << " " << fisico_cuerpo_dim[2] << "\""
                << " fisico_cuerpo_color:=" << fisico_cuerpo_color
                << " fisico_cuerpo_masa:=" << fisico_cuerpo_masa
                << " fisico_cuerpo_matriz_inercia:=" << "\"" << fisico_cuerpo_matriz_inercia[0] << " " << fisico_cuerpo_matriz_inercia[1] << " " << fisico_cuerpo_matriz_inercia[2] << " " << fisico_cuerpo_matriz_inercia[3] << " " << fisico_cuerpo_matriz_inercia[4] << " " << fisico_cuerpo_matriz_inercia[5] << "\""
                << " fisico_brazos_numero:=" << fisico_brazos_numero
                << " fisico_brazos_grados:=" << fisico_brazos_grados
                << " fisico_brazos_dim:=" << "\"" << fisico_brazos_dim[0] << " " << fisico_brazos_dim[1] << "\""
                << " fisico_brazos_color:=" << fisico_brazos_color
                << " fisico_brazos_masa:=" << fisico_brazos_masa
                << " fisico_brazos_matriz_inercia:=" << "\"" << fisico_brazos_matriz_inercia[0] << " " << fisico_brazos_matriz_inercia[1] << " " << fisico_brazos_matriz_inercia[2] << " " << fisico_brazos_matriz_inercia[3] << " " << fisico_brazos_matriz_inercia[4] << " " << fisico_brazos_matriz_inercia[5] << "\""
                << " fisico_motores_dim:=" << "\"" << fisico_motores_dim[0] << " " << fisico_motores_dim[1] << "\""
                << " fisico_motores_color:=" << fisico_motores_color
                << " fisico_motores_masa:=" << fisico_motores_masa
                << " fisico_motores_matriz_inercia:=" << "\"" << fisico_motores_matriz_inercia[0] << " " << fisico_motores_matriz_inercia[1] << " " << fisico_motores_matriz_inercia[2] << " " << fisico_motores_matriz_inercia[3] << " " << fisico_motores_matriz_inercia[4] << " " << fisico_motores_matriz_inercia[5] << "\""
                << " fisico_camara_dim:=" << "\"" << fisico_camara_dim[0] << " " << fisico_camara_dim[1] << " " << fisico_camara_dim[2] << "\""
                << " fisico_camara_color:=" << fisico_camara_color
                << " fisico_camara_masa:=" << fisico_camara_masa
                << " fisico_camara_matriz_inercia:=" << "\"" << fisico_camara_matriz_inercia[0] << " " << fisico_camara_matriz_inercia[1] << " " << fisico_camara_matriz_inercia[2] << " " << fisico_camara_matriz_inercia[3] << " " << fisico_camara_matriz_inercia[4] << " " << fisico_camara_matriz_inercia[5] << "\""
                << " actuadores_conversor_fuerza2torque:=" << actuadores_conversor_fuerza2torque
                << " sensores_ground_truth_publish_rate:=" << sensores_ground_truth_publish_rate
                << " sensores_camara_porcentaje_estereo:=" << sensores_camara_porcentaje_estereo
                << " sensores_camara_publish_rate:=" << sensores_camara_publish_rate
                << " sensores_camara_mostrar_gazebo:=" << sensores_camara_mostrar_gazebo;

            std::string command = "xacro " + xacro_path + args.str();      // Ejecutamos el procesador xacro por terminal
            FILE* pipe = popen(command.c_str(), "r");         // Abrimos el comando como un stream
            if (!pipe)
            {
                RCLCPP_ERROR(this->get_logger(), "Error al ejecutar xacro.");
                return "";
            }

            char buffer[256];
            std::stringstream result;
            while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
            {
                result << buffer;                             // Leemos la salida del comando xacro
            }

            pclose(pipe);                                     // Cerramos el proceso
            return result.str();                              // Retornamos el contenido generado por xacro
        }
        

        rclcpp::Client<SpawnEntity>::SharedPtr objeto_cliente;
        std::shared_ptr<SpawnEntity::Request> request;


        /* PARÁMETROS YAML */
        std::vector<double> fisico_cuerpo_dim;
        std::string fisico_cuerpo_color;
        double fisico_cuerpo_masa;
        std::vector<double> fisico_cuerpo_matriz_inercia;

        int fisico_brazos_numero;
        double fisico_brazos_grados;
        std::vector<double> fisico_brazos_dim;
        std::string fisico_brazos_color;
        double fisico_brazos_masa;
        std::vector<double> fisico_brazos_matriz_inercia;

        std::vector<double> fisico_motores_dim;
        std::string fisico_motores_color;
        double fisico_motores_masa;
        std::vector<double> fisico_motores_matriz_inercia;

        std::vector<double> fisico_camara_dim;
        std::string fisico_camara_color;
        double fisico_camara_masa;
        std::vector<double> fisico_camara_matriz_inercia;

        double actuadores_conversor_fuerza2torque;

        double sensores_ground_truth_publish_rate;
        double sensores_camara_porcentaje_estereo;
        double sensores_camara_publish_rate;
        std::string sensores_camara_mostrar_gazebo;

        std::string name_space;

        int dron_numero;
        std::vector<double> dron_spawn_box;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto objeto_nodo = std::make_shared<Clase_Cliente>();

    objeto_nodo->enviar_datos();

    // Elimino spin ya que quiero generar el URDF y eliminar el nodo

    rclcpp::shutdown();
    return 0;
}