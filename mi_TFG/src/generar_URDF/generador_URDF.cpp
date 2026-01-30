#include "rclcpp/rclcpp.hpp"

#include <gazebo_msgs/srv/spawn_entity.hpp>                 // Añadir interfaz usada en el servicio.
#include <ament_index_cpp/get_package_share_directory.hpp>  // Obtener la dirección del paquete y del Xacro

#include <fstream>
#include <sstream>                // Para usar stringstream
#include <cstdlib>               // Para usar popen y pclose

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
                
                request->name = "model";
                request->robot_namespace = "/model";

                request->reference_frame = "world";

                request->initial_pose.position.x = 0.0;
                request->initial_pose.position.y = 0.0;
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
            std::string nombre_paquete = "mi_TFG";         // Nombre del paquete
            std::string ruta_xacro = "/urdf/dron.xacro";       // Path del .xacro

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
                << " fisico_motores_matriz_inercia:=" << "\"" << fisico_motores_matriz_inercia[0] << " " << fisico_motores_matriz_inercia[1] << " " << fisico_motores_matriz_inercia[2] << " " << fisico_motores_matriz_inercia[3] << " " << fisico_motores_matriz_inercia[4] << " " << fisico_motores_matriz_inercia[5] << "\"";

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