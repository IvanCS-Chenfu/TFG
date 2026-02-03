#include "rclcpp/rclcpp.hpp"
#include "mi_tfg/action/tray_action.hpp"

#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <eigen3/Eigen/Dense>

#include <chrono> 
using namespace Eigen;
using namespace std::chrono_literals;

class Clase_Publisher : public rclcpp::Node
{
    public:
        Clase_Publisher() : rclcpp::Node("control_calcular_fuerzas")
        {
            pub_torque_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("control/tray/torque",10);
            pub_fuerza_ = this->create_publisher<std_msgs::msg::Float64>("control/tray/fuerza",10);
            objeto_timer = this->create_wall_timer(20ms, std::bind(&Clase_Publisher::enviar_fuerzas, this));

            sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "sensor/GT/pose", 10, std::bind(&Clase_Publisher::callback_pose, this, std::placeholders::_1));
            sub_vel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "sensor/GT/vel", 10, std::bind(&Clase_Publisher::callback_vel, this, std::placeholders::_1));
            sub_fb_ = this->create_subscription<mi_tfg::action::TrayAction_FeedbackMessage>(
            "AccionTrayectoria/_action/feedback", 10, std::bind(&Clase_Publisher::callback_feedback, this, std::placeholders::_1));

            // Declarar parámetro (tipo, nombre, valor por defecto)
            this->declare_parameter<double>("fisico.cuerpo.masa", 1.4);
            this->declare_parameter<std::vector<double>>("fisico.cuerpo.matriz_inercia", {1e-4, 1e-4, 1e-4, 0.0, 0.0, 0.0});
            this->declare_parameter<double>("fisico.constante.gravedad", -9.81);
            this->declare_parameter<double>("control.fuerza.kp", 2.5);
            this->declare_parameter<double>("control.fuerza.kv", 5.0);
            this->declare_parameter<double>("control.torque.kr", 0.5);
            this->declare_parameter<double>("control.torque.kw", 0.5);

            // Obtener parámetro (decir tipo)
            m = this->get_parameter("fisico.cuerpo.masa").as_double();
            inercia = this->get_parameter("fisico.cuerpo.matriz_inercia").as_double_array();
            gravedad = this->get_parameter("fisico.constante.gravedad").as_double();
            Kp = this->get_parameter("control.fuerza.kp").as_double();
            Kv = this->get_parameter("control.fuerza.kv").as_double();
            Kr = this->get_parameter("control.torque.kr").as_double();
            Kw = this->get_parameter("control.torque.kw").as_double();

            snap_des.setZero();
        }
    private:

        void callback_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {   
            x << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

            double x = msg->pose.orientation.x;
            double y = msg->pose.orientation.y;
            double z = msg->pose.orientation.z;
            double w = msg->pose.orientation.w;

            R_act << 1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
                    2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
                    2*(x*z-w*y), 2*(z*y+w*x), 1-2*(x*x+y*y);
        }

        void callback_vel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
        {
            x_dot << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;

            // Pasar la velocidad del sistema "world" al sistema "body"
            Vector3d w_world(msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z); 
            w_b = R_act.transpose()*w_world;
        }

        void callback_feedback(const mi_tfg::action::TrayAction_FeedbackMessage::SharedPtr msg)
        {
            x_des << msg->feedback.x.data[0], msg->feedback.y.data[0], msg->feedback.z.data[0];
            x_dot_des << msg->feedback.x.data[1], msg->feedback.y.data[1], msg->feedback.z.data[1];
            x_ddot_des << msg->feedback.x.data[2], msg->feedback.y.data[2], msg->feedback.z.data[2];

            yaw_des = msg->feedback.yaw.data[0];
            yaw_dot_des = msg->feedback.yaw.data[1];
            yaw_ddot_des = msg->feedback.yaw.data[2];

            jerk_des << msg->feedback.x.data[3], msg->feedback.y.data[3], msg->feedback.z.data[3];

            feedback_activado = true;
        }




        double mod(const Vector3d &v)
        {
            return std::sqrt(v.squaredNorm());
        }
        Vector3d vee(const Matrix3d &M)
        {
            // Para una matriz skew: [ 0 -z  y; z  0 -x; -y x 0 ] -> (x,y,z)
            return Vector3d(M(2,1), M(0,2), M(1,0));
        }



        void enviar_fuerzas()
        {
            if (feedback_activado)
            {
                // Fuerzas
                Vector3d ep = x - x_des;
                Vector3d ev = x_dot - x_dot_des;
                Vector3d g(0.0, 0.0, gravedad);

                Vector3d F_des = - (Kp * ep) - (Kv * ev) + m * (x_ddot_des - g);        // Desde World
                Vector3d f = F_des;                                                     // Desde World
                Vector3d jerk_F = jerk_des * m;
                F_des = R_act.transpose()*F_des;                                        // Desde Cuerpo

                // Torques
                Matrix3d I = Matrix3d::Identity();
                // Todo desde World
                Vector3d c_yaw(std::cos(yaw_des), std::sin(yaw_des), 0.0);
                Vector3d c_yaw_dot = yaw_dot_des * Vector3d(-std::sin(yaw_des), std::cos(yaw_des), 0.0);
                Vector3d c_yaw_ddot = yaw_ddot_des * Vector3d(-std::sin(yaw_des), std::cos(yaw_des), 0.0) - yaw_dot_des * yaw_dot_des * Vector3d(std::cos(yaw_des), std::sin(yaw_des), 0.0); 

                Vector3d eje_z = f/mod(f);
                Vector3d eje_z_dot = (I - eje_z*eje_z.transpose())*(jerk_F/mod(f));
                Vector3d eje_z_ddot = (I - eje_z*eje_z.transpose())*(snap_des/mod(f) - (jerk_F*(f.transpose()*jerk_F)/(mod(f)*mod(f)*mod(f)))) - eje_z_dot * (eje_z.transpose() * eje_z_dot);

                Vector3d h = eje_z.cross(c_yaw);
                Vector3d h_dot = eje_z_dot.cross(c_yaw) + eje_z.cross(c_yaw_dot);
                Vector3d h_ddot = eje_z_ddot.cross(c_yaw) + 2*eje_z_dot.cross(c_yaw_dot) + eje_z.cross(c_yaw_ddot);

                Vector3d eje_y = h/mod(h);
                Vector3d eje_y_dot = (I - eje_y*eje_y.transpose())*(h_dot/mod(h));
                Vector3d eje_y_ddot = (I - eje_y*eje_y.transpose())*(h_ddot/mod(h) - (h_dot*(h.transpose()*h_dot)/(mod(h)*mod(h)*mod(h)))) - eje_y_dot * eje_y_dot.transpose() * eje_y;

                Vector3d eje_x = eje_y.cross(eje_z);
                Vector3d eje_x_dot = eje_y_dot.cross(eje_z) + eje_y.cross(eje_z_dot);
                Vector3d eje_x_ddot = eje_y_ddot.cross(eje_z) + 2*eje_y_dot.cross(eje_z_dot) + eje_y.cross(eje_z_ddot);

                // Todo desde Cuerpo
                Matrix3d R_des;
                Matrix3d R_dot_des;
                Matrix3d R_ddot_des;

                R_des.col(0) = eje_x; R_des.col(1) = eje_y; R_des.col(2) = eje_z;
                R_dot_des.col(0) = eje_x_dot; R_dot_des.col(1) = eje_y_dot; R_dot_des.col(2) = eje_z_dot;
                R_ddot_des.col(0) = eje_x_ddot; R_ddot_des.col(1) = eje_y_ddot; R_ddot_des.col(2) = eje_z_ddot;


                Vector3d Omega_des = vee(R_des.transpose()*R_dot_des);
                //Vector3d Omega_dot_des = vee(R_des.transpose()*R_ddot_des-(R_des.transpose()*R_dot_des)*(R_des.transpose()*R_dot_des));
                Vector3d Omega_dot_des(0,0,0);

                Vector3d er = vee(R_des.transpose()*R_act - R_act.transpose()*R_des)/2;
                Vector3d ew = w_b - R_act.transpose()*R_des*Omega_des;


                Matrix3d J;
                J <<    inercia[0], inercia[3], inercia[4],
                        inercia[3], inercia[1], inercia[5],
                        inercia[4], inercia[5], inercia[2];

                Vector3d tau_des = -Kr*er - Kw*ew + J*R_act.transpose()*R_des*Omega_dot_des+ w_b.cross(J*w_b);  // Desde Cuerpo


                // Enviar Fuerza y Torque
                std_msgs::msg::Float64 fuerza_deseada;
                fuerza_deseada.data = F_des.z();
                pub_fuerza_->publish(fuerza_deseada);

                geometry_msgs::msg::Vector3Stamped torque_deseado;
                torque_deseado.header.frame_id = "cuerpo";
                torque_deseado.vector.x = tau_des.x();
                torque_deseado.vector.y = tau_des.y();
                torque_deseado.vector.z = tau_des.z();

                pub_torque_->publish(torque_deseado);
            }
        }   
        
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_torque_;  
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fuerza_;
        rclcpp::TimerBase::SharedPtr objeto_timer;                                  

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vel_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_fuerza_;
        rclcpp::Subscription<mi_tfg::action::TrayAction_FeedbackMessage>::SharedPtr sub_fb_;

        Matrix3d R_act;
        Vector3d jerk_des;
        Vector3d snap_des;
        Vector3d w_b;
        Vector3d f;
        Vector3d x;
        Vector3d x_dot;
        Vector3d x_ddot;
        Vector3d x_des;
        Vector3d x_dot_des;
        Vector3d x_ddot_des;

        bool feedback_activado = false;

        double yaw_des;
        double yaw_dot_des;
        double yaw_ddot_des;
        double m;
        std::vector<double> inercia;
        double gravedad;
        double Kp;
        double Kv;
        double Kr;
        double Kw;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto objeto_nodo = std::make_shared<Clase_Publisher>();
    rclcpp::spin(objeto_nodo);

    rclcpp::shutdown();
    return 0;
}