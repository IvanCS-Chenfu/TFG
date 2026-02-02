#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>

#include "mi_tfg/action/tray_action.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>

#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

#include <chrono> 
using namespace std::chrono_literals;

class Clase_Subscriber : public rclcpp::Node
{
    public:
        Clase_Subscriber() : rclcpp::Node("graficar_GTvsTray")
        {
            objeto_subscriber_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>
            ("sensor/GT/pose", 10, std::bind(&Clase_Subscriber::pose_callback, this, std::placeholders::_1));
            //objeto_subscriber_vel = this->create_subscription<geometry_msgs::msg::TwistStamped>
            //("sensor/GT/vel", 10, std::bind(&Clase_Subscriber::vel_callback, this, std::placeholders::_1));
            //objeto_subscriber_acc = this->create_subscription<geometry_msgs::msg::AccelStamped>
            //("sensor/GT/acc", 10, std::bind(&Clase_Subscriber::acc_callback, this, std::placeholders::_1));

            objeto_subscriber_feedback = this->create_subscription<mi_tfg::action::TrayAction_FeedbackMessage>(
            "AccionTrayectoria/_action/feedback", 10, std::bind(&Clase_Subscriber::feedback_callback, this, std::placeholders::_1));

            objeto_publisher_num = create_publisher<std_msgs::msg::Float64MultiArray>("/numeric_array", 10);
            objeto_publisher_str = create_publisher<std_msgs::msg::UInt8MultiArray>("/labels_array", 10);

            objeto_timer = this->create_wall_timer(20ms, std::bind(&Clase_Subscriber::enviar_graficas, this));
        }
    private:

        static std_msgs::msg::UInt8MultiArray make_labels_2d(const std::vector<std::string>& labels)
        {
            // M = ancho fijo = max_len (mínimo 1)
            size_t n = labels.size();
            size_t max_len = 1;
            for (auto &s : labels) max_len = std::max(max_len, s.size());

            std_msgs::msg::UInt8MultiArray msg;

            // Layout 2D: [N x M]
            msg.layout.dim.resize(2);
            msg.layout.dim[0].label  = "words";
            msg.layout.dim[0].size   = static_cast<uint32_t>(n);
            msg.layout.dim[0].stride = static_cast<uint32_t>(n * max_len);

            msg.layout.dim[1].label  = "chars";
            msg.layout.dim[1].size   = static_cast<uint32_t>(max_len);
            msg.layout.dim[1].stride = static_cast<uint32_t>(max_len);

            msg.layout.data_offset = 0;

            msg.data.assign(n * max_len, 0); // relleno NUL

            for (size_t i = 0; i < n; ++i)
            {
            const auto &s = labels[i];
            for (size_t j = 0; j < s.size() && j < max_len; ++j)
                msg.data[i * max_len + j] = static_cast<uint8_t>(s[j]); // ASCII/UTF-8 byte
            }
            return msg;
        }



        double pose2yaw(const geometry_msgs::msg::PoseStamped &pose_stamped)
        {
            double x = pose_stamped.pose.orientation.x;
            double y = pose_stamped.pose.orientation.y;
            double z = pose_stamped.pose.orientation.z;
            double w = pose_stamped.pose.orientation.w;

            double seno = 2.0 * (w * z + x * y);
            double coseno = 1.0 - 2.0 * (y * y + z * z);

            double yaw = std::atan2(seno, coseno);
            return yaw;
        }



        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_pose)
        {
            valor_real.data = {msg_pose->pose.position.x, msg_pose->pose.position.y, msg_pose->pose.position.z, pose2yaw(*msg_pose)};
            
            labels_real = {"x_real", "y_real", "z_real", "yaw_real"};
        }

        void vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg_vel)
        {
            valor_real.data = {msg_vel->twist.linear.x, msg_vel->twist.linear.y, msg_vel->twist.linear.z, msg_vel->twist.angular.z};

            labels_real = {"vx_real", "vy_real", "vz_real", "vyaw_real"};
        }

        void acc_callback(const geometry_msgs::msg::AccelStamped::SharedPtr msg_acc)
        {
            valor_real.data = {msg_acc->accel.linear.x, msg_acc->accel.linear.y, msg_acc->accel.linear.z, msg_acc->accel.angular.z};

            labels_real = {"ax_real", "ay_real", "az_real", "ayaw_real"};
        }

        void feedback_callback(const mi_tfg::action::TrayAction_FeedbackMessage::SharedPtr msg)
        {
            std::vector<double> x = msg->feedback.x.data;
            std::vector<double> y = msg->feedback.y.data;
            std::vector<double> z = msg->feedback.z.data;
            std::vector<double> yaw = msg->feedback.yaw.data;
            
            if (x.size() > 0 && y.size() > 0 && z.size() > 0 && yaw.size() > 0)
            {
                valor_tray.data = {x[0], y[0], z[0], yaw[0]};
                // valor_tray.data = {x[1], y[1], z[1], yaw[1]};
                // valor_tray.data = {x[2], y[2], z[2], yaw[2]};

                labels_tray = {"x_tray", "y_tray", "z_tray", "yaw_tray"};
                // labels_tray = {"vx_tray", "vy_tray", "vz_tray", "vyaw_tray"};
                // labels_tray = {"ax_tray", "ay_tray", "az_tray", "ayaw_tray"};

                tray_activada = true;
            }

        }



        void enviar_graficas()
        {
            if (tray_activada)
            {
                /*
                std_msgs::msg::Float64MultiArray enviar;
                enviar.data = {valor_real.data[0], valor_real.data[1], valor_real.data[2], valor_real.data[3], valor_tray.data[0], valor_tray.data[1], valor_tray.data[2], valor_tray.data[3]};
                objeto_publisher_num->publish(enviar);

                std::vector<std::string> labels;
                labels = {labels_real[0], labels_real[1], labels_real[2], labels_real[3], labels_tray[0], labels_tray[1], labels_tray[2], labels_tray[3]};
                auto lbl_msg = make_labels_2d(labels);
                objeto_publisher_str->publish(lbl_msg);
                */
                
                std_msgs::msg::Float64MultiArray enviar;
                enviar.data = {valor_tray.data[0]-valor_real.data[0], valor_tray.data[1]-valor_real.data[1], valor_tray.data[2]-valor_real.data[2], valor_tray.data[3]-valor_real.data[3]};
                objeto_publisher_num->publish(enviar);

                std::vector<std::string> labels;
                labels = {"e_x", "e_y", "e_z", "e_yaw"};
                auto lbl_msg = make_labels_2d(labels);
                objeto_publisher_str->publish(lbl_msg);
              
            }
        }

        
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr objeto_subscriber_pose;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr objeto_subscriber_vel; 
        rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr objeto_subscriber_acc; 

        rclcpp::Subscription<mi_tfg::action::TrayAction_FeedbackMessage>::SharedPtr objeto_subscriber_feedback;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr objeto_publisher_num;   
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr objeto_publisher_str;
        rclcpp::TimerBase::SharedPtr objeto_timer;  

        std_msgs::msg::Float64MultiArray valor_real;
        std::vector<std::string> labels_real;

        std_msgs::msg::Float64MultiArray valor_tray;
        std::vector<std::string> labels_tray;

        bool tray_activada = false;
};  

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto objeto_nodo = std::make_shared<Clase_Subscriber>();
    rclcpp::spin(objeto_nodo);

    rclcpp::shutdown();
    return 0;
}