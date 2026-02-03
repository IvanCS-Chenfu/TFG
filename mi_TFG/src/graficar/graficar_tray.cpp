#include "rclcpp/rclcpp.hpp"
#include "mi_tfg/action/tray_action.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <vector>
#include <string>
#include <algorithm>

class Clase_Subscriber : public rclcpp::Node
{
    public:
        Clase_Subscriber() : rclcpp::Node("graficar_tray")
        {
            objeto_subscriber = this->create_subscription<mi_tfg::action::TrayAction_FeedbackMessage>(
            "/AccionTrayectoria/_action/feedback", 10, std::bind(&Clase_Subscriber::feedback_callback, this, std::placeholders::_1));

            objeto_publisher_num = create_publisher<std_msgs::msg::Float64MultiArray>("/numeric_array", 10);
            objeto_publisher_str = create_publisher<std_msgs::msg::UInt8MultiArray>("/labels_array", 10);
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

        void feedback_callback(const mi_tfg::action::TrayAction_FeedbackMessage::SharedPtr msg)
        {
            std::vector<double> x = msg->feedback.x.data;
            std::vector<double> y = msg->feedback.y.data;
            std::vector<double> z = msg->feedback.z.data;
            std::vector<double> yaw = msg->feedback.yaw.data;
            
            if (x.size() > 0 && y.size() > 0 && z.size() > 0 && yaw.size() > 0)
            {
                // std::vector<std::string> labels = {"pose", "vel", "acc", "yerk"};
                std::vector<std::string> labels = {"x", "y", "z", "yaw"};

                std_msgs::msg::Float64MultiArray enviar;
                //enviar.data = {x[0], y[0], z[0], yaw[0]};
                enviar.data = {x[1], y[1], z[1], yaw[1]};
                objeto_publisher_num->publish(enviar);

                auto lbl_msg = make_labels_2d(labels);
                objeto_publisher_str->publish(lbl_msg);
            }
        }


        
        rclcpp::Subscription<mi_tfg::action::TrayAction_FeedbackMessage>::SharedPtr objeto_subscriber;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr objeto_publisher_num;   
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr objeto_publisher_str;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto objeto_nodo = std::make_shared<Clase_Subscriber>();
    rclcpp::spin(objeto_nodo);

    rclcpp::shutdown();
    return 0;
}