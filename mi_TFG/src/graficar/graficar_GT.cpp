#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <vector>
#include <string>
#include <algorithm>

class Clase_Subscriber : public rclcpp::Node
{
    public:
        Clase_Subscriber() : rclcpp::Node("graficar_GT")
        {
            objeto_subscriber_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>
            ("/model/sensor/GT/pose", 10, std::bind(&Clase_Subscriber::pose_callback, this, std::placeholders::_1));
            //objeto_subscriber_vel = this->create_subscription<geometry_msgs::msg::TwistStamped>
            //("/model/sensor/GT/vel", 10, std::bind(&Clase_Subscriber::vel_callback, this, std::placeholders::_1));
            //objeto_subscriber_acc = this->create_subscription<geometry_msgs::msg::AccelStamped>
            //("/model/sensor/GT/acc", 10, std::bind(&Clase_Subscriber::acc_callback, this, std::placeholders::_1));

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

        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_pose)
        {
            std_msgs::msg::Float64MultiArray num;
            num.data = {msg_pose->pose.position.x, msg_pose->pose.position.y, msg_pose->pose.position.z};
            // num.data = {msg_pose.pose.orientation.x, msg_pose.pose.orientation.y, msg_pose.pose.orientation.z, msg_pose.pose.orientation.w};
            // num.data = {msg_pose.pose.position.x, msg_pose.pose.position.y, msg_pose.pose.position.z, msg_pose.pose.orientation.x, msg_pose.pose.orientation.y, msg_pose.pose.orientation.z, msg_pose.pose.orientation.w};
            objeto_publisher_num->publish(num);

            std::vector<std::string> labels = {"x", "y", "z"};
            // std::vector<std::string> labels = {"qx", "qy", "qz", "qw"};
            // std::vector<std::string> labels = {"x", "y", "z","qx", "qy", "qz", "qw"};
            auto lbl_msg = make_labels_2d(labels);
            objeto_publisher_str->publish(lbl_msg);
        }

        void vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg_vel)
        {
            std_msgs::msg::Float64MultiArray num;
            num.data = {msg_vel->twist.linear.x, msg_vel->twist.linear.y, msg_vel->twist.linear.z};
            // num.data = {msg_vel.twist.angular.x, msg_vel.twist.angular.y, msg_vel.twist.angular.z};
            // num.data = {msg_vel.twist.linear.x, msg_vel.twist.linear.y, msg_vel.twist.linear.z, msg_vel.twist.angular.x, msg_vel.twist.angular.y, msg_vel.twist.angular.z};
            objeto_publisher_num->publish(num);

            std::vector<std::string> labels = {"vx", "vy", "vz"};
            // std::vector<std::string> labels = {"wx", "wy", "wz"};
            // std::vector<std::string> labels = {"vx", "vy", "vz","wx", "wy", "wz"};
            auto lbl_msg = make_labels_2d(labels);
            objeto_publisher_str->publish(lbl_msg);
        }

        void acc_callback(const geometry_msgs::msg::AccelStamped::SharedPtr msg_acc)
        {
            std_msgs::msg::Float64MultiArray num;
            num.data = {msg_acc->accel.linear.x, msg_acc->accel.linear.y, msg_acc->accel.linear.z};
            // num.data = {msg_acc.accel.angular.x, msg_acc.accel.angular.y, msg_acc.accel.angular.z};
            // num.data = {msg_acc.accel.linear.x, msg_acc.accel.linear.y, msg_acc.accel.linear.z, msg_acc.accel.angular.x, msg_acc.accel.angular.y, msg_acc.accel.angular.z};
            objeto_publisher_num->publish(num);

            std::vector<std::string> labels = {"ax", "ay", "az"};
            // std::vector<std::string> labels = {"alphax", "alphay", "alphaz"};
            // std::vector<std::string> labels = {"ax", "ay", "az","alphax", "alphay", "alphaz"};
            auto lbl_msg = make_labels_2d(labels);
            objeto_publisher_str->publish(lbl_msg);
        }



        
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr objeto_subscriber_pose;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr objeto_subscriber_vel; 
        rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr objeto_subscriber_acc; 

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