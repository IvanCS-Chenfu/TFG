#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>

using namespace std::chrono_literals;

class PwmPublisher : public rclcpp::Node
{
public:
    PwmPublisher() : Node("pwm_publisher"), pwm_value_(4000)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/pwm_signals", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&PwmPublisher::publish_pwm, this));

        std::thread(&PwmPublisher::keyboard_loop, this).detach();
    }

private:
    int pwm_value_;

    void publish_pwm()
    {
        auto msg = std_msgs::msg::Int32MultiArray();
        msg.data = {pwm_value_, pwm_value_, pwm_value_, pwm_value_};
        publisher_->publish(msg);
    }

    void keyboard_loop()
    {
        set_terminal_mode();
        std::cout << "Control con teclado: W (subir), S (bajar), Q (salir)\n";
        char c;
        while (rclcpp::ok())
        {
            c = getchar();
            if (c == 'q' || c == 'Q') {
                break;
            } else if (c == 'w' || c == 'W') {
                pwm_value_ = std::min(20000, pwm_value_ + 25);
            } else if (c == 's' || c == 'S') {
                pwm_value_ = std::max(0, pwm_value_ - 25);
            }
            std::cout << "PWM actual: " << pwm_value_ << std::endl;
        }
        reset_terminal_mode();
        rclcpp::shutdown();
    }

    void set_terminal_mode()
    {
        tcgetattr(0, &orig_termios_);
        struct termios new_termios = orig_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(0, TCSANOW, &new_termios);
    }

    void reset_terminal_mode()
    {
        tcsetattr(0, TCSANOW, &orig_termios_);
    }

    struct termios orig_termios_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PwmPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
