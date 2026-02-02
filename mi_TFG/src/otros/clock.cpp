#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class ClockPub : public rclcpp::Node {
public:
  ClockPub() : Node("clock_pub") {
    this->set_parameter(rclcpp::Parameter("use_sim_time", false)); // este nodo usa tiempo real

    pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
      [this]()
      {
        rosgraph_msgs::msg::Clock c;
        c.clock = this->get_clock()->now(); // tiempo real
        pub_->publish(c);
      });
  }
private:
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockPub>());
  rclcpp::shutdown();
  return 0;
}
