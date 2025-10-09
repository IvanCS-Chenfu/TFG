#include "my_quadracopter/quadcopter_plugin.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <ignition/math/Vector3.hh>

using namespace gazebo;

QuadcopterPlugin::QuadcopterPlugin() : links_initialized_(false) {}

void gazebo::QuadcopterPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    model_ = _model;

    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    ros_node_ = rclcpp::Node::make_shared("quadcopter_plugin_node");

    auto yaml_path = ament_index_cpp::get_package_share_directory("my_quadracopter") + "/config/dron.yaml";
    auto config = YAML::LoadFile(yaml_path);

    coef_pwm_w_ = config["plugin"]["coeficiente_pwm_w"].as<double>();
    coef_force_ = config["helices"]["coeficiente_fuerza"].as<double>();
    coef_torque_ = config["helices"]["coeficiente_par"].as<double>();
    radius_ = config["helices"]["radio_helice"].as<double>();

    motor_links_ = { "front_motor", "back_motor", "left_motor", "right_motor" };
    pwm_values_.resize(4, 0);

    std::string topic = "/pwm_signals";
    if (_sdf->HasElement("ros_topic")) {
        topic = _sdf->Get<std::string>("ros_topic");
    }

    sub_ = ros_node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        topic, 10, std::bind(&QuadcopterPlugin::OnRosMsg, this, std::placeholders::_1)
    );

    std::thread([node = ros_node_]() { rclcpp::spin(node); }).detach();

    gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&QuadcopterPlugin::ApplyForces, this));
}

void QuadcopterPlugin::OnRosMsg(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    pwm_values_ = msg->data;
    ApplyForces();
}

void gazebo::QuadcopterPlugin::ApplyForces()
{
    if (!model_) return;

    if (!links_initialized_) {
        links_.clear();
        for (const auto& name : motor_links_) {
            auto link = model_->GetLink(name);
            if (!link) return;
            links_.push_back(link);
        }
        links_initialized_ = true;
    }

    for (size_t i = 0; i < pwm_values_.size() && i < links_.size(); ++i) {
        double w = coef_pwm_w_ * pwm_values_[i];
        double thrust = coef_force_ * w * w * radius_ * radius_;
        double torque_z = coef_torque_ * w;

        links_[i]->AddRelativeForce({0, 0, thrust});
        links_[i]->AddRelativeTorque({0, 0, torque_z});
    }
}

GZ_REGISTER_MODEL_PLUGIN(QuadcopterPlugin)
