#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <memory>
#include <string>

namespace gazebo {
class MultiWrenchPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_    = model;
    ros_node_ = gazebo_ros::Node::Get(sdf);

    // --- Leer tres pares <link_name, topic_name> del SDF ---
    link_names_[0]  = sdf->Get<std::string>("link_name_motor1");
    link_names_[1]  = sdf->Get<std::string>("link_name_motor2");
    link_names_[2]  = sdf->Get<std::string>("link_name_motor3");
    link_names_[3]  = sdf->Get<std::string>("link_name_motor4");

    topic_names_[0] = sdf->Get<std::string>("topic_motor1");
    topic_names_[1] = sdf->Get<std::string>("topic_motor2");
    topic_names_[2] = sdf->Get<std::string>("topic_motor3");
    topic_names_[3] = sdf->Get<std::string>("topic_motor4");

    // --- Obtener punteros a links y crear suscripciones ---
    for (int i = 0; i < 4; ++i) {
      links_[i] = model_->GetLink(link_names_[i]);
      if (!links_[i]) {
        RCLCPP_ERROR(ros_node_->get_logger(),
          "MultiWrenchPlugin: no encuentro link %s",
          link_names_[i].c_str());
        return;
      }
      // Crear suscripción a cada tópico
      subs_[i] = ros_node_->create_subscription<geometry_msgs::msg::Wrench>(
        topic_names_[i], 10,
        [this, i](const geometry_msgs::msg::Wrench::SharedPtr msg) {
            this->OnWrench(i, msg);
        });

      RCLCPP_INFO(ros_node_->get_logger(),
        "Subscrito a [%s] para aplicar wrench en [%s]",
        topic_names_[i].c_str(), link_names_[i].c_str());
    }

    // Conectar al bucle de simulación
    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MultiWrenchPlugin::OnUpdate, this));
  }

private:
  // Callback parametrizado por índice
  void OnWrench(int idx, const geometry_msgs::msg::Wrench::SharedPtr msg) {
    last_forces_[idx]  = {msg->force.x,  msg->force.y,  msg->force.z};
    last_torques_[idx] = {msg->torque.x, msg->torque.y, msg->torque.z};
  }

  // En cada tick, reaplicamos todas las fuerzas/torques
  void OnUpdate() {
    for (int i = 0; i < 4; ++i) {
      links_[i]->AddRelativeForce(last_forces_[i]);
      links_[i]->AddRelativeTorque(last_torques_[i]);
    }
  }

  // Miembros
  physics::ModelPtr                      model_;
  std::shared_ptr<rclcpp::Node>          ros_node_;
  event::ConnectionPtr                   update_conn_;

  std::string                            link_names_[4];
  std::string                            topic_names_[4];
  physics::LinkPtr                       links_[4];
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr subs_[4];
  ignition::math::Vector3d               last_forces_[4]{{0,0,0},{0,0,0},{0,0,0}};
  ignition::math::Vector3d               last_torques_[4]{{0,0,0},{0,0,0},{0,0,0}};
};

GZ_REGISTER_MODEL_PLUGIN(MultiWrenchPlugin)
}  // namespace gazebo
