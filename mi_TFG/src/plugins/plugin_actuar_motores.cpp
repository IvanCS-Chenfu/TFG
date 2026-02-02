#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <memory>
#include <string>

namespace gazebo {
class PluginActuarMotores : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override 
  {
    model_    = model;
    ros_node_ = gazebo_ros::Node::Get(sdf);

    // --- Leer cuatro pares <link_name, topic_name> del SDF ---
    link_names_[0]  = sdf->Get<std::string>("motor1");
    link_names_[1]  = sdf->Get<std::string>("motor2");
    link_names_[2]  = sdf->Get<std::string>("motor3");
    link_names_[3]  = sdf->Get<std::string>("motor4");

    topic_names_[0] = sdf->Get<std::string>("topic1");
    topic_names_[1] = sdf->Get<std::string>("topic2");
    topic_names_[2] = sdf->Get<std::string>("topic3");
    topic_names_[3] = sdf->Get<std::string>("topic4");

    fuerza2torque = sdf->Get<double>("fuerza2torque", 0.02).first;

    // Caso NO quadracopter
    if (sdf->HasElement("motor5"))
    {
        link_names_[4]  = sdf->Get<std::string>("motor5");
        topic_names_[4] = sdf->Get<std::string>("topic5");
        link_names_[5]  = sdf->Get<std::string>("motor6");
        topic_names_[5] = sdf->Get<std::string>("topic6");
        numero_motores = 6;
    }
    if (sdf->HasElement("motor7"))
    {
        link_names_[6]  = sdf->Get<std::string>("motor7");
        topic_names_[6] = sdf->Get<std::string>("topic7");
        link_names_[7]  = sdf->Get<std::string>("motor8");
        topic_names_[7] = sdf->Get<std::string>("topic8");
        numero_motores = 8;
    }

    // --- Obtener punteros a links y crear suscripciones ---
    for (int i = 0; i < numero_motores; ++i)
    {
      links_[i] = model_->GetLink(link_names_[i]);

      if (!links_[i]) 
      {
        RCLCPP_ERROR(ros_node_->get_logger(), "PluginActuarMotores: no encuentro link %s", link_names_[i].c_str());
      }
      else
      {
        // Crear suscripción a cada tópico
        subs_[i] = ros_node_->create_subscription<std_msgs::msg::Float64>
        (topic_names_[i], 10, [this, i](const std_msgs::msg::Float64::SharedPtr msg)
        {
            this->OnWrench(i, msg);
        });

        RCLCPP_INFO(ros_node_->get_logger(),"Subscrito a [%s] para aplicar wrench en [%s]", topic_names_[i].c_str(), link_names_[i].c_str());
      }
    }

    // Conectar al bucle de simulación
    update_conn_ = event::Events::ConnectWorldUpdateBegin(std::bind(&PluginActuarMotores::OnUpdate, this));
  }

private:
  // Callback parametrizado por índice
  void OnWrench(int idx, const std_msgs::msg::Float64::SharedPtr msg) 
  {
    last_forces_[idx]  = {0, 0, msg->data};
    last_torques_[idx] = {0, 0, msg->data*fuerza2torque};

    if (idx%2 == 0)
    {
        last_torques_[idx] = -last_torques_[idx];
    }
  }

  // En cada tick, reaplicamos todas las fuerzas/torques
  void OnUpdate() 
  {
    for (int i = 0; i < numero_motores; ++i)
    {
      if (links_[i])
      {
        links_[i]->AddRelativeForce(last_forces_[i]);     // Sobre el centro de masas
        links_[i]->AddRelativeTorque(last_torques_[i]);   // Sobre el centro de masas
      }
    }
  }

  // Miembros
  physics::ModelPtr                      model_;
  std::shared_ptr<rclcpp::Node>          ros_node_;
  event::ConnectionPtr                   update_conn_;

  int numero_motores = 4;
  double fuerza2torque;

  std::string                            link_names_[8];
  std::string                            topic_names_[8];
  physics::LinkPtr                       links_[8];

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subs_[8];
  ignition::math::Vector3d               last_forces_[8]{{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
  ignition::math::Vector3d               last_torques_[8]{{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
};

GZ_REGISTER_MODEL_PLUGIN(PluginActuarMotores)
}  // namespace gazebo