#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <gazebo/common/UpdateInfo.hh>
#include <builtin_interfaces/msg/time.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>

#include <memory>
#include <string>

namespace gazebo {

class PluginSensorGroundtruth : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_    = model;
    ros_node_ = gazebo_ros::Node::Get(sdf);

    // Leer parámetros
    link_name_ = sdf->Get<std::string>("link_name");

    topic_pose_name_ = sdf->Get<std::string>("topic_pose");
    topic_vel_name_ = sdf->Get<std::string>("topic_vel");
    topic_acc_name_ = sdf->Get<std::string>("topic_acc");

    publish_rate_ = sdf->Get<double>("publish_rate",50.0).first;
    frame_id_ = sdf->Get<std::string>("frame_id","world").first;
    frame_propio_ = sdf->Get<bool>("frame_propio",false).first;


    // Obtener links + crear publishers
    link_ = model_->GetLink(link_name_);

    if (!link_)
    {
        RCLCPP_ERROR(ros_node_->get_logger(),"PluginSensorGroundtruth: no encuentro link %s",link_name_.c_str());
    }
    else
    {
        pub_pose_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_pose_name_, 10);
        pub_vel_ = ros_node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic_vel_name_, 10);
        pub_acc_ = ros_node_->create_publisher<geometry_msgs::msg::AccelStamped>(topic_acc_name_, 10);

        RCLCPP_INFO(ros_node_->get_logger(),"Publicando pose de [%s] en [%s] (rate=%.1f Hz)", link_name_.c_str(), topic_pose_name_.c_str(), publish_rate_);
        RCLCPP_INFO(ros_node_->get_logger(),"Publicando velocidad de [%s] en [%s] (rate=%.1f Hz)", link_name_.c_str(), topic_vel_name_.c_str(), publish_rate_);
        RCLCPP_INFO(ros_node_->get_logger(),"Publicando aceleración de [%s] en [%s] (rate=%.1f Hz)", link_name_.c_str(), topic_acc_name_.c_str(), publish_rate_);
    }


    last_sim_time_ = gazebo::common::Time(0, 0);
    prev_time_     = gazebo::common::Time(0, 0);

    // Conectar al bucle de simulación
    update_conn_ = event::Events::ConnectWorldUpdateBegin(std::bind(&PluginSensorGroundtruth::OnUpdate, this, std::placeholders::_1));
  }

private:
  void OnUpdate(const gazebo::common::UpdateInfo& info)
  {  
    // Publicar segun el "publish_rate" deseado
    const gazebo::common::Time sim_time = info.simTime;
    bool enviar = true;
    
    if (publish_rate_ > 0.0)
    {
        enviar = false;
        const double dt = (sim_time - last_sim_time_).Double();
        if (dt >= (1.0 / publish_rate_))
        {
            enviar = true;
            last_sim_time_ = sim_time;
        }
    }
    



    if (enviar)
    {
        // Para pasar al header del mensaje
        builtin_interfaces::msg::Time stamp;
        stamp.sec = static_cast<int32_t>(sim_time.sec);
        stamp.nanosec = static_cast<uint32_t>(sim_time.nsec);


        if (link_ && pub_pose_ && pub_vel_ && pub_acc_)
        {   
            // POSE
            const auto pose = link_->WorldPose();

            geometry_msgs::msg::PoseStamped msg_pose;
            msg_pose.header.stamp = stamp;
            msg_pose.header.frame_id = frame_id_;

            msg_pose.pose.position.x = pose.Pos().X();
            msg_pose.pose.position.y = pose.Pos().Y();
            msg_pose.pose.position.z = pose.Pos().Z();

            msg_pose.pose.orientation.x = pose.Rot().X();
            msg_pose.pose.orientation.y = pose.Rot().Y();
            msg_pose.pose.orientation.z = pose.Rot().Z();
            msg_pose.pose.orientation.w = pose.Rot().W();

            pub_pose_->publish(msg_pose);

            // VELOCIDAD
            auto v_lin = link_->WorldLinearVel();   // ignition::math::Vector3d
            auto v_ang = link_->WorldAngularVel();  // ignition::math::Vector3d

            if (frame_propio_)
            {
                const auto R = pose.Rot();  // world -> link

                v_lin  = R.RotateVectorReverse(v_lin);
                v_ang  = R.RotateVectorReverse(v_ang);

                nombre_header = link_name_;
            }
            else
            {
                nombre_header = frame_id_;
            }


            geometry_msgs::msg::TwistStamped msg_vel;
            msg_vel.header.stamp = stamp;
            msg_vel.header.frame_id = nombre_header;

            msg_vel.twist.linear.x  = v_lin.X();
            msg_vel.twist.linear.y  = v_lin.Y();
            msg_vel.twist.linear.z  = v_lin.Z();

            msg_vel.twist.angular.x = v_ang.X();
            msg_vel.twist.angular.y = v_ang.Y();
            msg_vel.twist.angular.z = v_ang.Z();

            pub_vel_->publish(msg_vel);

            // ACELERACIÓN
            const double dt = (sim_time - prev_time_).Double(); // Tiempo para derivada (desde última publicación)

            v_lin = link_->WorldLinearVel();   // ignition::math::Vector3d
            v_ang = link_->WorldAngularVel();  // ignition::math::Vector3d

            // Si es la primera vez (o dt muy pequeño), no publicamos derivada
            if (has_prev_ && dt >1e-9)
            {
                // Aceleración en world (derivada numérica)
                ignition::math::Vector3d a_lin = (v_lin - prev_v_lin_) / dt;
                ignition::math::Vector3d a_ang = (v_ang - prev_v_ang_) / dt;

                // Actualizar prev para la próxima
                prev_v_lin_ = v_lin;
                prev_v_ang_ = v_ang;

                if (frame_propio_)
                {
                    const auto R = pose.Rot();  // world -> link

                    a_lin  = R.RotateVectorReverse(a_lin);
                    a_ang  = R.RotateVectorReverse(a_ang);

                    nombre_header = link_name_;
                }
                else
                {
                    nombre_header = frame_id_;
                }

                geometry_msgs::msg::AccelStamped msg_acc;
                msg_acc.header.stamp = stamp;
                msg_acc.header.frame_id = nombre_header;

                msg_acc.accel.linear.x  = a_lin.X();
                msg_acc.accel.linear.y  = a_lin.Y();
                msg_acc.accel.linear.z  = a_lin.Z();

                msg_acc.accel.angular.x = a_ang.X();
                msg_acc.accel.angular.y = a_ang.Y();
                msg_acc.accel.angular.z = a_ang.Z();

                pub_acc_->publish(msg_acc);
            }
            else
            {
                prev_v_lin_ = v_lin;
                prev_v_ang_ = v_ang;
                has_prev_   = true;
            }
        }
        prev_time_ = sim_time;
    }
  }

  physics::ModelPtr model_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  event::ConnectionPtr update_conn_;

  std::string link_name_;
  std::string topic_pose_name_;
  std::string topic_vel_name_;
  std::string topic_acc_name_;
  physics::LinkPtr link_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_vel_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr pub_acc_;

  double publish_rate_{50.0};
  std::string frame_id_{"world"};
  std::string nombre_header;
  bool frame_propio_{true};

  gazebo::common::Time last_sim_time_;
  gazebo::common::Time prev_time_;

  ignition::math::Vector3d prev_v_lin_{0,0,0};
  ignition::math::Vector3d prev_v_ang_{0,0,0};
  bool has_prev_{false};
};

GZ_REGISTER_MODEL_PLUGIN(PluginSensorGroundtruth)

}  // namespace gazebo