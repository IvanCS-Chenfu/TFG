#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>                      // Librería necesaria para crear la acción.

#include "mi_tfg/action/tray_action.hpp"             // Añadir interfaz usada en el servicio.
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <cmath>
#include <algorithm>
#include <thread>
#include <mutex>


class Clase_Servicio_Accion : public rclcpp::Node
{
    public:
        using TrayAction = mi_tfg::action::TrayAction;
        using GoalHandleTrayAction = rclcpp_action::ServerGoalHandle<TrayAction>;

        Clase_Servicio_Accion() : rclcpp::Node("gen_tray_punto")
        {
            // Suscripción (siempre activa). En execute solo usamos la primera muestra que llegue.
            sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "sensor/GT/pose", rclcpp::QoS(10), std::bind(&Clase_Servicio_Accion::pose_actual_callback, this, std::placeholders::_1));
            sub_vel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "sensor/GT/vel", rclcpp::QoS(10), std::bind(&Clase_Servicio_Accion::vel_actual_callback, this, std::placeholders::_1));

            // Añadimos al objeto del servidor de la acción el tipo de interfaz a utilizar, el nombre de la acción el cual
            // el cliente de la acción debe llamar para acceder a él, y el nombre de las función callback (explicadas después).
            objeto_servicio_accion = rclcpp_action::create_server<TrayAction>(this, "AccionTrayectoria",
                std::bind(&Clase_Servicio_Accion::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&Clase_Servicio_Accion::handle_cancel, this, std::placeholders::_1),
                std::bind(&Clase_Servicio_Accion::handle_accepted, this, std::placeholders::_1));

        }
    private:

        // Obtener Pose Actual
        void pose_actual_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {   
            std::lock_guard<std::mutex> lk(pose_callback_mtx_);
            if(!bloquear_callback)
            {
                last_pose_ = *msg;
                bloquear_callback = true;
                RCLCPP_INFO(this->get_logger(), "pose_actualizada (callback)");
            }
        }

        // Obtener Vel Actual
        void vel_actual_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lk(vel_callback_mtx_);
            if (!bloquear_vel)
            {
                last_vel_ = *msg;
                bloquear_vel = true;
                RCLCPP_INFO(this->get_logger(), "vel_actualizada (callback)");
            }
        }

        // Estructura para obtener los coeficientes de cada polinomio cúbico
        struct coef_cubic
        {
            double a0;
            double a1;
            double a2;
            double a3;

            double tf;
        };

        // Obtener "Yaw" de la variable tipo "Pose"
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

        // Para una variable, calcula los coeficientes de su polinomio cúbico
        coef_cubic calcular_coeficientes(double q0, double qf, double v0, double vf, double tf)
        {
            coef_cubic c;
            c.tf = tf;
            c.a0 = q0;
            c.a1 = v0;
            c.a2 = (3.0*(qf - q0) - (2.0*v0 + vf)*tf) / (tf*tf);
            c.a3 = (2.0*(q0 - qf) + (v0 + vf)*tf) / (tf*tf*tf);

            return c;
        }

        // Para una variable, utiliza los coeficientes para calcular las variables deseadas
        std_msgs::msg::Float64MultiArray eval_cubic(const coef_cubic &c, double t)
        {
            std_msgs::msg::Float64MultiArray msg;
            msg.data.resize(5);
            
            double tt  = t;
            double tt2 = t * t;
            double tt3 = t * t * t;
            int a_cero;         // Para que los valores sean 0 en el instante final

            if (c.tf > t)
            {
                tt  = t;
                tt2 = t * t;
                tt3 = t * t * t;
                a_cero = 1;
            }
            else
            {
                tt  = c.tf;
                tt2 = c.tf * c.tf;
                tt3 = c.tf * c.tf * c.tf;
                a_cero = 0;
            }

            msg.data[0] = c.a0 + c.a1*tt + c.a2*tt2 + c.a3*tt3;
            msg.data[1] = a_cero*(c.a1 + 2.0*c.a2*tt + 3.0*c.a3*tt2);
            msg.data[2] = a_cero*(2.0*c.a2 + 6.0*c.a3*tt);
            msg.data[3] = a_cero*(6.0*c.a3);
            msg.data[4] = (tt / c.tf) * 100.0;

            return msg;
        }
        









        // Se llama cuando el cliente de la acción realiza la petición
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const TrayAction::Goal> goal)
        {
            (void)goal;
            // Si devolvemos (ACCEPT_AND_EXECUTE) se llama a la función "handle_accepted" (y se interrumpe la acción anterior)
            // Si devolvemos (ACCEPT_AND_DEFER) se espera a que termine la acción anterior y se llama a "handle_accepted"
            // Si devolvemos (REJECT) rechazamos el goal y no se llama a "handle_accepted"

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // Se llama cuando el cliente cancela la acción (por ejemplo: goal_handle.cancel_goal_async()).
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTrayAction> /*goal_handle*/)
        {
            RCLCPP_INFO(this->get_logger(), "Petición de cancelación recibida");

            // Si devolvemos (ACCEPT), la acción se cancela.
            // Si devolvemos (REJECT), la acción sigue.
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // Se llama cuando se acepta la petición en "handle_goal"
        void handle_accepted(const std::shared_ptr<GoalHandleTrayAction> goal_handle)
        {
            // Llama a la función "execute" como hilo (en segundo plano) con el fin de no bloquear la acción ante otras posibles llamadas
            std::shared_ptr<GoalHandleTrayAction> prev_goal;

            active_mtx_.lock();
            prev_goal = active_goal_;
            active_goal_ = goal_handle;
            active_mtx_.unlock(); 

            if (prev_goal && prev_goal->is_active())
            {
                auto res = std::make_shared<TrayAction::Result>();
                res->success = false;
                res->t_total = 0.0f;
                prev_goal->abort(res);
            }

            std::thread(&Clase_Servicio_Accion::execute, this, goal_handle).detach();
        }








        void execute(const std::shared_ptr<GoalHandleTrayAction> goal_handle)
        {
            // Para que cada vez que se llame a la acción se reinicien las variables.
            bool eliminado = false;         // al declararse aquí, es variable local de cada thread
            bool cancelado = false;         // necesario para goal_handle->canceled
            pose_callback_mtx_.lock();
            bloquear_callback = false;   
            pose_callback_mtx_.unlock();
            vel_callback_mtx_.lock();
            bloquear_vel = false;   
            vel_callback_mtx_.unlock();
            double t = 0.0;

            rclcpp::Rate wait_rate(100.0);

            bool pose_ready = false, vel_ready = false;;        // creada para aislarla de la variable global "bloquear_callback"

            while (rclcpp::ok() && !eliminado && (!pose_ready || !vel_ready))   // Bloquear hasta que se reciba la pose actual.
            {
                wait_rate.sleep();  // Esperar para evitar espera activa.
                if (goal_handle->is_canceling())
                {
                    eliminado = true;
                    cancelado = true;
                }
                active_mtx_.lock();
                if (active_goal_ != goal_handle && !eliminado)
                {
                    eliminado = true;
                }
                active_mtx_.unlock();

                pose_callback_mtx_.lock();
                pose_ready = bloquear_callback;
                pose_callback_mtx_.unlock();
                vel_callback_mtx_.lock();
                vel_ready = bloquear_vel;
                vel_callback_mtx_.unlock();
            }

            double t_total;
            auto result = std::make_shared<TrayAction::Result>();    // Creamos el objeto de nuestra interfaz que devolveremos al cliente al final del callback.
            if (!eliminado)
            {
                auto feedback_msg = std::make_shared<TrayAction::Feedback>();     // Creamos el objeto de nuestra interfaz que utilizaremos como feedback de la acción

                // Obtenemos los valores del cliente de la acción para utilizarlos en el bucle
                auto goal = goal_handle->get_goal();

                double tx = static_cast<double>(goal->tx);
                double ty = static_cast<double>(goal->ty);
                double tz = static_cast<double>(goal->tz);
                double tyaw = static_cast<double>(goal->tyaw);
                t_total = std::max({tx, ty, tz, tyaw});
                
                // Obtener Poses iniciales y finales
                pose_callback_mtx_.lock();
                double x0 = last_pose_.pose.position.x;
                double y0 = last_pose_.pose.position.y;
                double z0 = last_pose_.pose.position.z;
                double yaw0 = pose2yaw(last_pose_);
                pose_callback_mtx_.unlock();

                double xf   = goal->target_pose.pose.position.x;
                double yf   = goal->target_pose.pose.position.y;
                double zf   = goal->target_pose.pose.position.z;
                double yawf = pose2yaw(goal->target_pose);

                // Obtener Velocidades iniciales y finales
                vel_callback_mtx_.lock();
                double vx0 = last_vel_.twist.linear.x;
                double vy0 = last_vel_.twist.linear.y;
                double vz0 = last_vel_.twist.linear.z;
                double vyaw0 = last_vel_.twist.angular.z;
                vel_callback_mtx_.unlock();

                double vxf=0.0, vyf=0.0, vzf=0.0, vyawf=0.0;

                coef_cubic cx = calcular_coeficientes(x0, xf, vx0, vxf, tx);
                coef_cubic cy = calcular_coeficientes(y0, yf, vy0, vyf, ty);
                coef_cubic cz = calcular_coeficientes(z0, zf, vz0, vzf, tz);
                coef_cubic cyaw = calcular_coeficientes(yaw0, yawf, vyaw0, vyawf, tyaw);

                rclcpp::Rate rate(30.0);     // Delay para verlo más claro en el tópico feedback (junto a rate.sleep())

                auto t0 = this->get_clock()->now();
                RCLCPP_INFO(this->get_logger(), "t=%.6f", t0.seconds());

                while (rclcpp::ok() && !eliminado && t <= t_total) 
                {
                    active_mtx_.lock();
                    if (active_goal_ != goal_handle)
                    {
                        eliminado = true;
                    }
                    active_mtx_.unlock();

                    if (!goal_handle->is_canceling())
                    {
                        t = (this->get_clock()->now() - t0).seconds();

                        std_msgs::msg::Float64MultiArray msg_x = eval_cubic(cx, t);
                        std_msgs::msg::Float64MultiArray msg_y = eval_cubic(cy, t);
                        std_msgs::msg::Float64MultiArray msg_z = eval_cubic(cz, t);
                        std_msgs::msg::Float64MultiArray msg_yaw = eval_cubic(cyaw, t);

                        feedback_msg->t_act = static_cast<float>(t);
                        feedback_msg->x   = msg_x;
                        feedback_msg->y   = msg_y;
                        feedback_msg->z   = msg_z;
                        feedback_msg->yaw = msg_yaw;

                        goal_handle->publish_feedback(feedback_msg);    // Envía periódicamente los valores al tópico /feedback

                        rate.sleep();
                    }
                    else
                    {
                        cancelado = true;
                        eliminado = true;
                    }
                }
            }

            if (eliminado)
            {
                result->success = false;
                result->t_total = 0.0f;
                RCLCPP_INFO(this->get_logger(), "Acción CANCELADA");
                if (cancelado)
                {
                    goal_handle->canceled(result);
                }
            }
            else
            {
                result->success = true;
                result->t_total = static_cast<float>(t_total);
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Acción finalizada OK");
            }
        }

        rclcpp_action::Server<TrayAction>::SharedPtr objeto_servicio_accion;     // Creamos el objeto del servicio de la acción
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vel_;

        bool bloquear_callback = false;
        bool bloquear_vel = false;

        geometry_msgs::msg::PoseStamped last_pose_;
        geometry_msgs::msg::TwistStamped last_vel_;

        std::mutex active_mtx_;
        std::mutex pose_callback_mtx_;
        std::mutex vel_callback_mtx_;
        std::shared_ptr<GoalHandleTrayAction> active_goal_;  // goal que consideramos "activo"
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto objeto_nodo = std::make_shared<Clase_Servicio_Accion>();
    rclcpp::spin(objeto_nodo); 

    rclcpp::shutdown(); 
    return 0;
}