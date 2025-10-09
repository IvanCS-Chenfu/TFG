from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Rutas (usa tu paquete; si no instalas el share, puedes dejar tu ruta HOME)
    pkg_home = os.path.join(os.getenv('HOME'), 'ros', 'Gazebo_ws', 'src', 'my_quadracopter')
    world_path = os.path.join(pkg_home, 'worlds', 'empty.world')
    params_path = os.path.join(pkg_home, 'config', 'plugins.yaml')

    return LaunchDescription([
        # Lanzar Gazebo (servidor) con params ROS 2
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                world_path,
                '-s', 'libgazebo_ros_factory.so',
                '--ros-args', '--params-file', params_path
            ],
            output='screen'
        ),

        # Nodo que inserta el robot (si lo necesitas en /drone, puedes añadir namespace aquí también)
        Node(
            package='my_quadracopter',
            executable='sim_quadcopter',
            name='spawn_quadcopter_node',
            output='screen',
            # namespace='drone',   # <- opcional; no afecta a los plugins
            # parameters=[params_path],  # <- normalmente no hace falta para este nodo
        ),
        
        # Nodo estimador baro+mag (C++) — pásale también el YAML
        Node(
            package='my_quadracopter',
            executable='baro_mag_estimator',
            name='baro_mag_estimator',
            output='screen',
            parameters=[params_path],
            # Si quieres otros nombres de tópicos, puedes añadir 'remappings=[...]'
        ),
        
        Node(
            package='my_quadracopter',
            executable='pose_estimator_fusion',
            name='pose_estimator_fusion',
            output='screen',
            parameters=[params_path],  # reutiliza tu plugins.yaml si metes aquí los params del nodo
        ),
        
        Node(
            package='my_quadracopter',
            executable='rotation_matrix_publisher',
            name='rotation_matrix_publisher',
            output='screen',
        ),
        
        Node(
            package='my_quadracopter',
            executable='test_gen_tray',
            name='test_gen_tray',
            output='screen',
        ),
        
        Node(
            package='my_quadracopter',
            executable='test_fuerzas_gt',
            name='test_fuerzas_gt',
            output='screen',
        ),
        
        Node(
            package='my_quadracopter',
            executable='test_calcular_rd',
            name='test_calcular_rd',
            output='screen',
        ),
        
        Node(
            package='my_quadracopter',
            executable='test_torques_gt',
            name='test_torques_gt',
            output='screen',
        ),
        
        Node(
            package='my_quadracopter',
            executable='test_wrench_tray_gt',
            name='test_wrench_tray_gt',
            output='screen',
        ),
    ])
