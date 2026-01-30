from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    
    params_file = PathJoinSubstitution  ([
                                            FindPackageShare('mi_TFG'), 'config', 'xacro_dron.yaml'
                                        ])

    
    return LaunchDescription([

        # Lanzar Gazebo con el mundo
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),
        
        # Lanzar el nodo que inserta el robot
        Node(
            package='mi_TFG',
            executable='generador_URDF',
            name='generador_URDF',
            output='screen',
            parameters=[params_file]
        )
    ])