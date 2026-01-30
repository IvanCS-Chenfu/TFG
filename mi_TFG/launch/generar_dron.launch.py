from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    
    pkg = get_package_share_directory('mi_TFG')
    xacro_file = os.path.join(pkg, 'urdf', 'dron.xacro')
    
    params_file = PathJoinSubstitution  ([
                                            FindPackageShare('mi_TFG'), 'config', 'xacro_dron.yaml'
                                        ])

    robot_description = Command([
                                    'xacro ', xacro_file
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
        
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                # opcional si tu URDF usa "world" en gazebo:
                # 'frame_prefix': 'model/'
            }]
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