from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

import yaml
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ## Obtener parámetros para el launch
    params_tray_path = os.path.join  (
                                    get_package_share_directory('mi_tfg'), 'config', 'tray_dron.yaml'
                                    )

    with open(params_tray_path, 'r') as f:
        params_tray_get = yaml.safe_load(f) or {}

    params_tray_get = params_tray_get.get('/**',{}).get('ros__parameters', {})        # debido a que los parámetros se encuentran en "/**: ros__parameters: "
    usar_veltrap = str(params_tray_get.get('control.tray.usar_veltrap', 'false'))
    
    
    ## usar parámetros en "LaunchDescription([])"
    usar_veltrap_arg = DeclareLaunchArgument(
                                                'usar_veltrap', default_value=usar_veltrap
                                            )
    usar_veltrap = LaunchConfiguration('usar_veltrap')
    
    start_gazebo_arg = DeclareLaunchArgument(
                                                'start_gazebo', default_value='true'    ## Si el launch "multi_dron" llama a este launch
                                            )
    start_gazebo = LaunchConfiguration('start_gazebo')
    
    
    
    
    ## Parámetros para los nodos
    params_xacro = PathJoinSubstitution  ([
                                            FindPackageShare('mi_tfg'), 'config', 'xacro_dron.yaml'
                                        ])
    params_tray = PathJoinSubstitution  ([
                                            FindPackageShare('mi_tfg'), 'config', 'tray_dron.yaml'
                                        ])
    params_sim = PathJoinSubstitution  ([
                                            FindPackageShare('mi_tfg'), 'config', 'sim_dron.yaml'
                                        ])

    
   
    
    
    
    
    return LaunchDescription([
        start_gazebo_arg,
        usar_veltrap_arg,
        
        # Lanzar Gazebo con el mundo
        ExecuteProcess(
            condition=IfCondition(start_gazebo),        ## No se ejecuta si "multi_dron" llama a este launch.
            cmd=[
                'gazebo',
                '--verbose',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),
        
        # Lanzar el nodo que inserta el robot
        Node(
            package='mi_tfg',
            executable='generador_URDF',
            name='generador_URDF',
            output='screen',
            parameters=[
                        params_xacro,
                        params_sim
                        ]
        ),
        
        # Nodo reloj (todos los que necesiten tiempo se suscribirán a /clock)
        Node(
            condition=IfCondition(start_gazebo),        ## No se ejecuta si "multi_dron" llama a este launch
            package='mi_tfg',
            executable='clock',
            name='clock',
        ),
        
        # Nodo acción trayectoria           (gen_tray_veltrap y gen_tray_punto)
        Node(
            condition=UnlessCondition(usar_veltrap),
            package='mi_tfg',
            executable='gen_tray_punto',
            name='gen_tray_punto',
            #output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            condition=IfCondition(usar_veltrap),
            package='mi_tfg',
            executable='gen_tray_veltrap',
            name='gen_tray_veltrap',
            #output='screen',
            parameters=[
                        {'use_sim_time': True},
                        params_tray
                        ]
        ),
        
        # Nodo calculador de fuerzas y torques (para trayectoria)
        Node(
            package='mi_tfg',
            executable='control_calcular_fuerzas',
            name='control_calcular_fuerzas',
            #output='screen',
            parameters=[params_tray]
        ),
        
        # Nodo aplicador de fuerzas a los motores del dron (para seguir la trayectoria)
        Node(
            package='mi_tfg',
            executable='aplicar_fuerzas_dron',
            name='aplicar_fuerzas_dron',
            #output='screen',
            parameters=[params_tray]
        ),
    ])