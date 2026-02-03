from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch_ros.actions import PushRosNamespace, Node

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import yaml
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    
    ## No se utiliza "PathJoinSubstitution" ya que eso solo es útil para meter parámetros a nodos dentro del launch.
    
    ## Obtener numero de drones
    params_sim_path = os.path.join  (
                                    get_package_share_directory('mi_tfg'), 'config', 'sim_dron.yaml'
                                    )

    with open(params_sim_path, 'r') as f:
        params_sim = yaml.safe_load(f) or {}

    params_sim = params_sim.get('/**',{}).get('ros__parameters', {})        # debido a que los parámetros se encuentran en "/**: ros__parameters: "
    n = int(params_sim.get('dron.numero', 1)) 
    namespace_base = str(params_sim.get('dron.namespace_base', 'drone'))
    
    
    ## Obtener path del launch
    single_launch_path = os.path.join   (
                                        get_package_share_directory('mi_tfg'), 'launch', 'generar_dron.launch.py'
                                        )
    
    ## Obtener path del world
    world_path = PathJoinSubstitution  ([
                                            FindPackageShare('mi_tfg'), 'worlds', 'house_1.world'
                                        ])
    
    ## Abrir gazebo y llamar al launch "n" veces
    ld = LaunchDescription()
    
    ld.add_action(
                    ExecuteProcess
                    (
                        cmd=[
                            'gazebo',
                            '--verbose', world_path,
                            '-s', 'libgazebo_ros_factory.so'],
                        output='screen'
                    )
                )
    ld.add_action(
                    Node(
                            package='mi_tfg',
                            executable='clock',
                            name='clock',
                        )
                )
    
    
    
    for i in range(n):
        ld.add_action(
                        GroupAction
                        ([
                            PushRosNamespace(f'{namespace_base}_{i}'),
                            IncludeLaunchDescription(
                                                        PythonLaunchDescriptionSource(single_launch_path),
                                                        launch_arguments={'start_gazebo': 'false'}.items()      ## Enviar al launch "generar_dron" para que no abra gazebo n veces
                                                    ),
                        ])
                    )
        
    return ld