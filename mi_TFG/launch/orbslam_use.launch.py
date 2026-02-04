# mi_paquete/launch/orbslam3_mono_stereo.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition, UnlessCondition

import yaml
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    ## Obtener parámetros para el launch (sim)
    params_sim_path = os.path.join(get_package_share_directory('mi_tfg'), 'config', 'sim_dron.yaml')

    with open(params_sim_path, 'r') as f:
        params_sim = yaml.safe_load(f) or {}

    params_sim = params_sim.get('/**',{}).get('ros__parameters', {})
    usar_estereo_default = str(params_sim.get('orbslam.estereo', 'true'))
    
    
    
    ## Parámetros para pasar a los nodos
    default_vocab = PathJoinSubstitution([FindPackageShare('mi_tfg'), 'config', 'orbslam', 'vocabulary', 'ORBvoc.txt'])
    default_yaml_mono = PathJoinSubstitution([FindPackageShare('mi_tfg'), 'config', 'orbslam', 'orbslam_mono.yaml'])
    default_yaml_stereo = PathJoinSubstitution([FindPackageShare('mi_tfg'), 'config', 'orbslam', 'orbslam_stereo.yaml'])

    vocab_arg = DeclareLaunchArgument('vocab', default_value=default_vocab)
    yaml_mono_arg = DeclareLaunchArgument('yaml_mono', default_value=default_yaml_mono)
    yaml_stereo_arg = DeclareLaunchArgument('yaml_stereo', default_value=default_yaml_stereo)
    rectify_arg = DeclareLaunchArgument('rectify', default_value='false')
    ## Parámetros para usar en launch
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    usar_estereo_arg = DeclareLaunchArgument('usar_estereo', default_value=usar_estereo_default)

    vocab = LaunchConfiguration('vocab')
    yaml_mono = LaunchConfiguration('yaml_mono')
    yaml_stereo = LaunchConfiguration('yaml_stereo')
    rectify = LaunchConfiguration('rectify')  # "true" / "false"
    ## Parámetros para usar en launch
    use_sim_time = LaunchConfiguration('use_sim_time')
    usar_estereo = LaunchConfiguration('usar_estereo')
    
   
   
   ## Nodos
    mono_node = Node(
                    condition=UnlessCondition(usar_estereo), 
                    package='orbslam3',
                    executable='mono',
                    name='orbslam3_mono',
                    output='screen',
                    arguments=[vocab, yaml_mono],
                    parameters=[{'use_sim_time': use_sim_time}],
                    remappings=[
                                ('camera', 'sensor/camara_mono/image_raw')
                                ],
                    )

    stereo_node = Node(
                    condition=IfCondition(usar_estereo), 
                    package='orbslam3',
                    executable='stereo',
                    name='orbslam3_stereo',
                    output='screen',
                    arguments=[vocab, yaml_stereo, rectify],
                    parameters=[{'use_sim_time': use_sim_time}],
                    remappings=[
                                ('camera/left',  'sensor/camara_izq/image_raw'),
                                ('camera/right', 'sensor/camara_der/image_raw')
                                ],
                    )

    
    
    ## Insertar Launch (argumentos y nodos)
    return LaunchDescription([
        vocab_arg,
        yaml_mono_arg,
        yaml_stereo_arg,
        rectify_arg,
        use_sim_time_arg,
        usar_estereo_arg,

        mono_node,
        stereo_node,
    ])
