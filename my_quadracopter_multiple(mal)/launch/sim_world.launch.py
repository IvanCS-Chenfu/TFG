from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
import os, yaml
from ament_index_python.packages import get_package_share_directory




def _load_cfg():
    # 1) Intenta primero el YAML de tu workspace (edición en caliente)
    share_dir = get_package_share_directory('my_quadracopter')
    src_cfg = os.path.expanduser('~/ros/Gazebo_ws/src/my_quadracopter/config/sim_settings.yaml')
    if os.path.exists(src_cfg):
        with open(src_cfg, 'r') as f:
            return yaml.safe_load(f) or {}, src_cfg, share_dir

    # 2) Si no está, usa el instalado en share/

    share_cfg = os.path.join(share_dir, 'config', 'sim_settings.yaml')
    if os.path.exists(share_cfg):
        with open(share_cfg, 'r') as f:
            return yaml.safe_load(f) or {}, share_cfg, share_dir

    # 3) Sin fichero: defaults
    return {}, '(defaults)'




def generate_launch_description():
    # Rutas (usa tu paquete; si no instalas el share, puedes dejar tu ruta HOME)
    cfg, cfg_path, pkg_home = _load_cfg()
    world_path = os.path.join(pkg_home, 'worlds', 'empty.world')
    params_path = os.path.join(pkg_home, 'config', 'plugins.yaml')

    # Defaults desde YAML (sobrescribibles por CLI)
    num_default = str(cfg.get('num_drones', 3))
    area_default  = str(cfg.get('area_size', 10.0))
    seed_default       = str(cfg.get('seed', 0))

    
    
    return LaunchDescription([
        
        DeclareLaunchArgument('num_drones', default_value=num_default),
        DeclareLaunchArgument('area_size',  default_value=area_default),
        DeclareLaunchArgument('seed',       default_value=seed_default),
        
        
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
            parameters=[{
                'num_drones': ParameterValue(LaunchConfiguration('num_drones'), value_type=int),
                'area_size': ParameterValue(LaunchConfiguration('area_size'),  value_type=float),
                'seed':      ParameterValue(LaunchConfiguration('seed'),       value_type=int),
                # opcional, si quieres reproducibilidad:
                # 'seed': 1234
            }],
            # namespace='drone',   # <- opcional; no afecta a los plugins
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
            parameters=[{
                'num_drones': ParameterValue(LaunchConfiguration('num_drones'), value_type=int),
                'ns_prefix': '/drone_',
                'pose_suffix': '/ground_truth/pose',
                'R_suffix': '/R',
            }],
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
            parameters=[{
                'num_drones': ParameterValue(LaunchConfiguration('num_drones'), value_type=int),
            }],
        ),
        
        Node(
            package='my_quadracopter',
            executable='test_wrench_tray_gt',
            name='test_wrench_tray_gt',
            output='screen',
            parameters=[{
                'num_drones': ParameterValue(LaunchConfiguration('num_drones'), value_type=int),
            }],
        ),
        
        Node(
            package='my_quadracopter',
            executable='test_traygui_launcher.py',
            parameters=[{
                'num_drones': ParameterValue(LaunchConfiguration('num_drones'), value_type=int),
            }],
        ),
    ])
