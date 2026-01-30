from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

import yaml

def generate_launch_description():
    pkg = FindPackageShare('my_quadracopter')
    world_path  = PathJoinSubstitution([pkg, 'worlds', 'parking_garage.world'])
    cfg_default = PathJoinSubstitution([pkg, 'config', 'sim.yaml'])

    cfg_arg = DeclareLaunchArgument(
        'config_file',
        default_value=cfg_default,
        description='Ruta a sim.yaml con number_of_drones y area_size'
    )

    models_dir = PathJoinSubstitution([pkg, 'models'])

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[models_dir, ':', EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')]
    )
    
    disable_fuel = SetEnvironmentVariable(
    name='GAZEBO_MODEL_DATABASE_URI',
    value=''  # evita que intente ir a models.gazebosim.org si no encuentra algo
    )   
    
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen'
    )

    def _build(context):
        cfg_path = LaunchConfiguration('config_file').perform(context)

        # Leer YAML con valores por defecto si no existe
        N = 1
        area = [10.0, 10.0]
        z = 0.5
        use_random = True
        try:
            with open(cfg_path, 'r') as f:
                data = yaml.safe_load(f) or {}
                N = int(data.get('number_of_drones', N))
                area = list(data.get('area_size', area))
                z = float(data.get('z', z))
                use_random = bool(data.get('use_random_spawn', use_random))
        except Exception as e:
            print(f"[WARN] No se pudo leer '{cfg_path}': {e}. Usando valores por defecto.")

        actions = [gazebo]

            
        actions.append(
            Node(
                package='my_quadracopter',
                executable='test_traygui_launcher.py',
                name='test_traygui_launcher',
                output='screen',
                parameters=[
                    {'number_of_drones': N},
                    {'config_file': cfg_path},   # opcional; tu GUI lo puede usar como fallback
                ],
            )
        )
        
        
        for i in range(N):
            ns = f"drone_{i}"
            model = f"my_quadcopter_{i}"

            per_drone_nodes = [
                # Spawner: genera URDF desde XACRO con ns y lo envía a /spawn_entity
                Node(
                    package='my_quadracopter',
                    executable='sim_quadcopter',
                    name='spawn_quadcopter_node',
                    output='screen',
                    parameters=[{
                        'robot_name': model,
                        'robot_namespace': ns,   # ¡sin barra para xacro!
                        'area_size': area,
                        'z': z,
                        'use_random_spawn': use_random,
                        'random_seed': i,
                    }],
                ),

                # Resto de nodos de tu paquete, uno por dron
                Node(
                    package='my_quadracopter',
                    executable='baro_mag_estimator',
                    name='baro_mag_estimator',
                    output='screen',
                    parameters=[{'drone_id': i}],
                ),
                Node(
                    package='my_quadracopter',
                    executable='pose_estimator_fusion',
                    name='pose_estimator_fusion',
                    output='screen',
                    parameters=[{'drone_id': i}],
                ),
                Node(
                    package='my_quadracopter',
                    executable='rotation_matrix_publisher',
                    name='rotation_matrix_publisher',
                    output='screen',
                    parameters=[{'drone_id': i}],
                ),
                Node(
                    package='my_quadracopter',
                    executable='test_gen_tray',
                    name='test_gen_tray',
                    output='screen',
                    parameters=[{'drone_id': i}],
                ),
                Node(
                    package='my_quadracopter',
                    executable='test_fuerzas_gt',
                    name='test_fuerzas_gt',
                    output='screen',
                    parameters=[{'drone_id': i}],
                ),
                Node(
                    package='my_quadracopter',
                    executable='test_calcular_rd',
                    name='test_calcular_rd',
                    output='screen',
                    parameters=[{'drone_id': i}],
                ),
                Node(
                    package='my_quadracopter',
                    executable='test_torques_gt',
                    name='test_torques_gt',
                    output='screen',
                    parameters=[{'drone_id': i}],
                ),
                Node(
                    package='my_quadracopter',
                    executable='test_wrench_tray_gt',
                    name='test_wrench_tray_gt',
                    output='screen',
                    parameters=[{'drone_id': i}],
                ),
            ]

            actions.append(
                GroupAction([
                    PushRosNamespace(ns),   # todo lo de este dron queda bajo /drone_i/...
                    *per_drone_nodes
                ])
            )
        return actions

    return LaunchDescription([
        set_model_path,
        disable_fuel,
        cfg_arg,
        OpaqueFunction(function=_build),
    ])
