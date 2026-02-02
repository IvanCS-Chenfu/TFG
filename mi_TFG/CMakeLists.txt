cmake_minimum_required(VERSION 3.8)
project(mi_tfg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  urdf
  config

  DESTINATION share/${PROJECT_NAME}/
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TrayAction.action"

  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rclcpp_action)


add_library(plugin_actuar_motores SHARED src/plugins/plugin_actuar_motores.cpp)
ament_target_dependencies(plugin_actuar_motores gazebo_ros rclcpp std_msgs)
target_include_directories(plugin_actuar_motores PRIVATE include ${gazebo_ros_INCLUDE_DIRS})
target_link_libraries(plugin_actuar_motores ${gazebo_ros_LIBRARIES})

add_library(plugin_sensor_groundtrurh SHARED src/plugins/plugin_sensor_groundtrurh.cpp)
ament_target_dependencies(plugin_sensor_groundtrurh gazebo_ros rclcpp geometry_msgs)
target_include_directories(plugin_sensor_groundtrurh PRIVATE include ${gazebo_ros_INCLUDE_DIRS})
target_link_libraries(plugin_sensor_groundtrurh ${gazebo_ros_LIBRARIES})


install(TARGETS
  plugin_actuar_motores
  plugin_sensor_groundtrurh

  LIBRARY DESTINATION lib
)

add_executable(clock src/otros/clock.cpp)
ament_target_dependencies(clock rclcpp )

add_executable(generador_URDF src/generar_URDF/generador_URDF.cpp)
ament_target_dependencies(generador_URDF rclcpp gazebo_msgs ament_index_cpp)



add_executable(gen_tray_punto src/control_tray/gen_tray_punto.cpp)
ament_target_dependencies(gen_tray_punto rclcpp geometry_msgs ament_index_cpp std_msgs rclcpp_action)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gen_tray_punto "${cpp_typesupport_target}")

add_executable(gen_tray_veltrap src/control_tray/gen_tray_veltrap.cpp)
ament_target_dependencies(gen_tray_veltrap rclcpp geometry_msgs ament_index_cpp std_msgs rclcpp_action)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gen_tray_veltrap "${cpp_typesupport_target}")



add_executable(control_calcular_fuerzas src/control_tray/control_calcular_fuerzas.cpp)
ament_target_dependencies(control_calcular_fuerzas rclcpp geometry_msgs ament_index_cpp std_msgs rclcpp_action Eigen3)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(control_calcular_fuerzas "${cpp_typesupport_target}")

add_executable(aplicar_fuerzas_dron src/control_tray/aplicar_fuerzas_dron.cpp)
ament_target_dependencies(aplicar_fuerzas_dron rclcpp geometry_msgs std_msgs Eigen3)



add_executable(graficar_GT src/graficar/graficar_GT.cpp)
ament_target_dependencies(graficar_GT rclcpp geometry_msgs ament_index_cpp std_msgs)

add_executable(graficar_tray src/graficar/graficar_tray.cpp)
ament_target_dependencies(graficar_tray rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(graficar_tray "${cpp_typesupport_target}")

add_executable(graficar_GTvsTray src/graficar/graficar_GTvsTray.cpp)
ament_target_dependencies(graficar_GTvsTray rclcpp std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(graficar_GTvsTray "${cpp_typesupport_target}")




install(TARGETS
  clock
  generador_URDF
  gen_tray_punto
  gen_tray_veltrap
  control_calcular_fuerzas
  aplicar_fuerzas_dron
  graficar_GT
  graficar_tray
  graficar_GTvsTray

  DESTINATION lib/${PROJECT_NAME}
)


install(PROGRAMS
  src/graficar/graficar.py
  src/control_tray/gui_tray.py
  src/control_tray/gui_tray_multi.py

  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
