cmake_minimum_required(VERSION 3.8)
project(my_quadracopter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 🔍 Buscar dependencias necesarias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(xacro REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_runtime REQUIRED)



# 📦 Si hay tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 📂 Instalar carpetas de recursos
install(DIRECTORY
  launch urdf worlds config
  DESTINATION share/${PROJECT_NAME}/
)

# Interfaces (Action)
set(action_files
  "action/Poly3Traj.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ▶️ Nodo para insertar el robot en Gazebo
add_executable(sim_quadcopter src/sim_quadcopter.cpp)

ament_target_dependencies(sim_quadcopter
  rclcpp
  gazebo_msgs
  ament_index_cpp
)

target_link_libraries(sim_quadcopter yaml-cpp)

install(TARGETS sim_quadcopter
  DESTINATION lib/${PROJECT_NAME}
)

# 🧩 Plugin de Gazebo Classic
add_library(wrenches_plugin SHARED src/plugins/wrenches_plugin.cpp)

ament_target_dependencies(wrenches_plugin
  rclcpp
  std_msgs
  gazebo_ros
  ament_index_cpp
  geometry_msgs
)

target_include_directories(wrenches_plugin PRIVATE
  include
  ${gazebo_ros_INCLUDE_DIRS}
)

target_link_libraries(wrenches_plugin
  ${gazebo_ros_LIBRARIES}
  yaml-cpp
)




add_library(imu_plugin SHARED src/plugins/imu_plugin.cpp)

ament_target_dependencies(imu_plugin
  rclcpp
  sensor_msgs
  gazebo_ros
  ament_index_cpp
)

target_include_directories(imu_plugin PRIVATE
  include
)

target_link_libraries(imu_plugin
  ${gazebo_ros_LIBRARIES}
)



add_library(barometer_plugin SHARED src/plugins/barometer_plugin.cpp)

ament_target_dependencies(barometer_plugin
  rclcpp
  sensor_msgs
  gazebo_ros
  ament_index_cpp
)

target_include_directories(barometer_plugin PRIVATE
  include
)

target_link_libraries(barometer_plugin
  ${gazebo_ros_LIBRARIES}
)



add_library(magnetometer_plugin SHARED src/plugins/magnetometer_plugin.cpp)

ament_target_dependencies(magnetometer_plugin
  rclcpp
  sensor_msgs
  gazebo_ros
  ament_index_cpp
)

target_include_directories(magnetometer_plugin PRIVATE
  include
)

target_link_libraries(magnetometer_plugin
  ${gazebo_ros_LIBRARIES}
)



add_library(ground_truth_plugin SHARED src/plugins/ground_truth_plugin.cpp)

ament_target_dependencies(ground_truth_plugin
  rclcpp
  sensor_msgs
  gazebo_ros
  ament_index_cpp
)

target_include_directories(ground_truth_plugin PRIVATE
  include
)

target_link_libraries(ground_truth_plugin
  ${gazebo_ros_LIBRARIES}
)



install(TARGETS wrenches_plugin imu_plugin barometer_plugin magnetometer_plugin ground_truth_plugin
  DESTINATION lib
)



add_executable(prueba src/prueba.cpp)
ament_target_dependencies(prueba rclcpp std_msgs)
install(TARGETS prueba DESTINATION lib/${PROJECT_NAME})

add_executable(pwm2wrench src/pwm2wrench.cpp)
ament_target_dependencies(pwm2wrench rclcpp std_msgs geometry_msgs)
install(TARGETS pwm2wrench DESTINATION lib/${PROJECT_NAME})

add_executable(baro_mag_estimator src/baro_mag_estimator.cpp)
ament_target_dependencies(baro_mag_estimator rclcpp std_msgs geometry_msgs sensor_msgs)
install(TARGETS baro_mag_estimator DESTINATION lib/${PROJECT_NAME})

add_executable(pose_estimator_fusion src/pose_estimator_fusion.cpp)
ament_target_dependencies(pose_estimator_fusion rclcpp std_msgs geometry_msgs sensor_msgs)
install(TARGETS pose_estimator_fusion DESTINATION lib/${PROJECT_NAME})

add_executable(rotation_matrix_publisher src/rotation_matrix_publisher.cpp)
ament_target_dependencies(rotation_matrix_publisher rclcpp std_msgs geometry_msgs)
install(TARGETS rotation_matrix_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(test_angulos_imu src/tests/test_angulos_imu.cpp)
ament_target_dependencies(test_angulos_imu rclcpp std_msgs geometry_msgs sensor_msgs)
install(TARGETS test_angulos_imu DESTINATION lib/${PROJECT_NAME})

add_executable(test_torques_gt src/tests/test_torques_gt.cpp)
ament_target_dependencies(test_torques_gt rclcpp std_msgs geometry_msgs)
install(TARGETS test_torques_gt DESTINATION lib/${PROJECT_NAME})

add_executable(test_wrench_tray_gt src/tests/test_wrench_tray_gt.cpp)
ament_target_dependencies(test_wrench_tray_gt rclcpp std_msgs geometry_msgs)
target_link_libraries(test_wrench_tray_gt Eigen3::Eigen)
install(TARGETS test_wrench_tray_gt DESTINATION lib/${PROJECT_NAME})

add_executable(test_tray_launcher src/tests/test_tray_launcher.cpp)
ament_target_dependencies(test_tray_launcher rclcpp std_msgs geometry_msgs rclcpp_action)
install(TARGETS test_tray_launcher DESTINATION lib/${PROJECT_NAME})

target_include_directories(test_tray_launcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

add_executable(test_fuerzas_gt src/tests/test_fuerzas_gt.cpp)
ament_target_dependencies(test_fuerzas_gt rclcpp rclcpp_action geometry_msgs)
install(TARGETS test_fuerzas_gt DESTINATION lib/${PROJECT_NAME})

target_include_directories(test_fuerzas_gt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

add_executable(test_gen_tray src/tests/test_gen_tray.cpp)
ament_target_dependencies(test_gen_tray rclcpp rclcpp_action geometry_msgs)
install(TARGETS test_gen_tray DESTINATION lib/${PROJECT_NAME})

target_include_directories(test_gen_tray PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

add_executable(test_calcular_rd src/tests/test_calcular_rd.cpp)
ament_target_dependencies(test_calcular_rd rclcpp rclcpp_action geometry_msgs)
install(TARGETS test_calcular_rd DESTINATION lib/${PROJECT_NAME})

target_include_directories(test_calcular_rd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

# El ejecutable depende de que se generen las interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(test_fuerzas_gt  ${cpp_typesupport_target})
target_link_libraries(test_gen_tray      ${cpp_typesupport_target})
target_link_libraries(test_calcular_rd      ${cpp_typesupport_target})
target_link_libraries(test_tray_launcher      ${cpp_typesupport_target})

add_dependencies(test_tray_launcher test_gen_tray test_fuerzas_gt test_calcular_rd ${cpp_typesupport_target})

install(PROGRAMS src/tests/test_traygui_launcher.py DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)

ament_package()
