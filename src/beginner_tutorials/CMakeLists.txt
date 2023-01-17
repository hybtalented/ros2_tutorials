cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
include(GenerateExportHeader)

# 添加运行时的依赖
ament_export_dependencies(rosidl_default_runtime)
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
set(ros_msgs msg/Num.msg)
set(ros_srvs srv/AddTwoInts.srv)
set(ros_actions action/Fibonacci.action)
rosidl_generate_interfaces(${PROJECT_NAME} ${ros_msgs} ${ros_srvs} ${ros_actions})
# 引用同一个包内的接口 - 方式二
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
add_executable(talker  src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener  src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp)
# 引用同一个包内的接口 - 方式二
target_link_libraries(add_two_ints_client ${cpp_typesupport_target})

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server 
  rclcpp 
)
target_link_libraries(add_two_ints_server ${cpp_typesupport_target})
# 自动生存
generate_export_header(${cpp_typesupport_target}
    BASE_NAME ${PROJECT_NAME}
    EXPORT_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/visibility_control.h
)

add_library(action_server SHARED src/action_server.cpp)
ament_target_dependencies(action_server rclcpp rclcpp_action rclcpp_components)
# 添加 ROSIDL_TYPESUPPORT_CPP_BUILDING_DLL 宏控制动态库接口可见性
target_compile_definitions(action_server PRIVATE ROSIDL_TYPESUPPORT_CPP_BUILDING_DLL)
# 添加本包内的相关接口引用
target_link_libraries(action_server ${cpp_typesupport_target})
target_include_directories(action_server PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> 
  $<INSTALL_INTERFACE:include>
)
# 注册组件方式的节点，PLUGIN 后指定节点的名词， 如果还指定了 EXECUTABLE，同时将会生成一个可执行程序，
# 从而节点也可以通过 ros2 run 的方式启动
rclcpp_components_register_node(action_server PLUGIN FibonacciActionServer EXECUTABLE fibonacci_action_server)

add_library(action_client SHARED src/action_client.cpp)
ament_target_dependencies(action_client rclcpp rclcpp_action rclcpp_components)
target_include_directories(action_client PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> 
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(action_client PRIVATE ROSIDL_TYPESUPPORT_CPP_BUILDING_DLL)
target_link_libraries(action_client ${cpp_typesupport_target})
# 向ROS注册组件
rclcpp_components_register_node(action_client 
  PLUGIN FibonacciActionClient 
  # 创建对应的可执行文件，可以通过 ros2 run 运行
  EXECUTABLE fibonacci_action_client)

install(TARGETS 
  talker listener add_two_ints_server add_two_ints_client
  action_server action_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}
  DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
