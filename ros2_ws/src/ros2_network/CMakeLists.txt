cmake_minimum_required(VERSION 3.8)
project(ros2_network)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)

set(msg_files
  "msg/CamJpeg.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(cam_publisher src/cam_publisher.cpp)
target_include_directories(cam_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  )
target_compile_features(cam_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  cam_publisher
  "rclcpp"
)
target_link_libraries(cam_publisher "${cpp_typesupport_target}")

add_executable(cam_subscriber_save src/cam_subscriber_save.cpp)
target_include_directories(cam_subscriber_save PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cam_subscriber_save PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  cam_subscriber_save
  "rclcpp"
)
target_link_libraries(cam_subscriber_save "${cpp_typesupport_target}" ${OpenCV_LIBRARIES})

ament_export_dependencies(rosidl_default_runtime)

install(TARGETS cam_publisher cam_subscriber_save
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
