cmake_minimum_required(VERSION 3.8)
project(driver_monitoring)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# -------------------------------
# cal_lane 노드
# -------------------------------
add_executable(cal_lane src/cal_lane.cpp)
ament_target_dependencies(cal_lane rclcpp std_msgs sensor_msgs cv_bridge OpenCV)
target_include_directories(cal_lane PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cal_lane ${OpenCV_LIBS})

# -------------------------------
# lcd 노드
# -------------------------------
add_executable(lcd src/lcd.cpp)
ament_target_dependencies(lcd rclcpp std_msgs)
target_link_libraries(lcd wiringPi)

# -------------------------------
# motor 노드
# -------------------------------
add_executable(motor src/motor.cpp)
ament_target_dependencies(motor rclcpp std_msgs geometry_msgs)

# -------------------------------
# receive_threat 노드
# -------------------------------
add_executable(receive_threat src/receive_threat.cpp)
ament_target_dependencies(receive_threat rclcpp std_msgs)

# -------------------------------
add_executable(KTH_motor src/KTH_motor.cpp)
ament_target_dependencies(KTH_motor rclcpp geometry_msgs std_msgs)

add_executable(motor2 src/motor2.cpp)
ament_target_dependencies(motor2 rclcpp geometry_msgs std_msgs)
# install
# -------------------------------
install(TARGETS
  cal_lane
  lcd
  motor
  receive_threat
  KTH_motor
  motor2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
