cmake_minimum_required(VERSION 3.8)
project(cv_red_center)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_msg REQUIRED)
find_package(OpenCV REQUIRED)

# 퍼블리셔
add_executable(red_center_publisher src/red_center_publisher.cpp)
ament_target_dependencies(red_center_publisher rclcpp cv_msg)
target_include_directories(red_center_publisher PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(red_center_publisher ${OpenCV_LIBS})

# 서브스크라이버
add_executable(red_center_subscriber src/red_center_subscriber.cpp)
ament_target_dependencies(red_center_subscriber rclcpp cv_msg)

install(TARGETS
  red_center_publisher
  red_center_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
