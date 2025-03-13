cmake_minimum_required(VERSION 3.5)
project(leader_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/real_image.py
  scripts/convert_real_image_to_fisheye.py
  scripts/convert_fisheye_to_image.py
  scripts/make_image_to_panorama.py
  scripts/make_image_to_panorama2.py
  scripts/tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

# Install Cpp executables
install(TARGETS
  cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  models
  meshes
  worlds
  launch
  scripts
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
