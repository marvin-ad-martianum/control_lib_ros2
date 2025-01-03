cmake_minimum_required(VERSION 3.8)
project(control_lib_ros2)

# We assume C++20 (consistent with your library)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
# External dependencies
find_package(control_lib REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(jsoncpp REQUIRED)
find_package(yaml-cpp REQUIRED) 

#add_executable(pid_controller src/pid_controller.cpp)
#ament_target_dependencies(pid_controller std_msgs rclcpp yaml-cpp ament_index_cpp)

add_executable(model_lti src/model_lti.cpp)
ament_target_dependencies(model_lti std_msgs rclcpp yaml-cpp ament_index_cpp)

add_executable(pid_controller_generalized src/pid_controller_generalized.cpp)
ament_target_dependencies(pid_controller_generalized std_msgs rclcpp yaml-cpp ament_index_cpp)


#target_link_libraries(pid_controller
#  control_lib::control_lib
#  yaml-cpp      
#)
target_link_libraries(model_lti
  control_lib::control_lib
  yaml-cpp 
  Eigen3::Eigen     
)
target_link_libraries(pid_controller_generalized
  control_lib::control_lib
  ${FFTW_LIBRARIES}
  yaml-cpp      
)

target_include_directories(pid_controller_generalized
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ament_index_cpp_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    )


# Standard ROS 2 installation pattern
install(TARGETS 
  model_lti
  #pid_controller
  pid_controller_generalized
  DESTINATION lib/${PROJECT_NAME}
)

# Install config directory
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
