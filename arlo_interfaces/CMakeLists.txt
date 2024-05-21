cmake_minimum_required(VERSION 3.8)
project(arlo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PesosStruct.msg"
  "msg/EstadoArlo.msg"
  "srv/EvaluateDriver.srv"
  DEPENDENCIES sensor_msgs nav_msgs 
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
