cmake_minimum_required(VERSION 3.5)
project(cpp_client)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fp_core_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

#add_subdirectory(include/cpp_client)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# add_library(${PROJECT_NAME} src/Sensor.cpp)
# target_include_directories(${PROJECT_NAME}
#     PUBLIC
#         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#         $<INSTALL_INTERFACE:include>
#     PRIVATE
# )

# target_link_libraries(${PROJECT_NAME}
#     PUBLIC 

#     PRIVATE 
# )


add_executable(client src/my_client.cpp)
ament_target_dependencies(client rclcpp fp_core_msgs)

add_executable(client_semplice src/my_simpleClient.cpp)
ament_target_dependencies(client_semplice rclcpp fp_core_msgs)

add_executable(client_v3 src/my_client_v3.cpp)
ament_target_dependencies(client_v3 rclcpp fp_core_msgs)

add_executable(client_v4 src/my_client_v4.cpp)
ament_target_dependencies(client_v4 rclcpp fp_core_msgs)

add_executable(sub_v3 src/subscriber_sensors.cpp)
ament_target_dependencies(sub_v3 rclcpp fp_core_msgs)

add_executable(client_mix src/client_mix.cpp)
ament_target_dependencies(client_mix rclcpp fp_core_msgs)

add_executable(line_follower_cpp src/tiago_line_follower.cpp)
ament_target_dependencies(line_follower_cpp rclcpp fp_core_msgs geometry_msgs sensor_msgs)

add_executable(line_follower_cpp_luce src/tiago_line_follower_luce.cpp)
ament_target_dependencies(line_follower_cpp_luce rclcpp fp_core_msgs geometry_msgs sensor_msgs)


add_executable(line_follower_cpp_olio src/tiago_line_follower_olio.cpp)
ament_target_dependencies(line_follower_cpp_olio rclcpp fp_core_msgs geometry_msgs sensor_msgs)


add_executable(tiago_line_client src/tiago_line_client.cpp)
ament_target_dependencies(tiago_line_client  rclcpp fp_core_msgs geometry_msgs sensor_msgs)


add_executable(tiago_line_completo src/tiago_line_completo.cpp)
ament_target_dependencies(tiago_line_completo  rclcpp fp_core_msgs geometry_msgs sensor_msgs)


add_executable(client_v5 src/my_client_v5.cpp)
ament_target_dependencies(client_v5 rclcpp fp_core_msgs)

add_executable(client_porta src/client_porta.cpp)
ament_target_dependencies(client_porta rclcpp fp_core_msgs)


add_executable(client_illuminazione src/client_illuminazione.cpp)
ament_target_dependencies(client_illuminazione rclcpp fp_core_msgs)


add_executable(line_finale src/line_follower_finale.cpp)
ament_target_dependencies(line_finale rclcpp fp_core_msgs geometry_msgs sensor_msgs)



install(TARGETS
  client
  client_semplice
  client_v3
  client_v4
  sub_v3
  client_mix
  line_follower_cpp
  line_follower_cpp_luce
  line_follower_cpp_olio
  tiago_line_client
  tiago_line_completo
  client_v5
  client_porta
  client_illuminazione
  line_finale
  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
