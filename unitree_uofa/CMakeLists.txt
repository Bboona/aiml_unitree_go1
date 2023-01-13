cmake_minimum_required(VERSION 2.8.3)
project(unitree_uofa)

add_compile_options(-std=c++11)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  unitree_legged_msgs
)

catkin_package()

# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

link_directories(lib/cpp/${ARCH})
link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/${ARCH})

set(EXTRA_LIBS -pthread libunitree_legged_sdk.so)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
)

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib)

catkin_package(
   CATKIN_DEPENDS roscpp unitree_legged_msgs message_runtime rospy
)

add_executable(joystick_listener src/joystick_listener.cpp)
target_link_libraries(joystick_listener ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(joystick_listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(green_ball src/green_ball.cpp)
target_link_libraries(green_ball ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(green_ball ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(movement src/movement.cpp)
target_link_libraries(movement ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(movement ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# install
install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY lib/cpp/${ARCH}/
  DESTINATION lib/unitree
  USE_SOURCE_PERMISSIONS)
