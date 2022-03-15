cmake_minimum_required(VERSION 3.0.2)
project(map_process)

set(
  LINK_LIBRARIES_PATH "/opt/ros/melodic/include")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  actionlib
  geometry_msgs
  move_base_msgs
  tf
  sensor_msgs
  message_generation
)

find_package(Bullet REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp QUIET)
if(NOT YAMLCPP_FOUND)
  find_package(yaml-cpp 0.6 REQUIRED)
  set(YAMLCPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
  set(YAMLCPP_LIBRARIES ${YAML_CPP_LIBRARIES})
  add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
else()
  if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
  endif()
  link_directories(${YAMLCPP_LIBRARY_DIRS})
endif()

add_message_files(FILES
  taregt_pose.msg
  task.msg
)

generate_messages(DEPENDENCIES
  # taregt_pose
  # std_msgs
  # geometry_msgs
)

catkin_package(
  # INCLUDE_DIRS include 
  CATKIN_DEPENDS
  message_runtime
  CATKIN_DEPENDS roscpp rospy roscpp nav_msgs tf2
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BULLET_INCLUDE_DIRS}
  ${SDL_INCLUDE_DIR}
  ${SDL_IMAGE_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/map_process_node.cpp src/map_server.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${BULLET_LIBRARIES}
  ${SDL_LIBRARY}
  ${SDL_IMAGE_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
)


