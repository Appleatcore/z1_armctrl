cmake_minimum_required(VERSION 3.0.2)
project(arm_controller)
# 全局强制定义 Boost 限制，解决 Pinocchio/Boost 冲突
add_definitions(-DBOOST_MPL_LIMIT_LIST_SIZE=30)
add_definitions(-DBOOST_MPL_LIMIT_VECTOR_SIZE=30)

# # Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# z1 robotic manipulator sdk
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_subdirectory(thirdparty/z1_sdk)
find_library(Z1SDK_LIB Z1_SDK_${CMAKE_SYSTEM_PROCESSOR} thirdparty/z1_sdk/lib)
set(Z1SDK_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/thirdparty/z1_sdk/include)

find_package(pinocchio REQUIRED)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  actionlib_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  arm_controller_srvs
  urdf
  # moveit
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  rviz_visual_tools
  moveit_visual_tools
  pcl_ros
  pcl_conversions
)

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################

# # To declare and build messages, services or actions from within this
# # package, follow these steps:
# # * Let MSG_DEP_SET be the set of packages whose message types you use in
# #   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# # * In the file package.xml:
# #   * add a build_depend tag for "message_generation"
# #   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
# #   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
# #     but can be declared for certainty nonetheless:
# #     * add a exec_depend tag for "message_runtime"
# # * In this file (CMakeLists.txt):
# #   * add "message_generation" and every package in MSG_DEP_SET to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * add "message_runtime" and every package in MSG_DEP_SET to
# #     catkin_package(CATKIN_DEPENDS ...)
# #   * uncomment the add_*_files sections below as needed
# #     and list every .msg/.srv/.action file to be processed
# #   * uncomment the generate_messages entry below
# #   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# # Generate messages in the 'msg' folder
# add_message_files(
# FILES
# Message1.msg
# Message2.msg
# )

# # Generate services in the 'srv' folder
# add_service_files(
# FILES
# BackToHome.srv
# CheckPoseInWorkspace.srv
# Plan.srv
# )

# # Generate actions in the 'action' folder
add_action_files(
  FILES
  Plan.action
)

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs actionlib_msgs
)

# ###############################################
# # Declare ROS dynamic reconfigure parameters ##
# ###############################################

# # To declare and build dynamic reconfigure parameters within this
# # package, follow these steps:
# # * In the file package.xml:
# #   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
# # * In this file (CMakeLists.txt):
# #   * add "dynamic_reconfigure" to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * uncomment the "generate_dynamic_reconfigure_options" section below
# #     and list every .cfg file to be processed

# # Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
# cfg/DynReconf1.cfg
# cfg/DynReconf2.cfg
# )

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include

  LIBRARIES ${PROJECT_NAME}

  CATKIN_DEPENDS
  message_runtime
  arm_controller_srvs
  moveit_core
  moveit_ros_planning_interface
  urdf
  DEPENDS EIGEN3
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

# 添加宏定义 (Pinocchio 可能需要)
add_definitions(${pinocchio_DEFINITIONS})

# # Declare a C++ library
add_library(${PROJECT_NAME}
  src/arm_api.cpp
  src/arm_controller.cpp
  src/arm_planner.cpp
  src/geometry_utils.cpp
  src/kinematics.cpp
  src/js_api.cpp
  src/pinocchio_ik.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${Z1SDK_INCLUDE_DIR})
# target_link_libraries(${PROJECT_NAME} PUBLIC ${Z1SDK_LIB} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC 
  ${Z1SDK_LIB} 
  ${catkin_LIBRARIES} # 包含了 urdf
  pinocchio::pinocchio
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

# # Add cmake target dependencies of the library
# # as an example, code may need to be generated before libraries
# # either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Declare a C++ executable
# # With catkin_make all packages are built within a single CMake context
# # The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME})

# Calculate node (standalone line calculation)
add_executable(calculate_node src/calculate_node.cpp)
target_link_libraries(calculate_node ${catkin_LIBRARIES})
add_dependencies(calculate_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# PoseParamUpdater node
add_executable(pose_param_updater_node src/pose_param_updater_node.cpp src/pose_param_updater.cpp)
target_include_directories(pose_param_updater_node PRIVATE include)
target_link_libraries(pose_param_updater_node PRIVATE ${catkin_LIBRARIES})
add_dependencies(pose_param_updater_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_subdirectory(src/test)

# add_executable(check_pose_in_workspace_server src/check_pose_in_workspace_server.cpp)
# target_link_libraries(check_pose_in_workspace_server PRIVATE ${PROJECT_NAME})
# add_dependencies(check_pose_in_workspace_server ${PROJECT_NAME}_gencpp)

# # Rename C++ executable without prefix
# # The above recommended prefix causes long target names, the following renames the
# # target back to the shorter version for ease of user use
# # e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# # Add cmake target dependencies of the executable
# # same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
# ${catkin_LIBRARIES}
# )

# ############
# # Install ##
# ############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
# scripts/my_python_script
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark executables for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark libraries for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# # Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# FILES_MATCHING PATTERN "*.h"
# PATTERN ".svn" EXCLUDE
# )

# # Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
# # myfile1
# # myfile2
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# ############
# # Testing ##
# ############

# # Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_arm_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# # Add folders to be run by python nosetests
# catkin_add_nosetests(test)
