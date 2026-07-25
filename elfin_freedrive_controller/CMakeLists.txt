cmake_minimum_required(VERSION 3.0.2)
project(elfin_freedrive_controller)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  controller_manager_msgs
  controller_interface
  elfin_robot_msgs
  hardware_interface
  kdl_parser
  message_generation
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  urdf
)

find_package(orocos_kdl REQUIRED)

add_message_files(
  FILES
  FreedriveTelemetry.msg
)

add_service_files(
  FILES
  SetDampingScales.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS controller_interface controller_manager_msgs elfin_robot_msgs hardware_interface kdl_parser message_runtime pluginlib realtime_tools roscpp sensor_msgs std_msgs std_srvs urdf
  DEPENDS orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/elfin_freedrive_controller.cpp
)

add_executable(elfin_freedrive_manager
  src/elfin_freedrive_manager.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

target_link_libraries(elfin_freedrive_manager
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(elfin_freedrive_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} elfin_freedrive_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES elfin_freedrive_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config launch test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)

catkin_install_python(PROGRAMS
  script/collect_elfin_gravity_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_math_test test/test_freedrive_math.cpp)
  if(TARGET ${PROJECT_NAME}_math_test)
    target_include_directories(${PROJECT_NAME}_math_test PRIVATE include)
    target_link_libraries(${PROJECT_NAME}_math_test ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(${PROJECT_NAME}_button_test test/test_tool_button_logic.cpp)
  if(TARGET ${PROJECT_NAME}_button_test)
    target_include_directories(${PROJECT_NAME}_button_test PRIVATE include)
    target_link_libraries(${PROJECT_NAME}_button_test ${catkin_LIBRARIES})
  endif()
endif()
