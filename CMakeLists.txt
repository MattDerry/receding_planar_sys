cmake_minimum_required(VERSION 2.8.3)
project(receding_planar_sys)

find_package(catkin REQUIRED COMPONENTS
  angles
  camera_calibration_parsers
  cmake_modules
  cv_bridge
  geometry_msgs
  image_transport
  interactive_markers
  message_filters
  nav_msgs
  puppeteer_msgs
  roscpp
  roslib
  rosparam
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf_conversions
  visualization_msgs
)

find_package(Eigen REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals system)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package( CATKIN_DEPENDS camera_calibration_parsers cv_bridge geometry_msgs image_transport interactive_markers message_filters nav_msgs puppeteer_msgs roslib rosparam rospy sensor_msgs std_msgs std_srvs tf tf_conversions visualization_msgs)
include_directories(${EIGEN_INCLUDE_DIRS})

include_directories(
 include ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
)

add_executable(receding_coordinator src/receding_coordinator.cpp)
add_executable(video_recorder src/video_recorder.cpp)
add_executable(interactive_marker_ref_gen src/interactive_marker_ref_gen.cpp)

add_dependencies(receding_coordinator ${catkin_EXPORTED_TARGETS})
add_dependencies(video_recorder ${catkin_EXPORTED_TARGETS})
add_dependencies(interactive_marker_ref_gen ${catkin_EXPORTED_TARGETS})

target_link_libraries(receding_coordinator ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(video_recorder ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(interactive_marker_ref_gen ${catkin_LIBRARIES}) 
install(TARGETS receding_coordinator video_recorder interactive_marker_ref_gen RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
find_library(LOG4CXX_LIBRARY log4cxx)
catkin_install_python(PROGRAMS produce_optimization_data_stats.py src/dynamic_crane_task.py src/marker_path.py src/crane_task_definition.py src/emergency_stopper_joy.py src/maze_crane_task.py src/receding_controller.py src/record_data.py src/joystick_marker_ref_gen.py src/metric_eval_task.py src/target_factory.py src/task_fsm.py src/interactive_marker_ref_gen.py src/obstacle_factory.py src/recalc_error.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})