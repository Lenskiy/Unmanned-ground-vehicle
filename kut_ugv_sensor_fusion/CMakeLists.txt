cmake_minimum_required(VERSION 2.8.3)
project(kut_ugv_sensor_fusion)

#
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge kut_ugv_msgs image_transport image_geometry laser_geometry message_filters pcl_ros roscpp sensor_msgs std_msgs message_generation tf)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	lidar_segmentation_labels.msg
	lidar_object_list.msg
	lidar_object.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs 
   std_msgs#   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES kut_ugv_sensor_fusion
  CATKIN_DEPENDS kut_ugv_msgs cv_bridge image_transport laser_geometry message_filters pcl pcl_ros roscpp sensor_msgs std_msgs message_runtime geometry_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(sensor_fusion
 src/SensorFusion.cpp
)

add_library(ImProcessingTools
 src/ImProcessingTools.cpp
)

## Declare a cpp library
add_library(cvplot
 src/cvplot.cpp
)

## Declare a cpp executable
add_executable(sensor_fusion_node src/sensor_fusion_node.cpp)
add_executable(calibration_features_extraction_node src/calibration_features_extraction_node.cpp)
add_executable(data_recorder_node src/data_recorder_node.cpp)
add_executable(lidar_segmentation_node src/lidar_segmentation_node.cpp)
add_executable(pedestrian_detector_node src/pedestrian_detector_node.cpp)
add_executable(moving_object_detector_node  src/moving_object_detector_node.cpp)
add_executable(traffic_light_detector_node  src/traffic_light_detector_node.cpp)
add_executable(chess_calibration_data_colection_node  src/chess_calibration_data_colection_node.cpp)
add_executable(laser_intensities_node  src/laser_intensities_node.cpp)
add_executable(traffic_sign_detector_node  src/traffic_sign_detector_node.cpp src/ColorSignDetector.cpp src/BlobLabeling.cpp src/ihls.cpp src/math_utils.cpp src/nhs.cpp src/PostProcessing.cpp src/RotationalOffset.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(kut_ugv_sensor_fusion_node kut_ugv_sensor_fusion_generate_messages_cpp)

add_dependencies(sensor_fusion ${kut_ugv_sensor_fusion_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(sensor_fusion ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(cvplot ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(ImProcessingTools ${catkin_LIBRARIES} ${OpenCV_LIBS})

target_link_libraries(sensor_fusion_node ${catkin_LIBRARIES} ${OpenCV_LIBS} sensor_fusion)
target_link_libraries(calibration_features_extraction_node sensor_fusion)
target_link_libraries(data_recorder_node sensor_fusion)
target_link_libraries(lidar_segmentation_node ${catkin_LIBRARIES} sensor_fusion)
target_link_libraries(pedestrian_detector_node ${catkin_LIBRARIES} sensor_fusion)
target_link_libraries(moving_object_detector_node ${catkin_LIBRARIES} sensor_fusion cvplot)

target_link_libraries(laser_intensities_node ${catkin_LIBRARIES} cvplot)
target_link_libraries(traffic_light_detector_node ${catkin_LIBRARIES} sensor_fusion ImProcessingTools)
target_link_libraries(chess_calibration_data_colection_node ${catkin_LIBRARIES})
target_link_libraries(traffic_sign_detector_node ${catkin_LIBRARIES} ImProcessingTools)





#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS kut_ugv_sensor_fusion kut_ugv_sensor_fusion_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kut_ugv_sensor_fusion.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
