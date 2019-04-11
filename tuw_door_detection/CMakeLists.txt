cmake_minimum_required(VERSION 2.8.3)
project(tuw_door_detection)
add_compile_options(-std=c++14) ## for c++ 2014


find_package(catkin REQUIRED COMPONENTS
        tf2_ros
        geometry_msgs
        roscpp
        rospy
        std_msgs
        sensor_msgs
        nav_msgs
        tuw_geometry
        tuw_geometry_msgs
        tuw_object_msgs
        tuw_measurement_utils
        dynamic_reconfigure
        cv_bridge
        image_geometry
        )

find_package(PCL 1.3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:

generate_dynamic_reconfigure_options(
        cfg/Door2DDetector.cfg
        cfg/DepthDetector.cfg
        #   cfg/DynReconf2.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES tuw_linedetection
        CATKIN_DEPENDS
        roscpp
        rospy
        tuw_geometry
        tuw_geometry_msgs
        tuw_object_msgs
        tf2_ros
        tf_conversions
        cv_bridge
        image_geometry
        tuw_measurement_utils
        nav_msgs

        DEPENDS
        OpenCV
        Eigen3
)

###########
## Build ##
###########

include_directories(
        ./include/
        ${catkin_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(door_detector_node
        src/door_detector_node.cpp
        src/laser_processor/door_detector_base.cpp
        src/laser_processor/door_line_detector.cpp
        src/laser_processor/door_depth_detector.cpp
        src/laser_processor/door_detection.cpp
        src/image_processor/door_detector_imgproc.cpp
        src/door_detector.cpp
        )

add_dependencies(door_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(door_detector_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Boost_LIBRARIES}
        ${EIGEN_LIBRARIES}
        )

#############
## Install ##
#############
