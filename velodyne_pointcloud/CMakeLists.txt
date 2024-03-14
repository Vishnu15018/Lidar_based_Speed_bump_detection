cmake_minimum_required(VERSION 2.8.3)
project(velodyne_pointcloud)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fopenmp")
# Set minimum C++ standard to C++11
if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_STANDARD 14)
endif()

set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    nodelet
    roscpp
    roslib
    sensor_msgs
    tf
    velodyne_driver
    velodyne_msgs
    dynamic_reconfigure
    diagnostic_updater
    pcl_ros
    geometry_msgs
    pcl_msgs
    message_generation
)

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS}
             roslint)

find_package(Boost COMPONENTS signals)
find_package(Eigen3 REQUIRED)
# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.


find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)


find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

generate_dynamic_reconfigure_options(
  cfg/CloudNode.cfg cfg/TransformNode.cfg
)

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

include_directories(include ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${EIGEN3_INCLUDE_DIR}
)

add_message_files(
   FILES
   stamped_steering.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)
catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    INCLUDE_DIRS include
    LIBRARIES velodyne_rawdata)

    
#add_executable(dynamic_reconfigure_node src/dynamic_reconfigure_node.cpp)
#target_link_libraries(dynamic_reconfigure_node
#   ${catkin_LIBRARIES}
# )





add_executable(curb_detection_ros src/curb_detection_ros.cpp)
target_link_libraries(curb_detection_ros ${catkin_LIBRARIES})

add_executable(road_detection_ros_intensity src/road_detection_ros_intensity.cpp)
target_link_libraries(road_detection_ros_intensity ${catkin_LIBRARIES})

add_executable(Speed_bump_code src/Speed_bump_code.cpp)
target_link_libraries(Speed_bump_code  ${catkin_LIBRARIES})

add_executable(test_2 src/test_2.cpp)
target_link_libraries(test_2 ${catkin_LIBRARIES})

add_executable(ground_removal_ring src/ground_removal_ring.cpp)
target_link_libraries(ground_removal_ring ${catkin_LIBRARIES})
add_executable(ground_removal_front src/ground_removal_front.cpp)
target_link_libraries(ground_removal_front ${catkin_LIBRARIES})
add_executable(ground_removal_ring_drone src/ground_removal_ring_drone.cpp)
target_link_libraries(ground_removal_ring_drone ${catkin_LIBRARIES})


add_executable(saver src/saver.cpp)
target_link_libraries(saver ${catkin_LIBRARIES})







add_subdirectory(src/lib)
add_subdirectory(src/conversions)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)
install(PROGRAMS scripts/gen_calibration.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

roslint_cpp()

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
