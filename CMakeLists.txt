cmake_minimum_required(VERSION 2.8.3)
project(cam_lidar_cluster_data_project)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  tf
  visualization_msgs
  pcl_ros
  geometry_msgs
  nav_msgs
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)


include_directories(include)
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

#Image Rect Visualization
add_executable(frustum_cluster
  include/frustum_cluster.h
  src/frustum_cluster.cpp
)

target_include_directories(frustum_cluster PRIVATE
  include
  ${OpenCV_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(frustum_cluster
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(frustum_cluster
  ${catkin_EXPORTED_TARGETS}
)

install(
    TARGETS frustum_cluster frustum_cluster
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
