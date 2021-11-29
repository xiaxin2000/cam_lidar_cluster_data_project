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
  grid_map_cv
  grid_map_msgs
  grid_map_ros
  jsk_rviz_plugins
  sensor_msgs
  std_msgs
  vector_map_server
)

find_package(OpenMP)
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include  
  CATKIN_DEPENDS 
  geometry_msgs nav_msgs roscpp std_msgs
  DEPENDS EIGEN3 PCL 
  vector_map_server
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  jsk_rviz_plugins
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIBRARY_DIRS})

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
  src/frustum_cluster.cpp
  src/cluster.cpp
)

target_include_directories(frustum_cluster PRIVATE
  include
  ${OpenCV_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

target_link_libraries(frustum_cluster
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

add_dependencies(frustum_cluster
  ${catkin_EXPORTED_TARGETS}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
    TARGETS frustum_cluster frustum_cluster
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
