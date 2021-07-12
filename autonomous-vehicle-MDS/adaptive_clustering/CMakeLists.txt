cmake_minimum_required(VERSION 2.8.3)
project(adaptive_clustering)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs visualization_msgs geometry_msgs pcl_conversions pcl_ros vision_msgs)

find_package(PCL REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

add_message_files(FILES ClusterArray.msg Bboxes2d.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs vision_msgs)

catkin_package()

add_executable(adaptive_clustering src/adaptive_clustering.cpp)
target_link_libraries(adaptive_clustering ${catkin_LIBRARIES})

add_dependencies(adaptive_clustering ${catkin_EXPORTED_TARGETS} )

add_executable(pointcloud_ROI_ src/pointcloud_ROI.cpp)
target_link_libraries(pointcloud_ROI_ ${catkin_LIBRARIES})
add_dependencies(pointcloud_ROI_ ${catkin_EXPORTED_TARGETS} )


