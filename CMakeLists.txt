cmake_minimum_required(VERSION 3.0.2)
project(pclprocess)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
)

find_package(PCL 1.1 REQUIRED)

catkin_package()

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "-O3")

add_executable(pclsplit src/pclsplit.cpp)
target_link_libraries(pclsplit ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(pclsplit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(pclsplit2 src/pclsplit2.cpp)
target_link_libraries(pclsplit2 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(pclsplit2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(pclsplit3 src/pclsplit3.cpp)
target_link_libraries(pclsplit3 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(pclsplit3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(pclsplit4 src/pclsplit4.cpp)
target_link_libraries(pclsplit4 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(pclsplit4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(pclsplit5 src/pclsplit5.cpp)
target_link_libraries(pclsplit5 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(pclsplit5 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(pclsplit6 src/pclsplit6.cpp)
target_link_libraries(pclsplit6 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(pclsplit6 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})