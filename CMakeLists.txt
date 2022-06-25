cmake_minimum_required(VERSION 2.8.3)
project(indemind)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
)

find_package(OpenCV REQUIRED)
message(INFO "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Eigen3)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    include
)

# catkin_package()

add_executable(indemind_node 
    src/indemind_node.cpp
)
target_link_libraries(indemind_node  
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS} 
    ${PROJECT_SOURCE_DIR}/lib/libusbdriver.so
) 

