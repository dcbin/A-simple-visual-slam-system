cmake_minimum_required(VERSION 3.16)
project(vo_frontend)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
# find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
set(SOURCES
    ${PROJECT_SOURCE_DIR}/src/camera.cc
    ${PROJECT_SOURCE_DIR}/src/config.cc
    ${PROJECT_SOURCE_DIR}/src/frame.cc
    ${PROJECT_SOURCE_DIR}/src/mappoint.cc
    ${PROJECT_SOURCE_DIR}/src/map.cc
    ${PROJECT_SOURCE_DIR}/src/odometry.cc
    ${PROJECT_SOURCE_DIR}/test/test.cc
)
add_executable(my_frontend ${SOURCES})

target_include_directories(my_frontend PRIVATE /usr/include/glog ${PROJECT_SOURCE_DIR}/include/myfrontend ${EIGEN3_INCLUDE_DIR})
link_directories(/usr/lib/x86_64-linux-gnu)
target_link_libraries(my_frontend glog ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${EIGEN3_LIBRARIES})