cmake_minimum_required(VERSION 3.16)
project(vo_frontend)
set(CMAKE_CXX_STANDARD 17)
# 指定可执行文件的输出路径
# set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )

find_package(OpenCV REQUIRED)
# find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)
set(SOURCES
    ${PROJECT_SOURCE_DIR}/src/camera.cc
    ${PROJECT_SOURCE_DIR}/src/config.cc
    ${PROJECT_SOURCE_DIR}/src/frame.cc
    ${PROJECT_SOURCE_DIR}/src/mappoint.cc
    ${PROJECT_SOURCE_DIR}/src/map.cc
    ${PROJECT_SOURCE_DIR}/src/odometry.cc
    ${PROJECT_SOURCE_DIR}/test/test.cc
    ${PROJECT_SOURCE_DIR}/src/g2o_types.cc
)
add_executable(my_frontend ${SOURCES})

target_include_directories(my_frontend PRIVATE 
    /usr/include/glog 
    ${PROJECT_SOURCE_DIR}/include/myfrontend 
    ${EIGEN3_INCLUDE_DIR} 
    ${G2O_INCLUDE_DIRS})
link_directories(/usr/lib/x86_64-linux-gnu /usr/local/lib)

target_link_libraries(my_frontend
    glog 
    ${OpenCV_LIBS} 
    ${Sophus_LIBRARIES} 
    ${EIGEN3_LIBRARIES} 
    g2o_core 
    g2o_stuff 
    g2o_types_sba 
    g2o_types_slam3d 
    g2o_solver_dense)