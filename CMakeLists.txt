cmake_minimum_required(VERSION 3.14)
project(Radar)

set(CMAKE_CXX_STANDARD 20)

#设置执行文件输出目录
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(INC_DIR     /opt/DahuaTech/MVViewer/include
        ${PROJECT_SOURCE_DIR}/include
        ${Boost_INCLUDE_DIRS})

set(LINK_DIR    /opt/DahuaTech/MVViewer/lib/GenICam/bin/Linux64_x64
        /opt/DahuaTech/MVViewer/lib
        ${PROJECT_SOURCE_DIR}/lib/cubot)

#寻找opencv库,并且添加头文件
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#寻找boost库，并且添加头文件
find_package(Boost REQUIRED COMPONENTS system atomic)
include_directories(${Boost_INCLUDE_DIRS})


#
#MESSAGE(STATUS "Projectggg: SERVER")               #打印相关消息消息
#
#find_package(apriltag REQUIRED)
#message(STATUS ${apriltag_LIBRARIES})
#include_directories(${apriltag_INCLUDE_DIRS})
#



#寻找eigen库，并且添加头文件
find_package(Eigen3  REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS} ${INC_DIR} ${EIGEN3_INCLUDE_DIR})

#添加动态库,#添加系统关联库
link_directories(${LINK_DIR} )


#头文件搜索目录
include_directories(${PROJECT_SOURCE_DIR}/my_radar/inc)
include_directories(${PROJECT_SOURCE_DIR}/camera/inc)

#添加子目录
add_subdirectory(camera/src)

#添加子目录
add_executable(Radar main.cpp camera/src/shm.cpp my_radar/src/radar_main.cpp my_radar/inc/radar_main.hpp
        my_radar/src/double_measurement.cpp my_radar/inc/double_measurement.h my_radar/src/find_darts.cpp
        my_radar/inc/find_darts.h my_radar/src/others.cpp my_radar/inc/others.h my_radar/inc/switch_function.h
        my_radar/src/apriltag_pose.cpp my_radar/inc/apriltag_pose.h my_radar/src/kalman_forecast.cpp my_radar/inc/kalman_forecast.h my_radar/src/serial_port.cpp my_radar/inc/serial_port.h)

target_link_libraries(Radar ${OpenCV_LIBS} ${Eigen3_LIBRARIES} ${Boost_LIBRARY} ${LINK_DIR}
        boost_thread boost_system rt -lpthread timer imgSubscrible -lboost_date_time
        /usr/local/lib/libapriltag.so  /usr/local/lib/libapriltag.so.3 /usr/local/lib/libapriltag.so.3.1.0)

