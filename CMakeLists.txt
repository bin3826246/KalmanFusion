# 设置工程名为SensorFusion
project(KalmanFusion)

# 设置所需要的CMake的最低版本号
cmake_minimum_required (VERSION 2.8)

# 使用C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

# 设置使用DEBUG模式还是RELEASE模式
set(CMAKE_BUILD_TYPE DEBUG)
#set(CMAKE_BUILD_TYPE RELEASE)

include_directories(include)
include_directories(3rdparty)

# 找到src目录及其子目录下的所有c文件和h文件
file(GLOB_RECURSE PROJECT_SOURCES "src/*.cpp" "src/*.c" "src/*.cc")
file(GLOB_RECURSE PROJECT_HEADERS "include/*.hpp" "include/*.h" "include/*.hh")

# 设置二进制文件的生成地址，即bin目录，编译后会有名为SensorFusion或SensorFusion_d的二进制文件生成在bin目录下
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# 根据RELEASE或DEBUG模式，生成不同名的可执行文件，用以区分
#add_executable(KalmanFusion_d "src/main.cpp" "src/kalmanfilter.cpp" "src/sensorFusion.cpp")
if( ${CMAKE_BUILD_TYPE} MATCHES "RELEASE" )
    add_executable(KalmanFusion ${PROJECT_SOURCES} ${PROJECT_HEADERS})
else()
    add_executable(KalmanFusion_d ${PROJECT_SOURCES} ${PROJECT_HEADERS})
endif()
