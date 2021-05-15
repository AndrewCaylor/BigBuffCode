cmake_minimum_required(VERSION 3.17)
project(bigbuff)
#
set(CMAKE_CXX_STANDARD 14)
#
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
#
add_executable(bigbuff main.cpp BigbufDetector.cpp BigbufDetector.h TargetDetector.cpp TargetDetector.h DataCollector.cpp DataCollector.h)
target_link_libraries(bigbuff ${OpenCV_LIBS})
target_link_libraries(bigbuff ${Boost_LIBRARIES})
