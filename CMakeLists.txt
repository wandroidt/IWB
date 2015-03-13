cmake_minimum_required(VERSION 2.8.4)
project(IWB)
find_package( OpenCV REQUIRED )

set(PROJECT_SOURCE_DIR "~/C++/IWB")
set(CMAKE_CURRENT_SOURCE_DIR "~/C++/IWB")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp)
add_executable(IWB ${SOURCE_FILES})
target_link_libraries( IWB ${OpenCV_LIBS} )