cmake_minimum_required(VERSION 2.8)
project(PathTracing)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++0x -march=native")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++0x -march=native -fopenmp")

set(SOURCE_FILES
	src/main.cpp
	src/pt.cpp
	src/bidir_pt.cpp
    src/scene.cpp
    )

add_executable(softbivpt.exe ${SOURCE_FILES})
