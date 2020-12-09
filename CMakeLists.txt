cmake_minimum_required(VERSION 3.1)
project(path_planning)

set(CMAKE_GXX_FLAGS "-Wall -Wextra -Werror -Wconversion  -pedantic -std=c++11")
set(source_dir "${PROJECT_SOURCE_DIR}/src/")

include_directories("${PROJECT_SOURCE_DIR}/include/")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file (GLOB source_files "${source_dir}/*.cpp")
add_executable(main.o ${source_files})
target_link_libraries(main.o ${OpenCV_LIBS})