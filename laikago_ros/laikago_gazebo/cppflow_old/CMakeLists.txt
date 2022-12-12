#cmake_minimum_required(VERSION 3.10)
cmake_minimum_required(VERSION 3.5)
project(example)

find_library(TENSORFLOW_LIB tensorflow HINT $ENV{HOME}/libtensorflow/lib)
include_directories(include)

#set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD 14)
add_executable(example examples/load_model/main.cpp src/Model.cpp src/Tensor.cpp)
target_include_directories(example PRIVATE include $ENV{HOME}/libtensorflow/include)
target_link_libraries(example "${TENSORFLOW_LIB}")
