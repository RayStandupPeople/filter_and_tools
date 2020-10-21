cmake_minimum_required(VERSION 3.4.1)
project(filter)

# add c++11 support
SET(CMAKE_C_COMPILER g++)
add_compile_options(-std=c++11)
#代码路径
aux_source_directory(./SRC SRC)
add_executable(filter ${SRC})
