cmake_minimum_required(VERSION 3.4.1)
project(OBSTALCE_SELECTION)

# add c++11 support
SET(CMAKE_C_COMPILER g++)
add_compile_options(-std=c++11)
#代码路径
aux_source_directory(. SRC)
# 添加头文件路径
include_directories("${PROJECT_SOURCE_DIR}/../common/libs")
# 添加 math 子目录
add_subdirectory("../common" "${CMAKE_CURRENT_BINARY_DIR}/testlib_build")

# 指定生成目标
add_executable(obstacle_selection ${SRC})
# 添加链接库
target_link_libraries(obstacle_selection CommonFun)
