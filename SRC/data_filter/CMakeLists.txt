cmake_minimum_required(VERSION 3.4.1)
project(OBSTALCE_SELECTION)

# add c++11 support
SET(CMAKE_C_COMPILER g++)
add_compile_options(-std=c++11)
#yuan代码路径
aux_source_directory(. SRC)

# 添加头文件路径
include_directories("${PROJECT_SOURCE_DIR}/../common/libs")
include_directories("${PROJECT_SOURCE_DIR}/../common/srcs")
include_directories("${PROJECT_SOURCE_DIR}/libs")

# 添加  子目录 
add_subdirectory("../common" "${CMAKE_CURRENT_BINARY_DIR}/testlib_build")
add_subdirectory("srcs")


# Find required protobuf package
find_package(Protobuf REQUIRED)
if(PROTOBUF_FOUND)
    message(STATUS "protobuf library found")
else()
    message(FATAL_ERROR "protobuf library is needed but cant be found")
endif()
 
include_directories(${PROTOBUF_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ../../Proto/types.proto)


# 指定生成目标
add_executable(data_filter ${SRC} ${PROTO_SRCS} ${PROTO_HDRS})
# 添加链接库
target_link_libraries(data_filter common_lib)
target_link_libraries(data_filter median_filter_lib)
target_link_libraries(data_filter ${PROTOBUF_LIBRARIES})