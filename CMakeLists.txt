cmake_minimum_required(VERSION 3.4.1)
project(LOGANALYSIS_ALGOS)

# add c++11 support
SET(CMAKE_C_COMPILER g++)
add_compile_options(-std=c++11)

# Find required protobuf package
find_package(Protobuf REQUIRED)
if(PROTOBUF_FOUND)
    message(STATUS "protobuf library found")
else()
    message(FATAL_ERROR "protobuf library is needed but cant be found")
endif()



include_directories(${PROTOBUF_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ../Proto/types.proto)

#代码路径
# aux_source_directory(./SRC SRC)
# add_executable(log_analysis_and_algos ${SRC})