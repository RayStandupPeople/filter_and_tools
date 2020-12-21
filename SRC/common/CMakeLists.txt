# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_LIB_SRCS 变量
aux_source_directory(./srcs DIR_LIB_SRCS)

# 增加编译选项
# add_compile_options(`pkg-config --cflags --libs protobuf`)
add_compile_options(-std=c++11)
add_compile_options(-I/usr/include/python2.7 )
add_compile_options(-lpython2.7)
# 生成链接库
add_library (common_lib ${DIR_LIB_SRCS})