# BH45B1225 驱动库
add_library(bh45b1225 STATIC
    bh45b1225.c
)

# 添加头文件包含目录
target_include_directories(bh45b1225 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
