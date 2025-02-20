cmake_minimum_required(VERSION 3.10)  # 设置 CMake 最低版本
project(MyDynamicLibrary)             # 定义项目名称

include_directories(${CMAKE_SOURCE_DIR})

# 添加动态链接库
add_library(Armor_detector SHARED
    conduction.cpp
    config.hpp
)