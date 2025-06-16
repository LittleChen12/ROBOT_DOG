cmake_minimum_required(VERSION 3.5.0)
project(ROBOT_DOG)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 引入头文件目录
INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_LIST_DIR}/inc
)

# 将cpp文件添加到变量SRC_LIST
AUX_SOURCE_DIRECTORY(. SRC_LIST)                          #添加main.cpp
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src SRC_LIST)    #添加src目录下cpp文件
message(${SRC_LIST})                                      #打印路径，检测是否正确

# 将所有cpp文件编译生成可执行ROBOT_DOG文件
add_executable(ROBOT_DOG ${SRC_LIST})
# 链接pthread库
target_link_libraries(ROBOT_DOG pthread)

# 开启调试功能
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

