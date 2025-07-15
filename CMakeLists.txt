cmake_minimum_required(VERSION 3.5.0)
project(ROBOT_DOG LANGUAGES CXX)

# ——— C++17 标准 ———
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ——— 包含自定义头 ———
include_directories("${CMAKE_CURRENT_LIST_DIR}/inc")

include_directories(/usr/local/include) 
include_directories(/usr/local/lib/python3.10/dist-packages/torch/include/torch/csrc/api/include)
include_directories(/usr/local/lib/python3.10/dist-packages/torch/include)

link_directories(/usr/local/lib)
link_directories(/usr/local/lib/python3.10/dist-packages/torch/lib)

# 找到 CUDA
set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.6/bin/nvcc")
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-12.6)
set(CUDA_INCLUDE_DIRS "/usr/local/cuda-12.6/include")
set(CUDA_LIBRARY_DIRS "/usr/local/cuda-12.6/lib64")
find_package(CUDA REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})


# ——— 设置 LibTorch 路径 并查找 ———
set(CMAKE_PREFIX_PATH /usr/local/lib/python3.10/dist-packages/torch)
set(Boost_USE_MULTITHREADED ON)
set(Torch_DIR /usr/local/lib/python3.10/dist-packages/torch)
find_package(Torch REQUIRED)

# （可选）将 LibTorch 的编译选项加入全局
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# ——— 收集源文件 ———
aux_source_directory(. SRC_LIST)
aux_source_directory(${CMAKE_SOURCE_DIR}/src SRC_LIST)

# ——— 可执行文件 &amp; 链接库 ———
add_executable(ROBOT_DOG ${SRC_LIST})

# 链接 pthread、CUDA，以及最关键的 LibTorch
target_link_libraries(ROBOT_DOG
    pthread
    ${TORCH_LIBRARIES}
)

# 确保可以找到 libtorch.so
set_property(TARGET ROBOT_DOG PROPERTY IMPORTED_LOCATION
    "${CMAKE_PREFIX_PATH}/lib/libtorch.so"
)