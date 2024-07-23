cmake_minimum_required(VERSION 2.8)
project(3DTo3D)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)


find_package(Pangolin REQUIRED)# 可视化工具库
include_directories(${Pangolin_INCLUDE_DIRS})



# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
 

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
     
# Eigen
# 不需要寻找 手动指定默认安装位置 只是一堆头文件 无需target_link_libraries
find_package(Eigen3 REQUIRED)
include_directories("/usr/local/include/eigen3")

# G2O
set(ALL_TARGET_LIBRARIES g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension)
include(/home/dongdong/V1_Project/v20_openvslam/v0_dll/g2o/cmake_modules/FindG2O.cmake) # 改成自己安装包
#include(cmake/FindG2O.cmake)

# 编译节点
add_executable(v1_node main.cpp)

# 链接目标
target_link_libraries(v1_node ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${ALL_TARGET_LIBRARIES}  -lgomp)

