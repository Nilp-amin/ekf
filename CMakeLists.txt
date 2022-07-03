cmake_minimum_required(VERSION 3.3)

# specify project name
project(EKF)

# specify required C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)
MESSAGE( [Main] " EIGEN3_INCLUDE_DIRS = ${EIGEN3_INCLUDE_DIRS}")

# Eigen 3 include dir
include_directories($(EIGEN3_INCLUDE_DIRS))
# EKF include dir
include_directories(include)

# add the executable
add_executable(main ./test/main.cpp)

# link Eigen3 library to target 
target_link_libraries(main Eigen3::Eigen)
