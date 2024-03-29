cmake_minimum_required(VERSION 3.0.2)
project(allan_variance)
set(CMAKE_CXX_STANDARD 11)

find_package(yaml-cpp REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
  message(STATUS "OpenMP found will try to link!")
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -fno-omit-frame-pointer")

find_package(Eigen3 REQUIRED)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(allan_variance 
        src/allan_variance.cpp
        src/yaml_parsers.cpp
        src/AllanVarianceComputor.cpp)

target_link_libraries(allan_variance ${YAML_CPP_LIBRARIES} boost_system boost_filesystem)

