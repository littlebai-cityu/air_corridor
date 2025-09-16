cmake_minimum_required(VERSION 3.15)
project(Graphsearch LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14 CACHE STRING "C++ version selection")
set(PYBIND11_PYTHON_VERSION 3.10)

# target_include_directories(example PUBLIC
#                            "${CMAKE_SOURCE_DIR}"
#                            )
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/ConOps1)
include_directories(${PROJECT_SOURCE_DIR}/ConOps2)
include_directories(${PROJECT_SOURCE_DIR}/ConOps3)
include_directories(${PROJECT_SOURCE_DIR}/ConOps4)

# file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/*.cpp)
file(GLOB SRC_FILES 
     ${PROJECT_SOURCE_DIR}/*.cpp
     ${PROJECT_SOURCE_DIR}/ConOps1/*.cpp
     ${PROJECT_SOURCE_DIR}/ConOps2/*.cpp
     ${PROJECT_SOURCE_DIR}/ConOps3/*.cpp
     ${PROJECT_SOURCE_DIR}/ConOps4/*.cpp)
find_package(pybind11 2.5 REQUIRED)
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

pybind11_add_module(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC glog::glog)

target_compile_options(${PROJECT_NAME} PUBLIC -Wall)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME 
    "Graphsearch" PREFIX "" SUFFIX ${CMAKE_SHARED_LIBRARY_SUFFIX})
install (TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_SOURCE_DIR}/..)

# add_subdirectory(extern/pybind11)
