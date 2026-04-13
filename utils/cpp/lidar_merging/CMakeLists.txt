cmake_minimum_required(VERSION 3.8) # compatibilites de CMake
project(lidar_merging)

set(CMAKE_BUILD_TYPE debug) # compilation en mode debug
set(CMAKE_CXX_FLAGS "-Wall -Wextra -std=c++17") # options

find_package(PCL REQUIRED)
find_package(nlohmann_json REQUIRED)
# find_package(PCL REQUIRED COMPONENTS common octree)
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# file(source_files # les fichiers suivants seront listes dans la variable source_files
# src/lidar_merging.cpp
# )

add_executable( # creation de l'executable binaire du projet
  ${PROJECT_NAME}
  src/lidar_merging.cpp
  # src/test.cpp
)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES}) # link des bibliotheques PCL

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)