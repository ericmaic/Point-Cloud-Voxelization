cmake_minimum_required(VERSION 2.6)
project(voxel_grid)
 
find_package(PCL 1.2 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_executable(voxel_grid voxel_grid.cpp)
 
target_link_libraries (voxel_grid ${PCL_LIBRARIES})
 
install(TARGETS voxel_grid RUNTIME DESTINATION bin)
