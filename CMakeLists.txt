cmake_minimum_required(VERSION 2.8.3)
project(razer_hydra_demo)

find_package(catkin_simple REQUIRED)

## System dependencies are found with CMake's conventions
catkin_simple()

set(CMAKE_BUILD_TYPE Debug)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()
cs_add_library(hydra_move_group src/hydra_move_group.cpp)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


###########
## Build ##
###########
cs_add_executable(demo src/main.cpp)

target_link_libraries(demo hydra_move_group)
## Specify additional locations of header files



cs_install()
cs_export()
    

    
