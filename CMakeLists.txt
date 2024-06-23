cmake_minimum_required(VERSION 2.8.3)
project(maxon_epos)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES maxon_epos
)

 ###########
## Build ##
###########

include_directories(
    include
    include/${PROJECT_NAME}/
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/EposMotor.cpp
    src/EposManager.cpp
    src/Device.cpp
    src/EposException.cpp
    src/ControlModeBase.cpp
    src/EposProfilePositionMode.cpp
    src/EposProfileVelocityMode.cpp
    src/EposCurrentMode.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    EposCmd
)
