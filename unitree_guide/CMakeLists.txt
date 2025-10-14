cmake_minimum_required(VERSION 3.14)
project(unitree_guide)
set(CMAKE_BUILD_TYPE "Release")

set(ROBOT_TYPE Go1)         # The type of robot, support Go1 and A1 currently
set(PLATFORM amd64)         # The platform to compile, support amd64 and arm64

set(CATKIN_BUILD ON)        # Use CATKIN_BUILD or not, ON or OFF
set(GAZEBO ON)              # Use Gazebo or not, ON or OFF
set(REAL_ROBOT OFF)          # Link real robot or not, ON or OFF


set(DEBUG ON)              # Use debug functions or not, ON or OFF
# set(DEBUG ON)              # Use debug functions or not, ON or OFF
set(MOVE_BASE ON)               # Need move_base or not, ON or OFF

# 做一些逻辑判断，判断使用的是实物还是仿真
if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()
if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()

if(${ROBOT_TYPE} STREQUAL "A1")
    add_definitions(-DROBOT_TYPE_A1)
elseif(${ROBOT_TYPE} STREQUAL "Go1")
    add_definitions(-DROBOT_TYPE_Go1)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()

if(((GAZEBO) AND (REAL_ROBOT)) OR ((NOT GAZEBO) AND (NOT REAL_ROBOT)))
    message(FATAL_ERROR "[CMake ERROR] The GAZEBO and REAL_ROBOT can only be one ON one OFF")
endif()

if(GAZEBO OR MOVE_BASE)
    add_definitions(-DRUN_ROS)
    set(CATKIN_BUILD ON)
endif()

set(CMAKE_CXX_STANDARD 11)
find_package(Boost)

if(CATKIN_BUILD)
    add_definitions(-DCOMPILE_WITH_ROS)
    if(MOVE_BASE)
        add_definitions(-DCOMPILE_WITH_MOVE_BASE)
    endif()
endif()

if(DEBUG)
    add_definitions(-DCOMPILE_DEBUG)
    find_package(Python COMPONENTS Interpreter Development NumPy)
endif()

if(CATKIN_BUILD)
    if(GAZEBO)
        add_definitions(-DCOMPILE_WITH_GAZEBO)
        find_package(catkin REQUIRED COMPONENTS
            controller_manager
            joint_state_controller
            gazebo_ros
            # gazebo
        )
    endif()

    find_package(catkin REQUIRED COMPONENTS
        genmsg
        robot_state_publisher
        roscpp
        std_msgs
        tf
        geometry_msgs
        unitree_legged_msgs
    )
    catkin_package(
        CATKIN_DEPENDS 
        unitree_legged_msgs
    )
endif()

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3")

if(REAL_ROBOT)
    add_definitions(-DCOMPILE_WITH_REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        include_directories(
            library/unitree_legged_sdk_3.2/include
        )
        link_directories(
            library/unitree_legged_sdk_3.2/lib
        )
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
        include_directories(
            library/unitree_legged_sdk-3.8.0/include
        )
        if(${PLATFORM} STREQUAL "amd64")
            link_directories(
                library/unitree_legged_sdk-3.8.0/lib/cpp/amd64
            )
        elseif(${PLATFORM} STREQUAL "arm64")
            link_directories(
                library/unitree_legged_sdk-3.8.0/lib/cpp/arm64
            )
        endif()
    endif()
endif()

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

# aux_source_directory(src SRC_LIST)
file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

add_executable(junior_ctrl src/main.cpp ${SRC_LIST})
if(CATKIN_BUILD)
    target_link_libraries(junior_ctrl ${catkin_LIBRARIES})
    add_dependencies(junior_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()
if(DEBUG)
    target_include_directories(junior_ctrl PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
    target_link_libraries(junior_ctrl Python::Python Python::NumPy)
endif()
if(REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        if(${PLATFORM} STREQUAL "amd64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_amd64.so)
        elseif(${PLATFORM} STREQUAL "arm64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_arm64.so)
        endif()
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
            target_link_libraries(junior_ctrl libunitree_legged_sdk.a)
    endif()
endif()
target_link_libraries(junior_ctrl -pthread lcm)

if(NOT CATKIN_BUILD)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
endif()