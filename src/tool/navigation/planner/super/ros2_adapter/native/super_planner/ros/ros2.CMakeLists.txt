cmake_minimum_required(VERSION 3.5)
project(super_planner)

set(CMAKE_CXX_STANDARD 17)
option(RELEASE "Enable Release mode" ON)
option(SUPER_ENABLE_STACKTRACE "Enable libdw signal stack traces" ON)
#if (RELEASE)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-O3 -Wall")
#else ()
#    set(CMAKE_BUILD_TYPE Debug)
#    set(CMAKE_CXX_FLAGS "-O -Wall -g")
#endif ()

add_compile_options(-Werror=return-type)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Define the voxelize and raycasting method
add_definitions(-DORIGIN_AT_CORNER)
add_definitions(-DFMT_HEADER_ONLY)
add_definitions(-DUSE_ROS2)


string(TOUPPER $ENV{ROS_DISTRO} ROS_VERSION)
message(STATUS "ROS version: ${ROS_VERSION}")

include_directories(BEFORE
        include
        ../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rog_map REQUIRED)
find_package(mars_quadrotor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
    set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
else()
    set(YAML_CPP_TARGET yaml-cpp)
endif()


set(includes
        include
        ${Eigen_INCLUDE_DIRS}
        #            ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(third_party_libs
        #            ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        ${YAML_CPP_TARGET}
)

if(SUPER_ENABLE_STACKTRACE)
    list(APPEND third_party_libs dw)
endif()

set(ros_libs
        rclcpp
        std_msgs
        mars_quadrotor_msgs
        sensor_msgs
        tf2_ros
        nav_msgs
        geometry_msgs
        visualization_msgs
        rog_map
        tf2_ros
        pcl_conversions

)

include_directories(
        ${includes}
)


#=================FOR libsuper ============================#
#    file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
add_library(super
        ${srcs}
)
if(SUPER_ENABLE_STACKTRACE)
    target_compile_definitions(super PUBLIC SUPER_ENABLE_STACKTRACE=1)
endif()
target_include_directories(super PUBLIC
        ${includes})
ament_target_dependencies(super PUBLIC
        ${ros_libs}
)

#=================FOR Benchmark Node ============================#
add_executable(fsm_node
        Apps/fsm_node_ros2.cpp
)
if(SUPER_ENABLE_STACKTRACE)
    target_compile_definitions(fsm_node PRIVATE SUPER_ENABLE_STACKTRACE=1)
endif()
target_link_libraries(fsm_node PUBLIC
        super
        ${third_party_libs}
)
ament_target_dependencies(fsm_node PUBLIC
        ${ros_libs}
)

#=================INSTALL all targets ============================#

install(
        TARGETS
        fsm_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(${includes})

install(DIRECTORY include/
        DESTINATION include/
)

# install the launch file
install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}/
)



# Opt-in integration regression; uses the real A* and an isolated ROS domain.
option(SUPER_BUILD_PATH_SEARCH_TEST "Build clipped-goal A* regression" OFF)
if(SUPER_BUILD_PATH_SEARCH_TEST)
    add_executable(astar_clipped_goal_test test/astar_clipped_goal_test.cpp)
    target_link_libraries(astar_clipped_goal_test PUBLIC super ${third_party_libs})
    ament_target_dependencies(astar_clipped_goal_test PUBLIC ${ros_libs})
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    enable_testing()
    add_test(NAME astar_clipped_goal
        COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/test/run_astar_clipped_goal.py
            $<TARGET_FILE:astar_clipped_goal_test>)
endif()

ament_package()
