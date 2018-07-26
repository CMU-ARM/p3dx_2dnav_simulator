cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
project(p3dx_2dnav)

find_package(catkin REQUIRED
    COMPONENTS
        actionlib
        clear_costmap_recovery
        cmake_modules
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        message_generation
        podi_move_base_msgs
        podi_nav_core
        nav_msgs
        navfn
        pluginlib
        roscpp
        rospy
        rotate_recovery
        std_srvs
        tf
)
find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        podi_move_base_msgs
        nav_msgs
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

add_executable(obstacle src/obstacle.cpp)
add_executable(transform src/transformRobotToGroundTruth.cpp)
add_executable(lmsToBase src/lmsToBase.cpp)

target_link_libraries(obstacle ${catkin_LIBRARIES})
target_link_libraries(transform ${catkin_LIBRARIES})
target_link_libraries(lmsToBase ${catkin_LIBRARIES})


install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )

install(
    PROGRAMS
       scripts/subtopic_forwarder.py
       scripts/subtopic_forwarder_node.py
       scripts/warner.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
