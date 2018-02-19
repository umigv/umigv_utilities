cmake_minimum_required(VERSION 2.8.3)

project(umigv_utilities)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(CATKIN_DEPENDS roscpp INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
