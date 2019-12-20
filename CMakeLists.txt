cmake_minimum_required(VERSION 2.8.3)
project(density_aware_mst)

set(CMAKE_CXX_STANDARD 11)

include(GNUInstallDirs)

find_package(catkin REQUIRED COMPONENTS
    roboskel_msgs
    sensor_msgs
    roscpp
)

set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

set(${LIBRARY_TARGET_NAME}_SRC
    src/damst.cpp
)

set(${LIBRARY_TARGET_NAME}_HDR
    include/damst/damst.hpp
)

find_package(Boost REQUIRED COMPONENTS
    graph
)

catkin_package()

file(TO_CMAKE_PATH "${PROJECT_BINARY_DIR}/CMakeLists.txt" LOC_PATH)
if(EXISTS "${LOC_PATH}")
    message(FATAL_ERROR "You cannot build in a source directory (or any directory with a CMakeLists.txt file). Please make a build subdirectory. Feel free to remove CMakeCache.txt and CMakeFiles.")
endif()

include_directories(
    include
    ${LIBRARY_TARGET_NAME}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(${LIBRARY_TARGET_NAME} SHARED
    src/damst.cpp   
)

install(TARGETS ${LIBRARY_TARGET_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

add_executable(test_damst1 test/test_damst1.cpp)
target_link_libraries(test_damst1 ${LIBRARY_TARGET_NAME})

