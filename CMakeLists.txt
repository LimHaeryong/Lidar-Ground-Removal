cmake_minimum_required(VERSION 3.14)
set(PROJECT_NAME Lidar-Ground-Removal)
project(${PROJECT_NAME} LANGUAGES CXX)

add_definitions(-std=c++14 -O3)

set(CXX_FLAGS "-pthread -fPIC -DNDEBUG -Wextra")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS} -Wl,-Bstatic -Wl,-Bdynamic")

include(Dependency.cmake)

add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}
    src/main.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${libCarla_INCLUDE_DIR}
    ${DEP_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    include
)

target_link_directories(${PROJECT_NAME} PUBLIC
    ${libCarla_LIB_DIR}
    ${PCL_LIBRARY_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${libCarla_LIBS}
    ${PCL_LIBRARIES}
)

