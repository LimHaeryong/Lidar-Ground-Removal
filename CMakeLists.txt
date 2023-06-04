cmake_minimum_required(VERSION 3.14)
set(PROJECT_NAME Lidar-Ground-Removal)
project(${PROJECT_NAME} LANGUAGES CXX)

add_definitions(-std=c++14 -O3)

set(CXX_FLAGS "-pthread -fPIC -DNDEBUG -Wextra")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS} -Wl,-Bstatic -Wl,-Bdynamic")

include(Dependency.cmake)

add_executable(ground_removal
    src/main.cpp
    src/carla_manager.cpp include/carla_manager.h
    src/utils.cpp include/utils.h
    src/lidar_manager.cpp include/lidar_manager.h
    src/lidar_processor.cpp include/lidar_manager.h
    include/threadsafe_queue.h
)

target_include_directories(ground_removal PUBLIC
    include    
    ${libCarla_INCLUDE_DIR}
    ${DEP_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

target_link_directories(ground_removal PUBLIC
    ${libCarla_LIB_DIR}
    ${DEP_LIB_DIR}
    ${PCL_LIBRARY_DIRS}
)

target_link_libraries(ground_removal PUBLIC
    ${libCarla_LIBS}
    ${DEP_LIBS}
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

add_dependencies(ground_removal
    ${DEP_LIST}
)