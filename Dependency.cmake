include(ExternalProject)

set(DEP_INSTALL_DIR ${PROJECT_BINARY_DIR}/install)
set(DEP_INCLUDE_DIR ${DEP_INSTALL_DIR}/include)
set(DEP_LIB_DIR ${DEP_INSTALL_DIR}/lib)

# spdlog
ExternalProject_Add(
    dep-spdlog
    GIT_REPOSITORY "https://github.com/gabime/spdlog.git"
    GIT_TAG "v1.x"
    GIT_SHALLOW 1
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${DEP_INSTALL_DIR}
    TEST_COMMAND ""
)

set(DEP_LIST ${DEP_LIST} dep-spdlog)
if (MSVC)
  set(DEP_LIBS ${DEP_LIBS} spdlog$<$<CONFIG:Debug>:d>)
else()
  set(DEP_LIBS ${DEP_LIBS} spdlog)
endif()

set(libCarla_INSTALL_DIR ${CMAKE_SOURCE_DIR}/libcarla-install)
set(libCarla_INCLUDE_DIR ${libCarla_INSTALL_DIR}/include)
set(libCarla_LIB_DIR ${libCarla_INSTALL_DIR}/lib)
set(libCarla_LIBS 
    ${libCarla_LIB_DIR}/libboost_filesystem.a 
    ${libCarla_LIB_DIR}/libboost_program_options.a 
    ${libCarla_LIB_DIR}/libboost_system.a 
    ${libCarla_LIB_DIR}/libcarla_client.a 
    ${libCarla_LIB_DIR}/librpc.a 
    ${libCarla_LIB_DIR}/libDebugUtils.a 
    ${libCarla_LIB_DIR}/libDetour.a 
    ${libCarla_LIB_DIR}/libDetourCrowd.a 
    ${libCarla_LIB_DIR}/libDetourTileCache.a 
    ${libCarla_LIB_DIR}/libRecast.a
    ${libCarla_LIB_DIR}/libboost_filesystem.so
    ${libCarla_LIB_DIR}/libboost_program_options.so
    ${libCarla_LIB_DIR}/libboost_system.so
)
set(libCarla_INCLUDE_DIR ${libCarla_INCLUDE_DIR} ${libCarla_INCLUDE_DIR}/system)

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(yaml-cpp REQUIRED)