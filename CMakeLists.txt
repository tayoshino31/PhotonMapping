cmake_minimum_required(VERSION 3.10)
project(PhotonMapping)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/lajolla/cmake")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Add lajolla ---
add_subdirectory(lajolla)

add_executable(PhotonMapping main.cpp photon.cpp utils.h nanoflann.hpp kdtree.cpp)
target_link_libraries(PhotonMapping PRIVATE lajolla_lib)

target_include_directories(PhotonMapping PRIVATE
    ${CMAKE_SOURCE_DIR}/lajolla/src
    ${CMAKE_SOURCE_DIR}/lajolla/embree/include
)

add_custom_command(TARGET PhotonMapping POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_SOURCE_DIR}/lajolla/embree/bin/embree4.dll
    ${CMAKE_SOURCE_DIR}/lajolla/embree/bin/tbb12.dll
    $<TARGET_FILE_DIR:PhotonMapping>
)

set_target_properties(PhotonMapping PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
)
