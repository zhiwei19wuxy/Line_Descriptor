cmake_minimum_required(VERSION 3.26)

find_package(PCL CONFIG REQUIRED)
find_package(CGAL CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

FILE(GLOB INCLUDE "./include/*.h")
FILE(GLOB SRC "./src/*.cpp")

#STATIC library

set(LIB_NAME "lineDes")
add_library(${LIB_NAME} STATIC)

target_include_directories(${LIB_NAME} PUBLIC ./include)
target_sources(${LIB_NAME}
        PUBLIC
        ${INCLUDE}
        PRIVATE
        ${SRC})

# link PCL
target_link_libraries(${LIB_NAME} PUBLIC ${PCL_LIBRARIES})

#link CGAL
target_link_libraries(${LIB_NAME} PUBLIC CGAL::CGAL spdlog::spdlog)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${LIB_NAME} PRIVATE /bigobj)
endif ()
