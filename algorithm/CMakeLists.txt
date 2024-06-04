cmake_minimum_required(VERSION 3.26)

find_package(PCL CONFIG REQUIRED)
find_package(CGAL CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(OpenMP REQUIRED)

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
target_link_libraries(${LIB_NAME} PUBLIC ${PCL_LIBRARIES} spdlog::spdlog)

#link CGAL
#set(CGAL_DIR "D:/3rdParty/vs22/CGAL_")
#target_include_directories(${LIB_NAME} PUBLIC ${CGAL_DIR}/include)
target_link_libraries(${LIB_NAME} PUBLIC CGAL::CGAL)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${LIB_NAME} PRIVATE /bigobj)
endif ()

# openMP 配置
FIND_PACKAGE(OpenMP REQUIRED)
if (OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()


