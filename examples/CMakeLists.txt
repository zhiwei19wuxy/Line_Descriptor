find_package(PCL CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

add_executable(exam_Line_descriptors main.cpp)
target_link_libraries(exam_Line_descriptors PRIVATE ${PCL_LIBRARIES} lineDes spdlog::spdlog)

#add_executable(exam_triangle exam_triangle.cpp)
#target_link_libraries(exam_triangle PRIVATE lineDes)