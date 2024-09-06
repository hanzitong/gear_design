
cmake_minimum_required(VERSION 3.8)
project(test_gear_design)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wall -Wextra -Wpedantic -v)
endif()


add_executable(test_coordinates_transformation 
    test_coordinates.cpp
    # test_coordinates_transformation.cpp
    ../src/objfunctions.cpp
    ../src/coordinates.cpp
)
target_include_directories(test_coordinates_transformation PRIVATE include)
target_include_directories(test_coordinates_transformation PRIVATE /usr/include/eigen3)
target_link_libraries(test_coordinates_transformation PRIVATE nlopt)
target_link_libraries(test_coordinates_transformation PRIVATE gtest)
target_link_libraries(test_coordinates_transformation PRIVATE pthread)




# add_executable(gendata_objfunc_elipse_circle example/gendata_objfunc_elipse_circle.cpp)
# target_include_directories(gendata_objfunc_elipse_circle PRIVATE include)
# target_link_libraries(calc_intersection PRIVATE nlopt)
# install(TARGETS gendata_objfunc_elipse_circle DESTINATION ${CMAKE_SOURCE_DIR}/plot)

