
cmake_minimum_required(VERSION 3.8)
project(test_gear_design)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wall -Wextra -Wpedantic -v)
endif()


add_executable(calc_ybase 
    test_f_gear_calc_y_base_from_xgear.cpp
    ../src/f_gear_objective_function.cpp
    ../src/f_gear_profile.cpp
)
target_include_directories(calc_ybase PRIVATE include)
target_include_directories(calc_ybase PRIVATE /usr/include/eigen3)
target_link_libraries(calc_ybase PRIVATE nlopt)
target_link_libraries(calc_ybase PRIVATE gtest)
target_link_libraries(calc_ybase PRIVATE pthread)




# add_executable(gendata_objfunc_elipse_circle example/gendata_objfunc_elipse_circle.cpp)
# target_include_directories(gendata_objfunc_elipse_circle PRIVATE include)
# target_link_libraries(calc_intersection PRIVATE nlopt)
# install(TARGETS gendata_objfunc_elipse_circle DESTINATION ${CMAKE_SOURCE_DIR}/plot)

