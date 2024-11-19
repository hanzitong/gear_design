
cmake_minimum_required(VERSION 3.8)
project(gear_design)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wall -Wextra -Wpedantic -v)
endif()


find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIRS}
)

add_executable(main
  example/main.cpp
  # src/f_gear_profile.cpp
  # src/f_gear_objective_function.cpp
  # src/coordinates.cpp
  # src/objfunctions.cpp
)
target_include_directories(main PRIVATE include)
target_link_libraries(main PRIVATE nlopt)

# add_executable(test_quadratic_gear 
#     test/test_quadratic_gear.cpp
#     src/quadratic_gear.cpp
# )
# target_include_directories(test_quadratic_gear PRIVATE include)
# target_link_libraries(test_quadratic_gear PRIVATE Eigen3::Eigen gtest pthread)

# target_include_directories(gear_design
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# ==============

# add_library(gear_design_lib
#   src/quadratic_gear.cpp
#   # src/involute_gear.cpp
# )
# add_executable(gear_design_test test/main.cpp)
# target_link_libraries(gear_design_test gear_design_lib)

# target_include_directories(gear_design_lib
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# target_include_directories(gear_design_test
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# ==============

