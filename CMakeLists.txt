
cmake_minimum_required(VERSION 3.8)
project(gear_design)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wall -Wextra -Wpedantic -v)
endif()


find_package(Eigen3 REQUIRED)

## old method
# include_directories(
#   ${PROJECT_SOURCE_DIR}/include
#   ${EIGEN3_INCLUDE_DIRS}
#   ${Boost_INCLUDE_DIRS}
# )

## ============== BUILD A CODE ==============

# add_executable(main
#   example/main.cpp
# )
# target_include_directories(main PRIVATE
#   ${PROJECT_SOURCE_DIR}/include
#   ${EIGEN3_INCLUDE_DIRS}
#   ${Boost_INCLUDE_DIRS}
# )
# target_link_libraries(main PRIVATE
#   nlopt
# )

## ============== BUILD A TEST CODE ==============

# add_executable(test_coordinates 
add_executable(test_code 
    test/test_coordinates.cpp
)
target_include_directories(test_code PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
target_link_libraries(test_code PRIVATE
  gtest
  pthread
  nlopt
)

# target_include_directories(gear_design PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

## ============== BUILD AS A LIBRARY ==============

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

