
cmake_minimum_required(VERSION 3.8)
project(gear_design)

# if (NOT CMAKE_BUILD_TYPE)
#   set (CMAKE_BUILD_TYPE Debug CACHE STRING "comment: Build type (Debug/Release)" FORCE)
# endif()

# if (NOT CMAKE_BUILD_TYPE)
#   set (CMAKE_BUILD_TYPE Release CACHE STRING "comment: Build type (Debug/Release)" FORCE)
# endif()

message("CMAKE_BUILD_TYPE is set to ${CMAKE_BUILD_TYPE}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic) # show more warnings
  add_compile_options(-v) # show verbose message

  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -O0) # set debug mode and remove optimization
  endif()

  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3)  # optimized build
  endif()
endif()

find_package(Eigen3 REQUIRED)

## ============== BUILD A MINI-CODE ==============

# add_executable(main 
#   example/mini_main.cpp
# )
# target_include_directories(main PRIVATE
#   ${PROJECT_SOURCE_DIR}/include
#   ${EIGEN3_INCLUDE_DIRS}
#   ${Boost_INCLUDE_DIRS}
# )
# target_link_libraries(main PRIVATE
#   nlopt
# )

## ============== BUILD A TEST-CODE ==============

# add_executable(test_code 
#   # test/test_coordinates.cpp
#   test/test_coordinates_optimization.cpp
# )
# target_include_directories(test_code PRIVATE
#   ${PROJECT_SOURCE_DIR}/include
#   ${EIGEN3_INCLUDE_DIRS}
#   ${Boost_INCLUDE_DIRS}
# )
# target_link_libraries(test_code PRIVATE
#   gtest
#   pthread
#   nlopt
# )


## ========== BUILD PLOT-DATUM-GENERATERS ==============

add_executable(gendata
  plot/objfunc_shape/src/gendata_objfunc_shape.cpp
)
target_include_directories(gendata PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(gendata PRIVATE
  nlopt
)


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


## ============== memo ===============

## this is old method
# include_directories(
#   ${PROJECT_SOURCE_DIR}/include
#   ${EIGEN3_INCLUDE_DIRS}
#   ${Boost_INCLUDE_DIRS}
# )




