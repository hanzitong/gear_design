
cmake_minimum_required(VERSION 3.8)
project(gear_design)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wall -Wextra -Wpedantic -v)
endif()


# find_package(REQUIRED)

include_directories(
  include
  # ${EIGEN3_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIRS}
)

add_library(gear_design_lib
  # src/involute_gear.cpp
  src/quadratic_gear.cpp
)

add_executable(gear_design_test test/main.cpp)
target_link_libraries(gear_design_test gear_design_lib)

target_include_directories(gear_design_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(gear_design_test
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS gear_design_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS gear_design_test
#   RUNTIME DESTINATION bin
# )



