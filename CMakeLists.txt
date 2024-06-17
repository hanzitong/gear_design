
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


add_executable(gear_design src/main.cpp)

target_include_directories(gear_design
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# target_link_libraries()


install(
  DIRECTORY include/
  DESTINATION include
)
