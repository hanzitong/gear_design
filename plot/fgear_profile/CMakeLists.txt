
cmake_minimum_required(VERSION 3.8)

if(CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen/Core)


add_executable(gear_code ../gendata_multi_sigbase_fgear_profile.cpp)
target_include_directories()
target_link_libraries(gear_code eigen)

