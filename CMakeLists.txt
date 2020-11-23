cmake_minimum_required( VERSION 3.10 )

project( Dynamics_Rayleigh_Ritz )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


#Optimization parameters
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Default Build Type: Release" FORCE)
endif()

set(CMAKE_CXX_FLAGS "-Os")

# Armadillo linking
find_package(Armadillo REQUIRED)

# Include directories
include_directories(
    ./include  
)

# Add third party directory
add_subdirectory(libs)

set(ALL_LIBS
  ${ARMADILLO_LIBRARIES}
  third_party)

set(SOURCES
  ./src/euler_rotations.cpp
  ./src/dynamics_math.cpp
  ./src/modes_magnitude.cpp
  ./src/input_trajectory.cpp
  ./src/handle.cpp
  ./src/rayleigh_ritz_beam.cpp
  ./src/system_rayleigh_ritz.cpp
    )

  
# target
add_executable(main main.cpp ${SOURCES})

target_link_libraries(main ${ALL_LIBS})
target_include_directories(main PRIVATE ${ARMADILLO_INCLUDE_DIRS})