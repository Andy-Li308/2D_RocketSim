cmake_minimum_required(VERSION 3.16)
project(RocketSimulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add warning about library requirements
message(STATUS "================================================")
message(STATUS "2D Rocket Simulation Build Configuration")
message(STATUS "================================================")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Generator: ${CMAKE_GENERATOR}")
message(STATUS "")
message(STATUS "Looking for required libraries...")

# Set up library paths
set(SFML_DIR "C:/Users/andyl/Libraries/SFML-3.0.2/lib/cmake/SFML" CACHE PATH "SFML CMake directory")
set(Eigen3_DIR "C:/Users/andyl/Libraries/eigen-5.0.0/cmake" CACHE PATH "Eigen3 CMake directory")

# Find SFML
find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System)
message(STATUS "✓ SFML found: ${SFML_DIR}")

# Find Eigen3 - try config file first, fall back to header-only
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
    message(STATUS "Eigen3 config not found, using header-only mode")
    # Eigen is header-only, so we just need to include the headers
    set(Eigen3_INCLUDE_DIR "C:/Users/andyl/Libraries/eigen-5.0.0")
    message(STATUS "✓ Eigen3 headers: ${Eigen3_INCLUDE_DIR}")
else()
    message(STATUS "✓ Eigen3 found via config")
endif()

# Create the executable
add_executable(RocketSim
    src/main.cpp
    src/Rocket.cpp
    src/Visualization.cpp
    src/Controller.cpp
)

# Set up include directories (before target_link_libraries for visibility)
target_include_directories(RocketSim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    "C:/Users/andyl/Libraries/SFML-3.0.2/include"
)

# Link libraries
target_link_libraries(RocketSim PRIVATE
    SFML::Graphics
    SFML::Window
    SFML::System
)

# Link Eigen3 if found via config, otherwise just include headers
if(Eigen3_FOUND)
    target_link_libraries(RocketSim PRIVATE Eigen3::Eigen)
else()
    target_include_directories(RocketSim PRIVATE ${Eigen3_INCLUDE_DIR})
endif()
