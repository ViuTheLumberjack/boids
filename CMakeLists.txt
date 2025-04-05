include(FetchContent)

cmake_minimum_required(VERSION 3.28)
project(Boids)

option(USE_PARALLEL "Build with parallel processing" OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_STANDARD 23)

FetchContent_Declare(SFML
        GIT_REPOSITORY https://github.com/SFML/SFML.git
        GIT_TAG 3.0.0
        GIT_SHALLOW ON
        EXCLUDE_FROM_ALL
        SYSTEM)
FetchContent_MakeAvailable(SFML)

add_library(BoidLib
        Options.h
        modelAOS/Boid.cpp
        modelAOS/Boid.h
        modelAOS/Simulator.cpp
        modelAOS/Simulator.h
        modelAOS/WindowManager.cpp
        modelAOS/WindowManager.h
        modelSOA/BoidsArray.h
        modelSOA/Simulator.cpp
        modelSOA/Simulator.h
        modelSOA/WindowManager.cpp
        modelSOA/WindowManager.h
)

target_link_libraries(BoidLib PUBLIC SFML::Graphics PUBLIC SFML::Window PUBLIC SFML::System)

add_executable(Boids_DEMO demo.cpp)
add_executable(Boids_Benchmark benchmark.cpp)


# Add the SEQUENTIAL define only if USE_SEQUENTIAL is ON
if(USE_PARALLEL)
    find_package(OpenMP REQUIRED)
    target_link_libraries(BoidLib PUBLIC OpenMP::OpenMP_CXX)
endif()

target_link_libraries(Boids_DEMO PRIVATE BoidLib)
target_link_libraries(Boids_Benchmark PRIVATE BoidLib)
