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

# Add the SEQUENTIAL define only if USE_SEQUENTIAL is ON
if(USE_PARALLEL)
    add_compile_definitions(-DPARALLEL)
endif()

add_executable(Boids main.cpp
        model/Boid.cpp
        model/Boid.h
        model/Options.h
        model/Simulator.cpp
        model/Simulator.h
        )
target_link_libraries(Boids PRIVATE SFML::Graphics)
