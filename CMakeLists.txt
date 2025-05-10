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
FetchContent_Declare(matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG origin/master)
FetchContent_MakeAvailable(SFML)
FetchContent_MakeAvailable(matplotplusplus)

add_library(BoidLib
        BoidLib/Options.h
        BoidLib/modelAOS/Boid.cpp
        BoidLib/modelAOS/Boid.h
        BoidLib/modelAOS/Simulator.cpp
        BoidLib/modelAOS/Simulator.h
        BoidLib/modelAOS/WindowManager.cpp
        BoidLib/modelAOS/WindowManager.h
        BoidLib/modelSOA/BoidsArray.h
        BoidLib/modelSOA/Simulator.cpp
        BoidLib/modelSOA/Simulator.h
        BoidLib/modelSOA/WindowManager.cpp
        BoidLib/modelSOA/WindowManager.h
        BoidLib/modelAOSOA/Boid.cpp
        BoidLib/modelAOSOA/Boid.h
        BoidLib/modelAOSOA/Simulator.cpp
        BoidLib/modelAOSOA/Simulator.h
        BoidLib/modelAOSOA/WindowManager.cpp
        BoidLib/modelAOSOA/WindowManager.h
        BoidLib/modelAOS/KDTree.h
        BoidLib/modelAOS/KDTree.cpp
)

target_link_libraries(BoidLib PUBLIC SFML::Graphics PUBLIC SFML::Window PUBLIC SFML::System PUBLIC matplot)

add_executable(Boids_DEMO demo.cpp)
add_executable(Boids_Benchmark benchmark.cpp)
add_executable(Boids_plotter plotter.cpp)

# Add the SEQUENTIAL define only if USE_SEQUENTIAL is ON
if(USE_PARALLEL)
    find_package(OpenMP REQUIRED)
    target_link_libraries(BoidLib PUBLIC OpenMP::OpenMP_CXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

target_link_libraries(Boids_DEMO PRIVATE BoidLib)
target_link_libraries(Boids_Benchmark PRIVATE BoidLib)
target_link_libraries(Boids_plotter PRIVATE matplot)