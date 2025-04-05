//
// Created by viu on 04/04/2025.
//

#include <chrono>
#include <iostream>
#include <memory>

#include "modelAOS/Simulator.h"
#include "modelSOA/Simulator.cpp"

int main()
{
    static constexpr int ITERATIONS = 10000;
    const auto soaSimulator = std::make_unique<modelSOA::Simulator>();

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i<ITERATIONS; i++)
    {
        soaSimulator->NextState();

        i++;
    }

    const auto soaDuration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
    std::cout << "SOA elapsed: " << soaDuration << std::endl;

    const auto aosSimulator = std::make_unique<modelAOS::Simulator>();

    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i<ITERATIONS; i++)
    {
        soaSimulator->NextState();

        i++;
    }

    const auto aosDuration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
    std::cout << "AOS elapsed: " << aosDuration << std::endl;

    std::cout << "Speedup: " << static_cast<double>(aosDuration.count()) / static_cast<double>(soaDuration.count()) << std::endl;
}
