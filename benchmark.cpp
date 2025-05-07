//
// Created by viu on 04/04/2025.
//

#include <chrono>
#include <iostream>
#include <memory>
#include <numeric>
#include <omp.h>
#include <fstream>
#include <filesystem>

#include "BoidLib/modelAOS/Simulator.h"
#include "BoidLib/modelSOA/Simulator.h"
#include "BoidLib/modelAOSOA/Simulator.h"


static constexpr int EXPERIMENTS = 100;
static constexpr int ITERATIONS = 1000;

template<typename SimulatorType>
std::vector<long> performParallelExperiment(const BoidOptions options) {
    auto results = std::vector<long>(EXPERIMENTS);

    for (int j = 0; j < EXPERIMENTS; j++) {
        const auto simulator = std::make_unique<SimulatorType>(options);
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
        simulator->RunSimulation(&SimulatorType::NextStateParallel, ITERATIONS);
        auto end = std::chrono::high_resolution_clock::now();
        results[j] = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count();
    }

    return results;
}

template<typename SimulatorType>
std::vector<long> performSequentialExperiment(const BoidOptions options) {
    auto results = std::vector<long>(EXPERIMENTS);

    for (int j = 0; j < EXPERIMENTS; j++) {
        const auto simulator = std::make_unique<SimulatorType>(options);
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
        simulator->RunSimulation(&SimulatorType::NextStateSequential, ITERATIONS);
        auto end = std::chrono::high_resolution_clock::now();
        results[j] = std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count();
    }

    return results;
}

void writeResults(const std::filesystem::path& resultsPath, const std::vector<long>& results) {
    std::ofstream file {};
    file.open(resultsPath);
    for (const auto &result : results) {
        file << result << std::endl;
    }

    file.close();
}

int main() {
    // LAST 14 250 500 10
    // std::vector<int> threads = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};
    // std::vector<int> boids = {100, 250, 500, 1000, 2500, 5000, 10000};
    //std::vector<int> visualRange = {10, 25, 50, 100, 250, 500};
    //std::vector<int> maxV = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::vector<int> threads = {6, 12, 13};
    std::vector<int> boids = {100};
    std::vector<int> visualRange = {100};
    std::vector<int> maxV = {10};

    std::filesystem::path resultFolder = std::filesystem::current_path();
    resultFolder /= "results";
    create_directory(resultFolder);

    for (const int boid : boids) {
        for (const int range : visualRange) {
            BoidOptions options = {};
            options.BoidNum = boid;
            options.VisualRange = range;

            std::cout << "Running " << boid << " boids, "
                          << range << " visual range" << std::endl;
            // SEQUENTIAL EXPERIMENT
            auto soaResultsSeq = performSequentialExperiment<modelSOA::Simulator>(options);
            auto aosResultsSeq = performSequentialExperiment<modelAOS::Simulator>(options);
            std::cout << "SOA: " << static_cast<float>(std::accumulate(soaResultsSeq.begin(), soaResultsSeq.end(), 0l)) / EXPERIMENTS
                       << " microseconds" << std::endl;
            std::cout << "AOS: " << static_cast<float>(std::accumulate(aosResultsSeq.begin(), aosResultsSeq.end(), 0l)) / EXPERIMENTS
                    << " microseconds" << std::endl;
            writeResults(resultFolder / std::format("soa_seq_{}_{}.txt", boid, range), soaResultsSeq);
            writeResults(resultFolder / std::format("aos_seq_{}_{}.txt", boid, range), aosResultsSeq);

            for (const int max : maxV) {
                options.MaxV = max;
                std::cout << "Running " << boid << " boids, " << max << " maxV, "
                          << range << " visual range" << std::endl;
                auto aosoaResultsSeq = performSequentialExperiment<modelAOSOA::Simulator>(options);

                std::cout << "AOSOA: " << static_cast<float>(std::accumulate(aosoaResultsSeq.begin(), aosoaResultsSeq.end(), 0l)) /
                        EXPERIMENTS << " microseconds" << std::endl;

                writeResults(resultFolder / std::format("aosoa_seq_{}_{}_{}.txt", boid, range, max), aosoaResultsSeq);
            }

            // PARALLEL EXPERIMENT
            for (const int thread : threads) {
                omp_set_num_threads(thread);
                std::cout << "Running with " << thread << " threads" << std::endl;

                auto soaResultsParallel = performParallelExperiment<modelSOA::Simulator>(options);
                auto aosResultsParallel = performParallelExperiment<modelAOS::Simulator>(options);

                std::cout << "SOA: " << static_cast<float>(std::accumulate(soaResultsParallel.begin(), soaResultsParallel.end(), 0l)) / EXPERIMENTS
                        << " microseconds" << std::endl;
                std::cout << "AOS: " << static_cast<float>(std::accumulate(aosResultsParallel.begin(), aosResultsParallel.end(), 0l)) / EXPERIMENTS
                        << " microseconds" << std::endl;
                writeResults(resultFolder / std::format("soa_par_{}_{}_{}.txt", thread, boid, range), soaResultsParallel);
                writeResults(resultFolder / std::format("aos_par_{}_{}_{}.txt", thread, boid, range), aosResultsParallel);

                for (const int max : maxV) {
                    options.MaxV = max;
                    auto aosoaResultsParallel = performParallelExperiment<modelAOSOA::Simulator>(options);

                    std::cout << "AOSOA: " << static_cast<float>(std::accumulate(aosoaResultsParallel.begin(), aosoaResultsParallel.end(), 0l)) /
                            EXPERIMENTS << " microseconds" << std::endl;

                    writeResults(resultFolder / std::format("aosoa_par_{}_{}_{}_{}.txt", thread, boid, range, max), aosoaResultsParallel);
                }
            }
        }
    }
}
