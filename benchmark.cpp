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
#include <vector>

#include "BoidLib/modelAOS/Simulator.h"
#include "BoidLib/modelSOA/Simulator.h"
#include "BoidLib/modelAOSOA/Simulator.h"

static constexpr int EXPERIMENTS = 10;
static constexpr int ITERATIONS = 1000;

template <typename SimulatorType>
std::vector<long> performExperiment(void (modelAOS::Simulator::*f)(), const BoidOptions options)
{
    auto results = std::vector<long>(EXPERIMENTS);

    for (int j = 0; j < EXPERIMENTS; j++)
    {
        const auto simulator = std::make_unique<SimulatorType>(options);
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
        simulator->RunSimulation(f, ITERATIONS);
        auto end = std::chrono::high_resolution_clock::now();
        results[j] = std::chrono::duration_cast<std::chrono::microseconds>(
                         end - start)
                         .count();
    }

    return results;
}

void writeResults(const std::filesystem::path &resultsPath, const std::vector<long> &results)
{
    std::ofstream file{};
    file.open(resultsPath);
    for (const auto &result : results)
    {
        file << result << std::endl;
    }

    file.close();
}

int main()
{
    // LAST 14 250 500 10
    int start_threads = 1;
    int end_threads = 50;
    std::vector<int> boids = {100, 250, 1000, 2500, 5000};
    std::vector<int> visualRange = {100};
    // std::vector<int> maxV = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    // std::vector<int> threads = {13};
    // std::vector<int> boids = {1000};
    // std::vector<int> visualRange = {10};
    std::vector<int> maxV = {100};

    std::filesystem::path baseResultFolder = std::filesystem::current_path();

    for (const int boid : boids)
    {
        for (const int range : visualRange)
        {
            BoidOptions options = {};
            options.BoidNum = boid;
            options.VisualRange = range;

            std::cout << "Running " << boid << " boids, "
                      << range << " visual range" << std::endl;
            // SEQUENTIAL EXPERIMENTS
            auto aosResultsSeq = performExperiment<modelAOS::Simulator>(&modelAOS::Simulator::NextStateSequential, options);
            std::cout << "AOS: " << static_cast<float>(std::accumulate(aosResultsSeq.begin(), aosResultsSeq.end(), 0l)) / EXPERIMENTS
                      << " microseconds" << std::endl;
            auto resultFolder = baseResultFolder / "results_seq";
            if (!std::filesystem::exists(resultFolder))
            {
                std::filesystem::create_directories(resultFolder);
            }
            writeResults(resultFolder / std::format("aos_seq_{}_{}.txt", boid, range), aosResultsSeq);

            auto aosResultsSeqKD = performExperiment<modelAOS::Simulator>(&modelAOS::Simulator::NextStateSequentialKD, options);
            std::cout << "AOS: " << static_cast<float>(std::accumulate(aosResultsSeqKD.begin(), aosResultsSeqKD.end(), 0l)) / EXPERIMENTS
                      << " microseconds" << std::endl;
            resultFolder = baseResultFolder / "results_seq_kd";
            if (!std::filesystem::exists(resultFolder))
            {
                std::filesystem::create_directories(resultFolder);
            }
            writeResults(resultFolder / std::format("aos_seq_{}_{}.txt", boid, range), aosResultsSeqKD);

            // PARALLEL EXPERIMENT
            for (int thread = start_threads; thread <= end_threads; thread++)
            {
                omp_set_num_threads(thread);
                std::cout << "Running with " << thread << " threads" << std::endl;

                auto aosParallelBarrier = performExperiment<modelAOS::Simulator>(&modelAOS::Simulator::NextStateParallelBarrier, options);
                std::cout << "AOS: " << static_cast<float>(std::accumulate(aosParallelBarrier.begin(), aosParallelBarrier.end(), 0l)) / EXPERIMENTS
                          << " microseconds" << std::endl;
                resultFolder = baseResultFolder / "results_par_barrier";
                if (!std::filesystem::exists(resultFolder))
                {
                    std::filesystem::create_directories(resultFolder);
                }
                writeResults(resultFolder / std::format("aos_par_{}_{}.txt", boid, range), aosResultsSeqKD);

                auto aosParallelNoBarrier = performExperiment<modelAOS::Simulator>(&modelAOS::Simulator::NextStateParallelNoBarrier, options);
                std::cout << "AOS: " << static_cast<float>(std::accumulate(aosParallelNoBarrier.begin(), aosParallelNoBarrier.end(), 0l)) / EXPERIMENTS
                          << " microseconds" << std::endl;
                resultFolder = baseResultFolder / "results_par_no_barrier";
                if (!std::filesystem::exists(resultFolder))
                {
                    std::filesystem::create_directories(resultFolder);
                }
                writeResults(resultFolder / std::format("aos_par_{}_{}_{}.txt", thread, boid, range), aosParallelNoBarrier);

                auto aosParallelKDBarrier = performExperiment<modelAOS::Simulator>(&modelAOS::Simulator::NextStateParallelKDBarrier, options);

                std::cout << "AOS: " << static_cast<float>(std::accumulate(aosParallelKDBarrier.begin(), aosParallelKDBarrier.end(), 0l)) / EXPERIMENTS
                          << " microseconds" << std::endl;
                resultFolder = baseResultFolder / "results_par_kd_barrier";
                if (!std::filesystem::exists(resultFolder))
                {
                    std::filesystem::create_directories(resultFolder);
                }
                writeResults(resultFolder / std::format("aos_par_{}_{}_{}.txt", thread, boid, range), aosParallelKDBarrier);

                auto aosParallelKDNoBarrier = performExperiment<modelAOS::Simulator>(&modelAOS::Simulator::NextStateParallelKDNoBarrier, options);
                std::cout << "AOS: " << static_cast<float>(std::accumulate(aosParallelKDNoBarrier.begin(), aosParallelKDNoBarrier.end(), 0l)) / EXPERIMENTS
                          << " microseconds" << std::endl;
                resultFolder = baseResultFolder / "results_par_kd_no_barrier";
                if (!std::filesystem::exists(resultFolder))
                {
                    std::filesystem::create_directories(resultFolder);
                }
                writeResults(resultFolder / std::format("aos_par_{}_{}_{}.txt", thread, boid, range), aosParallelKDNoBarrier);
            }
        }
    }
}
