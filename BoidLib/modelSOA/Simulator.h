//
// Created by viu on 28/01/2025.
//

#ifndef SIMULATOR_SOA_H
#define SIMULATOR_SOA_H

#include <functional>

#include "BoidsArray.h"
#include "../Options.h"

namespace modelSOA {
    class Simulator {
        BoidOptions options;
        BoidsArray boids = BoidsArray();
        float *velocitiesX, *velocitiesY;

    public:
        Simulator(const BoidOptions &options);

        void RunSimulation(void(Simulator::*f)(), int iterations);
        void NextStateParallel();
        void NextStateSequential();

        [[nodiscard]] const BoidsArray &getState() const;

        ~Simulator() = default;
    };
}
#endif // SIMULATOR_SOA_H
