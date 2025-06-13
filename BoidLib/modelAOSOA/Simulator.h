//
// Created by viu on 28/01/2025.
//


#ifndef SIMULATOR_AOSOA_H
#define SIMULATOR_AOSOA_H


#include <functional>

#include "Boid.h"
#include "../Options.h"

namespace modelAOSOA {
    class Simulator {
        BoidOptions options;
        Boid *boids;
        float *velocitiesX, *velocitiesY;

    public:
        int numBlocks;
        Simulator(const BoidOptions &options);

        void RunSimulation(void(Simulator::*f)(), int iterations);
        void NextStateParallel();
        void NextStateSequential();

        [[nodiscard]] const Boid* getState() const;

        ~Simulator() = default;
    };
}
#endif //SIMULATOR_AOSOA_H
