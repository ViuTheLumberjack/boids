//
// Created by viu on 28/01/2025.
//


#ifndef SIMULATOR_AOS_H
#define SIMULATOR_AOS_H

#include <array>
#include <functional>

#include "Boid.h"
#include "../Options.h"

namespace modelAOS {
    class Simulator {
        BoidOptions options;
        Boid *boids;
        float *velocitiesX, *velocitiesY;

    public:
        explicit Simulator(BoidOptions options);

        void RunSimulation(void(Simulator::*f)(), int iterations);
        void NextStateParallel();
        void NextStateSequential();

        [[nodiscard]] const Boid* getState() const;

        ~Simulator() = default;
    };
}
#endif //SIMULATOR_AOS_H
