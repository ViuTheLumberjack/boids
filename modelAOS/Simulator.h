//
// Created by viu on 28/01/2025.
//


#ifndef SIMULATOR_AOS_H
#define SIMULATOR_AOS_H

#include <array>

#include "Boid.h"
#include "../Options.h"

namespace modelAOS {
    class Simulator {
        std::array<Boid, BoidOptions::BoidNum> boids = std::array<Boid, BoidOptions::BoidNum>();
        std::array<double, BoidOptions::BoidNum> velocitiesX{}, velocitiesY{};

    public:
        Simulator();

        void NextState();

        const std::array<Boid, BoidOptions::BoidNum> &getState() const;

        ~Simulator() = default;
    };
}
#endif //SIMULATOR_AOS_H
