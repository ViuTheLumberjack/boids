//
// Created by viu on 28/01/2025.
//

#ifndef SIMULATOR_SOA_H
#define SIMULATOR_SOA_H

#include "BoidsArray.h"

namespace modelSOA {
    class Simulator {
        BoidsArray boids = BoidsArray();
        std::array<double, BoidOptions::BoidNum> velocitiesX{}, velocitiesY{};

    public:
        Simulator();

        void NextState();

        const BoidsArray &getState() const;

        ~Simulator() = default;
    };
}
#endif // SIMULATOR_SOA_H
