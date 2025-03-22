//
// Created by viu on 28/01/2025.
//

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <array>

#include "Boid.h"
#include "Options.h"

class Simulator {
    std::array<Boid, BoidOptions::BoidNum> boids = std::array<Boid, BoidOptions::BoidNum>();

public:
    Simulator();
    void NextState();

    const std::array<Boid, BoidOptions::BoidNum>& getState() const;

    ~Simulator() = default;
};

#endif //SIMULATOR_H
