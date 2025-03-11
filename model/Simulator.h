//
// Created by viu on 28/01/2025.
//

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <vector>

#include "Boid.h"

class Simulator {
    std::vector<Boid> boids = std::vector<Boid>();

    public:
    Simulator();

    [[nodiscard]] std::vector<Boid> getState() const;
    void NextState();

    ~Simulator() = default;
};

#endif //SIMULATOR_H
