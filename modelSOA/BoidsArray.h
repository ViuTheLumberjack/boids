//
// Created by viu on 28/01/2025.
//

#ifndef BOIDSARRAY_H
#define BOIDSARRAY_H
#include <array>
#include "../Options.h"

namespace modelSOA {
    struct BoidsArray {
        std::array<double, BoidOptions::BoidNum> xPosition{};
        std::array<double, BoidOptions::BoidNum> yPosition{};
        std::array<double, BoidOptions::BoidNum> xVelocity{};
        std::array<double, BoidOptions::BoidNum> yVelocity{};
    };
}

#endif //BOIDSARRAY_H
