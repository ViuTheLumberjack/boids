//
// Created by viu on 28/01/2025.
//

#ifndef BOID_AOSOA_H
#define BOID_AOSOA_H
#include "../Options.h"

namespace modelAOSOA {
    struct Boid {
        float *xPosition;
        float *yPosition;
        float *xVelocity;
        float *yVelocity;

        void move(int num);

        Boid() = default;

        Boid(const float *x, const float *y, const float *velocityX, const float *velocityY) {
            *xPosition = *x;
            *yPosition = *y;
            *xVelocity = *velocityX;
            *yVelocity = *velocityY;
        }
    };
}
#endif //BOID_H
