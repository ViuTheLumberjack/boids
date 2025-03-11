//
// Created by viu on 28/01/2025.
//

#ifndef BOID_H
#define BOID_H

#include "Options.h"

#if not PARALLEL

enum BoidBias { LEFT, RIGHT };

struct BoidState {
    double xPosition;
    double yPosition;
    double xVelocity;
    double yVelocity;
};

class Boid {
    BoidState state;
    BoidBias bias;
    double biasVal = BoidOptions::BiasVal;

    public:
    double getX() const { return state.xPosition; }
    double getY() const { return state.yPosition; }
    double getXVelocity() const { return state.xVelocity; }
    double getYVelocity() const { return state.yVelocity; }
    BoidBias getBias() const { return this->bias; }
    double getBiasVal() const { return this->biasVal; }

    void setXVelocity(double const v) { state.xVelocity = v; }
    void setYVelocity(double const v) { state.yVelocity = v; }
    void setBiasVal(double const v) { this->biasVal = v; }

    void move();

    Boid() = default;

    Boid(const double x, const double y, const double velocityX, const double velocityY, const double bias) : state(x, y, velocityX, velocityY), bias(static_cast<BoidBias>(bias)) {}
};

#else

#endif

#endif //BOID_H
