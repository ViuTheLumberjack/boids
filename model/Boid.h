//
// Created by viu on 28/01/2025.
//

#ifndef BOID_H
#define BOID_H

struct BoidState {
    double xPosition;
    double yPosition;
    double xVelocity;
    double yVelocity;
};

class Boid {
    BoidState state;

    public:
    double getX() const { return state.xPosition; }
    double getY() const { return state.yPosition; }
    double getXVelocity() const { return state.xVelocity; }
    double getYVelocity() const { return state.yVelocity; }

    void setXVelocity(double const v) { state.xVelocity = v; }
    void setYVelocity(double const v) { state.yVelocity = v; }

    void move();

    Boid() = default;

    Boid(const double x, const double y, const double velocityX, const double velocityY) : state(x, y, velocityX, velocityY) {}
};

#endif //BOID_H
