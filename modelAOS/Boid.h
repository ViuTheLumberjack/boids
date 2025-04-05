//
// Created by viu on 28/01/2025.
//

#ifndef BOID_H
#define BOID_H

namespace modelAOS {
    struct Boid {
        double xPosition;
        double yPosition;
        double xVelocity;
        double yVelocity;

        void move();

        Boid() = default;

        Boid(const double x, const double y, const double velocityX, const double velocityY) : xPosition(x),
            yPosition(y), xVelocity(velocityX), yVelocity(velocityY) {
        }
    };
}
#endif //BOID_H
