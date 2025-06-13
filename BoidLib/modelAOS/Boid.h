//
// Created by viu on 28/01/2025.
//

#ifndef BOID_H
#define BOID_H

namespace modelAOS {
    struct Boid {
        float xPosition;
        float yPosition;
        float xVelocity;
        float yVelocity;

        void move();

        Boid() = default;

        Boid(const float x, const float y, const float velocityX, const float velocityY) : xPosition(x),
            yPosition(y), xVelocity(velocityX), yVelocity(velocityY) {
        }
    };
}
#endif //BOID_H
