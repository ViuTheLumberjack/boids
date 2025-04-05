//
// Created by viu on 28/01/2025.
//

#include "Boid.h"
#include "../Options.h"

namespace modelAOS {
    void Boid::move() {
        xPosition = xPosition + xVelocity / ScreenOptions::FrameRate;
        yPosition = yPosition + yVelocity / ScreenOptions::FrameRate;
    }
}
