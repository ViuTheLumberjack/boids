//
// Created by viu on 28/01/2025.
//

#include "Boid.h"
#include "Options.h"

void Boid::move() {
    state.xPosition = state.xPosition + state.xVelocity / ScreenOptions::FrameRate;
    state.yPosition = state.yPosition + state.yVelocity / ScreenOptions::FrameRate;
}