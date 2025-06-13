//
// Created by viu on 28/01/2025.
//

#include "Boid.h"
#include "../Options.h"

namespace modelAOSOA {
    void Boid::move(const int num) {
        for (int i = 0; i < num; ++i) {
            xPosition[i] = xPosition[i] + xVelocity[i] / ScreenOptions::FrameRate;
            yPosition[i] = yPosition[i] + yVelocity[i] / ScreenOptions::FrameRate;
        }
    }
}
