//
// Created by viu on 28/01/2025.
//

#include "Simulator.h"

#include "../Options.h"
#include <algorithm>
#include <array>
#include <random>
#include <iostream>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace modelSOA {
    using engine = std::mt19937;

    Simulator::Simulator() {
        std::random_device os_seed;
        const uint32_t seed = 42;

        engine generator(seed);
        std::uniform_real_distribution positionXDistribution(ScreenOptions::LeftMargin,
                                                             ScreenOptions::WindowWidth - ScreenOptions::RightMargin);
        std::uniform_real_distribution positionYDistribution(ScreenOptions::TopMargin,
                                                             ScreenOptions::WindowHeight - ScreenOptions::BottomMargin);
        std::uniform_real_distribution velocityXDistribution(-BoidOptions::MinSpeed, BoidOptions::MinSpeed);
        std::uniform_real_distribution velocityYDistribution(-BoidOptions::MinSpeed, BoidOptions::MinSpeed);


        for (int i = 0; i < BoidOptions::BoidNum; ++i) {
            boids.xPosition[i] = positionXDistribution(generator);
            boids.yPosition[i] = positionYDistribution(generator);
            boids.xVelocity[i] = velocityXDistribution(generator);
            boids.yVelocity[i] = velocityYDistribution(generator);
        }
    }

    const BoidsArray &Simulator::getState() const {
        return this->boids;
    }

    void Simulator::NextState() {
#pragma omp parallel shared(boids, velocitiesX, velocitiesY)
        {
            int i;

#pragma omp for private(i)
            for (i = 0; i < BoidOptions::BoidNum; ++i) {
                double xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0, neighboring_boids = 0, close_dx = 0,
                        close_dy = 0;

                double new_vx = boids.xVelocity[i];
                double new_vy = boids.yVelocity[i];

                for (int j = 0; j < BoidOptions::BoidNum; ++j) {
                    if (j == i) continue;

                    double dx = boids.xPosition[j] - boids.xPosition[i];
                    double dy = boids.yPosition[j] - boids.yPosition[i];

                    // We are inside the Visual Range so we can see each other
                    if (std::abs(dx) < BoidOptions::VisualRange && std::abs(dy) < BoidOptions::VisualRange) {
                        double squared_distance = dx * dx + dy * dy;

                        if (squared_distance < BoidOptions::ProtectedRange * BoidOptions::ProtectedRange) {
                            close_dx += boids.xPosition[i] - boids.xPosition[j];
                            close_dy += boids.yPosition[i] - boids.yPosition[j];
                        } else if (squared_distance <= BoidOptions::VisualRange * BoidOptions::VisualRange) {
                            xpos_avg += boids.xPosition[j];
                            ypos_avg += boids.yPosition[j];
                            xvel_avg += boids.xVelocity[j];
                            yvel_avg += boids.yVelocity[j];

                            neighboring_boids += 1;
                        }
                    }
                }
                // FLOCKING BEHAVIOUR
                if (neighboring_boids > 0) {
                    xpos_avg = xpos_avg / neighboring_boids;
                    ypos_avg = ypos_avg / neighboring_boids;
                    xvel_avg = xvel_avg / neighboring_boids;
                    yvel_avg = yvel_avg / neighboring_boids;

                    new_vx = new_vx +
                             (xpos_avg - boids.xPosition[i]) * BoidOptions::CenteringFactor +
                             (xvel_avg - new_vx) * BoidOptions::MatchingFactor;

                    new_vy = new_vy +
                             (ypos_avg - boids.yPosition[i]) * BoidOptions::CenteringFactor +
                             (yvel_avg - new_vy) * BoidOptions::MatchingFactor;
                }

                new_vx = new_vx + (close_dx * BoidOptions::AvoidFactor);
                new_vy = new_vy + (close_dy * BoidOptions::AvoidFactor);

                // MANAGE TURN
                // (0, 0) is top left
                // if outside top margin:
                if (boids.yPosition[i] < ScreenOptions::TopMargin) {
                    new_vy += BoidOptions::TurnFactor;
                }
                // if outside right margin:
                if (boids.xPosition[i] > ScreenOptions::WindowWidth - ScreenOptions::RightMargin) {
                    new_vx -= BoidOptions::TurnFactor;
                }
                //if outside left margin:
                if (boids.xPosition[i] < ScreenOptions::LeftMargin) {
                    new_vx += BoidOptions::TurnFactor;
                }
                //if outside bottom margin:
                if (boids.yPosition[i] > ScreenOptions::WindowHeight - ScreenOptions::BottomMargin) {
                    new_vy -= BoidOptions::TurnFactor;
                }

                // I AM SPEED
                double speed = std::sqrt(new_vx * new_vx + new_vy * new_vy);

                if (speed < BoidOptions::MinSpeed) {
                    new_vx = new_vx / speed * BoidOptions::MinSpeed;
                    new_vy = new_vy / speed * BoidOptions::MinSpeed;
                }
                if (speed > BoidOptions::MaxSpeed) {
                    new_vx = new_vx / speed * BoidOptions::MaxSpeed;
                    new_vy = new_vy / speed * BoidOptions::MaxSpeed;
                }

                velocitiesX[i] = new_vx;
                velocitiesY[i] = new_vy;
            }
#pragma omp barrier
#pragma omp for private(i)
            for (i = 0; i < BoidOptions::BoidNum; i++) {
                boids.xPosition[i] = boids.xPosition[i] + velocitiesX[i] / ScreenOptions::FrameRate;
                boids.yPosition[i] = boids.yPosition[i] + velocitiesY[i] / ScreenOptions::FrameRate;
            }
        }

        boids.xVelocity = velocitiesX;
        boids.yVelocity = velocitiesY;
    }
}
