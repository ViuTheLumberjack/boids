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

    Simulator::Simulator(const BoidOptions &options) : options(options) {
        boids.xPosition = new float[options.BoidNum];
        boids.yPosition = new float[options.BoidNum];
        boids.xVelocity = new float[options.BoidNum];
        boids.yVelocity = new float[options.BoidNum];
        velocitiesX = new float[options.BoidNum];
        velocitiesY = new float[options.BoidNum];

        std::random_device os_seed;
        const uint32_t seed = 42;

        engine generator(seed);
        std::uniform_real_distribution positionXDistribution(ScreenOptions::LeftMargin,
                                                             ScreenOptions::WindowWidth - ScreenOptions::RightMargin);
        std::uniform_real_distribution positionYDistribution(ScreenOptions::TopMargin,
                                                             ScreenOptions::WindowHeight - ScreenOptions::BottomMargin);
        std::uniform_real_distribution velocityXDistribution(-options.MinSpeed, options.MinSpeed);
        std::uniform_real_distribution velocityYDistribution(-options.MinSpeed, options.MinSpeed);


        for (int i = 0; i < options.BoidNum; ++i) {
            boids.xPosition[i] = positionXDistribution(generator);
            boids.yPosition[i] = positionYDistribution(generator);
            boids.xVelocity[i] = velocityXDistribution(generator);
            boids.yVelocity[i] = velocityYDistribution(generator);
        }
    }

    const BoidsArray & Simulator::getState() const {
        return this->boids;
    }

    void Simulator::RunSimulation(void(Simulator::*f)(), const int iterations) {
        // Copy the boids to a new array
        velocitiesX = new float[options.BoidNum];
        velocitiesY = new float[options.BoidNum];
// #pragma omp parallel default(none) shared(boids, options, velocitiesX, velocitiesY, f) firstprivate(iterations) if (f != &Simulator::NextStateSequential)
        {
            for (int i = 0; i < iterations; ++i) {
                (this->*f)();
            }
        }
    }

    void Simulator::NextStateParallel() {
        int active_levels = omp_get_active_level();
#pragma omp parallel shared(options, boids, velocitiesX, velocitiesY) if (active_levels < 1)
        {
            int i;

#pragma omp for private(i)
            for (i = 0; i < options.BoidNum; ++i) {
                float xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0, neighboring_boids = 0, close_dx = 0,
                        close_dy = 0;

                float new_vx = boids.xVelocity[i];
                float new_vy = boids.yVelocity[i];

                for (int j = 0; j < options.BoidNum; ++j) {
                    if (j == i) continue;

                    float dx = boids.xPosition[j] - boids.xPosition[i];
                    float dy = boids.yPosition[j] - boids.yPosition[i];

                    // We are inside the Visual Range so we can see each other
                    if (std::abs(dx) < options.VisualRange && std::abs(dy) < options.VisualRange) {
                        float squared_distance = dx * dx + dy * dy;

                        if (squared_distance < options.ProtectedRange * options.ProtectedRange) {
                            close_dx += boids.xPosition[i] - boids.xPosition[j];
                            close_dy += boids.yPosition[i] - boids.yPosition[j];
                        } else if (squared_distance <= options.VisualRange * options.VisualRange) {
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
                             (xpos_avg - boids.xPosition[i]) * options.CenteringFactor +
                             (xvel_avg - new_vx) * options.MatchingFactor;

                    new_vy = new_vy +
                             (ypos_avg - boids.yPosition[i]) * options.CenteringFactor +
                             (yvel_avg - new_vy) * options.MatchingFactor;
                }

                new_vx = new_vx + (close_dx * options.AvoidFactor);
                new_vy = new_vy + (close_dy * options.AvoidFactor);

                // MANAGE TURN
                // (0, 0) is top left
                // if outside top margin:
                if (boids.yPosition[i] < ScreenOptions::TopMargin) {
                    new_vy += options.TurnFactor;
                }
                // if outside right margin:
                if (boids.xPosition[i] > ScreenOptions::WindowWidth - ScreenOptions::RightMargin) {
                    new_vx -= options.TurnFactor;
                }
                //if outside left margin:
                if (boids.xPosition[i] < ScreenOptions::LeftMargin) {
                    new_vx += options.TurnFactor;
                }
                //if outside bottom margin:
                if (boids.yPosition[i] > ScreenOptions::WindowHeight - ScreenOptions::BottomMargin) {
                    new_vy -= options.TurnFactor;
                }

                // I AM SPEED
                float speed = std::sqrt(new_vx * new_vx + new_vy * new_vy);

                if (speed < options.MinSpeed) {
                    new_vx = new_vx / speed * options.MinSpeed;
                    new_vy = new_vy / speed * options.MinSpeed;
                }
                if (speed > options.MaxSpeed) {
                    new_vx = new_vx / speed * options.MaxSpeed;
                    new_vy = new_vy / speed * options.MaxSpeed;
                }

                velocitiesX[i] = new_vx;
                velocitiesY[i] = new_vy;
            }

#pragma omp for private(i)
            for (i = 0; i < options.BoidNum; i++) {
                boids.xPosition[i] = boids.xPosition[i] + velocitiesX[i] / ScreenOptions::FrameRate;
                boids.yPosition[i] = boids.yPosition[i] + velocitiesY[i] / ScreenOptions::FrameRate;
                boids.xVelocity[i] = velocitiesX[i];
                boids.yVelocity[i] = velocitiesY[i];
            }
        }
    }

    void Simulator::NextStateSequential() {
        {
            int i;

            for (i = 0; i < options.BoidNum; ++i) {
                float xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0, neighboring_boids = 0, close_dx = 0,
                        close_dy = 0;

                float new_vx = boids.xVelocity[i];
                float new_vy = boids.yVelocity[i];

                for (int j = 0; j < options.BoidNum; ++j) {
                    if (j == i) continue;

                    float dx = boids.xPosition[j] - boids.xPosition[i];
                    float dy = boids.yPosition[j] - boids.yPosition[i];

                    // We are inside the Visual Range so we can see each other
                    if (std::abs(dx) < options.VisualRange && std::abs(dy) < options.VisualRange) {
                        float squared_distance = dx * dx + dy * dy;

                        if (squared_distance < options.ProtectedRange * options.ProtectedRange) {
                            close_dx += boids.xPosition[i] - boids.xPosition[j];
                            close_dy += boids.yPosition[i] - boids.yPosition[j];
                        } else if (squared_distance <= options.VisualRange * options.VisualRange) {
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
                             (xpos_avg - boids.xPosition[i]) * options.CenteringFactor +
                             (xvel_avg - new_vx) * options.MatchingFactor;

                    new_vy = new_vy +
                             (ypos_avg - boids.yPosition[i]) * options.CenteringFactor +
                             (yvel_avg - new_vy) * options.MatchingFactor;
                }

                new_vx = new_vx + (close_dx * options.AvoidFactor);
                new_vy = new_vy + (close_dy * options.AvoidFactor);

                // MANAGE TURN
                // (0, 0) is top left
                // if outside top margin:
                if (boids.yPosition[i] < ScreenOptions::TopMargin) {
                    new_vy += options.TurnFactor;
                }
                // if outside right margin:
                if (boids.xPosition[i] > ScreenOptions::WindowWidth - ScreenOptions::RightMargin) {
                    new_vx -= options.TurnFactor;
                }
                //if outside left margin:
                if (boids.xPosition[i] < ScreenOptions::LeftMargin) {
                    new_vx += options.TurnFactor;
                }
                //if outside bottom margin:
                if (boids.yPosition[i] > ScreenOptions::WindowHeight - ScreenOptions::BottomMargin) {
                    new_vy -= options.TurnFactor;
                }

                // I AM SPEED
                float speed = std::sqrt(new_vx * new_vx + new_vy * new_vy);

                if (speed < options.MinSpeed) {
                    new_vx = new_vx / speed * options.MinSpeed;
                    new_vy = new_vy / speed * options.MinSpeed;
                }
                if (speed > options.MaxSpeed) {
                    new_vx = new_vx / speed * options.MaxSpeed;
                    new_vy = new_vy / speed * options.MaxSpeed;
                }

                velocitiesX[i] = new_vx;
                velocitiesY[i] = new_vy;
            }

            for (i = 0; i < options.BoidNum; i++) {
                boids.xPosition[i] = boids.xPosition[i] + velocitiesX[i] / ScreenOptions::FrameRate;
                boids.yPosition[i] = boids.yPosition[i] + velocitiesY[i] / ScreenOptions::FrameRate;
                boids.xVelocity[i] = velocitiesX[i];
                boids.yVelocity[i] = velocitiesY[i];
            }
        }
    }
}
