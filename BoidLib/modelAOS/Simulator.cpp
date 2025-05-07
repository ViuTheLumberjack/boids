//
// Created by viu on 28/01/2025.
//

#include "Simulator.h"
#include "../Options.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <omp.h>
#include <random>

using engine = std::mt19937;

namespace modelAOS {
    Simulator::Simulator(const BoidOptions options) : options(options) {
        boids = new Boid[options.BoidNum];
        velocitiesX = new float[options.BoidNum];
        velocitiesY = new float[options.BoidNum];
        
        std::random_device os_seed;
        const uint32_t seed = 42;

        engine generator(seed);
        std::uniform_real_distribution<float> positionXDistribution(ScreenOptions::LeftMargin,
                                                                    ScreenOptions::WindowWidth -
                                                                    ScreenOptions::RightMargin);
        std::uniform_real_distribution<float> positionYDistribution(ScreenOptions::TopMargin,
                                                                    ScreenOptions::WindowHeight -
                                                                    ScreenOptions::BottomMargin);
        std::uniform_real_distribution<float> velocityXDistribution(-options.MinSpeed, options.MinSpeed);
        std::uniform_real_distribution<float> velocityYDistribution(-options.MinSpeed, options.MinSpeed);

        for (int repetition = 0; repetition < options.BoidNum; ++repetition) {
            const Boid boid{
                (positionXDistribution(generator)), (positionYDistribution(generator)),
                (velocityXDistribution(generator)), (velocityYDistribution(generator))
            };

            boids[repetition] = boid;
        }
    }

    const Boid *Simulator::getState() const {
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
        // Copy the boids to a new array
        int active_levels = omp_get_active_level();
#pragma omp parallel shared(boids, options, velocitiesX, velocitiesY) if (active_levels < 1)
        {
            int i;

#pragma omp for private(i) schedule(dynamic)
            for (i = 0; i < options.BoidNum; ++i) {
                float xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0, neighboring_boids = 0, close_dx = 0,
                        close_dy = 0;

                float pos_x = boids[i].xPosition;
                float pos_y = boids[i].yPosition;
                float new_vx = boids[i].xVelocity;
                float new_vy = boids[i].yVelocity;

                for (int j = 0; j < options.BoidNum; ++j) {
                    if (j == i) continue;

                    float dx = boids[j].xPosition - pos_x;
                    float dy = boids[j].yPosition - pos_y;

                    // We are inside the Visual Range so we can see each other
                    if (std::abs(dx) < options.VisualRange && std::abs(dy) < options.VisualRange) {
                        float squared_distance = dx * dx + dy * dy;

                        if (squared_distance < options.ProtectedRange * options.ProtectedRange) {
                            close_dx += pos_x - boids[j].xPosition;
                            close_dy += pos_y - boids[j].yPosition;
                        } else if (squared_distance <= options.VisualRange * options.VisualRange) {
                            xpos_avg += boids[j].xPosition;
                            ypos_avg += boids[j].yPosition;
                            xvel_avg += boids[j].xVelocity;
                            yvel_avg += boids[j].yVelocity;

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
                             (xpos_avg - pos_x) * options.CenteringFactor +
                             (xvel_avg - new_vx) * options.MatchingFactor;

                    new_vy = new_vy +
                             (ypos_avg - pos_y) * options.CenteringFactor +
                             (yvel_avg - boids[i].yVelocity) * options.MatchingFactor;
                }

                new_vx = new_vx + (close_dx * options.AvoidFactor);
                new_vy = new_vy + (close_dy * options.AvoidFactor);

                // MANAGE TURN
                // (0, 0) is top left
                // if outside top margin:
                if (pos_y < ScreenOptions::TopMargin) {
                    new_vy += options.TurnFactor;
                }
                // if outside right margin:
                if (pos_x > ScreenOptions::WindowWidth - ScreenOptions::RightMargin) {
                    new_vx -= options.TurnFactor;
                }
                //if outside left margin:
                if (pos_x < ScreenOptions::LeftMargin) {
                    new_vx += options.TurnFactor;
                }
                //if outside bottom margin:
                if (pos_y > ScreenOptions::WindowHeight - ScreenOptions::BottomMargin) {
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
            for (i = 0; i < options.BoidNum; ++i) {
                boids[i].xVelocity = velocitiesX[i];
                boids[i].yVelocity = velocitiesY[i];
                boids[i].move();
            }
        }
    }

    void Simulator::NextStateSequential() {
        int i;
        for (i = 0; i < options.BoidNum; ++i) {
            float xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0, neighboring_boids = 0, close_dx = 0,
                    close_dy = 0;

            float pos_x = boids[i].xPosition;
            float pos_y = boids[i].yPosition;
            float new_vx = boids[i].xVelocity;
            float new_vy = boids[i].yVelocity;

            for (int j = 0; j < options.BoidNum; ++j) {
                if (j == i) continue;

                float dx = boids[j].xPosition - pos_x;
                float dy = boids[j].yPosition - pos_y;

                // We are inside the Visual Range so we can see each other
                if (std::abs(dx) < options.VisualRange && std::abs(dy) < options.VisualRange) {
                    float squared_distance = dx * dx + dy * dy;

                    if (squared_distance < options.ProtectedRange * options.ProtectedRange) {
                        close_dx += pos_x - boids[j].xPosition;
                        close_dy += pos_y - boids[j].yPosition;
                    } else if (squared_distance <= options.VisualRange * options.VisualRange) {
                        xpos_avg += boids[j].xPosition;
                        ypos_avg += boids[j].yPosition;
                        xvel_avg += boids[j].xVelocity;
                        yvel_avg += boids[j].yVelocity;

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
                         (xpos_avg - pos_x) * options.CenteringFactor +
                         (xvel_avg - new_vx) * options.MatchingFactor;

                new_vy = new_vy +
                         (ypos_avg - pos_y) * options.CenteringFactor +
                         (yvel_avg - boids[i].yVelocity) * options.MatchingFactor;
            }

            new_vx = new_vx + (close_dx * options.AvoidFactor);
            new_vy = new_vy + (close_dy * options.AvoidFactor);

            // MANAGE TURN
            // (0, 0) is top left
            // if outside top margin:
            if (pos_y < ScreenOptions::TopMargin) {
                new_vy += options.TurnFactor;
            }
            // if outside right margin:
            if (pos_x > ScreenOptions::WindowWidth - ScreenOptions::RightMargin) {
                new_vx -= options.TurnFactor;
            }
            //if outside left margin:
            if (pos_x < ScreenOptions::LeftMargin) {
                new_vx += options.TurnFactor;
            }
            //if outside bottom margin:
            if (pos_y > ScreenOptions::WindowHeight - ScreenOptions::BottomMargin) {
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

        for (i = 0; i < options.BoidNum; ++i) {
            boids[i].xVelocity = velocitiesX[i];
            boids[i].yVelocity = velocitiesY[i];
            boids[i].move();
        }
    }
}
