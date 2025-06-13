//
// Created by viu on 28/01/2025.
//

#include "Simulator.h"
#include "../Options.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <iostream>
#include <omp.h>
#include <random>

using engine = std::mt19937;

namespace modelAOSOA {
    Simulator::Simulator(const BoidOptions &options) : options(options) {
        numBlocks = (options.BoidNum + options.MaxV - 1) / options.MaxV;
        boids = new Boid[numBlocks];
        velocitiesX = new float[options.BoidNum];
        velocitiesY = new float[options.BoidNum];

        std::random_device os_seed;
        const uint32_t seed = 42;
        std::mt19937 generator(seed);
        std::uniform_real_distribution<float> positionXDistribution(ScreenOptions::LeftMargin,
                                                                    ScreenOptions::WindowWidth -
                                                                    ScreenOptions::RightMargin);
        std::uniform_real_distribution<float> positionYDistribution(ScreenOptions::TopMargin,
                                                                    ScreenOptions::WindowHeight -
                                                                    ScreenOptions::BottomMargin);
        std::uniform_real_distribution<float> velocityXDistribution(-options.MinSpeed, options.MinSpeed);
        std::uniform_real_distribution<float> velocityYDistribution(-options.MinSpeed, options.MinSpeed);

        for (int blockIdx = 0; blockIdx < numBlocks; ++blockIdx) {
            boids[blockIdx].xPosition = new float[options.MaxV];
            boids[blockIdx].yPosition = new float[options.MaxV];
            boids[blockIdx].xVelocity = new float[options.MaxV];
            boids[blockIdx].yVelocity = new float[options.MaxV];

            for (int localIdx = 0; localIdx  < options.MaxV; ++localIdx) {
                boids[blockIdx].xPosition[localIdx] = positionXDistribution(generator);
                boids[blockIdx].yPosition[localIdx] = positionYDistribution(generator);
                boids[blockIdx].xVelocity[localIdx] = velocityXDistribution(generator);
                boids[blockIdx].yVelocity[localIdx] = velocityYDistribution(generator);
            }
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
        int active_levels = omp_get_active_level();
#pragma omp parallel shared(boids, velocitiesX, velocitiesY) if (active_levels < 1)
        {
            int blockIdx = 0;
#pragma omp for private(blockIdx)
            for (blockIdx = 0; blockIdx < numBlocks; ++blockIdx) {
                for (int localIdx = 0; localIdx < options.MaxV; ++localIdx) {
                    const int i = blockIdx * options.MaxV + localIdx;
                    if (i >= options.BoidNum) continue;

                    const Boid &block = boids[blockIdx];
                    float xi = block.xPosition[localIdx];
                    float yi = block.yPosition[localIdx];
                    float vxi = block.xVelocity[localIdx];
                    float vyi = block.yVelocity[localIdx];

                    float xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0, neighboring_boids = 0, close_dx = 0,
                            close_dy = 0;
                    float new_vx = vxi;
                    float new_vy = vyi;

                    for (int otherBlockIdx = 0; otherBlockIdx < numBlocks; ++otherBlockIdx) {
                        const Boid &otherBlock = boids[otherBlockIdx];

                        int maxLocal = (otherBlockIdx == numBlocks - 1)
                                           ? (numBlocks % options.MaxV)
                                           : options.MaxV;
                        if (maxLocal == 0) maxLocal = options.MaxV;

                        for (int j = 0; j < maxLocal; ++j) {
                            int global_j = otherBlockIdx * options.MaxV + j;
                            if (global_j == i) continue;

                            float dx = otherBlock.xPosition[j] - xi;
                            float dy = otherBlock.yPosition[j] - yi;

                            if (std::abs(dx) < options.VisualRange && std::abs(dy) < options.VisualRange) {
                                float squared_distance = dx * dx + dy * dy;

                                if (squared_distance < options.ProtectedRange * options.ProtectedRange) {
                                    close_dx += xi - otherBlock.xPosition[j];
                                    close_dy += yi - otherBlock.yPosition[j];
                                } else if (squared_distance <= options.VisualRange * options.VisualRange) {
                                    xpos_avg += otherBlock.xPosition[j];
                                    ypos_avg += otherBlock.yPosition[j];
                                    xvel_avg += otherBlock.xVelocity[j];
                                    yvel_avg += otherBlock.yVelocity[j];
                                    neighboring_boids += 1;
                                }
                            }
                        }
                    }

                    if (neighboring_boids > 0) {
                        xpos_avg = xpos_avg / neighboring_boids;
                        ypos_avg = ypos_avg / neighboring_boids;
                        xvel_avg = xvel_avg / neighboring_boids;
                        yvel_avg = yvel_avg / neighboring_boids;

                        new_vx = new_vx + (xpos_avg - xi) * options.CenteringFactor + (xvel_avg - vxi) * options.
                                 MatchingFactor;
                        new_vy = new_vy + (ypos_avg - yi) * options.CenteringFactor + (yvel_avg - vyi) * options.
                                 MatchingFactor;
                    }

                    new_vx += close_dx * options.AvoidFactor;
                    new_vy += close_dy * options.AvoidFactor;

                    if (yi < ScreenOptions::TopMargin) new_vy += options.TurnFactor;
                    if (xi > ScreenOptions::WindowWidth - ScreenOptions::RightMargin) new_vx -= options.TurnFactor;
                    if (xi < ScreenOptions::LeftMargin) new_vx += options.TurnFactor;
                    if (yi > ScreenOptions::WindowHeight - ScreenOptions::BottomMargin) new_vy -= options.TurnFactor;

                    float speed = std::sqrt(new_vx * new_vx + new_vy * new_vy);
                    if (speed < options.MinSpeed) {
                        new_vx = new_vx / speed * options.MinSpeed;
                        new_vy = new_vy / speed * options.MinSpeed;
                    }
                    if (speed > options.MaxSpeed) {
                        new_vx = new_vx / speed * options.MaxSpeed;
                        new_vy = new_vy / speed * options.MaxSpeed;
                    }

                    // Update velocities
                    velocitiesX[i] = new_vx;
                    velocitiesY[i] = new_vy;
                }
            }

#pragma omp for private(blockIdx)
            for (blockIdx = 0; blockIdx < numBlocks; ++blockIdx) {
                Boid &block = boids[blockIdx];
                for (int localIdx = 0; localIdx < options.MaxV; ++localIdx) {
                    block.xVelocity[localIdx] = velocitiesX[blockIdx * options.MaxV + localIdx];
                    block.yVelocity[localIdx] = velocitiesY[blockIdx * options.MaxV + localIdx];
                }
                block.move(options.MaxV);
            }
        }
    }

    void Simulator::NextStateSequential() {
        for (int blockIdx = 0; blockIdx < numBlocks; ++blockIdx) {
            for (int localIdx = 0; localIdx < options.MaxV; ++localIdx) {
                const int i = blockIdx * options.MaxV + localIdx;
                if (i >= options.BoidNum) continue;

                const Boid &block = boids[blockIdx];
                const float xi = block.xPosition[localIdx];
                const float yi = block.yPosition[localIdx];
                const float vxi = block.xVelocity[localIdx];
                const float vyi = block.yVelocity[localIdx];

                float xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0, neighboring_boids = 0, close_dx = 0,
                        close_dy = 0;
                float new_vx = vxi;
                float new_vy = vyi;

                for (int otherBlockIdx = 0; otherBlockIdx < numBlocks; ++otherBlockIdx) {
                    const Boid &otherBlock = boids[otherBlockIdx];

                    int maxLocal = (otherBlockIdx == numBlocks - 1)
                                       ? (numBlocks % options.MaxV)
                                       : options.MaxV;
                    if (maxLocal == 0) maxLocal = options.MaxV;

                    for (int j = 0; j < maxLocal; ++j) {
                        if (const int global_j = otherBlockIdx * options.MaxV + j; global_j == i) continue;

                        float dx = otherBlock.xPosition[j] - xi;
                        float dy = otherBlock.yPosition[j] - yi;

                        if (std::abs(dx) < options.VisualRange && std::abs(dy) < options.VisualRange) {
                            float squared_distance = dx * dx + dy * dy;

                            if (squared_distance < options.ProtectedRange * options.ProtectedRange) {
                                close_dx += xi - otherBlock.xPosition[j];
                                close_dy += yi - otherBlock.yPosition[j];
                            } else if (squared_distance <= options.VisualRange * options.VisualRange) {
                                xpos_avg += otherBlock.xPosition[j];
                                ypos_avg += otherBlock.yPosition[j];
                                xvel_avg += otherBlock.xVelocity[j];
                                yvel_avg += otherBlock.yVelocity[j];
                                neighboring_boids += 1;
                            }
                        }
                    }
                }

                if (neighboring_boids > 0) {
                    xpos_avg = xpos_avg / neighboring_boids;
                    ypos_avg = ypos_avg / neighboring_boids;
                    xvel_avg = xvel_avg / neighboring_boids;
                    yvel_avg = yvel_avg / neighboring_boids;

                    new_vx = new_vx + (xpos_avg - xi) * options.CenteringFactor + (xvel_avg - vxi) * options.
                             MatchingFactor;
                    new_vy = new_vy + (ypos_avg - yi) * options.CenteringFactor + (yvel_avg - vyi) * options.
                             MatchingFactor;
                }

                new_vx += close_dx * options.AvoidFactor;
                new_vy += close_dy * options.AvoidFactor;

                if (yi < ScreenOptions::TopMargin) new_vy += options.TurnFactor;
                if (xi > ScreenOptions::WindowWidth - ScreenOptions::RightMargin) new_vx -= options.TurnFactor;
                if (xi < ScreenOptions::LeftMargin) new_vx += options.TurnFactor;
                if (yi > ScreenOptions::WindowHeight - ScreenOptions::BottomMargin) new_vy -= options.TurnFactor;

                float speed = std::sqrt(new_vx * new_vx + new_vy * new_vy);
                if (speed < options.MinSpeed) {
                    new_vx = new_vx / speed * options.MinSpeed;
                    new_vy = new_vy / speed * options.MinSpeed;
                }
                if (speed > options.MaxSpeed) {
                    new_vx = new_vx / speed * options.MaxSpeed;
                    new_vy = new_vy / speed * options.MaxSpeed;
                }

                // Update velocities
                velocitiesX[i] = new_vx;
                velocitiesY[i] = new_vy;
            }
        }

        for (int blockIdx = 0; blockIdx < numBlocks; ++blockIdx) {
            Boid &block = boids[blockIdx];
            for (int localIdx = 0; localIdx < options.MaxV; ++localIdx) {
                block.xVelocity[localIdx] = velocitiesX[blockIdx * options.MaxV + localIdx];
                block.yVelocity[localIdx] = velocitiesY[blockIdx * options.MaxV + localIdx];
            }
            block.move(options.MaxV);
        }
    }
}
