//
// Created by viu on 28/01/2025.
//

#include "Simulator.h"
#include "Options.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <random>

using engine = std::mt19937;

Simulator::Simulator() {
    std::random_device os_seed;
    const uint32_t seed = os_seed();

    engine generator(seed);
    std::uniform_real_distribution positionXDistribution(ScreenOptions::LeftMargin, ScreenOptions::WindowWidth - ScreenOptions::RightMargin);
    std::uniform_real_distribution positionYDistribution(ScreenOptions::TopMargin, ScreenOptions::WindowHeight - ScreenOptions::BottomMargin);
    std::uniform_real_distribution velocityXDistribution( -BoidOptions::MinSpeed, BoidOptions::MinSpeed);
    std::uniform_real_distribution velocityYDistribution( -BoidOptions::MinSpeed, BoidOptions::MinSpeed);

    for(int repetition = 0; repetition < BoidOptions::BoidNum; ++repetition) {
        const Boid boid {(positionXDistribution(generator)), (positionYDistribution(generator)),
            (velocityXDistribution(generator)), (velocityYDistribution(generator))};

        boids[repetition] = boid;
    }
}

const std::array<Boid, BoidOptions::BoidNum>& Simulator::getState() const {
    return this->boids;
}

void Simulator::NextState() {
    std::array<double, BoidOptions::BoidNum> velocitiesX{}, velocitiesY{};

#pragma omp parallel shared(boids, velocitiesX, velocitiesY)
    {
        int i;

#pragma omp for private(i)
        for(i = 0; i < boids.size(); ++i) {
            double xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0, neighboring_boids = 0, close_dx = 0, close_dy = 0;

            double new_vx = boids[i].getXVelocity();
            double new_vy = boids[i].getYVelocity();

            for(int j = 0; j < boids.size(); ++j) {
                if(j == i) continue;

                double dx = boids[j].getX() - boids[i].getX();
                double dy = boids[j].getY() - boids[i].getY();

                // We are inside the Visual Range so we can see each other
                if(std::abs(dx) < BoidOptions::VisualRange && std::abs(dy) < BoidOptions::VisualRange) {
                    double squared_distance = dx * dx + dy * dy;

                    if (squared_distance < BoidOptions::ProtectedRange * BoidOptions::ProtectedRange) {
                        close_dx += boids[i].getX() - boids[j].getX();
                        close_dy += boids[i].getY() - boids[j].getY();
                    } else if(squared_distance <= BoidOptions::VisualRange * BoidOptions::VisualRange) {
                        xpos_avg += boids[j].getX();
                        ypos_avg += boids[j].getY();
                        xvel_avg += boids[j].getXVelocity();
                        yvel_avg += boids[j].getYVelocity();

                        neighboring_boids += 1;
                    }
                }
            }
            // FLOCKING BEHAVIOUR
            if (neighboring_boids > 0) {
                xpos_avg = xpos_avg/neighboring_boids;
                ypos_avg = ypos_avg/neighboring_boids;
                xvel_avg = xvel_avg/neighboring_boids;
                yvel_avg = yvel_avg/neighboring_boids;

                new_vx = new_vx +
                                   (xpos_avg - boids[i].getX()) * BoidOptions::CenteringFactor +
                                   (xvel_avg - new_vx) * BoidOptions::MatchingFactor;

                new_vy = new_vy +
                                   (ypos_avg - boids[i].getY()) * BoidOptions::CenteringFactor +
                                   (yvel_avg - boids[i].getYVelocity()) * BoidOptions::MatchingFactor;

            }

            new_vx = new_vx + (close_dx * BoidOptions::AvoidFactor);
            new_vy = new_vy + (close_dy * BoidOptions::AvoidFactor);

            // MANAGE TURN
            // (0, 0) is top left
            // if outside top margin:
            if(boids[i].getY() < ScreenOptions::TopMargin) {
                new_vy += BoidOptions::TurnFactor;
            }
            // if outside right margin:
            if(boids[i].getX() > ScreenOptions::WindowWidth - ScreenOptions::RightMargin) {
                new_vx -= BoidOptions::TurnFactor;
            }
            //if outside left margin:
            if(boids[i].getX() < ScreenOptions::LeftMargin) {
                new_vx += BoidOptions::TurnFactor;
            }
            //if outside bottom margin:
            if(boids[i].getY() > ScreenOptions::WindowHeight - ScreenOptions::BottomMargin) {
                new_vy -= BoidOptions::TurnFactor;
            }

            // I AM SPEED
            double speed = std::sqrt(new_vx * new_vx + new_vy * new_vy);

            if(speed < BoidOptions::MinSpeed) {
                new_vx = new_vx / speed * BoidOptions::MinSpeed;
                new_vy = new_vy / speed * BoidOptions::MinSpeed;
            }
            if(speed > BoidOptions::MaxSpeed) {
                new_vx = new_vx / speed * BoidOptions::MaxSpeed;
                new_vy = new_vy / speed * BoidOptions::MaxSpeed;
            }

            velocitiesX[i] = new_vx;
            velocitiesY[i] = new_vy;
        }
#pragma omp barrier
#pragma omp for private(i)
        for(i = 0; i < boids.size(); ++i) {
            boids[i].setXVelocity(velocitiesX[i]);
            boids[i].setYVelocity(velocitiesY[i]);
            boids[i].move();
        }
    }
}
