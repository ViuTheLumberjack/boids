//
// Created by viu on 28/01/2025.
//

#include "Simulator.h"
#include "Options.h"

#include <algorithm>
#include <iostream>
#include <random>

using engine = std::mt19937;

Simulator::Simulator() {
    std::random_device os_seed;
    const uint32_t seed = os_seed();

    engine generator(seed);
    std::uniform_real_distribution<> velocityXDistribution( -BoidOptions::MaxSpeed, BoidOptions::MaxSpeed);
    std::uniform_real_distribution<> velocityYDistribution( -BoidOptions::MaxSpeed, BoidOptions::MaxSpeed);
    std::uniform_int_distribution<> biasDistribution( 0, 1);

    for(int repetition = 0; repetition < BoidOptions::BoidNum; ++repetition) {
        Boid boid {static_cast<float>(ScreenOptions::WindowWidth) / 2, static_cast<float>(ScreenOptions::WindowHeight) / 2,
            (velocityXDistribution(generator)), (velocityYDistribution(generator)),
            (biasDistribution(generator))};

        boids.push_back(boid);
    }
}

std::vector<Boid> Simulator::getState() const {
    return this->boids;
}

void Simulator::NextState() {
    for(int i = 0; i < boids.size(); ++i) {
        double xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0, neighboring_boids = 0, close_dx = 0, close_dy = 0;
        for(int j = 0; j < boids.size(); ++j) {
            if(j == i) continue;

            double dx = boids[j].getX() - boids[i].getY();
            double dy = boids[j].getY() - boids[i].getX();

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

            // FLOCKING BEHAVIOUR
            if (neighboring_boids > 0) {
                xpos_avg = xpos_avg/neighboring_boids;
                ypos_avg = ypos_avg/neighboring_boids;
                xvel_avg = xvel_avg/neighboring_boids;
                yvel_avg = yvel_avg/neighboring_boids;

                const double vx = (boids[i].getXVelocity() +
                                   (xpos_avg - boids[i].getX()) * BoidOptions::CenteringFactor +
                                   (xvel_avg - boids[i].getXVelocity()) * BoidOptions::MatchingFactor);

                const double vy = (boids[i].getYVelocity() +
                                   (ypos_avg - boids[i].getY()) * BoidOptions::CenteringFactor +
                                   (yvel_avg - boids[i].getYVelocity()) * BoidOptions::MatchingFactor);

                boids[i].setXVelocity(vx);
                boids[i].setYVelocity(vy);
            }

            boids[i].setXVelocity(boids[i].getXVelocity() + (close_dx * BoidOptions::AvoidFactor));
            boids[i].setYVelocity(boids[i].getYVelocity() + (close_dy * BoidOptions::AvoidFactor));

            // MANAGE TURN
            // (0, 0) is top left
            // if outside top margin:
            if(boids[i].getY() < ScreenOptions::TopMargin) {
                boids[i].setYVelocity(boids[i].getYVelocity() + BoidOptions::TurnFactor);
            }
            // if outside right margin:
            if(boids[i].getX() > ScreenOptions::WindowWidth - ScreenOptions::RightMargin) {
                boids[i].setXVelocity(boids[i].getXVelocity() - BoidOptions::TurnFactor);
            }
            //if outside left margin:
            if(boids[i].getX() < ScreenOptions::LeftMargin) {
                boids[i].setXVelocity(boids[i].getXVelocity() + BoidOptions::TurnFactor);
            }
            //if outside bottom margin:
            if(boids[i].getY() > ScreenOptions::WindowHeight - ScreenOptions::BottomMargin) {
                boids[i].setYVelocity(boids[i].getYVelocity() - BoidOptions::TurnFactor);
            }


            // GROUP BIAS
            switch(boids[i].getBias()) {
                case RIGHT:
                    if(boids[i].getXVelocity() > 0) {
                        boids[i].setBiasVal(std::min(boids[i].getBiasVal() + BoidOptions::BiasIncrement, BoidOptions::MaxBias));
                    } else {
                        boids[i].setBiasVal(std::max(boids[i].getBiasVal() - BoidOptions::BiasIncrement, BoidOptions::BiasIncrement));
                    }

                    boids[i].setXVelocity((1 - boids[i].getBiasVal()) * boids[i].getXVelocity() + (boids[i].getBiasVal() * -1));
                    break;
                case LEFT:
                    if(boids[i].getXVelocity() < 0) {
                        boids[i].setBiasVal(std::min(boids[i].getBiasVal() + BoidOptions::BiasIncrement, BoidOptions::MaxBias));
                    } else {
                        boids[i].setBiasVal(std::max(boids[i].getBiasVal() - BoidOptions::BiasIncrement, BoidOptions::BiasIncrement));
                    }

                    boids[i].setXVelocity((1 - boids[i].getBiasVal()) * boids[i].getXVelocity() + boids[i].getBiasVal());
                    break;
                default:
                    std::cerr << "Invalid bias value" << std::endl;
            }

            // I AM SPEED
            double speed = std::sqrt(boids[i].getXVelocity() * boids[i].getXVelocity() + boids[i].getYVelocity() * boids[i].getYVelocity());

            if(speed < BoidOptions::MinSpeed) {
                boids[i].setXVelocity(boids[i].getXVelocity() / speed * BoidOptions::MinSpeed);
                boids[i].setYVelocity(boids[i].getYVelocity() / speed * BoidOptions::MinSpeed);
            }
            if(speed > BoidOptions::MaxSpeed) {
                boids[i].setXVelocity(boids[i].getXVelocity() / speed * BoidOptions::MaxSpeed);
                boids[i].setYVelocity(boids[i].getYVelocity() / speed * BoidOptions::MaxSpeed);
            }

            boids[i].move();
        }
    }
}
