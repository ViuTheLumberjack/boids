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

#include "KDTree.h"

using engine = std::mt19937;

namespace modelAOS
{
    Simulator::Simulator(const BoidOptions &options) : options(options)
    {
        boids = new Boid[options.BoidNum];
        velocitiesX = new float[options.BoidNum];
        velocitiesY = new float[options.BoidNum];
        positionX = new float[options.BoidNum];
        positionY = new float[options.BoidNum];

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

        for (int repetition = omp_get_thread_num(); repetition < options.BoidNum; repetition += omp_get_num_threads())
        {
            const Boid boid{
                (positionXDistribution(generator)), (positionYDistribution(generator)),
                (velocityXDistribution(generator)), (velocityYDistribution(generator))};

            boids[repetition] = boid;
        }
    }

    const Boid *Simulator::getState() const
    {
        return this->boids;
    }

    void Simulator::RunSimulation(void (Simulator::*f)(), const int iterations)
    {
        // Copy the boids to a new array
        // velocitiesX = new float[options.BoidNum];
        // velocitiesY = new float[options.BoidNum];

        for (int i = 0; i < iterations; ++i)
        {
            (this->*f)();
        }
    }

    void Simulator::NextStateParallel()
    {
        // Copy the boids to a new array

        const auto kdTree = std::make_unique<ModelAOS::KDTree>(boids, options.BoidNum, options.VisualRange);
#pragma omp parallel default(none) shared(boids, options, kdTree, velocitiesX, velocitiesY, positionX, positionY)
        {
#pragma omp for
            for (int i = 0; i < options.BoidNum; ++i)
            {
                velocitiesX[i] = boids[i].xVelocity;
                velocitiesY[i] = boids[i].yVelocity;
                positionX[i] = boids[i].xPosition;
                positionY[i] = boids[i].yPosition;
            }

            // const int numThreads = omp_get_num_threads();
#pragma omp for schedule(dynamic)
            for (int i = 0; i < options.BoidNum; ++i)
            //            for (int i = omp_get_thread_num(); i < options.BoidNum; i += numThreads)
            {
                const Boid *current = &boids[i];
                float xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0, close_dx = 0, neighboring_boids = 0,
                      close_dy = 0;

                const float pos_x = positionX[i];
                const float pos_y = positionY[i];
                float new_vx = velocitiesX[i];
                float new_vy = velocitiesY[i];

                const auto neighbors = kdTree->query(&boids[i]);

                // for (int j = 0; j < options.BoidNum; ++j)
                for (const auto temp : neighbors)
                {
                    const auto j = temp - boids;
                    const float dx = positionX[j] - pos_x;
                    const float dy = positionY[j] - pos_y;

                    if (const float squared_distance = dx * dx + dy * dy; squared_distance < options.ProtectedRange * options.ProtectedRange)
                    {
                        close_dx += pos_x - positionX[j];
                        close_dy += pos_y - positionY[j];
                    }

                    xpos_avg += positionX[j];
                    ypos_avg += positionY[j];
                    xvel_avg += velocitiesX[j];
                    yvel_avg += velocitiesY[j];

                    neighboring_boids++;
                }

                // if (!neighbors.empty()) {
                if (neighboring_boids > 0)
                {
                    xpos_avg = xpos_avg / neighboring_boids;
                    ypos_avg = ypos_avg / neighboring_boids;
                    xvel_avg = xvel_avg / neighboring_boids;
                    yvel_avg = yvel_avg / neighboring_boids;

                    new_vx = new_vx +
                             (xpos_avg - pos_x) * options.CenteringFactor +
                             (xvel_avg - new_vx) * options.MatchingFactor;

                    new_vy = new_vy +
                             (ypos_avg - pos_y) * options.CenteringFactor +
                             (yvel_avg - new_vy) * options.MatchingFactor;
                }

                new_vx = new_vx + (close_dx * options.AvoidFactor);
                new_vy = new_vy + (close_dy * options.AvoidFactor);

                // MANAGE TURN
                // (0, 0) is top left
                // if outside top margin:
                if (pos_y < ScreenOptions::TopMargin)
                {
                    new_vy += options.TurnFactor;
                }
                // if outside right margin:
                if (pos_x > ScreenOptions::WindowWidth - ScreenOptions::RightMargin)
                {
                    new_vx -= options.TurnFactor;
                }
                // if outside left margin:
                if (pos_x < ScreenOptions::LeftMargin)
                {
                    new_vx += options.TurnFactor;
                }
                // if outside bottom margin:
                if (pos_y > ScreenOptions::WindowHeight - ScreenOptions::BottomMargin)
                {
                    new_vy -= options.TurnFactor;
                }

                // I AM SPEED
                float speed = std::sqrt(new_vx * new_vx + new_vy * new_vy);

                if (speed < options.MinSpeed)
                {
                    new_vx = new_vx / speed * options.MinSpeed;
                    new_vy = new_vy / speed * options.MinSpeed;
                }
                if (speed > options.MaxSpeed)
                {
                    new_vx = new_vx / speed * options.MaxSpeed;
                    new_vy = new_vy / speed * options.MaxSpeed;
                }

                boids[i].xVelocity = new_vx;
                boids[i].yVelocity = new_vy;
                boids[i].move();
            }
        }
    }

    void Simulator::NextStateSequential()
    {
        const auto kdTree = std::make_unique<ModelAOS::KDTree>(boids, options.BoidNum, options.VisualRange);
        for (int i = 0; i < options.BoidNum; ++i)
        {
            float xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0, neighboring_boids = 0, close_dx = 0,
                  close_dy = 0;

            float pos_x = boids[i].xPosition;
            float pos_y = boids[i].yPosition;
            float new_vx = boids[i].xVelocity;
            float new_vy = boids[i].yVelocity;

            auto neighbors = kdTree->query(&boids[i]);
            for (const auto neighbor : neighbors)
            // for (int j = 0; j < options.BoidNum; ++j)
            {
                // const auto neighbor = neighbors[j];
                // const auto neighbor = &boids[j];

                const float dx = neighbor->xPosition - pos_x;
                const float dy = neighbor->yPosition - pos_y;

                if (const float squared_distance = dx * dx + dy * dy; squared_distance < options.ProtectedRange * options.ProtectedRange)
                {
                    close_dx += pos_x - neighbor->xPosition;
                    close_dy += pos_y - neighbor->yPosition;
                }

                xpos_avg += neighbor->xPosition;
                ypos_avg += neighbor->yPosition;
                xvel_avg += neighbor->xVelocity;
                yvel_avg += neighbor->yVelocity;

                neighboring_boids++;
            }
            // FLOCKING BEHAVIOUR
            if (neighboring_boids > 0)
            {
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
            if (pos_y < ScreenOptions::TopMargin)
            {
                new_vy += options.TurnFactor;
            }
            // if outside right margin:
            if (pos_x > ScreenOptions::WindowWidth - ScreenOptions::RightMargin)
            {
                new_vx -= options.TurnFactor;
            }
            // if outside left margin:
            if (pos_x < ScreenOptions::LeftMargin)
            {
                new_vx += options.TurnFactor;
            }
            // if outside bottom margin:
            if (pos_y > ScreenOptions::WindowHeight - ScreenOptions::BottomMargin)
            {
                new_vy -= options.TurnFactor;
            }

            // I AM SPEED
            float speed = std::sqrt(new_vx * new_vx + new_vy * new_vy);

            if (speed < options.MinSpeed)
            {
                new_vx = new_vx / speed * options.MinSpeed;
                new_vy = new_vy / speed * options.MinSpeed;
            }
            if (speed > options.MaxSpeed)
            {
                new_vx = new_vx / speed * options.MaxSpeed;
                new_vy = new_vy / speed * options.MaxSpeed;
            }

            velocitiesX[i] = new_vx;
            velocitiesY[i] = new_vy;
        }

        for (int i = 0; i < options.BoidNum; ++i)
        {
            boids[i].xVelocity = velocitiesX[i];
            boids[i].yVelocity = velocitiesY[i];
            boids[i].move();
        }
    }
}
