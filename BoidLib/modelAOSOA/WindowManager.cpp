//
// Created by viu on 22/03/2025.
//

#include "WindowManager.h"

#include <cmath>

#include "../Options.h"
#include "SFML/Graphics/RenderWindow.hpp"
#include "SFML/Graphics/Sprite.hpp"

namespace modelAOSOA {
    WindowManager::WindowManager() {
        window = sf::RenderWindow(sf::VideoMode({ScreenOptions::WindowWidth, ScreenOptions::WindowHeight}), "Boids");
        const auto desktop = sf::VideoMode::getDesktopMode();
        window.setPosition({
            static_cast<int>(desktop.size.x / 2 - window.getSize().x / 2),
            static_cast<int>(desktop.size.y / 2 - window.getSize().y / 2)
        });
        window.setFramerateLimit(ScreenOptions::FrameRate);
    }

    void WindowManager::initWindow() {
        window.clear(sf::Color::Black);

        constexpr std::array boxLines =
        {
            sf::Vertex{sf::Vector2f(ScreenOptions::LeftMargin, ScreenOptions::TopMargin)},
            sf::Vertex{
                sf::Vector2f(ScreenOptions::LeftMargin, ScreenOptions::WindowHeight - ScreenOptions::BottomMargin)
            },
            sf::Vertex{
                sf::Vector2f(ScreenOptions::WindowWidth - ScreenOptions::RightMargin,
                             ScreenOptions::WindowHeight - ScreenOptions::BottomMargin)
            },
            sf::Vertex{sf::Vector2f(ScreenOptions::WindowWidth - ScreenOptions::RightMargin, ScreenOptions::TopMargin)},
            sf::Vertex{sf::Vector2f(ScreenOptions::LeftMargin, ScreenOptions::TopMargin)},
        };

        window.draw(boxLines.data(), boxLines.size(), sf::PrimitiveType::LineStrip);
    }

    void WindowManager::updateWindow(const Boid* flockState, const int boidsNum, const int numV) {
        initWindow();

        for (int i = 0; i < boidsNum; ++i) {
            const auto boid = flockState[i];
            for (int j = 0; j < numV; j++) {

                sf::Sprite sprite(texture);
                sprite.setScale(sf::Vector2f(0.025, 0.025));
                sprite.setPosition(sf::Vector2f(boid.xPosition[j], boid.yPosition[j]));
                sprite.setRotation(sf::radians(std::atan2(boid.yVelocity[j], boid.xVelocity[j]) + M_PI));

                window.draw(sprite);
            }
        }

        window.display();
    }

    sf::RenderWindow &WindowManager::getWindow() {
        return window;
    }
}
