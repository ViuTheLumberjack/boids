#include <SFML/Window.hpp>

#include "SFML/Graphics/RenderWindow.hpp"
#include "SFML/Graphics/Sprite.hpp"
#include "SFML/Graphics/Texture.hpp"

#include "model/Simulator.h"

const sf::Texture texture("textures/triangle.png");

void initWindow(sf::RenderWindow* window) {
    std::vector<sf::Sprite> sprites;

    window->clear(sf::Color::Black);

    constexpr std::array boxLines =
    {
        sf::Vertex{sf::Vector2f(ScreenOptions::LeftMargin, ScreenOptions::TopMargin)},
        sf::Vertex{sf::Vector2f(ScreenOptions::LeftMargin, ScreenOptions::WindowHeight - ScreenOptions::BottomMargin)},
        sf::Vertex{sf::Vector2f(ScreenOptions::WindowWidth - ScreenOptions::RightMargin, ScreenOptions::WindowHeight - ScreenOptions::BottomMargin)},
        sf::Vertex{sf::Vector2f(ScreenOptions::WindowWidth - ScreenOptions::RightMargin, ScreenOptions::TopMargin)},
        sf::Vertex{sf::Vector2f(ScreenOptions::LeftMargin, ScreenOptions::TopMargin)},
    };
    window->draw(boxLines.data(), boxLines.size(), sf::PrimitiveType::LineStrip);
}

void updateWindow(sf::RenderWindow* window, const std::vector<Boid>& flockState) {
    initWindow(window);

    for (const auto& boid : flockState) {
        sf::Sprite sprite(texture);
        sprite.setScale(sf::Vector2f(0.0025, 0.0025));
        sprite.setPosition(sf::Vector2f(boid.getX(), boid.getY()));
        sprite.setColor(boid.getBias() == BoidBias::LEFT ? sf::Color::Red : sf::Color::Blue);

        window->draw(sprite);
    }
    window->display();
}

int main()
{
    const auto s = std::make_unique<Simulator>();
    sf::RenderWindow window(sf::VideoMode({ScreenOptions::WindowWidth, ScreenOptions::WindowHeight}), "Boids");
    const auto desktop = sf::VideoMode::getDesktopMode();
    window.setPosition({static_cast<int>(desktop.size.x/2 - window.getSize().x/2), static_cast<int>(desktop.size.y/2 - window.getSize().y/2)});
    window.setFramerateLimit(ScreenOptions::FrameRate);

    initWindow(&window);

    while (window.isOpen())
    {
        s->NextState();

        updateWindow(&window, s->getState());

        while (const std::optional event = window.pollEvent())
        {
            // "close requested" event: we close the window
            if (event->is<sf::Event::Closed>())
                window.close();
        }
    }
}
