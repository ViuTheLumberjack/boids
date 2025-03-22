#include <iostream>
#include <valarray>
#include <SFML/Window.hpp>

#include "SFML/Graphics/RenderWindow.hpp"

#include "model/Simulator.h"
#include "model/WindowManager.h"


int main()
{
    const auto s = std::make_unique<Simulator>();
    const auto w = std::make_unique<WindowManager>();

    while (w->getWindow().isOpen())
    {
        s->NextState();
        w->updateWindow(s->getState());

        while (const std::optional event = w->getWindow().pollEvent())
        {
            if (event->is<sf::Event::Closed>()) {
                w->getWindow().close();
            }
        }
    }
}
