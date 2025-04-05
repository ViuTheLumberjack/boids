//
// Created by viu on 22/03/2025.
//

#ifndef WINDOW_AOS_H
#define WINDOW_AOS_H

#include "Boid.h"
#include "../Options.h"
#include "SFML/Graphics/RenderWindow.hpp"
#include "SFML/Graphics/Texture.hpp"

namespace modelAOS {
    class WindowManager {
    private :
        sf::Texture texture {"triangle.png"};
        sf::RenderWindow window;

        void initWindow();

    public:
        WindowManager();

        void updateWindow(const std::array<Boid, BoidOptions::BoidNum> &flockState);

        sf::RenderWindow& getWindow();
    };
}
#endif //WINDOW_AOS_H
