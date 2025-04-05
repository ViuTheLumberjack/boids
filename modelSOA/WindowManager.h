//
// Created by viu on 22/03/2025.
//

#ifndef WINDOW_SOA_H
#define WINDOW_SOA_H
#include "BoidsArray.h"

#include "SFML/Graphics/RenderWindow.hpp"
#include "SFML/Graphics/Texture.hpp"

namespace modelSOA {
    class WindowManager{
    private :
        sf::Texture texture {"triangle.png"};
        sf::RenderWindow window;

        void initWindow();

    public:
        WindowManager();

        void updateWindow(const BoidsArray& flockState);

        sf::RenderWindow& getWindow();
    };
}

#endif //WINDOW_SOA_H
