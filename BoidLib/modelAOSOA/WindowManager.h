//
// Created by viu on 22/03/2025.
//

#ifndef WINDOW_AOSOA_H
#define WINDOW_AOSOA_H

#include "Boid.h"
#include "../Options.h"
#include "SFML/Graphics/RenderWindow.hpp"
#include "SFML/Graphics/Texture.hpp"

namespace modelAOSOA {
    class WindowManager {
    private :
        sf::Texture texture {"triangle.png"};
        sf::RenderWindow window;

        void initWindow();

    public:
        WindowManager();

        void updateWindow(const Boid* flockState, const int boidsNum, const int numV);

        sf::RenderWindow& getWindow();
    };
}
#endif //WINDOW_AOS_H
