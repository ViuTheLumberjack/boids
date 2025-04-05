#include <memory>
#include <iostream>

#include "SFML/Graphics/RenderWindow.hpp"
#include "SFML/Window/Event.hpp"
#include "SFML/Graphics.hpp"

#include "modelAOS/Simulator.h"
#include "modelAOS/WindowManager.h"
#include "modelSOA/Simulator.h"
#include "modelSOA/WindowManager.h"

#ifdef _OPENMP
#include <omp.h> // for OpenMP library functions
#endif

using namespace modelSOA;

int main() {
    const auto s = std::make_unique<Simulator>();
    const auto w = std::make_unique<WindowManager>();

#ifdef _OPENMP
    std::cout << "Num processors (Phys+HT): " << omp_get_num_procs() << std::endl;
#endif


    while (w->getWindow().isOpen()) {

        s->NextState();
        {
            // Update the window with the new state
            w->updateWindow(s->getState());

            while (const std::optional event = w->getWindow().pollEvent())
            {
                // Close window: exit
                if (event->is<sf::Event::Closed>())
                    w->getWindow().close();
            }
        }
    }
}
