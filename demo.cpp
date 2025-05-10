#include <memory>
#include <iostream>

#include "SFML/Graphics/RenderWindow.hpp"
#include "SFML/Window/Event.hpp"
#include "SFML/Graphics.hpp"

#include "BoidLib/modelAOS/Simulator.h"
#include "BoidLib/modelAOS/WindowManager.h"
#include "BoidLib/modelSOA/Simulator.h"
#include "BoidLib/modelSOA/WindowManager.h"
#include "BoidLib/modelAOSOA/Simulator.h"
#include "BoidLib/modelAOSOA/WindowManager.h"

#ifdef _OPENMP
#include <omp.h> // for OpenMP library functions
#endif

using namespace modelAOS;

int main() {
    BoidOptions options {};
    options.BoidNum = 1000;
    options.MaxV = 2;
    options.VisualRange = 50;

    const auto s = std::make_unique<Simulator>(options);
    const auto w = std::make_unique<WindowManager>();

#ifdef _OPENMP
    std::cout << "Max Threads: " << omp_get_max_threads() << std::endl;
    std::cout << "Num Threads: " << omp_get_num_threads() << std::endl;
    std::cout << "Num processors (Phys+HT): " << omp_get_num_procs() << std::endl;
    omp_set_num_threads(13);
#endif


    while (w->getWindow().isOpen()) {
#ifdef _OPENMP
        s->NextStateParallel();
#else
        s->NextStateSequential();
#endif

        // Update the window with the new state
        w->updateWindow(s->getState(), options.BoidNum);

        while (const std::optional event = w->getWindow().pollEvent()) {
            // Close window: exit
            if (event->is<sf::Event::Closed>())
                w->getWindow().close();
        }
    }
}
