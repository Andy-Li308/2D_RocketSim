#include <iostream>
#include <algorithm>
#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include "Rocket.h"
#include "Visualization.h"
#include "Controller.h"

const float SIM_DT = 1.0f / 60.0f;

int main() {
    std::cout << "===========================================\n";
    std::cout << "  FAST Controller - High Control Effort\n";
    std::cout << "===========================================\n\n";

    Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(6);
    initial_state(0) = 2.0;
    initial_state(2) = 10.0;

    Rocket rocket(1.0, 0.5, initial_state);

    std::cout << "[+] Initial position: (" << initial_state(0) << ", " << initial_state(2) << ") m\n\n";

    sf::RenderWindow window(sf::VideoMode(sf::Vector2u(1200, 800)), "Rocket Sim - FAST Controller");
    window.setFramerateLimit(60);

    sf::Font font("C:\\Windows\\Fonts\\consola.ttf");

    Controller controller(100.0, M_PI / 2);
    if (!controller.load_gains_from_json("../lqr_gains_fast.json")) {
        std::cerr << "[!] Could not load lqr_gains_fast.json\n";
        return 1;
    }

    Eigen::VectorXd setpoint = Eigen::VectorXd::Zero(6);
    setpoint(2) = 0.5;

    std::cout << "[+] Setpoint: hover at (0, 0.5) m\n\n";

    int frame = 0;
    double simulation_time = 0.0;

    while (window.isOpen()) {
        std::optional<sf::Event> event = window.pollEvent();
        while (event) {
            if (event->is<sf::Event::Closed>()) window.close();
            event = window.pollEvent();
        }

        Eigen::VectorXd state = rocket.get_state();

        if (state(2) <= setpoint(2) && std::abs(state(3)) < 1.0) {
            rocket.set_control(0.0, 0.0);
        } else {
            Eigen::Vector2d control = controller.compute_control(state, setpoint);
            double total_thrust = rocket.m * rocket.g + control(0);
            total_thrust = std::clamp(total_thrust, 0.0, 100.0);
            rocket.set_control(total_thrust, control(1));
        }

        rocket.update(SIM_DT);
        rocket.check_ground_collision(0.0);
        simulation_time += SIM_DT;
        frame++;

        window.clear(sf::Color::Black);
        Visualization::draw_axes(window, font);
        Visualization::draw_ground(window);
        Visualization::draw_rocket(window, rocket, 30.0f);
        window.display();

        Visualization::print_telemetry(rocket, frame);

        if (state(0) < -20.0 || state(0) > 20.0) {
            std::cout << "\n[!] Rocket left bounds.\n";
            break;
        }
    }

    std::cout << "\n[+] Fast sim ended. Time: " << simulation_time << "s, Frames: " << frame << "\n";
    return 0;
}
