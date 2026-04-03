#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include "Rocket.h"
#include "Visualization.h"
#include "Controller.h"


const float SIM_DT = 1.0f / 60.0f;

int main(int argc, char* argv[]) {
    std::string profile = "slow";
    if (argc > 1) profile = argv[1];
    if (profile != "slow" && profile != "fast") {
        std::cerr << "Usage: RocketSim [slow|fast]\n";
        return 1;
    }

    std::string gains_file = "../data/lqr_gains_" + profile + ".json";
    std::string window_title = std::string("Rocket Sim - ") + (profile == "fast" ? "FAST" : "SLOW");

    Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(6);
    initial_state(0) = 2.0;
    initial_state(2) = 10.0;

    Rocket rocket(1.0, 0.5, initial_state);

    std::cout << "Rocket mass: " << rocket.m << " kg\n";
    std::cout << "Initial position: (" << initial_state(0) << ", " << initial_state(2) << ")\n";

    sf::RenderWindow window(sf::VideoMode(sf::Vector2u(1200, 800)), window_title);
    window.setFramerateLimit(60);

    sf::Font font("C:\\Windows\\Fonts\\consola.ttf");

    Controller controller(100.0, M_PI / 2, rocket.m, rocket.g);
    if (!controller.load_gains_from_json(gains_file)) {
        std::cerr << "Could not load " << gains_file << "\n";
        return 1;
    }

    Eigen::VectorXd setpoint = Eigen::VectorXd::Zero(6);
    setpoint(2) = 0.5;
    std::cout << "Setpoint: hover at (0, " << setpoint(2) << ")\n";
    std::cout << "    Simulation Running...\n";
    std::cout << "===========================================\n\n";

    int frame = 0;
    double simulation_time = 0.0;
    bool engine_cut = false;

    std::ofstream log_file("../data/sim_log.csv");
    log_file << "time,x,xdot,y,ydot,theta,thetadot,thrust,alpha,x_err,y_err\n";

    std::ofstream meta_file("../data/sim_meta.txt");
    meta_file << profile << "\n";
    meta_file.close();

    while (window.isOpen()) {
        std::optional<sf::Event> event = window.pollEvent();
        while (event) {
            if (event->is<sf::Event::Closed>()) window.close();
            event = window.pollEvent();
        }

        Eigen::VectorXd state = rocket.get_state();

        if (!engine_cut) {
            Eigen::VectorXd error = state - setpoint;
            bool stable = std::abs(error(0)) < 0.05
                       && std::abs(error(1)) < 0.05
                       && std::abs(error(2)) < 0.05
                       && std::abs(error(3)) < 0.05
                       && std::abs(error(4)) < 0.02
                       && std::abs(error(5)) < 0.02;
            if (stable) {
                engine_cut = true;
                std::cout << "[+] Hover stable at t=" << simulation_time << "s — engine cut\n";
            } else {
                Eigen::Vector2d control = controller.compute_control(state, setpoint);
                rocket.set_control(control(0), control(1));
            }
        }

        if (engine_cut) {
            rocket.set_control(0.0, 0.0);
        }

        rocket.update(SIM_DT);
        rocket.check_ground_collision(0.0);

        simulation_time += SIM_DT;
        frame++;

        // Log data for plotting
        Eigen::VectorXd log_state = rocket.get_state();
        log_file << simulation_time << ","
                 << log_state(0) << "," << log_state(1) << ","
                 << log_state(2) << "," << log_state(3) << ","
                 << log_state(4) << "," << log_state(5) << ","
                 << rocket.thrust << "," << rocket.alpha << ","
                 << (log_state(0) - setpoint(0)) << ","
                 << (log_state(2) - setpoint(2)) << "\n";

        window.clear(sf::Color::Black);
        Visualization::draw_axes(window, font);
        Visualization::draw_ground(window);
        Visualization::draw_setpoint(window, setpoint(0), setpoint(2));
        Visualization::draw_rocket(window, rocket, 30.0f);
        window.display();

        Visualization::print_telemetry(rocket, frame);

        if (state(0) < -20.0 || state(0) > 20.0) {
            std::cout << "Rocket left horizontal bounds. Ending simulation.\n";
            break;
        }
    }

    log_file.close();
    std::cout << "Simulation ended. Time: " << simulation_time << "s, Frames: " << frame << "\n";
    std::cout << "===========================================\n";

    return 0;
}
