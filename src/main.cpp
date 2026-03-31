#include <iostream>
#include <algorithm>
#include <string>
#include <SFML/Graphics.hpp>
#include <Eigen/Dense>
#include "Rocket.h"
#include "Visualization.h"
#include "Controller.h"

// Simulation timestep
const float SIM_DT = 1.0f / 60.0f;

int main(int argc, char* argv[]) {
    // Parse profile from command line: slow or fast
    std::string profile = "slow";
    if (argc > 1) {
        profile = argv[1];
    }
    if (profile != "slow" && profile != "fast") {
        std::cerr << "Usage: RocketSim [slow|fast]\n";
        return 1;
    }

    std::string gains_file = "../lqr_gains_" + profile + ".json";
    std::string window_title = "Rocket Sim - " + std::string(profile == "fast" ? "FAST" : "SLOW") + " Controller";

    // ========== Initialization ==========
    std::cout << "===========================================\n";
    std::cout << "  2D Rocket Simulation - " << (profile == "fast" ? "FAST" : "SLOW") << " Controller\n";
    std::cout << "===========================================\n\n";

    Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(6);
    initial_state(0) = 2.0;      // x = 2 m (lateral offset)
    initial_state(2) = 10.0;     // y = 10 m (high altitude)

    Rocket rocket(1.0, 0.5, initial_state);  // m=1kg, L=0.5m

    std::cout << "[+] Rocket initialized:\n";
    std::cout << "    Mass: " << rocket.m << " kg\n";
    std::cout << "    Initial position: (" << initial_state(0) << ", " << initial_state(2) << ") m\n\n";

    sf::RenderWindow window(sf::VideoMode(sf::Vector2u(1200, 800)), window_title);
    window.setFramerateLimit(60);

    sf::Font font("C:\\Windows\\Fonts\\consola.ttf");

    // Initialize controller and load LQR gains for selected profile
    Controller controller(100.0, M_PI / 2);
    if (!controller.load_gains_from_json(gains_file)) {
        std::cerr << "[!] Could not load " << gains_file << "\n";
        return 1;
    }
    
    Eigen::VectorXd setpoint = Eigen::VectorXd::Zero(6);
    setpoint(2) = 0.5;      // y = 0.5 m (just above ground)
    
    std::cout << "[+] Control setpoint: hover at (0, " << setpoint(2) << ") m\n\n";

    // ========== Simulation Loop ==========
    std::cout << "===========================================\n";
    std::cout << "    Simulation Running...\n";
    std::cout << "===========================================\n\n";

    int frame = 0;
    double simulation_time = 0.0;

    while (window.isOpen()) {
        // Handle window events
        std::optional<sf::Event> event = window.pollEvent();
        while (event) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
            event = window.pollEvent();
        }

        // Get current state and compute LQR control
        Eigen::VectorXd state = rocket.get_state();
        
        // Once at or below setpoint, cut engines (landed)
        if (state(2) <= setpoint(2) && std::abs(state(3)) < 1.0) {
            rocket.set_control(0.0, 0.0);
        } else {
            Eigen::Vector2d control = controller.compute_control(state, setpoint);
            double total_thrust = rocket.m * rocket.g + control(0);
            total_thrust = std::clamp(total_thrust, 0.0, 100.0);
            rocket.set_control(total_thrust, control(1));
        }

        // Update physics (RK4 integration)
        rocket.update(SIM_DT);
        
        // Handle ground collision
        rocket.check_ground_collision(0.0);
        
        simulation_time += SIM_DT;
        frame++;

        // Render frame
        window.clear(sf::Color::Black);
        Visualization::draw_axes(window, font);
        Visualization::draw_ground(window);
        Visualization::draw_rocket(window, rocket, 30.0f);
        window.display();

        // Print telemetry
        Visualization::print_telemetry(rocket, frame);

        // Check simulation bounds (only horizontal)
        if (state(0) < -20.0 || state(0) > 20.0) {
            std::cout << "\n[!] Rocket left horizontal bounds. Ending simulation.\n";
            break;
        }
    }

    // ========== Cleanup ==========
    std::cout << "\n===========================================\n";
    std::cout << "[+] Simulation ended successfully!\n";
    std::cout << "    Total time: " << simulation_time << " seconds\n";
    std::cout << "    Total frames: " << frame << "\n";
    std::cout << "===========================================\n";

    return 0;
}
