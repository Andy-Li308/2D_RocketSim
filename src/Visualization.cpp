#include "Visualization.h"
#include <iostream>
#include <cmath>
#include <iomanip>

namespace Visualization {
    // Define rendering constants
    const float PIXELS_PER_METER = 50.0f;
    const float WINDOW_WIDTH = 1200.0f;
    const float WINDOW_HEIGHT = 800.0f;
    const float GROUND_Y = WINDOW_HEIGHT - 50.0f;

    sf::Vector2f world_to_screen(double x, double y) {
        float screen_x = WINDOW_WIDTH / 2.0f + static_cast<float>(x) * PIXELS_PER_METER;
        float screen_y = GROUND_Y - static_cast<float>(y) * PIXELS_PER_METER;
        return sf::Vector2f(screen_x, screen_y);
    }

    void draw_rocket(sf::RenderWindow& window, const Rocket& rocket, float thrust_scale) {
        Eigen::VectorXd state = rocket.get_state();
        double x = state(0);
        double y = state(2);
        double theta = state(4);

        // Convert world position to screen position (rocket center)
        sf::Vector2f center = world_to_screen(x, y);

        // === Draw rocket body as white rectangle ===
        float rocket_width = 6.0f;    // pixels  
        float rocket_height = 60.0f;  // pixels

        sf::RectangleShape rocket_body(sf::Vector2f(rocket_width, rocket_height));
        rocket_body.setFillColor(sf::Color::White);
        rocket_body.setOrigin(sf::Vector2f(rocket_width / 2.0f, rocket_height / 2.0f));
        rocket_body.setPosition(center);
        rocket_body.setRotation(sf::degrees(static_cast<float>(theta * 180.0f / M_PI)));
        window.draw(rocket_body);

        // Precompute rotation values
        float cos_theta = std::cos(static_cast<float>(theta));
        float sin_theta = std::sin(static_cast<float>(theta));

        // === Calculate rocket tail position ===
        // Tail offset when rocket rotates by theta
        // Nose points in direction: (sin(theta), cos(theta)) in world space
        // In screen space (y inverted): (sin(theta), -cos(theta))
        // Tail is opposite: (-sin(theta), cos(theta))
        sf::Vector2f tail_offset = sf::Vector2f(
            -sin_theta * rocket_height / 2.0f,
            cos_theta * rocket_height / 2.0f
        );
        sf::Vector2f rocket_tail = center + tail_offset;

        // === Draw fins at tail ===
        float fin_length = 10.0f;
        float fin_width = 2.0f;
        
        // Perpendicular direction to rocket (90 degrees rotated)
        float perp_sin = cos_theta;
        float perp_cos = -sin_theta;
        
        for (int i = -1; i <= 1; i += 2) {  // Left and right fins
            sf::RectangleShape fin(sf::Vector2f(fin_width, fin_length));
            fin.setFillColor(sf::Color::White);
            fin.setOrigin(sf::Vector2f(fin_width / 2.0f, fin_length / 2.0f));
            
            // Offset perpendicular to rocket body
            sf::Vector2f fin_offset = sf::Vector2f(
                i * perp_sin * rocket_width / 2.0f,
                i * perp_cos * rocket_width / 2.0f
            );
            fin.setPosition(rocket_tail + fin_offset);
            fin.setRotation(sf::degrees(static_cast<float>(theta * 180.0f / M_PI)));
            window.draw(fin);
        }

        // === Draw thrust line (proportional to thrust, angled by alpha) ===
        if (rocket.thrust > 0.01f) {
            float thrust_px_length = std::min(static_cast<float>(rocket.thrust) * thrust_scale, 60.0f);
            
            // Thrust angle = theta + alpha
            float thrust_angle_rad = static_cast<float>(theta + rocket.alpha);
            
            // Draw thin red rectangle for thrust line from tail
            sf::RectangleShape thrust_line(sf::Vector2f(2.0f, thrust_px_length));
            thrust_line.setFillColor(sf::Color::Red);
            thrust_line.setPosition(rocket_tail);
            thrust_line.setOrigin(sf::Vector2f(1.0f, 0.0f));  // Pivot at top (head of arrow)
            // Thrust extends in direction theta + alpha (away from rocket nozzle)
            thrust_line.setRotation(sf::degrees(thrust_angle_rad * 180.0f / static_cast<float>(M_PI)));
            window.draw(thrust_line);
        }

        // === Draw center of mass marker ===
        sf::CircleShape com_marker(3.0f);
        com_marker.setPosition(center - sf::Vector2f(3.0f, 3.0f));
        com_marker.setFillColor(sf::Color::Yellow);
        window.draw(com_marker);
    }

    void draw_ground(sf::RenderWindow& window) {
        sf::RectangleShape ground(sf::Vector2f(WINDOW_WIDTH, 50.0f));
        ground.setPosition(sf::Vector2f(0, GROUND_Y));
        ground.setFillColor(sf::Color::Green);
        window.draw(ground);
    }

    void draw_axes(sf::RenderWindow& window, const sf::Font& font) {
        // Colors
        sf::Color axis_color(100, 100, 100);       // dark gray for axes
        sf::Color grid_color(40, 40, 40);           // subtle grid
        sf::Color label_color(180, 180, 180);       // light gray labels

        // Y-axis (vertical) at x=0 world
        sf::Vector2f y_axis_top = world_to_screen(0, 15);
        sf::Vector2f y_axis_bot = world_to_screen(0, 0);
        sf::RectangleShape y_axis(sf::Vector2f(1.0f, y_axis_bot.y - y_axis_top.y));
        y_axis.setPosition(sf::Vector2f(y_axis_top.x, y_axis_top.y));
        y_axis.setFillColor(axis_color);
        window.draw(y_axis);

        // X-axis (horizontal) at y=0 world
        sf::Vector2f x_axis_left = world_to_screen(-12, 0);
        sf::Vector2f x_axis_right = world_to_screen(12, 0);
        sf::RectangleShape x_axis(sf::Vector2f(x_axis_right.x - x_axis_left.x, 1.0f));
        x_axis.setPosition(sf::Vector2f(x_axis_left.x, x_axis_left.y));
        x_axis.setFillColor(axis_color);
        window.draw(x_axis);

        // Y-axis tick marks and labels (every 1 meter)
        for (int y = 0; y <= 14; y++) {
            sf::Vector2f pos = world_to_screen(0, y);

            // Horizontal grid line
            sf::RectangleShape grid_line(sf::Vector2f(WINDOW_WIDTH, 1.0f));
            grid_line.setPosition(sf::Vector2f(0, pos.y));
            grid_line.setFillColor(grid_color);
            window.draw(grid_line);

            // Tick mark
            sf::RectangleShape tick(sf::Vector2f(6.0f, 1.0f));
            tick.setPosition(sf::Vector2f(pos.x - 3.0f, pos.y));
            tick.setFillColor(axis_color);
            window.draw(tick);

            // Label
            sf::Text label(font, std::to_string(y) + "m", 11);
            label.setFillColor(label_color);
            label.setPosition(sf::Vector2f(pos.x + 6.0f, pos.y - 7.0f));
            window.draw(label);
        }

        // X-axis tick marks and labels (every 2 meters)
        for (int x = -10; x <= 10; x += 2) {
            if (x == 0) continue;  // skip origin (already labeled by Y)
            sf::Vector2f pos = world_to_screen(x, 0);

            // Vertical grid line
            sf::RectangleShape grid_line(sf::Vector2f(1.0f, GROUND_Y));
            grid_line.setPosition(sf::Vector2f(pos.x, 0));
            grid_line.setFillColor(grid_color);
            window.draw(grid_line);

            // Tick mark
            sf::RectangleShape tick(sf::Vector2f(1.0f, 6.0f));
            tick.setPosition(sf::Vector2f(pos.x, pos.y - 3.0f));
            tick.setFillColor(axis_color);
            window.draw(tick);

            // Label
            sf::Text label(font, std::to_string(x) + "m", 11);
            label.setFillColor(label_color);
            label.setPosition(sf::Vector2f(pos.x - 8.0f, pos.y + 4.0f));
            window.draw(label);
        }

        // Origin label
        sf::Vector2f origin = world_to_screen(0, 0);
        sf::Text origin_label(font, "0", 11);
        origin_label.setFillColor(label_color);
        origin_label.setPosition(sf::Vector2f(origin.x + 5.0f, origin.y + 4.0f));
        window.draw(origin_label);
    }

    void print_telemetry(const Rocket& rocket, int frame) {
        // Print every 60 frames (~1 second at 60 Hz)
        if (frame % 60 == 0) {
            Eigen::VectorXd state = rocket.get_state();
            double x = state(0);
            double y = state(2);
            double xdot = state(1);
            double ydot = state(3);
            double theta = state(4);
            double thetadot = state(5);

            double sim_time = frame * (1.0 / 60.0);
            double theta_deg = theta * 180.0 / M_PI;
            double alpha_deg = rocket.alpha * 180.0 / M_PI;

            std::cout << "\n--- Frame " << frame << " (t=" << std::fixed << std::setprecision(2) 
                      << sim_time << "s) ---\n";
            std::cout << "Position: x=" << std::setprecision(4) << x << " m, y=" << y << " m\n";
            std::cout << "Velocity: vx=" << xdot << " m/s, vy=" << ydot << " m/s\n";
            std::cout << "Angle: theta=" << std::setprecision(2) << theta_deg << " deg, omega=" 
                      << std::setprecision(4) << thetadot << " rad/s\n";
            std::cout << "Thrust: T=" << rocket.thrust << " N, alpha=" << std::setprecision(2) 
                      << alpha_deg << " deg\n";
        }
    }
}
