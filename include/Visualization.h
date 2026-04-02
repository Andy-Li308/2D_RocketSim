#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <SFML/Graphics.hpp>
#include "Rocket.h"


namespace Visualization {
    extern const float PIXELS_PER_METER;
    extern const float WINDOW_WIDTH;
    extern const float WINDOW_HEIGHT;
    extern const float GROUND_Y;
    sf::Vector2f world_to_screen(double x, double y);
    void draw_rocket(sf::RenderWindow& window, const Rocket& rocket, float thrust_scale = 30.0f);
    void draw_ground(sf::RenderWindow& window);
    void draw_axes(sf::RenderWindow& window, const sf::Font& font);
    void print_telemetry(const Rocket& rocket, int frame);
}

#endif // VISUALIZATION_H
