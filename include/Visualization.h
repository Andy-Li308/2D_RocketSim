#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <SFML/Graphics.hpp>
#include "Rocket.h"

/**
 * @namespace Visualization
 * @brief SFML rendering utilities for rocket simulation
 */
namespace Visualization {
    // Rendering constants
    extern const float PIXELS_PER_METER;
    extern const float WINDOW_WIDTH;
    extern const float WINDOW_HEIGHT;
    extern const float GROUND_Y;

    /**
     * @brief Convert world coordinates to screen coordinates
     * @param x World X position (meters)
     * @param y World Y position (meters)
     * @return Screen position in pixels
     */
    sf::Vector2f world_to_screen(double x, double y);

    /**
     * @brief Draw the rocket as a rod with thrust vector and COM marker
     * @param window SFML render window
     * @param rocket Rocket object containing physics state
     * @param thrust_scale Scale factor for thrust arrow length
     */
    void draw_rocket(sf::RenderWindow& window, const Rocket& rocket, float thrust_scale = 30.0f);

    /**
     * @brief Draw the ground plane
     * @param window SFML render window
     */
    void draw_ground(sf::RenderWindow& window);

    /**
     * @brief Draw x/y coordinate axes with grid and labels
     * @param window SFML render window
     * @param font Font for axis labels
     */
    void draw_axes(sf::RenderWindow& window, const sf::Font& font);

    /**
     * @brief Print state telemetry to console
     * @param rocket Rocket object
     * @param frame Current frame number
     */
    void print_telemetry(const Rocket& rocket, int frame);
}

#endif // VISUALIZATION_H
