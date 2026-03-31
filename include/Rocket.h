#ifndef ROCKET_H
#define ROCKET_H

#include <Eigen/Dense>
#include <vector>

/**
 * @class Rocket
 * @brief 6-DOF rigid body rocket simulator for 2D physics
 * 
 * State vector: [x, xdot, y, ydot, theta, thetadot]
 * Where:
 *   x, y = position in world frame
 *   xdot, ydot = velocity
 *   theta = angle from vertical (radians)
 *   thetadot = angular velocity
 */
class Rocket {
public:
    // Physics parameters
    double m;  // mass (kg)
    double L;  // length (m)
    double g;  // gravity (m/s^2)

    // Derived parameters
    double l;  // L/2 = distance from center to thrust application point
    double I;  // moment of inertia = (1/12)*m*L^2

    // State vector: [x, xdot, y, ydot, theta, thetadot]
    Eigen::VectorXd state;

    // Control inputs
    double thrust;  // T (N)
    double alpha;   // engine tilt angle relative to rocket (rad)

    // Constructor with customizable initial conditions
    Rocket(double mass = 1.0, 
           double length = 0.5,
           Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(6),
           double gravity = 9.81);

    /**
     * @brief Get current state vector
     * @return Copy of state [x, xdot, y, ydot, theta, thetadot]
     */
    Eigen::VectorXd get_state() const;

    /**
     * @brief Set control inputs
     * @param T Thrust magnitude (N)
     * @param alpha Engine tilt angle relative to rocket body (rad)
     */
    void set_control(double T, double alpha);

    /**
     * @brief Integrate state forward by dt using RK4
     * @param dt Time step (seconds)
     */
    void update(double dt);

    /**
     * @brief Check and handle ground collision
     * When rocket hits ground (y < 0), stop vertical motion and constrain to y=0
     * @param ground_level Ground height in world coordinates (default: y=0)
     */
    void check_ground_collision(double ground_level = 0.0);

private:
    /**
     * @brief Compute state derivatives
     * @param state Current state vector
     * @return State derivatives [xdot, xdotdot, ydot, ydotdot, thetadot, thetadotdot]
     */
    Eigen::VectorXd compute_derivatives(const Eigen::VectorXd& state) const;
};

#endif // ROCKET_H
