#include "Rocket.h"
#include <cmath>

Rocket::Rocket(double mass, double length, Eigen::VectorXd initial_state, double gravity)
    : m(mass), L(length), g(gravity) {
    
    // Derived parameters
    l = L / 2.0;  // distance from center to thrust point
    I = (1.0 / 12.0) * m * L * L;  // moment of inertia for uniform rod

    // Initialize state
    if (initial_state.size() == 6) {
        state = initial_state;
    } else {
        // Default: at origin, vertical orientation, at rest
        state = Eigen::VectorXd::Zero(6);
        state(0) = 0.0;   // x = 0
        state(1) = 0.0;   // xdot = 0
        state(2) = 5.0;   // y = 5 (above ground)
        state(3) = 0.0;   // ydot = 0
        state(4) = 0.0;   // theta = 0 (vertical)
        state(5) = 0.0;   // thetadot = 0
    }

    // Initialize control inputs
    thrust = 0.0;
    alpha = 0.0;
}

Eigen::VectorXd Rocket::get_state() const {
    return state;
}

void Rocket::set_control(double T, double alpha_cmd) {
    thrust = T;
    alpha = alpha_cmd;
}

Eigen::VectorXd Rocket::compute_derivatives(const Eigen::VectorXd& s) const {
    // State: [x, xdot, y, ydot, theta, thetadot]
    double x = s(0);
    double xdot = s(1);
    double y = s(2);
    double ydot = s(3);
    double theta = s(4);
    double thetadot = s(5);

    // Compute accelerations from physics equations
    // Translational:
    //   xdotdot = (T/m) * sin(theta + alpha)
    //   ydotdot = (T/m) * cos(theta + alpha) - g
    // Rotational:
    //   thetadotdot = (l*T/I) * sin(alpha)

    double thrust_angle = theta + alpha;
    double xdotdot = (thrust / m) * std::sin(thrust_angle);
    double ydotdot = (thrust / m) * std::cos(thrust_angle) - g;
    double thetadotdot = (l * thrust / I) * std::sin(alpha);

    // Return derivatives: [xdot, xdotdot, ydot, ydotdot, thetadot, thetadotdot]
    Eigen::VectorXd derivatives = Eigen::VectorXd::Zero(6);
    derivatives(0) = xdot;
    derivatives(1) = xdotdot;
    derivatives(2) = ydot;
    derivatives(3) = ydotdot;
    derivatives(4) = thetadot;
    derivatives(5) = thetadotdot;

    return derivatives;
}

void Rocket::update(double dt) {
    // RK4 integration
    Eigen::VectorXd k1 = compute_derivatives(state);
    Eigen::VectorXd k2 = compute_derivatives(state + 0.5 * dt * k1);
    Eigen::VectorXd k3 = compute_derivatives(state + 0.5 * dt * k2);
    Eigen::VectorXd k4 = compute_derivatives(state + dt * k3);

    state = state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

void Rocket::check_ground_collision(double ground_level) {
    // Check if rocket center of mass has hit the ground
    if (state(2) < ground_level) {
        // Constrain position to ground
        state(2) = ground_level;
        
        // Stop vertical motion
        state(3) = 0.0;  // ydot = 0
        
        // Dampen angular motion (friction/landing effect)
        state(5) *= 0.5;  // thetadot *= 0.5
    }
}
