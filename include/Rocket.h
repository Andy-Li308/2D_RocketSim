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
    Eigen::VectorXd state;


class Rocket {
public:
    double m;
    double L;
    double g;
    double l;
    double I;
    Eigen::VectorXd state;
    double thrust;
    double alpha;

    Rocket(double mass = 1.0, double length = 0.5,
           Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(6),
           double gravity = 9.81);
    Eigen::VectorXd get_state() const;
    void set_control(double T, double alpha);
    void update(double dt);
    void check_ground_collision(double ground_level = 0.0);

private:
    Eigen::VectorXd compute_derivatives(const Eigen::VectorXd& state) const;
};
     * @return Copy of state [x, xdot, y, ydot, theta, thetadot]
