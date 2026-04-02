#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Eigen/Dense>
#include <string>

class Controller {
private:
    Eigen::Matrix<double, 2, 6> K;
    double max_thrust;
    double max_gimbal;
public:
    Controller(double max_thrust = 10.0, double max_gimbal = 0.785);
    bool load_gains_from_json(const std::string& filename);
    Eigen::Vector2d compute_control(const Eigen::VectorXd& state,
                                    const Eigen::VectorXd& state_ref);
    const Eigen::Matrix<double, 2, 6>& get_gains() const { return K; }
    void set_gains(const Eigen::Matrix<double, 2, 6>& new_K);
};

#endif // CONTROLLER_H
