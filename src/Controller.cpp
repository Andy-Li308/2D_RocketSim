#include "Controller.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

Controller::Controller(double max_thrust, double max_gimbal)
    : max_thrust(max_thrust), max_gimbal(max_gimbal) {
    K = Eigen::Matrix<double, 2, 6>::Zero();
}

bool Controller::load_gains_from_json(const std::string& filename) {
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "[!] Error: Could not open " << filename << std::endl;
            return false;
        }
        
        // Read entire file into string
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string content = buffer.str();
        
        // Find the k_matrix section
        size_t k_start = content.find("\"k_matrix\"");
        if (k_start == std::string::npos) {
            std::cerr << "[!] Error: k_matrix not found in JSON" << std::endl;
            return false;
        }
        
        // Find the array opening bracket after k_matrix
        size_t array_start = content.find('[', k_start);
        if (array_start == std::string::npos) {
            std::cerr << "[!] Error: Could not find array in k_matrix" << std::endl;
            return false;
        }
        
        // Extract just the array part
        size_t array_end = array_start;
        int bracket_count = 0;
        bool in_brackets = false;
        for (size_t i = array_start; i < content.length(); i++) {
            if (content[i] == '[') {
                bracket_count++;
                in_brackets = true;
            } else if (content[i] == ']') {
                bracket_count--;
                if (bracket_count == 0 && in_brackets) {
                    array_end = i;
                    break;
                }
            }
        }
        
        std::string array_str = content.substr(array_start, array_end - array_start + 1);
        
        // Parse the 2D array manually
        int matrix_row = 0;
        int matrix_col = 0;
        std::string number_str;
        
        for (char c : array_str) {
            if (std::isdigit(c) || c == '.' || c == '-' || c == 'e' || c == 'E' || c == '+') {
                number_str += c;
            } else if ((c == ',' || c == ']' || c == '[') && !number_str.empty()) {
                try {
                    double value = std::stod(number_str);
                    K(matrix_row, matrix_col) = value;
                    matrix_col++;
                    number_str.clear();
                } catch (const std::exception& e) {
                    // Skip invalid numbers
                    number_str.clear();
                }
            }
            
            if (c == ']' && matrix_col > 0) {
                matrix_row++;
                matrix_col = 0;
            }
        }
        
        if (matrix_row < 2) {
            std::cerr << "[!] Error: Could not parse full K matrix (got " << matrix_row << " rows)" << std::endl;
            return false;
        }
        
        std::cout << "[+] Loaded LQR gains from " << filename << std::endl;
        std::cout << "    K matrix (2x6):" << std::endl;
        std::cout << "    Thrust:  ";
        for (int i = 0; i < 6; i++) std::cout << K(0, i) << " ";
        std::cout << std::endl;
        std::cout << "    Gimbal:  ";
        for (int i = 0; i < 6; i++) std::cout << K(1, i) << " ";
        std::cout << std::endl << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "[!] Error loading gains: " << e.what() << std::endl;
        return false;
    }
}

Eigen::Vector2d Controller::compute_control(const Eigen::VectorXd& state,
                                            const Eigen::VectorXd& state_ref) {
    if (state.size() != 6 || state_ref.size() != 6) {
        std::cerr << "[!] Error: State vector must be size 6" << std::endl;
        return Eigen::Vector2d::Zero();
    }
    
    // Compute state error
    Eigen::VectorXd error = state - state_ref;
    
    // LQR control law: u = -K * error
    Eigen::Vector2d control = -K * error;
    
    // Saturate controls
    // Thrust perturbation can be negative (reduce thrust below hover)
    // Total thrust clamping happens in main after adding feedforward (mg)
    control(0) = std::clamp(control(0), -max_thrust, max_thrust);
    
    // Gimbal angle constrained to [-max_gimbal, max_gimbal]
    control(1) = std::clamp(control(1), -max_gimbal, max_gimbal);
    
    return control;
}

void Controller::set_gains(const Eigen::Matrix<double, 2, 6>& new_K) {
    K = new_K;
}
