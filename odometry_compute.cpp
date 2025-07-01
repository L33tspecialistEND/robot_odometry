#include "odometry_compute.hpp"
#include <cmath>

namespace Ackerman
{
    // Constructor to initialise the member variables and set
    // the initial robot state
    OdometryCompute::OdometryCompute(double wheel_radius_m, double wheelbase_m) :
    wheel_radius_m_(wheel_radius_m),
    wheelbase_m_(wheelbase_m)
    {
        robot_velocity_ = 0.0;
        current_state_ = {0.0, 0.0, 0.0, 0.0};

        // Before calling any methods, previous_ must be updated with the values
        // of current_ and current_ must be updated from the program
        previous_ = {0.0, 0.0};                 
        current_ = {0.0, 0.0};
    }

    void OdometryCompute::update_robot_state(double steering_angle)
    {
        // Conversion from degrees to radians
        // theta = radians * pi / 180
        current_state_.phi = steering_angle * M_PI / 180;

        compute_linear_velocity();
        compute_orientation();
        compute_x_y_coordinates();
    }

    // Method to calculate the overall linear velocity of the robot
    void OdometryCompute::compute_linear_velocity()
        {
            // Angular displacements of the left and right wheels
            // Angular displacement (theta) = (current_ticks - previous_ticks) * 2 * pi / ticks_per_revolution
            double left_angular_displacement{ 
                (current_.total_left_ticks - previous_.total_left_ticks) *
                2 * M_PI / Odometry::encoder_ticks_per_revolution };
            double right_angular_displacement{ 
                (current_.total_right_ticks - previous_.total_right_ticks) *
                2 * M_PI / Odometry::encoder_ticks_per_revolution };

            // Angular velocities of the left and right wheels
            // ω = Δθ / Δt
            double left_omega{ left_angular_displacement / Odometry::update_interval_s };
            double right_omega{ right_angular_displacement / Odometry::update_interval_s };

            // Linear velocities of the left and right wheels
            // u = ωr
            double left_velocity{ left_omega * wheel_radius_m_ };
            double right_velocity{ right_omega * wheel_radius_m_ };

            // Robot linear velocity, v
            robot_velocity_ =  (left_velocity + right_velocity) / 2;
        }

    void OdometryCompute::compute_orientation()
    {
        // The instantaneous rate of change of the robot's heading
        // θ' = (v / l) * tan(ϕ)
        double theta_dot{ robot_velocity_ / wheelbase_m_ * std::tan(current_state_.phi) };

        // Current heading
        // θ = θ + (θ' * Δt)
        current_state_.theta += theta_dot * Odometry::update_interval_s;
    }

    void OdometryCompute::compute_x_y_coordinates()
    {
        // The instantaneous rate of change of the x and y coordinates
        // x' = v * cos(θ)
        double x_dot{ robot_velocity_ * std::cos(current_state_.theta)};
    
        // y' = v * sin(θ)
        double y_dot{ robot_velocity_ * std::sin(current_state_.theta)};

        // Current x-coordinate
        // x = x + (x' * Δt)
        current_state_.x += x_dot * Odometry::update_interval_s;

        // Current y-coordinate
        // y = y + (y' * Δt)
        current_state_.y += y_dot * Odometry::update_interval_s;
    }
}