#ifndef ODOMETRY_COMPUTE_HPP
#define ODOMETRY_COMPUTE_HPP

// A structure to hold an ackerman vehicle's state
struct RobotState
{
    double x;       // X-coordinate in meters
    double y;       // Y-coordinate in meters
    double theta;   // Orientation (heading) in radians
    double phi;     // Overall steering angle in radians
};

// A structure to hold the total number of clicks from
// the left and right wheel when either rotates
struct WheelEncoders
{
    double total_left_ticks;
    double total_right_ticks;
};

// Namespace to hold constants used in the implementation
namespace Odometry
{
    double update_interval_s{ 0.1 };            // Value will be used in a loop that will update the state
    double encoder_ticks_per_revolution{ };     // The value will depend on the wheel encoder
}

namespace Ackerman
{
    class OdometryCompute
    {
        public:
            // Constructor passes values for wheel radius and wheelbase
            OdometryCompute(double wheel_radius_m, double wheelbase_m);

            // Calls the methods that will be used to calculate and update 
            // the parameters of the robot's state.
            // steering_angle is in degrees. It will be converted to radians
            void update_robot_state(double steering_angle);

        private:
            double wheel_radius_m_;     // Radius of a wheel
            double wheelbase_m_;        // Distance between front and rear wheel axle
            double robot_velocity_;     // Overall velocity from the center of the rear-wheel axle
            RobotState current_state_;  // Current robot position and orientation
            WheelEncoders previous_;    // Last recorded tick number from left and right encoders
            WheelEncoders current_;     // Current total tick number from left and right encoders
        
            // Calculate the overall linear velocity of the robot using the
            // number of ticks from the wheel encoders
            void compute_linear_velocity();

            // Calculate the orientation of the robot
            void compute_orientation();

            // Calculate the current x and y coordinates of the robot
            void compute_x_y_coordinates();
    };
}

#endif 