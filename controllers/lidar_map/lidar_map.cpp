#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>

#include <cmath>
#include <iostream>
#include <vector>
#include <iomanip>

using namespace webots;

// Clamp a value between a low and high boundary.
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

// Represents a 3D waypoint for the drone to navigate to.
struct Waypoint {
    double x;
    double y;
    double z;
};

// Defines the operational states of the drone.
enum DroneState {
    TAKEOFF,
    NAVIGATING,
    MISSION_COMPLETE
};

int main() {
    // 1. INITIALIZATION
    //-------------------
    Robot *robot = new Robot();
    int timeStep = (int)robot->getBasicTimeStep();

    // Initialize devices
    InertialUnit *imu = robot->getInertialUnit("inertial unit");
    imu->enable(timeStep);
    GPS *gps = robot->getGPS("gps");
    gps->enable(timeStep);
    Gyro *gyro = robot->getGyro("gyro");
    gyro->enable(timeStep);
    Lidar *lidar = robot->getLidar("lidar");
    lidar->enable(timeStep);
    lidar->enablePointCloud();

    // Get motor pointers
    Motor *front_left_motor = robot->getMotor("front left propeller");
    Motor *front_right_motor = robot->getMotor("front right propeller");
    Motor *rear_left_motor = robot->getMotor("rear left propeller");
    Motor *rear_right_motor = robot->getMotor("rear right propeller");

    Motor *motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
    for (int i = 0; i < 4; ++i) {
        motors[i]->setPosition(INFINITY);
        motors[i]->setVelocity(1.0);
    }

    // 2. MISSION & CONTROL PARAMETERS
    //---------------------------------
    // Define the 3D flight path for the drone
    std::vector<Waypoint> waypoints = {
        {2.0, 0.0, 2.5},
        {2.0, 2.0, 3.0},
        {0.0, 2.5, 3.5},
        {-1.5, 1.5, 2.0},
        {0.0, 0.0, 2.0} // Return home
    };

    size_t current_waypoint_index = 0;
    DroneState currentState = TAKEOFF; // Start with the takeoff sequence
    double takeoff_altitude = 2.0;   // Safe altitude to reach before navigating

    std::cout << "Drone mission starting. State: TAKEOFF" << std::endl;
    std::cout << "Total waypoints: " << waypoints.size() << std::endl;

    // PID and motor control constants
    const double k_vertical_thrust = 68.5;
    const double k_vertical_offset = 0.6;
    const double k_vertical_p = 3.0;
    const double k_roll_p = 50.0;
    const double k_pitch_p = 30.0;
    const double k_yaw_p = 1.0;
    const double k_pos_p = 0.4;
    const double k_max_pos_disturbance = 2.0;
    const double k_max_yaw_disturbance = 1.5;

    // Obstacle avoidance parameters
    const double obstacle_detection_distance = 1.0;      // (m) How close to get before avoiding
    const double obstacle_avoidance_yaw_gain = 0.7;      // How sharply to turn away
    const float obstacle_avoidance_zone_angle_deg = 90.0; // (degrees) Field of view for detection

    double last_print_time = 0.0;

    // 3. MAIN CONTROL LOOP
    //----------------------
    while (robot->step(timeStep) != -1) {
        double time = robot->getTime();

        // Get sensor readings
        const double roll = imu->getRollPitchYaw()[0];
        const double pitch = imu->getRollPitchYaw()[1];
        const double yaw = imu->getRollPitchYaw()[2];
        const double* gps_values = gps->getValues();
        const double current_x = gps_values[0];
        const double current_y = gps_values[1];
        const double altitude = gps_values[2];
        const double roll_velocity = gyro->getValues()[0];
        const double pitch_velocity = gyro->getValues()[1];
        
        double roll_disturbance = 0.0;
        double pitch_disturbance = 0.0;
        double yaw_disturbance = 0.0;
        double vertical_input = 0.0;

        // --- State Machine Logic ---
        switch (currentState) {
            case TAKEOFF: {
                // Goal: Climb to takeoff_altitude
                double target_altitude = takeoff_altitude;
                double clamped_diff_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
                vertical_input = k_vertical_p * std::pow(clamped_diff_altitude, 3.0);
                
                // Maintain stable hover (no roll, pitch, or yaw disturbances)
                roll_disturbance = 0.0;
                pitch_disturbance = 0.0;
                yaw_disturbance = 0.0;

                // Check for state transition
                if (altitude > takeoff_altitude - 0.1) {
                    currentState = NAVIGATING;
                    std::cout << "Takeoff complete. State: NAVIGATING" << std::endl;
                }
                break;
            }

            case NAVIGATING: {
                // Check if mission is complete
                if (current_waypoint_index >= waypoints.size()) {
                    currentState = MISSION_COMPLETE;
                    std::cout << "All waypoints reached. State: MISSION_COMPLETE" << std::endl;
                    break;
                }

                Waypoint target = waypoints[current_waypoint_index];

                // Calculate distance and bearing to the target
                double dx = target.x - current_x;
                double dy = target.y - current_y;
                double distance_to_target = std::sqrt(dx * dx + dy * dy); // 2D distance for bearing
                double target_bearing = std::atan2(dy, dx);
                
                // --- Lidar Obstacle Detection Logic ---
                bool obstacle_detected = false;
                double avoidance_turn_direction = 0.0; // -1 for left, 1 for right

                if (lidar->getRangeImage() != nullptr) {
                    const float *ranges = lidar->getRangeImage();
                    int resolution = lidar->getHorizontalResolution();
                    double fov = lidar->getFov();
                    
                    int center_index = resolution / 2;
                    int zone_indices = (obstacle_avoidance_zone_angle_deg / 360.0) * resolution;
                    int start_index = CLAMP(center_index - zone_indices / 2, 0, resolution - 1);
                    int end_index = CLAMP(center_index + zone_indices / 2, 0, resolution - 1);
                    
                    double min_front_distance = lidar->getMaxRange();
                    double min_left_distance = lidar->getMaxRange();
                    double min_right_distance = lidar->getMaxRange();

                    // Find closest obstacle in the right-front sector
                    for (int i = start_index; i < center_index; ++i) {
                        if (ranges[i] < min_right_distance) min_right_distance = ranges[i];
                    }
                    // Find closest obstacle in the left-front sector
                    for (int i = center_index; i <= end_index; ++i) {
                        if (ranges[i] < min_left_distance) min_left_distance = ranges[i];
                    }
                    
                    min_front_distance = std::min(min_left_distance, min_right_distance);

                    if (min_front_distance < obstacle_detection_distance) {
                        obstacle_detected = true;
                        // Turn towards the side with more free space
                        avoidance_turn_direction = (min_left_distance > min_right_distance) ? -1.0 : 1.0;
                    }
                }

                // --- Control Logic (Navigation vs. Avoidance) ---
                if (obstacle_detected) {
                    // Obstacle avoidance is active
                    yaw_disturbance = obstacle_avoidance_yaw_gain * avoidance_turn_direction;
                    // Slow down forward movement to give time to turn
                    pitch_disturbance = -0.5; // Apply slight backward pitch
                    roll_disturbance = 0; // Don't roll while turning to avoid
                } else {
                    // Normal waypoint navigation
                    double heading_error = target_bearing - yaw;
                    if (heading_error > M_PI) heading_error -= 2 * M_PI;
                    if (heading_error < -M_PI) heading_error += 2 * M_PI;

                    yaw_disturbance = CLAMP(k_yaw_p * heading_error, -k_max_yaw_disturbance, k_max_yaw_disturbance);
                    
                    // Use transformed coordinates to calculate forward/sideways error
                    double forward_error = dx * std::cos(yaw) + dy * std::sin(yaw);
                    double sideways_error = -dx * std::sin(yaw) + dy * std::cos(yaw);
                    
                    pitch_disturbance = CLAMP(-k_pos_p * forward_error, -k_max_pos_disturbance, k_max_pos_disturbance);
                    roll_disturbance = CLAMP(-k_pos_p * sideways_error, -k_max_pos_disturbance, k_max_pos_disturbance);
                }

                // Altitude control is always active to reach target Z
                double clamped_diff_altitude = CLAMP(target.z - altitude + k_vertical_offset, -1.0, 1.0);
                vertical_input = k_vertical_p * std::pow(clamped_diff_altitude, 3.0);

                // --- Waypoint Completion Check ---
                double distance_3d = std::sqrt(dx * dx + dy * dy + std::pow(target.z - altitude, 2));
                if (distance_3d < 0.5) { // Use a 3D distance threshold
                    std::cout << "Waypoint " << current_waypoint_index + 1 << " reached!" << std::endl;
                    current_waypoint_index++;
                }
                
                // Print status
                if (time - last_print_time > 0.5) {
                    std::cout << std::fixed << std::setprecision(2);
                    std::cout << "Target: (" << target.x << ", " << target.y << ", " << target.z << ") | ";
                    std::cout << "Pos: (" << current_x << ", " << current_y << ", " << altitude << ") | ";
                    std::cout << "Dist: " << distance_3d << "m | ";
                    std::cout << "Obstacle: " << (obstacle_detected ? "YES" : "NO") << std::endl;
                    last_print_time = time;
                }
                break;
            }

            case MISSION_COMPLETE: {
                // Mission is done, just hover in place.
                vertical_input = 0;
                roll_disturbance = 0;
                pitch_disturbance = 0;
                yaw_disturbance = 0;
                break;
            }
        }

        // 4. MOTOR CONTROL
        //------------------
        // Combine disturbances and vertical input to calculate motor speeds
        double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
        double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
        
        double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_disturbance;
        double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_disturbance;
        double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_disturbance;
        double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_disturbance;

        // Set motor velocities
        front_left_motor->setVelocity(front_left_motor_input);
        front_right_motor->setVelocity(-front_right_motor_input);
        rear_left_motor->setVelocity(-rear_left_motor_input);
        rear_right_motor->setVelocity(rear_right_motor_input);
    }

    // Cleanup
    delete robot;
    return 0;
}