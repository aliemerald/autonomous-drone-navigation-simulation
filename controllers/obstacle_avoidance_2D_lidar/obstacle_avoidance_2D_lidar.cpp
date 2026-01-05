#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Lidar.hpp>

#include <cmath>
#include <iostream>
#include <vector>
#include <iomanip>
#include <numeric> // For std::accumulate

using namespace webots;

#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

struct Waypoint {
    double x;
    double y;
};

int main() {
    Robot *robot = new Robot();
    int timeStep = (int)robot->getBasicTimeStep();

    InertialUnit *imu = robot->getInertialUnit("inertial unit");
    imu->enable(timeStep);
    GPS *gps = robot->getGPS("gps");
    gps->enable(timeStep);
    Gyro *gyro = robot->getGyro("gyro");
    gyro->enable(timeStep);
    Lidar *lidar = robot->getLidar("lidar");
    lidar->enable(timeStep);
    // Ensure Lidar receives range image data
    lidar->enablePointCloud(); 


    Motor *front_left_motor = robot->getMotor("front left propeller");
    Motor *front_right_motor = robot->getMotor("front right propeller");
    Motor *rear_left_motor = robot->getMotor("rear left propeller");
    Motor *rear_right_motor = robot->getMotor("rear right propeller");

    Motor *motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
    for (int i = 0; i < 4; ++i) {
        motors[i]->setPosition(INFINITY);
        motors[i]->setVelocity(1.0);
    }

    std::vector<Waypoint> waypoints = {
        {2.0, 0.0},
        {2.0, 2.0},
        {0.0, 2.5},
        {-1.5, 1.5}
    };

    std::cout << "Starting waypoint mission..." << std::endl;
    std::cout << "Total waypoints: " << waypoints.size() << std::endl;

    size_t current_waypoint_index = 0;

    // Takeoff wait
    while (robot->step(timeStep) != -1) {
        if (robot->getTime() > 1.0)
            break;
    }

    const double k_vertical_thrust = 68.5;
    const double k_vertical_offset = 0.6;
    const double k_vertical_p = 3.0;
    const double k_roll_p = 50.0;
    const double k_pitch_p = 30.0;
    const double k_yaw_p = 1.0;
    const double k_max_yaw_disturbance = 1.5;

    const double k_pos_p = 0.4;
    const double k_max_pos_disturbance = 2.0;

    // Obstacle Avoidance Parameters
    const double obstacle_detection_distance = 0.8; // Distance in meters to consider an obstacle
    const double obstacle_avoidance_yaw_gain = 0.7; // How strongly to turn away from obstacles
    const double obstacle_avoidance_roll_gain = 0.5; // How strongly to move sideways (optional)
    const double obstacle_avoidance_zone_angle = M_PI / 3.0; // Angle (in radians) to check for obstacles in front (e.g., 45 degrees each side)

    double target_altitude = 2.0;
    double last_print_time = 0.0;

    while (robot->step(timeStep) != -1 && current_waypoint_index < waypoints.size()) {
        double time = robot->getTime();

        double roll = imu->getRollPitchYaw()[0];
        double pitch = imu->getRollPitchYaw()[1];
        double yaw = imu->getRollPitchYaw()[2];

        const double *gps_values = gps->getValues();
        double altitude = gps_values[2];
        double current_x = gps_values[0];
        double current_y = gps_values[1];

        double roll_velocity = gyro->getValues()[0];
        double pitch_velocity = gyro->getValues()[1];

        Waypoint target = waypoints[current_waypoint_index];

        double dx = target.x - current_x;
        double dy = target.y - current_y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_bearing = std::atan2(dy, dx);

        double heading_error = target_bearing - yaw;
        if (heading_error > M_PI) heading_error -= 2 * M_PI;
        if (heading_error < -M_PI) heading_error += 2 * M_PI;

        double pitch_disturbance = 0.0;
        double roll_disturbance = 0.0;
        double yaw_disturbance = 0.0;

        bool obstacle_detected = false;
        double avoidance_turn_direction = 0.0; // -1 for left, 1 for right, 0 for no turn

        // Lidar Obstacle Detection
        if (lidar->getRangeImage() != nullptr) {
            const float *ranges = lidar->getRangeImage();
            int resolution = lidar->getHorizontalResolution();
            double lidar_angle_step = lidar->getFov() / resolution;

            // Define the angular range for obstacle detection (e.g., +/- 45 degrees from front)
            int center_index = resolution / 2;
            int start_index = center_index - static_cast<int>(obstacle_avoidance_zone_angle / lidar_angle_step / 2);
            int end_index = center_index + static_cast<int>(obstacle_avoidance_zone_angle / lidar_angle_step / 2);

            // Clamp indices to ensure they are within bounds
            start_index = CLAMP(start_index, 0, resolution - 1);
            end_index = CLAMP(end_index, 0, resolution - 1);

            double min_front_distance = lidar->getMaxRange(); // Initialize with max possible range
            int closest_angle_index = -1;

            for (int i = start_index; i <= end_index; ++i) {
                if (ranges[i] < min_front_distance) {
                    min_front_distance = ranges[i];
                    closest_angle_index = i;
                }
            }
            
            // Check for obstacles in the left and right sectors within the detection zone
            // Prioritise turning away from the side with the closer obstacle
            double min_left_distance = lidar->getMaxRange();
            double min_right_distance = lidar->getMaxRange();

            for (int i = start_index; i < center_index; ++i) { // Left side
                if (ranges[i] < min_left_distance) {
                    min_left_distance = ranges[i];
                }
            }

            for (int i = center_index + 1; i <= end_index; ++i) { // Right side
                if (ranges[i] < min_right_distance) {
                    min_right_distance = ranges[i];
                }
            }


            if (min_front_distance < obstacle_detection_distance) {
                obstacle_detected = true;
                // Determine avoidance turn direction based on which side is "clearer" or has no obstacle
                if (min_left_distance > min_right_distance) {
                    avoidance_turn_direction = 1.0; // Turn right
                    // std::cout << "Obstacle detected, turning right. Front: " << min_front_distance << "m, Left: " << min_left_distance << "m, Right: " << min_right_distance << "m" << std::endl;
                } else {
                    avoidance_turn_direction = -1.0; // Turn left
                    // std::cout << "Obstacle detected, turning left. Front: " << min_front_distance << "m, Left: " << min_left_distance << "m, Right: " << min_right_distance << "m" << std::endl;
                }
            }
        }

        if (time - last_print_time >= 0.5) {
            last_print_time = time;
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Time: " << time << "s | ";
            std::cout << "Waypoint: " << current_waypoint_index + 1 << "/" << waypoints.size() << " | ";
            std::cout << "Current Pos: (" << current_x << ", " << current_y << ", Alt: " << altitude << ") | ";
            std::cout << "Target Pos: (" << target.x << ", " << target.y << ") | ";
            std::cout << "Dist: " << distance << "m | ";
            std::cout << "Heading Error: " << heading_error << " rad | ";
            std::cout << "Obstacle: " << (obstacle_detected ? "YES" : "NO") << std::endl;
        }

        if (distance < 0.2) {
            std::cout << "Waypoint " << current_waypoint_index + 1 << " reached!" << std::endl;
            current_waypoint_index++;
            if (current_waypoint_index >= waypoints.size()) {
                std::cout << "All waypoints visited. Mission complete. Hovering." << std::endl;
            }
        }

        if (obstacle_detected) {
            // Override waypoint navigation for obstacle avoidance
            yaw_disturbance = obstacle_avoidance_yaw_gain * avoidance_turn_direction;
            // roll_disturbance = obstacle_avoidance_roll_gain * avoidance_turn_direction; 
            pitch_disturbance = 0.0; // Stop forward movement when avoiding
        } else {
            // Normal waypoint navigation
            yaw_disturbance = k_yaw_p * heading_error;
            yaw_disturbance = CLAMP(yaw_disturbance, -k_max_yaw_disturbance, k_max_yaw_disturbance);

            double forward_error = dx * std::cos(yaw) + dy * std::sin(yaw);
            double sideways_error = -dx * std::sin(yaw) + dy * std::cos(yaw);

            pitch_disturbance = -k_pos_p * forward_error;
            roll_disturbance  = -k_pos_p * sideways_error;

            roll_disturbance = CLAMP(roll_disturbance, -k_max_pos_disturbance, k_max_pos_disturbance);
            pitch_disturbance = CLAMP(pitch_disturbance, -k_max_pos_disturbance, k_max_pos_disturbance);
        }

        double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
        double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
        double clamped_diff_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
        double vertical_input = k_vertical_p * std::pow(clamped_diff_altitude, 3.0);

        double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_disturbance;
        double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_disturbance;
        double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_disturbance;
        double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_disturbance;

        front_left_motor->setVelocity(front_left_motor_input);
        front_right_motor->setVelocity(-front_right_motor_input);
        rear_left_motor->setVelocity(-rear_left_motor_input);
        rear_right_motor->setVelocity(rear_right_motor_input);
    }

    std::cout << "Mission ended. Bye." << std::endl;
    front_left_motor->setVelocity(0.0);
    front_right_motor->setVelocity(0.0);
    rear_left_motor->setVelocity(0.0);
    rear_right_motor->setVelocity(0.0);

    delete robot;
    return 0;
}