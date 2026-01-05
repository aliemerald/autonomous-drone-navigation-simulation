#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/Node.hpp>        // Required to interact with simulation nodes
#include <webots/Supervisor.hpp>  // Use Supervisor to control the world
#include <webots/Field.hpp>       // Required for field manipulation

#include <cmath>
#include <iostream>
#include <vector>
#include <iomanip>
#include <string> // Required for creating the path dot string

using namespace webots;

#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

struct Waypoint {
    double x;
    double y;
};

// Function to create a dot string for the flight path or a reached waypoint
std::string createDotString(double x, double y, double z, double transparency = 0.0) {
    std::string dot_string = "Transform { translation " +
                             std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) +
                             " children [ Shape { appearance PBRAppearance { baseColor 0.145 0.588 0.745 ";
    if (transparency > 0.0) {
        dot_string += "transparency " + std::to_string(transparency);
    }
    dot_string += "} geometry Sphere { radius 0.01 } } ] }";
    return dot_string;
}

int main() {
    Supervisor *robot = new Supervisor();
    int timeStep = (int)robot->getBasicTimeStep();

    InertialUnit *imu = robot->getInertialUnit("inertial unit");
    imu->enable(timeStep);
    GPS *gps = robot->getGPS("gps");
    gps->enable(timeStep);
    Gyro *gyro = robot->getGyro("gyro");
    gyro->enable(timeStep);

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
        {0.0, 2.0},
        {0.0, 0.0}
    };

    Waypoint home_position = {0.0, 0.0};

    std::cout << "Starting waypoint mission..." << std::endl;
    std::cout << "Total waypoints: " << waypoints.size() << std::endl;

    Node *waypoint_marker_node = robot->getFromDef("WAYPOINT_MARKER");
    // Change: Use "translation" field for Pose node instead of "position"
    Field *marker_translation_field = waypoint_marker_node ? waypoint_marker_node->getField("translation") : nullptr;

    Node *flight_path_group_node = robot->getFromDef("OLD_FLIGHT_PATH");
    Field *flight_path_children_field = flight_path_group_node ? flight_path_group_node->getField("children") : nullptr;

    size_t current_waypoint_index = 0;
    bool landing_sequence_started = false;
    bool waypoint_marker_initialized = false; // Flag to ensure marker is set only once for each new waypoint

    while (robot->step(timeStep) != -1 && robot->getTime() < 1.0) {
        // Initial wait or setup time if needed
    }

    const double k_vertical_thrust = 68.5;
    const double k_vertical_offset = 0.6;
    const double k_vertical_p = 3.0;
    const double k_roll_p = 50.0;
    const double k_pitch_p = 30.0;
    const double k_yaw_p = 1.5;
    const double k_max_yaw_disturbance = 1.5;
    const double k_pos_p = 0.4;
    const double k_max_pos_disturbance = 8.0;

    double target_altitude = 2.0;
    double landing_target_altitude = 0.1;
    double landing_descent_rate = 0.05;

    double last_print_time = 0.0;
    double last_path_dot_time = 0.0;

    while (robot->step(timeStep) != -1) {
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

        Waypoint target;
        if (!landing_sequence_started) {
            if (current_waypoint_index < waypoints.size()) {
                target = waypoints[current_waypoint_index];
            } else {
                landing_sequence_started = true;
                target = home_position;
                // Hide the main waypoint marker when landing sequence starts
                if (marker_translation_field) {
                    const double hidden_position[3] = {0.0, 0.0, -100.0};
                    marker_translation_field->setSFVec3f(hidden_position);
                }
                std::cout << "All waypoints visited. Returning to home for landing." << std::endl;
            }
        } else {
            target = home_position;
        }

        // Update waypoint marker translation if a new waypoint is active
        if (marker_translation_field && !landing_sequence_started && !waypoint_marker_initialized) {
            const double marker_position[3] = {target.x, target.y, 2.0}; // Assuming 2.0 is your desired marker height
            marker_translation_field->setSFVec3f(marker_position);
            waypoint_marker_initialized = true; // Mark as initialized for the current waypoint
            std::cout << "Waypoint marker moved to: (" << target.x << ", " << target.y << ")" << std::endl;
        }

        if (flight_path_children_field && (time - last_path_dot_time > 0.1)) {
            last_path_dot_time = time;
            std::string path_dot_string = createDotString(current_x, current_y, altitude);
            flight_path_children_field->importMFNodeFromString(-1, path_dot_string);
        }

        double dx = target.x - current_x;
        double dy = target.y - current_y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double target_bearing = std::atan2(dy, dx);

        double heading_error = target_bearing - yaw;
        if (heading_error > M_PI) heading_error -= 2 * M_PI;
        if (heading_error < -M_PI) heading_error += 2 * M_PI;

        if (time - last_print_time >= 0.5) {
            last_print_time = time;
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Time: " << time << "s | ";
            if (!landing_sequence_started) {
                std::cout << "Waypoint: " << current_waypoint_index + 1 << "/" << waypoints.size() << " | ";
            } else {
                std::cout << "Landing Sequence | ";
            }
            std::cout << "Current Pos: (" << current_x << ", " << current_y << ", Alt: " << altitude << ") | ";
            std::cout << "Target Pos: (" << target.x << ", " << target.y << ") | ";
            std::cout << "Dist: " << distance << "m | ";
            std::cout << "Heading Error: " << heading_error << " rad" << std::endl;
        }

        double pitch_disturbance = 0.0;
        double roll_disturbance = 0.0;
        double yaw_disturbance = 0.0;

        if (!landing_sequence_started) {
            if (distance < 0.2) { // Waypoint reached threshold
                std::cout << "Waypoint " << current_waypoint_index + 1 << " reached!" << std::endl;
                
                // Add a semi-transparent marker for the reached waypoint to the flight path group
                if (flight_path_children_field) {
                    std::string reached_waypoint_marker_string = createDotString(target.x, target.y, 2.0, 0.5); // 0.5 for semi-transparent
                    flight_path_children_field->importMFNodeFromString(-1, reached_waypoint_marker_string);
                }

                current_waypoint_index++;
                waypoint_marker_initialized = false; // Reset flag to move marker to the next waypoint
            }
        }

        if (current_waypoint_index >= waypoints.size() && !landing_sequence_started) {
            landing_sequence_started = true;
            std::cout << "All waypoints visited. Initiating return to home and landing sequence." << std::endl;
            if (marker_translation_field) {
                const double hidden_position[3] = {0.0, 0.0, -100.0};
                marker_translation_field->setSFVec3f(hidden_position);
            }
        }

        if (landing_sequence_started) {
            if (altitude > landing_target_altitude + 0.1) {
                yaw_disturbance = k_yaw_p * heading_error;
                yaw_disturbance = CLAMP(yaw_disturbance, -k_max_yaw_disturbance, k_max_yaw_disturbance);

                double forward_error = dx * std::cos(yaw) + dy * std::sin(yaw);
                double sideways_error = -dx * std::sin(yaw) + dy * std::cos(yaw);

                pitch_disturbance = -k_pos_p * forward_error;
                roll_disturbance = -k_pos_p * sideways_error;

                roll_disturbance = CLAMP(roll_disturbance, -k_max_pos_disturbance, k_max_pos_disturbance);
                pitch_disturbance = CLAMP(pitch_disturbance, -k_max_pos_disturbance, k_max_pos_disturbance);
            } else {
                yaw_disturbance = 0.0;
                pitch_disturbance = 0.0;
                roll_disturbance = 0.0;
            }

            target_altitude = std::max(landing_target_altitude, target_altitude - landing_descent_rate * timeStep / 1000.0);

            if (altitude < landing_target_altitude + 0.05) {
                std::cout << "Landed at home position. Mission complete." << std::endl;
                break;
            }
        } else {
            yaw_disturbance = k_yaw_p * heading_error;
            yaw_disturbance = CLAMP(yaw_disturbance, -k_max_yaw_disturbance, k_max_yaw_disturbance);

            double forward_error = dx * std::cos(yaw) + dy * std::sin(yaw);
            double sideways_error = -dx * std::sin(yaw) + dy * std::cos(yaw);

            pitch_disturbance = -k_pos_p * forward_error;
            roll_disturbance = -k_pos_p * sideways_error;

            roll_disturbance = CLAMP(roll_disturbance, -k_max_pos_disturbance, k_max_pos_disturbance);
            pitch_disturbance = CLAMP(pitch_disturbance, -k_max_pos_disturbance, k_max_pos_disturbance);

            target_altitude = 2.0; // Maintain altitude during waypoint navigation
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

    std::cout << "Mission ended. Shutting down motors." << std::endl;
    front_left_motor->setVelocity(0.0);
    front_right_motor->setVelocity(0.0);
    rear_left_motor->setVelocity(0.0);
    rear_right_motor->setVelocity(0.0);

    delete robot;
    return 0;
}