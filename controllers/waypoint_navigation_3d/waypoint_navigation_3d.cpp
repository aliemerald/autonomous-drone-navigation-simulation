#include <webots/Supervisor.hpp>
#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>

#include <cmath>
#include <iostream>
#include <vector>
#include <iomanip>

using namespace webots;

#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

struct Waypoint {
    double x;
    double y;
    double z;
};

int main() {
    Supervisor *supervisor = new Supervisor();
    int timeStep = (int)supervisor->getBasicTimeStep();

    // --- SENSOR & MOTOR INITIALIZATION (No changes here) ---
    InertialUnit *imu = supervisor->getInertialUnit("inertial unit");
    imu->enable(timeStep);
    GPS *gps = supervisor->getGPS("gps");
    gps->enable(timeStep);
    Gyro *gyro = supervisor->getGyro("gyro");
    gyro->enable(timeStep);
    Motor *front_left_motor = supervisor->getMotor("front left propeller");
    Motor *front_right_motor = supervisor->getMotor("front right propeller");
    Motor *rear_left_motor = supervisor->getMotor("rear left propeller");
    Motor *rear_right_motor = supervisor->getMotor("rear right propeller");
    Motor *motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
    for (int i = 0; i < 4; ++i) {
        motors[i]->setPosition(INFINITY);
        motors[i]->setVelocity(1.0);
    }

    // --- WAYPOINTS ---
    std::vector<Waypoint> waypoints = {
        {2.0, 0.0, 2.0},
        {2.0, 2.0, 3.0},
        {0.0, 2.0, 2.5},
        {0.0, 0.0, 2.0}
    };

    std::cout << "Starting waypoint mission..." << std::endl;
    size_t current_waypoint_index = 0;

    // --- SCENE TREE AND VISUALIZATION SETUP ---
    Node *waypoint_marker_node = supervisor->getFromDef("WAYPOINT_MARKER");
    Field *waypoint_marker_translation = waypoint_marker_node ? waypoint_marker_node->getField("translation") : nullptr;
    Node *flight_path_line_node = supervisor->getFromDef("FLIGHT_PATH_LINE");
    Field *flight_path_points_field = nullptr;
    Field *flight_path_coord_index_field = nullptr;
    if (flight_path_line_node) {
        flight_path_coord_index_field = flight_path_line_node->getField("coordIndex");
        Node *coord_node = flight_path_line_node->getField("coord")->getSFNode();
        if (coord_node) {
            flight_path_points_field = coord_node->getField("point");
        }
    }
    
    // NEW: --- TRAIL OPTIMIZATION SETUP ---
    // Only add a point to the trail if the drone has moved more than this distance.
    const double MIN_TRAIL_DISTANCE = 0.2; // meters
    double last_trail_point[3] = {0.0, 0.0, 0.0};
    bool is_first_trail_point = true; // Flag to ensure the first point is always added.
    int path_point_count = 0;
    // --- END NEW SECTION ---

    if (waypoint_marker_translation && !waypoints.empty()) {
        const double initial_waypoint_pos[3] = {waypoints[0].x, waypoints[0].y, waypoints[0].z};
        waypoint_marker_translation->setSFVec3f(initial_waypoint_pos);
    }

    // Takeoff wait
    while (supervisor->step(timeStep) != -1) {
        if (supervisor->getTime() > 1.0)
            break;
    }

    // --- FLIGHT CONSTANTS (No changes here) ---
    const double k_vertical_thrust = 68.5;
    const double k_vertical_offset = 0.6;
    const double k_vertical_p = 3.0;
    const double k_roll_p = 50.0;
    const double k_pitch_p = 30.0;
    const double k_yaw_p = 0.5;
    const double k_max_yaw_disturbance = 1.5;
    const double k_pos_p = 0.4;
    const double k_max_pos_disturbance = 2.0;
    double last_print_time = 0.0;

    // --- MAIN FLIGHT LOOP ---
    while (supervisor->step(timeStep) != -1 && current_waypoint_index < waypoints.size()) {
        double time = supervisor->getTime();

        // Get sensor data
        const double *imu_values = imu->getRollPitchYaw();
        double roll = imu_values[0], pitch = imu_values[1], yaw = imu_values[2];
        const double *gps_values = gps->getValues();
        double current_x = gps_values[0], current_y = gps_values[1], current_z = gps_values[2];
        const double *gyro_values = gyro->getValues();
        double roll_velocity = gyro_values[0], pitch_velocity = gyro_values[1];

        // NEW: --- OPTIMIZED FLIGHT PATH TRAIL ---
        double dist_from_last = std::sqrt(pow(current_x - last_trail_point[0], 2) +
                                          pow(current_y - last_trail_point[1], 2) +
                                          pow(current_z - last_trail_point[2], 2));

        // Check if it's the first point or if the drone has moved far enough.
        if (is_first_trail_point || dist_from_last > MIN_TRAIL_DISTANCE) {
            if (flight_path_points_field && flight_path_coord_index_field) {
                const double new_point[3] = {current_x, current_y, current_z};
                flight_path_points_field->insertMFVec3f(-1, new_point);
                
                if (path_point_count > 0) {
                    flight_path_coord_index_field->insertMFInt32(-1, path_point_count - 1);
                    flight_path_coord_index_field->insertMFInt32(-1, path_point_count);
                    flight_path_coord_index_field->insertMFInt32(-1, -1);
                }
                path_point_count++;

                // Update the last point's position to the current position.
                last_trail_point[0] = current_x;
                last_trail_point[1] = current_y;
                last_trail_point[2] = current_z;
                is_first_trail_point = false;
            }
        }
        // --- END NEW SECTION ---

        // Calculate distance and bearing to target
        Waypoint target = waypoints[current_waypoint_index];
        double dx = target.x - current_x, dy = target.y - current_y, dz = target.z - current_z;
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        double target_bearing = std::atan2(dy, dx);
        double heading_error = target_bearing - yaw;
        if (heading_error > M_PI) heading_error -= 2 * M_PI;
        if (heading_error < -M_PI) heading_error += 2 * M_PI;

        // Print status... (No changes here)
        
        double pitch_disturbance = 0.0, roll_disturbance = 0.0, yaw_disturbance = 0.0;

        // Check for waypoint arrival
        if (distance < 0.3) {
            std::cout << "Waypoint " << current_waypoint_index + 1 << " reached!" << std::endl;
            current_waypoint_index++;

            if (current_waypoint_index >= waypoints.size()) {
                std::cout << "All waypoints visited. Mission complete. Hovering." << std::endl;
                if (waypoint_marker_translation) {
                    const double hidden_pos[3] = {0.0, 0.0, -0.1};
                    waypoint_marker_translation->setSFVec3f(hidden_pos);
                }
            } else {
                std::cout << "Proceeding to waypoint " << current_waypoint_index + 1 << std::endl;
                if (waypoint_marker_translation) {
                    const Waypoint& next_target = waypoints[current_waypoint_index];
                    const double next_waypoint_pos[3] = {next_target.x, next_target.y, next_target.z};
                    waypoint_marker_translation->setSFVec3f(next_waypoint_pos);
                }
            }
        } else {
            // Navigation logic... (No changes here)
            yaw_disturbance = k_yaw_p * heading_error;
            yaw_disturbance = CLAMP(yaw_disturbance, -k_max_yaw_disturbance, k_max_yaw_disturbance);
            double forward_error = dx * std::cos(yaw) + dy * std::sin(yaw);
            double sideways_error = -dx * std::sin(yaw) + dy * std::cos(yaw);
            pitch_disturbance = -k_pos_p * forward_error;
            roll_disturbance = -k_pos_p * sideways_error;
            roll_disturbance = CLAMP(roll_disturbance, -k_max_pos_disturbance, k_max_pos_disturbance);
            pitch_disturbance = CLAMP(pitch_disturbance, -k_max_pos_disturbance, k_max_pos_disturbance);
        }

        // Motor input calculations... (No changes here)
        double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
        double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
        double clamped_diff_altitude = CLAMP(target.z - current_z + k_vertical_offset, -1.0, 1.0);
        double vertical_input = k_vertical_p * std::pow(clamped_diff_altitude, 3.0);
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

    std::cout << "Mission ended. Bye." << std::endl;
    for (int i = 0; i < 4; ++i) {
        motors[i]->setVelocity(0.0);
    }

    delete supervisor;
    return 0;
}