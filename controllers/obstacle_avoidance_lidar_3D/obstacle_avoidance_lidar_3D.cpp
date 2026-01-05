// ADDED FOR VISUALIZATION
#include <webots/Supervisor.hpp>
// ---
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
// ADDED FOR VISUALIZATION
#include <webots/Node.hpp>
#include <webots/Field.hpp>
// ---

#include <cmath>
#include <iostream>
#include <vector>
#include <iomanip>

// NOTE: This using statement is from your original code.
using namespace webots;

// CLAMP macro is from your original code.
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

// Waypoint struct is from your original code.
struct Waypoint {
    double x;
    double y;
    double z;
};

// DroneState enum is from your original code.
enum DroneState {
    TAKEOFF,
    NAVIGATING,
    MISSION_COMPLETE
};

int main() {
    // 1. INITIALIZATION
    //-------------------
    // CHANGED FOR VISUALIZATION: Using Supervisor instead of Robot
    Supervisor *supervisor = new Supervisor();
    int timeStep = (int)supervisor->getBasicTimeStep();

    // Device initialization is identical to your original code.
    InertialUnit *imu = supervisor->getInertialUnit("inertial unit");
    imu->enable(timeStep);
    GPS *gps = supervisor->getGPS("gps");
    gps->enable(timeStep);
    Gyro *gyro = supervisor->getGyro("gyro");
    gyro->enable(timeStep);
    Lidar *lidar = supervisor->getLidar("lidar");
    lidar->enable(timeStep);
    lidar->enablePointCloud();

    // Motor setup is identical to your original code.
    Motor *front_left_motor = supervisor->getMotor("front left propeller");
    Motor *front_right_motor = supervisor->getMotor("front right propeller");
    Motor *rear_left_motor = supervisor->getMotor("rear left propeller");
    Motor *rear_right_motor = supervisor->getMotor("rear right propeller");

    Motor *motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
    for (int i = 0; i < 4; ++i) {
        motors[i]->setPosition(INFINITY);
        motors[i]->setVelocity(1.0);
    }

    // 2. MISSION & CONTROL PARAMETERS
    //---------------------------------
    // Waypoint definition is identical to your original code.
    std::vector<Waypoint> waypoints = {
        {2.0, 0.0, 2.5},
        {2.0, 2.0, 3.0},
        {0.0, 2.5, 3.5},
        {-1.5, 1.5, 2.0},
        {0.0, 0.0, 2.0} // Return home
    };

    // State machine and mission variables are identical to your original code.
    size_t current_waypoint_index = 0;
    DroneState currentState = TAKEOFF; 
    double takeoff_altitude = 2.0;

    std::cout << "Drone mission starting. State: TAKEOFF" << std::endl;
    std::cout << "Total waypoints: " << waypoints.size() << std::endl;

    // --- ADDED FOR VISUALIZATION ---
    // Get references to the marker and flight path nodes in the world.
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
    
    // Variables to manage drawing the flight trail.
    const double MIN_TRAIL_DISTANCE = 0.2; 
    double last_trail_point[3] = {0.0, 0.0, 0.0};
    bool is_first_trail_point = true; 
    int path_point_count = 0;
    // --- END OF VISUALIZATION ADDITIONS ---

    // All control constants are identical to your original code.
    const double k_vertical_thrust = 68.5;
    const double k_vertical_offset = 0.6;
    const double k_vertical_p = 3.0;
    const double k_roll_p = 50.0;
    const double k_pitch_p = 30.0;
    const double k_yaw_p = 1.0;
    const double k_pos_p = 0.4;
    const double k_max_pos_disturbance = 2.0;
    const double k_max_yaw_disturbance = 1.5;
    const double obstacle_detection_distance = 1.0;
    const double obstacle_avoidance_yaw_gain = 0.7;
    const float obstacle_avoidance_zone_angle_deg = 90.0;

    double last_print_time = 0.0;

    // 3. MAIN CONTROL LOOP
    //----------------------
    while (supervisor->step(timeStep) != -1) {
        // NOTE: The `supervisor->` calls here are the only change in this section.
        double time = supervisor->getTime();

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

        // State Machine Logic is identical to your original code.
        switch (currentState) {
            case TAKEOFF: {
                // TAKEOFF logic is identical.
                double target_altitude = takeoff_altitude;
                double clamped_diff_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
                vertical_input = k_vertical_p * std::pow(clamped_diff_altitude, 3.0);
                
                roll_disturbance = 0.0;
                pitch_disturbance = 0.0;
                yaw_disturbance = 0.0;

                if (altitude > takeoff_altitude - 0.1) {
                    currentState = NAVIGATING;
                    std::cout << "Takeoff complete. State: NAVIGATING" << std::endl;
                    
                    // --- ADDED FOR VISUALIZATION ---
                    // Set the marker to the first waypoint's position.
                    if (waypoint_marker_translation && !waypoints.empty()) {
                        const Waypoint& target = waypoints[current_waypoint_index];
                        const double waypoint_pos[3] = {target.x, target.y, target.z};
                        waypoint_marker_translation->setSFVec3f(waypoint_pos);
                    }
                    // ---
                }
                break;
            }

            case NAVIGATING: {
                if (current_waypoint_index >= waypoints.size()) {
                    currentState = MISSION_COMPLETE;
                    std::cout << "All waypoints reached. State: MISSION_COMPLETE" << std::endl;
                    break;
                }

                // --- ADDED FOR VISUALIZATION ---
                // This entire block draws the flight path trail.
                double dist_from_last = std::sqrt(pow(current_x - last_trail_point[0], 2) +
                                                  pow(current_y - last_trail_point[1], 2) +
                                                  pow(altitude - last_trail_point[2], 2));

                if (is_first_trail_point || dist_from_last > MIN_TRAIL_DISTANCE) {
                    if (flight_path_points_field && flight_path_coord_index_field) {
                        const double new_point[3] = {current_x, current_y, altitude};
                        flight_path_points_field->insertMFVec3f(-1, new_point);
                        
                        if (path_point_count > 0) {
                            flight_path_coord_index_field->insertMFInt32(-1, path_point_count - 1);
                            flight_path_coord_index_field->insertMFInt32(-1, path_point_count);
                            flight_path_coord_index_field->insertMFInt32(-1, -1);
                        }
                        path_point_count++;

                        last_trail_point[0] = current_x;
                        last_trail_point[1] = current_y;
                        last_trail_point[2] = altitude;
                        is_first_trail_point = false;
                    }
                }
                // --- END OF VISUALIZATION ADDITION ---

                // The rest of the NAVIGATING logic is IDENTICAL to your original code.
                Waypoint target = waypoints[current_waypoint_index];
                double dx = target.x - current_x;
                double dy = target.y - current_y;
                double target_bearing = std::atan2(dy, dx);
                
                bool obstacle_detected = false;
                double avoidance_turn_direction = 0.0;

                if (lidar->getRangeImage() != nullptr) {
                    const float *ranges = lidar->getRangeImage();
                    int resolution = lidar->getHorizontalResolution();
                    int center_index = resolution / 2;
                    int zone_indices = (obstacle_avoidance_zone_angle_deg / 360.0) * resolution;
                    int start_index = CLAMP(center_index - zone_indices / 2, 0, resolution - 1);
                    int end_index = CLAMP(center_index + zone_indices / 2, 0, resolution - 1);
                    double min_left_distance = lidar->getMaxRange();
                    double min_right_distance = lidar->getMaxRange();

                    for (int i = start_index; i < center_index; ++i) {
                        if (ranges[i] < min_right_distance) min_right_distance = ranges[i];
                    }
                    for (int i = center_index; i <= end_index; ++i) {
                        if (ranges[i] < min_left_distance) min_left_distance = ranges[i];
                    }
                    double min_front_distance = std::min(min_left_distance, min_right_distance);

                    if (min_front_distance < obstacle_detection_distance) {
                        obstacle_detected = true;
                        avoidance_turn_direction = (min_left_distance > min_right_distance) ? -1.0 : 1.0;
                    }
                }

                if (obstacle_detected) {
                    yaw_disturbance = obstacle_avoidance_yaw_gain * avoidance_turn_direction;
                    pitch_disturbance = -0.5;
                    roll_disturbance = 0;
                } else {
                    double heading_error = target_bearing - yaw;
                    if (heading_error > M_PI) heading_error -= 2 * M_PI;
                    if (heading_error < -M_PI) heading_error += 2 * M_PI;
                    yaw_disturbance = CLAMP(k_yaw_p * heading_error, -k_max_yaw_disturbance, k_max_yaw_disturbance);
                    double forward_error = dx * std::cos(yaw) + dy * std::sin(yaw);
                    double sideways_error = -dx * std::sin(yaw) + dy * std::cos(yaw);
                    pitch_disturbance = CLAMP(-k_pos_p * forward_error, -k_max_pos_disturbance, k_max_pos_disturbance);
                    roll_disturbance = CLAMP(-k_pos_p * sideways_error, -k_max_pos_disturbance, k_max_pos_disturbance);
                }

                double clamped_diff_altitude = CLAMP(target.z - altitude + k_vertical_offset, -1.0, 1.0);
                vertical_input = k_vertical_p * std::pow(clamped_diff_altitude, 3.0);

                double distance_3d = std::sqrt(dx * dx + dy * dy + std::pow(target.z - altitude, 2));
                if (distance_3d < 0.5) {
                    std::cout << "Waypoint " << current_waypoint_index + 1 << " reached!" << std::endl;
                    current_waypoint_index++;
                    
                    // --- ADDED FOR VISUALIZATION ---
                    // Move the marker to the next waypoint.
                    if (waypoint_marker_translation) {
                        if (current_waypoint_index < waypoints.size()) {
                            const Waypoint& next_target = waypoints[current_waypoint_index];
                            const double next_pos[3] = {next_target.x, next_target.y, next_target.z};
                            waypoint_marker_translation->setSFVec3f(next_pos);
                        }
                    }
                    // ---
                }
                
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
                // --- ADDED FOR VISUALIZATION ---
                // Hide the marker when the mission is finished.
                if (waypoint_marker_translation) {
                     const double hidden_pos[3] = {0.0, 0.0, -10.0}; 
                     waypoint_marker_translation->setSFVec3f(hidden_pos);
                     waypoint_marker_translation = nullptr; 
                }
                // ---

                // Hover logic is identical to your original code.
                vertical_input = 0;
                roll_disturbance = 0;
                pitch_disturbance = 0;
                yaw_disturbance = 0;
                break;
            }
        }

        // 4. MOTOR CONTROL
        //------------------
        // This entire section is identical to your original code.
        double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
        double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
        
        double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_disturbance;
        double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_disturbance;
        double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_disturbance;
        double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_disturbance;

        front_left_motor->setVelocity(front_left_motor_input);
        front_right_motor->setVelocity(-front_right_motor_input);
        rear_left_motor->setVelocity(-rear_left_motor_input);
        rear_right_motor->setVelocity(rear_right_motor_input);
    }

    // Cleanup is identical.
    delete supervisor;
    return 0;
}