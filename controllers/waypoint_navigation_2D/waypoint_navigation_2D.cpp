#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>

#include <cmath>
#include <iostream>
#include <vector>
#include <iomanip>

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

  Motor *front_left_motor = robot->getMotor("front left propeller");
  Motor *front_right_motor = robot->getMotor("front right propeller");
  Motor *rear_left_motor = robot->getMotor("rear left propeller");
  Motor *rear_right_motor = robot->getMotor("rear right propeller");

  Motor *motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
  for (int i = 0; i < 4; ++i) {
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(1.0);
  }

  // Waypoints
  std::vector<Waypoint> waypoints = {
    {2.0, 0.0},
    {2.0, 2.0},
    {0.0, 2.0},
    {0.0, 0.0}
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
  
  const double k_yaw_p = 0.5;
  const double k_max_yaw_disturbance = 1.5;

  const double k_pos_p = 0.4;
  const double k_max_pos_disturbance = 8.0;

  double target_altitude = 2.0; // Drone will maintain this altitude for all waypoints

  double last_print_time = 0.0;

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

    // Check if all waypoints have been reached
    if (current_waypoint_index >= waypoints.size()) {
      std::cout << "All waypoints reached. Stopping motors and mission complete." << std::endl;
      // Immediately stop all motors
      front_left_motor->setVelocity(0.0);
      front_right_motor->setVelocity(0.0);
      rear_left_motor->setVelocity(0.0);
      rear_right_motor->setVelocity(0.0);
      break; // Exit the simulation loop
    }

    // Current target is always the next waypoint
    Waypoint target = waypoints[current_waypoint_index];
      
    double dx = target.x - current_x;
    double dy = target.y - current_y;
    double distance = std::sqrt(dx * dx + dy * dy);
    double target_bearing = std::atan2(dy, dx);
      
    double heading_error = target_bearing - yaw;
    if (heading_error > M_PI) heading_error -= 2 * M_PI;
    if (heading_error < -M_PI) heading_error += 2 * M_PI;

    if (time - last_print_time >= 2.0) {
      last_print_time = time;
      std::cout << std::fixed << std::setprecision(2);
      std::cout << "Time: " << time << "s | ";
      std::cout << "Waypoint: " << current_waypoint_index + 1 << "/" << waypoints.size() << " | ";
      std::cout << "Current Pos: (" << current_x << ", " << current_y << ", Alt: " << altitude << ") | ";
      std::cout << "Target Pos: (" << target.x << ", " << target.y << ") | ";
      std::cout << "Dist: " << distance << "m | ";
      std::cout << "Heading Error: " << heading_error << " rad" << std::endl;
    }

    // Check if current waypoint is reached
    if (distance < 0.2) {
      std::cout << "Waypoint " << current_waypoint_index + 1 << " reached" << std::endl;
      current_waypoint_index++;
    }
    
    // Control logic for navigation (always active until last waypoint is reached)
    double yaw_disturbance = k_yaw_p * heading_error;
    yaw_disturbance = CLAMP(yaw_disturbance, -k_max_yaw_disturbance, k_max_yaw_disturbance);
      
    double forward_error  = dx * std::cos(yaw) + dy * std::sin(yaw);
    double sideways_error = -dx * std::sin(yaw) + dy * std::cos(yaw);
          
    double pitch_disturbance = -k_pos_p * forward_error;
    double roll_disturbance  = -k_pos_p * sideways_error;
      
    roll_disturbance = CLAMP(roll_disturbance, -k_max_pos_disturbance, k_max_pos_disturbance);
    pitch_disturbance = CLAMP(pitch_disturbance, -k_max_pos_disturbance, k_max_pos_disturbance);

    // Vertical control logic (maintaining target_altitude)
    double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
    double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
    
    double clamped_diff_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    double vertical_input = k_vertical_p * std::pow(clamped_diff_altitude, 3.0);

    // Calculate individual motor velocities
    double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_disturbance;
    double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_disturbance;
    double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_disturbance;
    double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_disturbance;

    front_left_motor->setVelocity(front_left_motor_input);
    front_right_motor->setVelocity(-front_right_motor_input);
    rear_left_motor->setVelocity(-rear_left_motor_input);
    rear_right_motor->setVelocity(rear_right_motor_input);
  }

  std::cout << "Mission ended." << std::endl;
  front_left_motor->setVelocity(0.0);
  front_right_motor->setVelocity(0.0);
  rear_left_motor->setVelocity(0.0);
  rear_right_motor->setVelocity(0.0);

  delete robot;
  return 0;
}