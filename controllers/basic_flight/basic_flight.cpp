#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>

#include <cmath>
#include <iostream>
#include <iomanip> // For std::fixed and std::setprecision

using namespace webots;

#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

int main() {
  Robot* robot = new Robot();
  int timeStep = static_cast<int>(robot->getBasicTimeStep());

  InertialUnit* imu = robot->getInertialUnit("inertial unit");
  imu->enable(timeStep);
  GPS* gps = robot->getGPS("gps");
  gps->enable(timeStep);
  Gyro* gyro = robot->getGyro("gyro");
  gyro->enable(timeStep);

  Motor* front_left_motor = robot->getMotor("front left propeller");
  Motor* front_right_motor = robot->getMotor("front right propeller");
  Motor* rear_left_motor = robot->getMotor("rear left propeller");
  Motor* rear_right_motor = robot->getMotor("rear right propeller");

  Motor* motors[4] = { front_left_motor, front_right_motor, rear_left_motor, rear_right_motor };
  for (int i = 0; i < 4; ++i) {
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0.0);
  }

  const double k_vertical_thrust = 68.5;
  const double k_vertical_offset = 0.6;
  const double k_vertical_p = 3.0;
  const double k_roll_p = 50.0;
  const double k_pitch_p = 30.0;

  const double lift_altitude = 1.0;
  double target_altitude = lift_altitude;

  // Simplified state flags and timers
  bool reached_altitude = false;
  bool initial_hover_done = false;
  bool move_forward = false;
  bool landing = false;

  double initial_hover_start_time = 0.0;
  double forward_motion_start_time = 0.0;
  
  double last_print_time = 0.0;

  while (robot->step(timeStep) != -1) {
    double time = robot->getTime();
    double roll = imu->getRollPitchYaw()[0];
    double pitch = imu->getRollPitchYaw()[1];
    double altitude = gps->getValues()[2];
    double roll_velocity = gyro->getValues()[0];
    double pitch_velocity = gyro->getValues()[1];
    
    double pitch_disturbance = 0.0;

    // 1. Takeoff phase
    if (!reached_altitude && altitude >= lift_altitude - 0.05) {
      reached_altitude = true;
      initial_hover_start_time = time;
      std::cout << "Reached target altitude. Hovering for 3 seconds.\n";
    }
    
    // 2. Initial hover phase
    if (reached_altitude && !initial_hover_done && time - initial_hover_start_time >= 3.0) {
      initial_hover_done = true;
      move_forward = true;
      forward_motion_start_time = time;
      std::cout << "Done with initial hover. Moving forwards for 5 seconds.\n";
    }
    
    // 3. Forward motion phase
    if (move_forward) {
      pitch_disturbance = -2.0;
      // After 5 seconds, stop moving forward and immediately start landing
      if (time - forward_motion_start_time >= 5.0) {
        move_forward = false;
        landing = true;
        std::cout << "Initiating landing.\n";
      }
    }
    
    // 4. Landing phase
    if (landing) {
      target_altitude = 0.0;
    }
    
    // PID control
    double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity;
    double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
    double clamped_altitude_diff = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    double vertical_input = k_vertical_p * pow(clamped_altitude_diff, 3.0);

    double fl = k_vertical_thrust + vertical_input - roll_input + pitch_input;
    double fr = k_vertical_thrust + vertical_input + roll_input + pitch_input;
    double rl = k_vertical_thrust + vertical_input - roll_input - pitch_input;
    double rr = k_vertical_thrust + vertical_input + roll_input - pitch_input;

    front_left_motor->setVelocity(fl);
    front_right_motor->setVelocity(-fr);
    rear_left_motor->setVelocity(-rl);
    rear_right_motor->setVelocity(rr);

    // Print current vs target altitude every 0.5s
    if (time - last_print_time >= 2) {
      last_print_time = time;
      std::cout << std::fixed << std::setprecision(2);
      std::cout << "Time: " << time << "s | Altitude: " << altitude 
                << " | Target: " << target_altitude << std::endl;
    }
  }

  delete robot;
  return 0;
}