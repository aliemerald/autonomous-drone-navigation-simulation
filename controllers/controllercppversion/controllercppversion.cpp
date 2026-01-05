#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <webots/Robot.hpp>

#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>

using namespace webots;

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

int main(int argc, char** argv) {
    Robot* robot = new Robot();
    int timeStep = (int)robot->getBasicTimeStep();
    
    // Get and enable devices.
    Camera* camera = robot->getCamera("camera");
    camera->enable(timeStep);
    LED* front_left_led = robot->getLED("front left led");
    LED* front_right_led = robot->getLED("front right led");
    InertialUnit* imu = robot->getInertialUnit("inertial unit");
    imu->enable(timeStep);
    GPS* gps = robot->getGPS("gps");
    gps->enable(timeStep);
    Compass* compass = robot->getCompass("compass");
    compass->enable(timeStep);
    Gyro* gyro = robot->getGyro("gyro");
    gyro->enable(timeStep);
    Keyboard* keyboard = robot->getKeyboard();
    keyboard->enable(timeStep);
    Motor* camera_roll_motor = robot->getMotor("camera roll");
    Motor* camera_pitch_motor = robot->getMotor("camera pitch");
    // Motor* camera_yaw_motor = robot->getMotor("camera yaw"); // Not used in this example.
    
    // Get propeller motors and set them to velocity mode.
    Motor* front_left_motor = robot->getMotor("front left propeller");
    Motor* front_right_motor = robot->getMotor("front right propeller");
    Motor* rear_left_motor = robot->getMotor("rear left propeller");
    Motor* rear_right_motor = robot->getMotor("rear right propeller");

    Motor* motors[4] = { front_left_motor, front_right_motor, rear_left_motor, rear_right_motor };
    for (int i = 0; i < 4; i++) {
        motors[i]->setPosition(INFINITY);
        motors[i]->setVelocity(1.0);
    }
    
    // Display the welcome message.
    std::cout<<"Start the drone...\n"<<std::endl;

    // Wait one second.
    while (robot->step(timeStep) != -1) {
        if (robot->getTime() > 1.0)
            break;
    }
    
    // Display manual control message.
    std::cout << "You can control the drone with your computer keyboard:" << std::endl;
    std::cout << "- 'up': move forward." << std::endl;
    std::cout << "- 'down': move backward." << std::endl;
    std::cout << "- 'right': turn right." << std::endl;
    std::cout << "- 'left': turn left." << std::endl;
    std::cout << "- 'shift + up': increase the target altitude." << std::endl;
    std::cout << "- 'shift + down': decrease the target altitude." << std::endl;
    std::cout << "- 'shift + right': strafe right." << std::endl;
    std::cout << "- 'shift + left': strafe left." << std::endl;

    // Constants, empirically found.
    const double k_vertical_thrust = 68.5;  // with this thrust, the drone lifts.
    const double k_vertical_offset = 0.6;   // Vertical offset where the robot actually targets to stabilize itself.
    const double k_vertical_p = 3.0;        // P constant of the vertical PID.
    const double k_roll_p = 50.0;           // P constant of the roll PID.
    const double k_pitch_p = 30.0;          // P constant of the pitch PID.

    // Variables.
    double target_altitude = 1.0;  // The target altitude. Can be changed by the user.

    // Main loop
    while (robot->step(timeStep) != -1) {
        const double time = robot->getTime();  // in seconds.

        // Retrieve robot position using the sensors.
        const double roll = imu->getRollPitchYaw()[0];
        const double pitch = imu->getRollPitchYaw()[1];
        const double altitude = gps->getValues()[2];
        const double roll_velocity = gyro->getValues()[0];
        const double pitch_velocity = gyro->getValues()[1];

        // Blink the front LEDs alternatively with a 1 second rate.
        const bool led_state = ((int)time) % 2;
        front_left_led->set(led_state);
        front_right_led->set(!led_state);

        // Stabilize the Camera by actuating the camera motors according to the gyro feedback.
        camera_roll_motor->setPosition(-0.115 * roll_velocity);
        camera_pitch_motor->setPosition(-0.1 * pitch_velocity);

        // Transform the keyboard input to disturbances on the stabilization algorithm.
        double roll_disturbance = 0.0;
        double pitch_disturbance = 0.0;
        double yaw_disturbance = 0.0;
        int key = keyboard->getKey();
        while (key > 0) {
            switch (key) {
            case Keyboard::UP:
                pitch_disturbance = -2.0;
                break;
            case Keyboard::DOWN:
                pitch_disturbance = 2.0;
                break;
            case Keyboard::RIGHT:
                yaw_disturbance = -1.3;
                break;
            case Keyboard::LEFT:
                yaw_disturbance = 1.3;
                break;
            case (Keyboard::SHIFT + Keyboard::RIGHT):
                roll_disturbance = -1.0;
                break;
            case (Keyboard::SHIFT + Keyboard::LEFT):
                roll_disturbance = 1.0;
                break;
            case (Keyboard::SHIFT + Keyboard::UP):
                target_altitude += 0.05;
                std::cout << "target altitude: " << target_altitude << " [m]" << std::endl;
                break;
            case (Keyboard::SHIFT + Keyboard::DOWN):
                target_altitude -= 0.05;
                std::cout << "target altitude: " << target_altitude << " [m]" << std::endl;
                break;
            }
            key = keyboard->getKey();
        }

        // Compute the roll, pitch, yaw and vertical inputs.
        const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
        const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
        const double yaw_input = yaw_disturbance;
        const double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
        const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

        // Actuate the motors taking into consideration all the computed inputs.
        const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
        const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
        const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
        const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
        front_left_motor->setVelocity(front_left_motor_input);
        front_right_motor->setVelocity(-front_right_motor_input);
        rear_left_motor->setVelocity(-rear_left_motor_input);
        rear_right_motor->setVelocity(rear_right_motor_input);
        
    }

    delete robot;
    return 0;

}