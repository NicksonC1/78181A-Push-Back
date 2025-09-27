#include "main.h"
#include "genesis/api.hpp"
#include "pros/rtos.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
 // <--------------------------------------------------------------- Setup ------------------------------------------------------------------>
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// DriveTrain
pros::MotorGroup leftMotors({21, 20, 3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({-4, -5, -6}, pros::MotorGearset::blue); 

namespace Motor{
  // pros::Motor intakeB(16, pros::MotorGearset::blue);
  pros::Motor intakeF(-1, pros::MotorGearset::blue);
  pros::Motor intakeM(2, pros::MotorGearset::blue);
  pros::Motor intakeU(-10, pros::MotorGearset::blue);
} // namespace Motor

namespace Sensor{
  pros::Distance d_front(19);
  pros::Distance d_right(11);
  pros::Optical o_colorSort(12);
  pros::adi::DigitalIn autonSwitch('A');
} // namspace Sensor

namespace Piston{
  pros::adi::DigitalOut loader('B'); 
  pros::adi::DigitalOut miniHood('C'); 
} // namespace Piston

// <------------------------------------------------------------- Odom Sensors ------------------------------------------------------------->
class CustomIMU : public pros::IMU {
  public:
    CustomIMU(int port, double scalar)
      : pros::IMU(port),
        m_port(port),
        m_scalar(scalar) {}
    virtual double get_rotation() const {
      return pros::c::imu_get_rotation(m_port) * m_scalar;
    }
  private:
    const int m_port;
    const double m_scalar;
};

CustomIMU s_imu(13, 1.0);
// pros::Imu imu(21);
pros::Rotation horizontalEnc(14);
pros::Rotation verticalEnc(15);
// port 14 broken

genesis::TrackingWheel vertical_tracking_wheel(&verticalEnc, 2.0 , -0.62); // Single
genesis::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, 2.0 , -2.75); // Double Stacked

// genesis::TrackingWheel vertical_tracking_wheel(&verticalEnc, 2.0 , -0.65); // Single
// genesis::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, 2.0 , -2.3); // Double Stacked


// genesis::TrackingWheel vertical_tracking_wheel(&verticalEnc, 2.0 , -0.5); // Single
// genesis::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, 2.0 , -2.7); // Double Stacked

// <---------------------------------------------------------------- Config ---------------------------------------------------------------->
genesis::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 11.5 inch track width
                              genesis::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

genesis::ControllerSettings linearController (6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3.75, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              50, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              250, // large error range timeout, in milliseconds
                                              60 // maximum acceleration (slew)
);

genesis::ControllerSettings angularController(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              18, // derivative gain (kD) 
                                              0, // anti windup
                                              1, // small error range, in inches
                                              50, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              250, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

genesis::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &s_imu // inertial sensor
);

// input curve for throttle input during driver control
genesis::ExpoDriveCurve throttleCurve(1, // joystick deadband out of 127
                                     0, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
);

// input curve for steer input during driver control
genesis::ExpoDriveCurve steerCurve(1, // joystick deadband out of 127
                                  0, // minimum output where drivetrain will move out of 127
                                  1 // expo curve gain
);

// create the chassis
genesis::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);