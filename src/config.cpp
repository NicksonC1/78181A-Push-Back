#include "main.h"
#include "genesis/api.hpp"
#include "pros/rtos.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
 // <--------------------------------------------------------------- Setup ------------------------------------------------------------------>
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-13, -12, -11}, pros::MotorGearset::blue); // checked
pros::MotorGroup rightMotors({18, 19, 20}, pros::MotorGearset::blue); // checked

namespace Motor{
  pros::Motor intakeF(-8, pros::MotorGearset::blue); // checked
  pros::Motor intakeM(2, pros::MotorGearset::blue); // checked
  pros::Motor intakeU(-10, pros::MotorGearset::blue); // checked
} // namespace Motor

namespace Sensor{
  pros::Distance d_front(14); // checked
  pros::Distance d_right(6); // checked
  pros::Distance d_filled(5); // checked
  pros::Optical o_colorSort(9); // checked
  pros::adi::DigitalIn autonSwitch('F');
} // namspace Sensor

namespace Piston{
  pros::adi::DigitalOut intake('G'); // checked
  pros::adi::DigitalOut loader('D'); // checked
  pros::adi::DigitalOut miniHood('H'); // checked
  pros::adi::DigitalOut hood('A'); // checked
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

CustomIMU s_imu(7, 1.00528659218); // checked
// CustomIMU s_imu(7, 1.0); // checked

pros::Rotation horizontalEnc(21);
pros::Rotation verticalEnc(21);

genesis::TrackingWheel vertical_tracking_wheel(&verticalEnc, 2.0 , -0.62); // Single
genesis::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, 2.0 , -2.75); // Double Stacked

// <---------------------------------------------------------------- Config ---------------------------------------------------------------->
// genesis::Drivetrain drivetrain(&leftMotors, // left motor group
//                               &rightMotors, // right motor group
//                               11.5, // 11.5 inch track width
//                               genesis::Omniwheel::NEW_325, // using new 3.25" omnis
//                               450, // drivetrain rpm is 450
//                               8 // horizontal drift is 2. If we had traction wheels, it would have been 8
// );

// genesis::ControllerSettings linearController (6, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               3.75, // derivative gain (kD)
//                                               0, // anti windup
//                                               1, // small error range, in inches
//                                               50, // small error range timeout, in milliseconds
//                                               3, // large error range, in inches
//                                               250, // large error range timeout, in milliseconds
//                                               60 // maximum acceleration (slew)
// );

// genesis::ControllerSettings angularController(3, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               18, // derivative gain (kD) 
//                                               0, // anti windup
//                                               1, // small error range, in inches
//                                               50, // small error range timeout, in milliseconds
//                                               3, // large error range, in inches
//                                               250, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

// genesis::OdomSensors sensors(nullptr, // vertical tracking wheel
//                             nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
//                             nullptr, // horizontal tracking wheel
//                             nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
//                             &s_imu // inertial sensor
// );

// // input curve for throttle input during driver control
// genesis::ExpoDriveCurve throttleCurve(1, // joystick deadband out of 127
//                                      0, // minimum output where drivetrain will move out of 127
//                                      1 // expo curve gain
// );

// // input curve for steer input during driver control
// genesis::ExpoDriveCurve steerCurve(1, // joystick deadband out of 127
//                                   0, // minimum output where drivetrain will move out of 127
//                                   1 // expo curve gain
// );

// // create the chassis
// genesis::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);