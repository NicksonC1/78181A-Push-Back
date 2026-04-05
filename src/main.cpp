// <--------------------------------------------------------------- Includes --------------------------------------------------------------->
#include <bits/stdc++.h>
#include <vector>
#include <functional>
#include <string>
#include "main.h"
#include "genesis/api.hpp"
#include "genesis/chassis/chassis.hpp"
#include "liblvgl/lvgl.h"
#include "liblvgl/llemu.hpp"
// #include "brainScreenLVGL.h"
// #include "config.h"
#include "RclTracking.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

ASSET(middle_txt); // PP
ASSET(long1_txt); // PP
ASSET(long2_txt); // PP
ASSET(park_txt); // PP

// <--------------------------------------------------------------- Setup ------------------------------------------------------------------>
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-11, 12, -13}, pros::MotorGearset::blue); // checked
pros::MotorGroup rightMotors({18, -19, 20}, pros::MotorGearset::blue); // checked

namespace Motor{
  pros::Motor intake1(-14, pros::MotorGearset::blue);
  pros::Motor intake2(17, pros::MotorGearset::blue); 
//   pros::MotorGroup intake({-14, 17}, pros::MotorGearset::blue); 
} // namespace Motor

namespace Sensor{
  pros::Distance d_front(22); 
  pros::Distance d_left(14);
  pros::Distance d_right(17); 
  pros::Optical o_colorSort(8); 
  pros::Optical o_crossed(17); 
  pros::adi::DigitalIn autonSwitch('Z'); 
} // namspace Sensor

namespace Piston{
  pros::adi::DigitalOut loader('F'); 
  pros::adi::DigitalOut hook('C'); // Checked
  pros::adi::DigitalOut low('A'); // Checked
  pros::adi::DigitalOut lift('B'); // Checked
  pros::adi::DigitalOut hood('E');
  pros::adi::DigitalOut odom('D');
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


// CustomIMU s_imu(21, 1.01152008991); // checked
CustomIMU s_imu(2, 1.01123595506); // checked

pros::Rotation horizontalEnc(-15);
pros::Rotation verticalEnc(-16); 

genesis::TrackingWheel vertical(&verticalEnc, 2.0 , 0.36); // Singleji
genesis::TrackingWheel horizontal(&horizontalEnc, 2.75 , -2.93); // Double Stacked

genesis::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 11.5 inch track width
                              genesis::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              10 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

genesis::ControllerSettings linearController (0, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

genesis::ControllerSettings angularController(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD) 
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// genesis::ControllerSettings linearController (7.5, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               6, // derivative gain (kD)
//                                               0, // anti windup
//                                               1, // small error range, in inches
//                                               100, // small error range timeout, in milliseconds
//                                               3, // large error range, in inches
//                                               500, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

// genesis::ControllerSettings angularController(2.75, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               17.5, // derivative gain (kD) 
//                                               0, // anti windup
//                                               1, // small error range, in inches
//                                               100, // small error range timeout, in milliseconds
//                                               3, // large error range, in inches
//                                               500, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

genesis::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &s_imu // inertial sensor
);

// input curve for throttle input during driver control
genesis::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     0, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
);

// input curve for steer input during driver control
genesis::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  0, // minimum output where drivetrain will move out of 127
                                  1.05 // expo curve gain
);

// create the chassis
genesis::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// RCL tracking sensors: right offset is +, forward offset is +, headings in degrees.
RclSensor rclFront(&Sensor::d_front, 0.0, 9.0, 0.0, 15.0);
RclSensor rclLeft(&Sensor::d_left, -7.1, 0.0, 270.0, 15.0);
RclSensor rclRight(&Sensor::d_right, 7.1, 0.0, 90.0, 15.0);
RclTracking RclMain(&chassis, 30, true, 0.5, 4.0, 10.0, 6.0, 20);

std::vector<std::pair<float, float>> points;

namespace TaskHandler {
    bool antiJam = false; bool antiJam2 = false; bool antiJam3 = false;
    bool autonSelect = true;
    bool colorSort = true;
    bool driver = true;
    bool intake = true;
    bool filled = false;
    bool spinning = false;
    bool intakeSpin = false;
} // namespace TaskHandler

// <------------------------------------------------------------ Miscellaneous ------------------------------------------------------------>
namespace Misc{
    constexpr int DELAY = 10;
    pros::motor_brake_mode_e_t brakeState = pros::E_MOTOR_BRAKE_HOLD;
    pros::motor_brake_mode_e_t brakeStateI = pros::E_MOTOR_BRAKE_COAST;
    int val = 0;
    bool turningRed = false;
    enum class FieldSide { AUTO, POS_Y, POS_X, NEG_Y, NEG_X };
    void led(){
        while(1){
            Sensor::o_colorSort.set_integration_time(5);
            Sensor::o_colorSort.set_led_pwm(100);
            Sensor::o_crossed.set_integration_time(5);
            Sensor::o_crossed.set_led_pwm(100);
            pros::delay(50);
        }
    }
    void togglePiston(pros::adi::DigitalOut &piston, bool &state) {
        state = !state;
        piston.set_value(state);
    }
    void cdrift(float lV, float rV, int timeout, bool cst = true){
        (cst == true) ? (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST), rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST)) : (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE), rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE));
        leftMotors.move(lV);
        rightMotors.move(rV);
        pros::delay(timeout);
        leftMotors.brake();
        rightMotors.brake();
    }
    void cdrift(float lV, float rV){
        leftMotors.move(lV);
        rightMotors.move(rV);
    }
    void cbrake(bool cst = true){
        (cst == true) ? (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST), rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST)) : (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE), rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE));
        leftMotors.brake();
        rightMotors.brake();
    }
    void chain(std::vector<std::pair<float, float>>& waypoints, int angular = 450, int lateral = 2300){
        while(!waypoints.empty()){
            std::pair<int, int> target = waypoints.front();
            chassis.turnToPoint(target.first,target.second,angular,{.minSpeed = 10,.earlyExitRange = 2});
            chassis.moveToPoint(target.first,target.second,lateral,{.minSpeed = 10,.earlyExitRange = 2});
            chassis.waitUntilDone();
            waypoints.erase(waypoints.begin());
        }
    }
    void linear(double dist, int timeout, genesis::MoveToPointParams p = {}, bool async = true){
        genesis::Pose pose = chassis.getPose(true);
        dist < 0 ? p.forwards = false : p.forwards = true;
        chassis.moveToPoint(
        pose.x + std::sin(pose.theta) * dist,
        pose.y + std::cos(pose.theta) * dist,
        timeout, p, async);
    }
    void driveFor(float distance, float maxSpeed, int timeout, float minspeed=0, float exit=0) {
        double headingRadians = chassis.getPose(true).theta;
        double startingX = chassis.getPose().x;
        double startingY = chassis.getPose().y;
        double deltaX = distance * sin(headingRadians);
        double deltaY = distance * cos(headingRadians);
        double newX = startingX + deltaX;
        double newY = startingY + deltaY;
        if (distance > 0) {
            chassis.moveToPoint(newX, newY, timeout, {.forwards=true, .maxSpeed=maxSpeed, .minSpeed=minspeed, .earlyExitRange=exit});
        }
        else if (distance < 0) {
            chassis.moveToPoint(newX, newY, timeout, {.forwards=false, .maxSpeed=maxSpeed, .minSpeed=minspeed, .earlyExitRange=exit});
        }
    }

    struct PoseSampleParams {
        double refX = std::numeric_limits<double>::quiet_NaN();
        double refY = std::numeric_limits<double>::quiet_NaN();
        double radiusIn = 8.0;
    };

    static bool poseWithinRadius(double x, double y, double refX, double refY, double radiusIn) {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(refX) || !std::isfinite(refY)) return false;
        const double dx = x - refX;
        const double dy = y - refY;
        return (dx * dx + dy * dy) <= (radiusIn * radiusIn);
    }

    static void applyPoseFallback(double& x, double& y, double estimateX, double estimateY, const PoseSampleParams& params) {
        const double refX = std::isfinite(params.refX) ? params.refX : estimateX;
        const double refY = std::isfinite(params.refY) ? params.refY : estimateY;
        const double radiusIn = std::isfinite(params.radiusIn) ? params.radiusIn : 0.0;
        if (radiusIn <= 0.0 || !poseWithinRadius(x, y, refX, refY, radiusIn)) {
            x = refX;
            y = refY;
        }
    }

    void resetWalls(bool useLeft = true, bool useRight = true, bool useFront = true, PoseSampleParams sampleParams = PoseSampleParams{}, bool useFallback = true) {
        constexpr double field = 144.0;
        constexpr double halfField = field / 2.0;
        constexpr double WALL_0_X = halfField;
        constexpr double WALL_1_Y = halfField;
        constexpr double WALL_2_X = -halfField;
        constexpr double WALL_3_Y = -halfField;
        constexpr double ANGLE_TOLERANCE = 15.0 * (M_PI / 180.0);
        constexpr double pi = M_PI;
        constexpr double mmToIn = 1.0 / 25.4;

        constexpr double leftOffsetR = -7.1;
        constexpr double rightOffsetR = 7.1;
        constexpr double frontOffsetF = 9.0;
        constexpr double frontOffsetR = 0.0;

        enum class Axis { NONE, X, Y };
        struct Result {
            Axis axis;
            double axisPosition;
        };

        genesis::Pose m_offset(0.0, 0.0, 0.0);
        double realReading = 0.0;

        auto getReading = [&](genesis::Pose pose, Result& result, bool force) {
            // determine sensor pose
            double sensorAngle = pose.theta + m_offset.theta;
            double sensorOffsetX = m_offset.x * std::cos(pose.theta) - m_offset.y * std::sin(pose.theta);
            double sensorOffsetY = m_offset.x * std::sin(pose.theta) + m_offset.y * std::cos(pose.theta);
            double sensorX = pose.x + sensorOffsetX;
            double sensorY = pose.y + sensorOffsetY;

            // determine predicted reading
            double predictedReading;
            double angleError;
            int wall;

            if (angleError = std::abs(std::remainder(0 - sensorAngle, 2 * pi)); angleError < ANGLE_TOLERANCE) {
                predictedReading = (WALL_0_X - sensorX) / std::cos(angleError);
                wall = 0;
            } else if (angleError = std::abs(std::remainder(0.5 * pi - sensorAngle, 2 * pi)); angleError < ANGLE_TOLERANCE) {
                predictedReading = (WALL_1_Y - sensorY) / std::cos(angleError);
                wall = 1;
            } else if (angleError = std::abs(std::remainder(pi - sensorAngle, 2 * pi)); angleError < ANGLE_TOLERANCE) {
                predictedReading = (sensorX - WALL_2_X) / std::cos(angleError);
                wall = 2;
            } else if (angleError = std::abs(std::remainder(1.5 * pi - sensorAngle, 2 * pi)); angleError < ANGLE_TOLERANCE) {
                predictedReading = (sensorY - WALL_3_Y) / std::cos(angleError);
                wall = 3;
            } else {
                wall = -1;
            }

            // determine the new position on the axis based on distance sensor readings
            if (wall == 0) {
                result.axis = Axis::X;
                result.axisPosition = (WALL_0_X - realReading * std::cos(angleError)) - sensorOffsetX;
            } else if (wall == 1) {
                result.axis = Axis::Y;
                result.axisPosition = (WALL_1_Y - realReading * std::cos(angleError)) - sensorOffsetY;
            } else if (wall == 2) {
                result.axis = Axis::X;
                result.axisPosition = (WALL_2_X + realReading * std::cos(angleError)) - sensorOffsetX;
            } else if (wall == 3) {
                result.axis = Axis::Y;
                result.axisPosition = (WALL_3_Y + realReading * std::cos(angleError)) - sensorOffsetY;
            }
        };

        genesis::Pose pose = chassis.getPose(true, true);
        double heading = chassis.getPose().theta;
        const double estimatedX = pose.x;
        const double estimatedY = pose.y;

        Result leftRes = {Axis::NONE, 0.0};
        Result rightRes = {Axis::NONE, 0.0};
        Result frontRes = {Axis::NONE, 0.0};

        if (useLeft) {
            m_offset = genesis::Pose(0.0, -leftOffsetR, M_PI_2);
            realReading = Sensor::d_left.get_distance() * mmToIn;
            getReading(pose, leftRes, false);
        }
        if (useRight) {
            m_offset = genesis::Pose(0.0, -rightOffsetR, -M_PI_2);
            realReading = Sensor::d_right.get_distance() * mmToIn;
            getReading(pose, rightRes, false);
        }
        if (useFront) {
            m_offset = genesis::Pose(frontOffsetF, -frontOffsetR, 0.0);
            realReading = Sensor::d_front.get_distance() * mmToIn;
            getReading(pose, frontRes, false);
        }

        double x = estimatedX;
        double y = estimatedY;
        double xSum = 0.0;
        double ySum = 0.0;
        int xCount = 0;
        int yCount = 0;

        auto accumulate = [&](const Result& res) {
            if (!std::isfinite(res.axisPosition)) return;
            if (res.axis == Axis::X) {
                xSum += res.axisPosition;
                xCount++;
            } else if (res.axis == Axis::Y) {
                ySum += res.axisPosition;
                yCount++;
            }
        };

        if (useLeft) accumulate(leftRes);
        if (useRight) accumulate(rightRes);
        if (useFront) accumulate(frontRes);

        if (xCount > 0) x = xSum / static_cast<double>(xCount);
        if (yCount > 0) y = ySum / static_cast<double>(yCount);

        if (useFallback) {
            applyPoseFallback(x, y, estimatedX, estimatedY, sampleParams);
        }

        chassis.setPose(x, y, heading);
        RclMain.setRclPose(chassis.getPose());

        printf("Pose -> X: %.2f, Y: %.2f, Heading: %.2f\n", x, y, heading);
    }

    void resetYLeftRightFacing270(PoseSampleParams sampleParams = PoseSampleParams{}, bool useFallback = true) {
        constexpr double field = 144.0;
        constexpr double halfField = field / 2.0;
        constexpr double mmToIn = 1.0 / 25.4;
        constexpr double leftOffsetR = -14.1;
        constexpr double rightOffsetR = 0.1;
        constexpr double headingRad = 1.5 * M_PI;

        const genesis::Pose estimatePose = chassis.getPose();
        const double estimateX = estimatePose.x;
        const double estimateY = estimatePose.y;

        const double d_left = Sensor::d_left.get_distance() * mmToIn;
        const double d_right = Sensor::d_right.get_distance() * mmToIn;

        auto yFromWall = [&](double readingIn, double offsetR, bool posWall) {
            const double sensorOffsetY = offsetR * std::cos(headingRad);
            return posWall ? (halfField - readingIn) - sensorOffsetY
                           : (-halfField + readingIn) - sensorOffsetY;
        };

        double x = estimateX;
        double y = estimateY;
        double ySum = 0.0;
        int yCount = 0;

        if (std::isfinite(d_left)) {
            ySum += yFromWall(d_left, leftOffsetR, true);
            yCount++;
        }
        if (std::isfinite(d_right)) {
            ySum += yFromWall(d_right, rightOffsetR, false);
            yCount++;
        }

        if (yCount > 0) y = ySum / static_cast<double>(yCount);

        if (useFallback) {
            const double refX = std::isfinite(sampleParams.refX) ? sampleParams.refX : estimateX;
            const double refY = std::isfinite(sampleParams.refY) ? sampleParams.refY : estimateY;
            const double radiusIn = std::isfinite(sampleParams.radiusIn) ? sampleParams.radiusIn : 0.0;
            if (radiusIn <= 0.0 || !poseWithinRadius(x, y, refX, refY, radiusIn)) {
                y = refY;
            }
        }

        chassis.setPose(-46, y+0.55, chassis.getPose().theta);
        RclMain.setRclPose(chassis.getPose());

        printf("Pose -> X: %.2f, Y: %.2f, Heading: %.2f\n", x, y, headingRad);
    }

    int curve(int input, double t = 5, bool activated = true) {
        if(!activated) return input;
        val = (std::exp(-t/10)) + std::exp((std::abs(input)-100)/10)*(1-std::exp(-t/10)) * input;
        return val;
    }

    void park(float lV, float rV, int timeout){
        leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        leftMotors.move(lV);
        rightMotors.move(rV);
        while(timeout > 0){
            timeout -= Misc::DELAY;
            if(Sensor::o_crossed.get_hue() > 0 && Sensor::o_crossed.get_hue() < 12){
                break;
            }
            pros::delay(Misc::DELAY);
        }
        leftMotors.brake();
        rightMotors.brake();
    }
} // namespace Misc

namespace OdomTune {
    bool active = false;
    int sampleCount = 0;
    int lastDirection = 0;
    double lastVertical = 0.0;
    double lastHorizontal = 0.0;
    double lastHeadingDeg = 0.0;
    double lastVerticalOffset = 0.0;
    double lastHorizontalOffset = 0.0;
    double avgVerticalOffset = 0.0;
    double avgHorizontalOffset = 0.0;

    constexpr double TARGET_DEGREES = 720.0;
    constexpr int MIN_SPIN_POWER = 18;
    constexpr int MAX_SPIN_POWER = 42;
    constexpr int TIMEOUT_MS = 7000;

    const char* status() {
        if (active) return "RUNNING";
        if (sampleCount > 0) return "READY";
        return "IDLE";
    }

    const char* directionLabel() {
        if (lastDirection > 0) return "CCW";
        if (lastDirection < 0) return "CW";
        return "--";
    }

    void run(int direction) {
        if (active || direction == 0) return;

        active = true;
        lastDirection = direction;
        TaskHandler::driver = false;
        TaskHandler::autonSelect = false;

        leftMotors.move(0);
        rightMotors.move(0);
        pros::delay(100);

        vertical.reset();
        horizontal.reset();
        s_imu.tare_rotation();
        chassis.setPose(0, 0, 0);
        pros::delay(50);

        int elapsed = 0;
        while (elapsed < TIMEOUT_MS && std::abs(s_imu.get_rotation()) < TARGET_DEGREES) {
            const double remaining = TARGET_DEGREES - std::abs(s_imu.get_rotation());
            const int power = std::clamp(static_cast<int>(remaining * 0.08) + MIN_SPIN_POWER, MIN_SPIN_POWER, MAX_SPIN_POWER);
            leftMotors.move(direction * power);
            rightMotors.move(-direction * power);
            pros::delay(Misc::DELAY);
            elapsed += Misc::DELAY;
        }

        leftMotors.brake();
        rightMotors.brake();
        pros::delay(300);

        lastVertical = vertical.getDistanceTraveled();
        lastHorizontal = horizontal.getDistanceTraveled();
        lastHeadingDeg = s_imu.get_rotation();

        const double headingRad = lastHeadingDeg * M_PI / 180.0;
        if (std::abs(headingRad) > 1e-5) {
            lastVerticalOffset = -lastVertical / headingRad;
            lastHorizontalOffset = -lastHorizontal / headingRad;
            avgVerticalOffset = (avgVerticalOffset * sampleCount + lastVerticalOffset) / (sampleCount + 1);
            avgHorizontalOffset = (avgHorizontalOffset * sampleCount + lastHorizontalOffset) / (sampleCount + 1);
            sampleCount++;
        }

        chassis.setPose(0, 0, 0);
        pros::delay(20);
        TaskHandler::driver = true;
        active = false;
        controller.rumble(".");
    }
} // namespace OdomTune

// <-------------------------------------------------------------- Anti Jam ----------------------------------------------------------->
namespace Jam{
    int counter = 0;
    int counter1 = 0;
    bool stuck = false;
    void antiJam(){
        if(TaskHandler::antiJam){
            counter+=Misc::DELAY;
            if(Motor::intake1.get_actual_velocity() == 0 && counter > 300) stuck = true;
            if (stuck == true) {
                // TaskHandler::colorSort = false;
                Motor::intake1.move(-127);
                pros::delay(100);
                Motor::intake1.move(127);
                stuck = false;
                counter = 0;  
            }
        }
    }
}

namespace Intake{
    enum class State { LOCK, SPIT, SCORE, LG, SKILLS };
    State currentState = State::LOCK;

    void setState(State state){
        currentState = state;
        int intake1 = 0;
        int intake2 = 0;
        bool hoodc = false;
        // bool liftc = true;

        switch (state) {
            case State::LOCK:
                intake1 = 127;
                intake2 = 127;
                hoodc = false;
                // liftc = true;
                break;
            case State::SPIT:
                intake1 = -127;
                intake2 = -127;
                hoodc = false;
                // liftc = true;
                break;
            case State::SCORE:
                intake1 = 127;
                intake2 = 127;
                hoodc = true;
                // liftc = true;
                break; 
            case State::LG:
                intake1 = -50;
                intake2 = -50;
                hoodc = false;
                // liftc = true;
                break;
            case State::SKILLS:
                intake1 = 50;
                intake2 = 50;
                hoodc = false;
                // liftc = false;
                break;
            
        }
        Motor::intake1.move(intake1);
        Motor::intake2.move(intake2);
        Piston::hood.set_value(hoodc);
        // Piston::lift.set_value(liftc);
    }
}

// <-------------------------------------------------------------- Auto Routes ----------------------------------------------------------->
namespace Auton{
    int state = 0;
    void test() { 
        Misc::cdrift(55,55,550);
    }
    void left(){

    }
    void right(){
        
    }  
    void solo(){          
        
    }
    void skills() { 
        
    }
} // namespace Auton

// <-------------------------------------------------------------- Driver Code ----------------------------------------------------------->
namespace Driver{
    bool b_loader = false, b_clamp = false, b_aligner = false, b_hook = false, b_driver = false, b_middle = false, b_lift = false;
    double curveVal = 7.0;
    void joystick(){
        while(1){
            if(TaskHandler::driver) {
                int leftY = Misc::curve(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), curveVal, false); 
                int rightX = Misc::curve(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), curveVal, false);
                leftMotors.move(leftY+rightX); 
                rightMotors.move(leftY-rightX);
            }
            pros::delay(Misc::DELAY);
        }
    }

    void intake(){
        while (1){
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) Intake::setState(Intake::State::LOCK);
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) Intake::setState(Intake::State::SCORE);
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) Intake::setState(Intake::State::SPIT);
            else {
                Motor::intake1.brake();
                Motor::intake2.brake();
                Piston::hood.set_value(false);
            }
            pros::delay(Misc::DELAY);
        }
    }

    void piston(){
        while(1){
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) { Misc::togglePiston(Piston::loader, b_loader); }
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { Misc::togglePiston(Piston::hook, b_hook); }
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) { Misc::togglePiston(Piston::lift, b_lift); }
            (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) ? Piston::low.set_value(true) : Piston::low.set_value(false);

            pros::delay(Misc::DELAY);
        }
    }
} // namespace Driver

// <-------------------------------------------------------------- Auton ----------------------------------------------------------->

using AutonFunc = void(*)();
std::vector<std::pair<std::string, AutonFunc>> autonRoutines = {
    {"Default Auton", Auton::left},
    {"Right Auton", Auton::right},
    {"Solo Auton", Auton::solo},
    {"Skills Auton", Auton::skills},
};

int autonState = 0;

void autonSwitch() {
    if(TaskHandler::autonSelect) {    
        pros::delay(Misc::DELAY);
        if (Sensor::autonSwitch.get_new_press()) { autonState++; if (autonState == autonRoutines.size()) autonState = 0; }
        pros::lcd::set_text(4, autonRoutines[autonState].first);
    }
}

LV_IMG_DECLARE(tdbg);
LV_IMG_DECLARE(WORLDS_logo);
LV_IMG_DECLARE(WO_logo);
LV_IMG_DECLARE(Final_log);
LV_IMG_DECLARE(screen);
LV_IMG_DECLARE(sixseven);
LV_IMG_DECLARE(gay);
lv_obj_t * sbg = lv_img_create(lv_scr_act());
lv_obj_t * sixSeven = lv_img_create(lv_scr_act());
lv_obj_t * sKiss = lv_img_create(lv_scr_act());
lv_obj_t * slogo = lv_img_create(lv_scr_act());
lv_obj_t * Wlogo = lv_img_create(lv_scr_act());
lv_obj_t * Slogo = lv_img_create(lv_scr_act());

// lv_img_set_src(Wlogo, &WORLDS_logo);
// lv_obj_set_pos(Wlogo, 10, 3);
// <------------------------------------------------------------ Initialize --------------------------------------------------------------->
void initialize() {
    pros::Task t_Select(autonSwitch);
    pros::lcd::initialize();
    chassis.setPose(0, 0, 0);
    chassis.calibrate(); 
    RclMain.setRclPose(chassis.getPose());
    RclMain.startTracking();
    Sensor::o_colorSort.set_led_pwm(100);
    Sensor::o_colorSort.set_integration_time(5);
    Sensor::o_crossed.set_led_pwm(100);
    Sensor::o_crossed.set_integration_time(5);
    Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intake2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // lv_init();
    // set_up();
    // pros::Task LVGL_upd([&]() { screen_upd(); });

	// lv_img_set_src(slogo, &Final_log);
	// lv_obj_set_pos(slogo, 20, 15);

    // lv_img_set_src(Slogo, &screen);
	// lv_obj_set_pos(Slogo, 0, 0);

    // lv_img_set_src(sixSeven, &sixseven);
	// lv_obj_set_pos(sixSeven, 160, 0);
    pros::Task screenTask([&]() {
        while (1) {
            // Misc::resetFinal();
            // Misc::resetWalls(true,true,true,{-46,0,10});
            // Misc::resetWalls(true,true,true,{46,1,5});
            // Misc::resetWalls(false,true,true);
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            // pros::lcd::print(3, "Tune UP=CCW LEFT=CW");
            // if (!TaskHandler::autonSelect) {
            //     pros::lcd::print(4, "Tune %s Dir:%s", OdomTune::status(), OdomTune::directionLabel());
            // }
            // pros::lcd::print(5, "IMU %.1f V %.2f H %.2f", OdomTune::lastHeadingDeg, OdomTune::lastVertical, OdomTune::lastHorizontal);
            // pros::lcd::print(6, "LastOff V %.2f H %.2f", OdomTune::lastVerticalOffset, OdomTune::lastHorizontalOffset);
            // pros::lcd::print(7, "Avg%02d V %.2f H %.2f", OdomTune::sampleCount, OdomTune::avgVerticalOffset, OdomTune::avgHorizontalOffset);
            pros::delay(50);
        }
    });

    pros::Task autonSelect([]{ while(1){ autonSwitch(); pros::delay(Misc::DELAY); }});
    // pros::Task stopIntake([]{ while(1){ Jam::intake(); pros::delay(Misc::DELAY); }});
    // pros::Task screenC([]{ while (1) { Screen::update(); pros::delay(100); }});
}
void disabled() {}
void competition_initialize() {}
ASSET(example_txt); // PP

// <------------------------------------------------------------- Auton ------------------------------------------------------------->
void autonomous() {
    // Auton::left();
    // pros::delay(1000000);
    
    chassis.turnToHeading(90,100000);
    pros::delay(10000000);
    // chassis.turnToHeading(90,1500);
    // pros::delay(3000);
    // chassis.turnToHeading(180,1500);
    // pros::delay(3000);
    // chassis.turnToHeading(270,1500);
    // pros::delay(3000);
    // chassis.turnToHeading(0,1500);
    // pros::delay(3000);
    // chassis.turnToHeading(90,1000);
    // chassis.turnToHeading(180,1000);
    // chassis.turnToHeading(270,1000);
    // chassis.turnToHeading(0,1000);
    // chassis.moveToPoint(0, 24, 1500);
    // chassis.turnToHeading(180,1000);
    // chassis.moveToPoint(0, 0, 1500);
    // chassis.turnToHeading(0,1000);
    // chassis.moveToPoint(-24,24,1750,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=0});

    
    (autonState < autonRoutines.size()) ? autonRoutines[autonState].second() : Auton::test();
}
//                          _ooOoo
//                         o8888888o
//                         88" . "88 
//                         (| -_- |)
//                         O\  =  /O
//                       ___/`---'\____
//                    .'  \\|     |//  `.
//                   /  \\|||  :  |||//  \
//                  /  _||||| -:- |||||_  \
//                  |   | \\\  -  /// |   |
//                  | \_|  ''\---/''  |   |
//                  \  .-\__       __/-.  /
//                ___`. .'  /--.--\ `. . __
//             ."" '<  `.___\_<|>_/__.'  >'"".
//            | | :  `- \`.;`\ _ /`;.`/ - ` : | |
//            \  \ `-.   \_ __\ /__ _/   .-` /  /
//       ======`-.____`-.___\_____/___.-`____.-'======
//                          `=---='
//    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//                   佛祖保佑       永無BUG
// <--------------------------------------------------------------- Driver --------------------------------------------------------------->
void opcontrol() {
    // pros::lcd::shutdown();
    // lv_init();
    // lv_img_set_src(sKiss, &gay);
	// lv_obj_set_pos(sKiss, 0, 0);

    // lv_img_set_src(sixSeven, &sixseven);
	// lv_obj_set_pos(sixSeven, 140, 0);

    pros::Task intakeTask(Driver::intake);
    pros::Task driverTask(Driver::joystick);
    pros::Task pistonTask(Driver::piston);
    TaskHandler::autonSelect = false;
    TaskHandler::colorSort = false;
    TaskHandler::antiJam = false;
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST); rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);  Motor::intake2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    while(1) {
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) OdomTune::run(1);
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) OdomTune::run(-1);
        pros::delay(Misc::DELAY);
    }
}
