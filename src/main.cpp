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
pros::MotorGroup rightMotors({18, -6, 20}, pros::MotorGearset::blue); // checked

namespace Motor{
  pros::Motor intake1(-14, pros::MotorGearset::blue);
  pros::Motor intake2(8, pros::MotorGearset::blue); 
//   pros::MotorGroup intake({-14, 17}, pros::MotorGearset::blue); 
} // namespace Motor

namespace Sensor{
  pros::Distance d_frontR(7); 
  pros::Distance d_frontL(5);
  pros::Distance d_left(3);
  pros::Distance d_right(9); 
  pros::Optical o_colorSort(10); 
  pros::Optical o_crossed(17); 
  pros::adi::DigitalIn autonSwitch('H'); 
} // namspace Sensor

namespace Piston{
  pros::adi::DigitalOut loader('F'); 
  pros::adi::DigitalOut hook('C'); // Checked
  pros::adi::DigitalOut low('D'); // Checked
  pros::adi::DigitalOut lift('B'); // Checked
  pros::adi::DigitalOut hood('E');
  pros::adi::DigitalOut odom('G');
pros::adi::DigitalOut descore('A');
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
CustomIMU s_imu(1, 1.01123595506); // checked

pros::Rotation horizontalEnc(-15);
pros::Rotation verticalEnc(-16); 

genesis::TrackingWheel vertical(&verticalEnc, 2.0 , -1.0); // Singleji // 0.046541 -1.0
genesis::TrackingWheel horizontal(&horizontalEnc, 2.75 , -2.469176); // Double Stacked -2.469176 -3.04

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
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

genesis::ControllerSettings angularController(2.75, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              12, // derivative gain (kD) 
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

namespace MotionPlusTuning {
    // constexpr float lateralKpInitial = 6767.0f;
    // constexpr float lateralKpFinal = 6767.0f;
    // constexpr float lateralKpKnee = 1.0f;
    // constexpr float lateralKpPower = 1.0f;
    // constexpr float lateralKi = 100.0f;
    // constexpr float lateralKd = 13.0f;
    // constexpr float lateralIntegralDeadband = 6.0f;
    // constexpr bool lateralIntegralSignReset = true;

    // constexpr float lateralKpInitial = 7000.0f;
    // constexpr float lateralKpFinal = 7000.0f;
    // constexpr float lateralKpKnee = 1.0f;
    // constexpr float lateralKpPower = 1.0f;
    // constexpr float lateralKi = 6000.0f;
    // constexpr float lateralKd = 70.0f;
    // constexpr float lateralIntegralDeadband = 0.5f;
    // constexpr bool lateralIntegralSignReset = true;

    constexpr float lateralKpInitial = 6500.0f;
    constexpr float lateralKpFinal = 6500.0f;
    constexpr float lateralKpKnee = 1.0f;
    constexpr float lateralKpPower = 1.0f;
    constexpr float lateralKi = 1000.0f;
    constexpr float lateralKd = 14.0f;
    constexpr float lateralIntegralDeadband = 0.5f;
    constexpr bool lateralIntegralSignReset = true;

    // AsymptoticGains lateralKp = AsymptoticGains(15000, 15000, 1, 1);
    // AsymptoticGains angularKp = AsymptoticGains(480, 220, 28, 1.7);
    // AsymptoticGains correctKp = AsymptoticGains(200, 200, 1, 1);

    // PID lateralPID = PID(lateralKp, 25000, 150, 0.5, true);
    // PID angularPID = PID(angularKp, 2000, 12, 5, true);
    // PID correctPID = PID(correctKp, 0, 40, 0, false);


    constexpr float turnKpInitial = 450.0f;
    constexpr float turnKpFinal = 220.0f;
    constexpr float turnKpKnee = 28.0f;
    constexpr float turnKpPower = 1.5f;
    constexpr float turnKi = 1500.0f; 
    constexpr float turnKd = 14.0f;
    constexpr float turnIntegralDeadband = 5.0f;
    constexpr bool turnIntegralSignReset = true;

    constexpr float headingKpInitial = 200.0f;
    constexpr float headingKpFinal = 200.0f;
    constexpr float headingKpKnee = 1.0f;
    constexpr float headingKpPower = 1.0f;
    constexpr float headingKi = 0.0f;
    constexpr float headingKd = 0.0f;
    constexpr float headingIntegralDeadband = 0.0f;
    constexpr bool headingIntegralSignReset = false;

    constexpr float pursuitMinLookahead = 4.0f;
    constexpr float pursuitMaxLookahead = 12.0f;
    constexpr float pursuitVelocityConst = 0.12f;
    constexpr float pursuitCurvatureConst = 0.02f;
    constexpr float pursuitCTEConst = 0.25f;
    constexpr float pursuitRadiusLookahead = 5.0f;

    void apply() {
        chassis.setLateralPIDPlus(lateralKpInitial, lateralKpFinal, lateralKpKnee, lateralKpPower, lateralKi, lateralKd,
                                  lateralIntegralDeadband, lateralIntegralSignReset);
        chassis.setTurnPIDPlus(turnKpInitial, turnKpFinal, turnKpKnee, turnKpPower, turnKi, turnKd,
                               turnIntegralDeadband, turnIntegralSignReset);
        chassis.setHeadingCorrectPIDPlus(headingKpInitial, headingKpFinal, headingKpKnee, headingKpPower, headingKi,
                                         headingKd, headingIntegralDeadband, headingIntegralSignReset);
        chassis.setPursuitSettingsPlus(pursuitMinLookahead, pursuitMaxLookahead, pursuitVelocityConst,
                                       pursuitCurvatureConst, pursuitCTEConst, pursuitRadiusLookahead);
    }
} // namespace MotionPlusTuning

// RCL tracking sensors: right offset is +, forward offset is +, headings in degrees.
RclSensor rclFrontR(&Sensor::d_frontR, 0.0, 9.0, 0.0, 15.0);
RclSensor rclFrontL(&Sensor::d_frontL, 0.0, 9.0, 0.0, 15.0);
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
    constexpr double FIELD_EDGE = 48.0;
    // constexpr double FRONT_SENSOR_OFFSET = 6.5;
    // constexpr double LEFT_SENSOR_OFFSET = -5;
    // constexpr double RIGHT_SENSOR_OFFSET = -5;
    constexpr double FRONT_SENSOR_OFFSET = 17; // 15
    constexpr double FRONT_SENSOR_2_OFFSET = 19;
    constexpr double LEFT_SENSOR_OFFSET = 19;
    constexpr double RIGHT_SENSOR_OFFSET = 19;
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

    void reset(int wall, int sensor, bool bypassSettleDelay = false) {
        leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
        rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
        leftMotors.brake();
        rightMotors.brake();

        if (!bypassSettleDelay) pros::delay(DELAY);

        double distanceReading = 0.0;
        double sensorOffset = 0.0;

        if (sensor == 1) {
            distanceReading = Sensor::d_frontR.get_distance() / 25.4;
            sensorOffset = FRONT_SENSOR_OFFSET;
        } else if (sensor == 4) {
            distanceReading = Sensor::d_frontL.get_distance() / 25.4;
            sensorOffset = FRONT_SENSOR_2_OFFSET;
        } else if (sensor == 2) {
            distanceReading = Sensor::d_left.get_distance() / 25.4;
            sensorOffset = LEFT_SENSOR_OFFSET;
        } else if (sensor == 3) {
            distanceReading = Sensor::d_right.get_distance() / 25.4;
            sensorOffset = RIGHT_SENSOR_OFFSET;
        } else {
            printf("Misc::reset invalid sensor %d\n", sensor);
            return;
        }

        if (!std::isfinite(distanceReading) || distanceReading <= 0.0) {
            printf("Misc::reset bad distance reading %.2f\n", distanceReading);
            return;
        }

        const double relativeOffset = sensorOffset - distanceReading;
        const double sign = (wall <= 2) ? 1.0 : -1.0;
        const double newPos = sign * (FIELD_EDGE + relativeOffset);
        const double heading = chassis.getPose().theta;

        if (wall == 1 || wall == 3) {
            chassis.setPose(newPos, chassis.getPose().y, heading);
        } else if (wall == 2 || wall == 4) {
            chassis.setPose(chassis.getPose().x, newPos, heading);
        } else {
            printf("Misc::reset invalid wall %d\n", wall);
            return;
        }

        RclMain.setRclPose(chassis.getPose());
        printf("Misc::reset -> X: %.2f, Y: %.2f, H: %.2f\n", chassis.getPose().x, chassis.getPose().y, heading);
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
        constexpr double front2OffsetF = 9.0;
        constexpr double front2OffsetR = 0.0;

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
        Result front2Res = {Axis::NONE, 0.0};

        auto validReading = [&](double reading) {
            return std::isfinite(reading) && reading > 0.0;
        };

        if (useLeft) {
            realReading = Sensor::d_left.get_distance() * mmToIn;
            if (validReading(realReading)) {
                m_offset = genesis::Pose(0.0, -leftOffsetR, M_PI_2);
                getReading(pose, leftRes, false);
            }
        }
        if (useRight) {
            realReading = Sensor::d_right.get_distance() * mmToIn;
            if (validReading(realReading)) {
                m_offset = genesis::Pose(0.0, -rightOffsetR, -M_PI_2);
                getReading(pose, rightRes, false);
            }
        }
        if (useFront) {
            realReading = Sensor::d_frontR.get_distance() * mmToIn;
            if (validReading(realReading)) {
                m_offset = genesis::Pose(frontOffsetF, -frontOffsetR, 0.0);
                getReading(pose, frontRes, false);
            }
            realReading = Sensor::d_frontL.get_distance() * mmToIn;
            if (validReading(realReading)) {
                m_offset = genesis::Pose(front2OffsetF, -front2OffsetR, 0.0);
                getReading(pose, front2Res, false);
            }
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
        if (useFront) {
            accumulate(frontRes);
            accumulate(front2Res);
        }

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

namespace Color {
    enum class colorVals { NONE, BLUE, RED };

    constexpr double rLow = 5.0;
    constexpr double rHigh = 38.0;
    constexpr double bLow = 175.0;
    constexpr double bHigh = 225.0;
    constexpr double minProx = 95.0;

    inline bool withinProx(double prox) { return prox > minProx; }
    inline bool isRed(double hue) { return hue > rLow && hue < rHigh; }
    inline bool isBlue(double hue) { return hue > bLow && hue < bHigh; }

    colorVals detectedColor() {
        const double hue = Sensor::o_colorSort.get_hue();
        const double prox = Sensor::o_colorSort.get_proximity();

        if (!withinProx(prox)) return colorVals::NONE;
        if (isBlue(hue)) return colorVals::BLUE;
        if (isRed(hue)) return colorVals::RED;
        return colorVals::NONE;
    }

    bool blueDetected() {
        return detectedColor() == colorVals::BLUE;
    }
}

namespace Intake{
    enum class State { LOCK, LOCKBOTTOM, SPIT, SCORE, SCORESLOW, LG1, LG2, SKILLS };
    State currentState = State::LOCK;

    void setState(State state){
        currentState = state;
        int intake1 = 0;
        int intake2 = 0;
        bool hoodc = false;
        bool intakec = false;
        // bool liftc = true;

        switch (state) {
            case State::LOCK:
                intake1 = 127;
                intake2 = 127;
                hoodc = false;
                // liftc = true;
                break;
            case State::LOCKBOTTOM:
                intake1 = 0;
                intake2 = 127;
                hoodc = false;
                // liftc = true;
                break;
            case State::SPIT:
                intake1 = -127;
                intake2 = -127;
                hoodc = false;
                intakec = true;
                // liftc = true;
                break;
            case State::SCORE:
                intake1 = 127;
                intake2 = 127;
                hoodc = true;
                // liftc = true;
                break; 
            case State::SCORESLOW:
                intake1 = 100;
                intake2 = 127;
                hoodc = true;
                // liftc = true;
                break; 
            case State::LG1:
                intake1 = -127;
                intake2 = -50; // -55. 
                // 53 49
                hoodc = false;
                intakec = true;
                // liftc = true;
                break;
            case State::LG2:
                intake1 = -127;
                intake2 = -50;
                hoodc = false;
                intakec = true;
                // liftc = true;
                break;
            case State::SKILLS:
                intake1 = 55;
                intake2 = 127;
                hoodc = false;
                // liftc = false;
                break;
            
        }
        Motor::intake1.move(intake1);
        Motor::intake2.move(intake2);
        Piston::hood.set_value(hoodc);
        Piston::low.set_value(intakec);
        // Piston::lift.set_value(liftc);
    }

    void indexUntilDetected(int timeout = 3500){
        int elapsed = 0;
        while (elapsed < timeout && !Color::isBlue(Sensor::o_colorSort.get_hue())) {
            // Piston::lift.set_value(false);
            // Intake::setState(Intake::State::SCORE);
            pros::delay(10);
            elapsed += 10;
        }

        if (Color::blueDetected()) {
            Motor::intake1.brake();
            Motor::intake2.brake();
            // Intake::setState(Intake::State::LOCK);
        }
    }
}

// <-------------------------------------------------------------- Auto Routes ----------------------------------------------------------->
namespace Auton{
    int state = 0;
    void test() { 
        Misc::cdrift(55,55,550);
    }
    void testPlus() {
        chassis.setPose(0, 0, 0);

        chassis.setIntakeExitSupplierPlus([] {
            return std::max(std::fabs(Motor::intake1.get_actual_velocity()),
                            std::fabs(Motor::intake2.get_actual_velocity()));
        }, 550.0f, 300.0f);

        Intake::setState(Intake::State::LOCK);

        // Simple turn tune check
        chassis.turnHeadingPlus(45, 2, 0, 1.0, 1200, false);
        pros::delay(200);

        // Straight point move
        chassis.movePointPlus(0, 24, 0, 0.85, 1800, true, false, false);
        pros::delay(200);

        // Chained point move with hot exit
        chassis.movePointPlus(12, 36, 0.10, 0.90, 1800, true, false, false);
        pros::delay(150);

        // Boomerang pose move
        chassis.movePosePlus(24, 48, 90, 10.0, 0.30, 0.10, 0.90, 2200, 0.04, true, false, false);
        pros::delay(150);

        // Intake-based exit example
        chassis.movePointPlus(24, 60, 0.10, 0.80, 2000, true, true, false);
        pros::delay(150);

        // Simple path example
        genesis::PathPlus path({
            {24, 60, 10000},
            {30, 72, 10000},
            {42, 84, 9000},
            {54, 96, 0}
        });
        chassis.followStanleyPlus(path, 2500, 1.2, -1, false, false, false);

        chassis.clearIntakeExitSupplierPlus();
        Intake::setState(Intake::State::SPIT);
        pros::delay(300);
        Motor::intake1.brake();
        Motor::intake2.brake();
    }

    void leftsevenwingback(){
        chassis.setPose(-49, 14, 90);
        Intake::setState(Intake::State::LOCK);
        // chassis.movePosePlus()
        // chassis.movePosePlus(-27, 21, 80, 0, 0, 0.0, 1, 1500, -1, false, false, false);
        chassis.moveToPointPro(-27,21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=3});
        chassis.waitUntil(8);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        chassis.movePosePlus(-49, 47, 300, 0, 0, 0.04, 1, 1500, -1, false, false, false);
        // chassis.movePosePlus(-51, 43, 120, 0, 0, 0.0, -1, 1500, -1, false, false, false);
        // chassis.movePointPlus(-51,43,0,-1,1500,false,false,false);
        // chassis.movePosePlus(-46, 48, 120, 0, 0, 0.0, -1, 1500, -1, true, false, false);

        // chassis.moveToPointPro(-27,21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        // chassis.waitUntilDone();
        // chassis.turnPointPlus(-65, 43.5, 2, 0.0, 1, 1500, false);

        // chassis.moveToPointPro(-46, 48, 1500,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
        // chassis.waitUntilDone();
        chassis.turnPointPlus(-65, 49, 2, 0.0, 1, 1500, false);
        
        // Misc::cdrift(60,60,150);
        Misc::cdrift(40,40,750);



        // chassis.turnHeadingPlus(315, 2, 0, 1.0, 400, false); // 310
        // // chassis.turnToPoint(58,41,400,{.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
        // chassis.moveToPosePro(-65,47.5,270,1500,{.forwards=true,.horizontalDrift=2,.lead=0.45,.maxSpeed=80,.minSpeed=0,.earlyExitRange=0}); 
        // chassis.waitUntilDone();
        // Misc::cdrift(55,55,110);
        // // // Misc::cdrift(40,40,670);
        // Misc::cdrift(40,40,550);
        chassis.moveToPointPro(-28.5, 49.3, 1500,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(23);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,500);
        Intake::setState(Intake::State::LOCK);

        chassis.movePosePlus(-36, 44, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0.35, 1.0, 1500, false);

        chassis.movePosePlus(-11, 39.5, 270, 0, 0, 0.0, -0.8, 300, -1, true, false, false); // 15 wing
        chassis.movePosePlus(-11, 39.5, 270, 0, 0, 0.0, -0.45, 1500, -1, true, false, false); // 15 wing
        // chassis.turnHeadingPlus(285, 2, 0.0, 0.25, 2000, false);
        Misc::cdrift(0,-15);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }

    void rightsevenwingback(){
    
    }

    void leftsevenwingbackhold(){
        chassis.setPose(-49, 14, 90);
        Intake::setState(Intake::State::LOCK);
        // chassis.movePosePlus()
        // chassis.movePosePlus(-27, 21, 80, 0, 0, 0.0, 1, 1500, -1, false, false, false);
        chassis.moveToPointPro(-27,21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=3});
        chassis.waitUntil(8);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        chassis.movePosePlus(-49, 47, 300, 0, 0, 0.04, 1, 1500, -1, false, false, false);
        // chassis.movePosePlus(-51, 43, 120, 0, 0, 0.0, -1, 1500, -1, false, false, false);
        // chassis.movePointPlus(-51,43,0,-1,1500,false,false,false);
        // chassis.movePosePlus(-46, 48, 120, 0, 0, 0.0, -1, 1500, -1, true, false, false);

        // chassis.moveToPointPro(-27,21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        // chassis.waitUntilDone();
        // chassis.turnPointPlus(-65, 43.5, 2, 0.0, 1, 1500, false);

        // chassis.moveToPointPro(-46, 48, 1500,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
        // chassis.waitUntilDone();
        chassis.turnPointPlus(-65, 49, 2, 0.0, 1, 1500, false);
        
        // Misc::cdrift(60,60,150);
        Misc::cdrift(40,40,750);



        // chassis.turnHeadingPlus(315, 2, 0, 1.0, 400, false); // 310
        // // chassis.turnToPoint(58,41,400,{.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
        // chassis.moveToPosePro(-65,47.5,270,1500,{.forwards=true,.horizontalDrift=2,.lead=0.45,.maxSpeed=80,.minSpeed=0,.earlyExitRange=0}); 
        // chassis.waitUntilDone();
        // Misc::cdrift(55,55,110);
        // // // Misc::cdrift(40,40,670);
        // Misc::cdrift(40,40,550);
        chassis.moveToPointPro(-28, 50, 1500,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(23);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,500);
        Intake::setState(Intake::State::LOCK);

        chassis.turnHeadingPlus(180, 1, 1.0, 1.0, 1500, false);
        chassis.turnHeadingPlus(270, 2, 0.0, 1.0, 1500, false);
        pros::delay(200);
        Misc::cdrift(-30,-30,300,false);


        // chassis.movePosePlus(-36, 44, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        // Piston::hook.set_value(true);
        // chassis.turnHeadingPlus(270, 2, 0.35, 1.0, 1500, false);

        // chassis.movePosePlus(-30, 39.5, 270, 0, 0, 0.0, -0.4, 300, -1, true, false, false); // 15 wing
        // chassis.movePosePlus(-11, 39.5, 270, 0, 0, 0.0, -0.45, 1500, -1, true, false, false); // 15 wing
        Misc::cdrift(0,-15);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }

    void leftsevenend(){
    
    }

    void rightsevenend(){
    
    }

    void leftsevenwingfront(){
        chassis.setPose(-49, 14, 90);
        Intake::setState(Intake::State::LOCK);
        chassis.moveToPointPro(-27,21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(7.5);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        chassis.turnHeadingPlus(315, 2, 0, 1.0, 400, false); // 310
        // chassis.turnToPoint(58,41,400,{.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
        chassis.moveToPosePro(-65,47.5,270,1500,{.forwards=true,.horizontalDrift=2,.lead=0.45,.maxSpeed=80,.minSpeed=0,.earlyExitRange=0}); 
        chassis.waitUntilDone();
        // Misc::cdrift(55,55,110);
        // Misc::cdrift(40,40,670);
        Misc::cdrift(40,40,550);
        chassis.moveToPointPro(-30, 48, 1500,{.forwards=false,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(20.5);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,500);
        Intake::setState(Intake::State::LOCK);

        chassis.movePosePlus(-36, 40, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0.35, 1.0, 1500, false);

        chassis.movePosePlus(-11, 39.5, 270, 0, 0, 0.0, -0.45, 1000, -1, true, false, false); // 15 wing
        // chassis.turnHeadingPlus(285, 2, 0.0, 0.25, 2000, false);
        Misc::cdrift(0,-15);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
    void rightsevenwingfront(){
        chassis.setPose(-49, -14, 90);
        Intake::setState(Intake::State::LOCK);
        chassis.moveToPointPro(-27,-21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(7);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        chassis.turnHeadingPlus(230, 2, 0, 1.0, 400, false);
        // chassis.turnToPoint(58,41,400,{.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
        chassis.moveToPosePro(-65,-44.5,270,1500,{.forwards=true,.horizontalDrift=2,.lead=0.485,.maxSpeed=80,.minSpeed=0,.earlyExitRange=0}); 
        chassis.waitUntilDone();
        // Misc::cdrift(55,55,110);
        // Misc::cdrift(40,40,670);
        Misc::cdrift(40,40,550);
        chassis.moveToPointPro(-30, -45, 1500,{.forwards=false,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(19.5);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,500);
        Intake::setState(Intake::State::LOCK);

        chassis.movePosePlus(-34, -53, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0.35, 1.0, 1500, false);

        chassis.movePosePlus(-11, -53.5, 270, 0, 0, 0.0, -0.45, 1000, -1, true, false, false); // 15 wing
        // chassis.turnHeadingPlus(285, 2, 0.0, 0.25, 2000, false);
        Misc::cdrift(0,-15);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
    void left43wing(){
        chassis.setPose(-49, 14, 90);
        Intake::setState(Intake::State::LOCK);
        chassis.moveToPointPro(-27,21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(7.5);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        chassis.moveToPointPro(-8,9,1000,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        Piston::lift.set_value(true);
        pros::delay(50);
        Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-10,-10,500);
        
        chassis.turnHeadingPlus(310, 2, 0, 1.0, 400, false);
        Piston::lift.set_value(false);
        Intake::setState(Intake::State::LOCK);
        // chassis.turnToPoint(58,41,400,{.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
        chassis.moveToPosePro(-65,47.5,270,1500,{.forwards=true,.horizontalDrift=2,.lead=0.45,.maxSpeed=80,.minSpeed=0,.earlyExitRange=0}); 
        chassis.waitUntilDone();
        // Misc::cdrift(55,55,110);
        // Misc::cdrift(40,40,670);
        Misc::cdrift(40,40,750);
        chassis.moveToPointPro(-30, 48, 1500,{.forwards=false,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(21.5);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,500);
        Intake::setState(Intake::State::LOCK);

        chassis.movePosePlus(-36, 40, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0.35, 1.0, 1500, false);

        chassis.movePosePlus(-12.5, 39.5, 270, 0, 0, 0.0, -0.85, 1000, -1, true, false, false); // 15 wing
        Misc::cdrift(0,-15);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
    void right43wing(){
        chassis.setPose(-49, -14, 90);
        Intake::setState(Intake::State::LOCKBOTTOM);
        Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        Motor::intake1.brake();
        chassis.moveToPointPro(-27,-21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(7.5);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();

        chassis.moveToPointPro(-11,-6,900,{.forwards=true,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(4);
        Piston::loader.set_value(false);
        chassis.waitUntilDone();
        chassis.turnHeadingPlus(45, 2, 0, 1.0, 300, false);
        // chassis.turnPointPlus(0, 0, 2, 0.0, 1, 500, false);
        
        // Piston::low.set_value(true);
        // chassis.movePosePlus(-12, -12, 45, 0, 0, 0.0, 0.35, 1500, -1, true, false, false);
        // chassis.moveDistPlus(15, 0.4, 1500, false);
        Intake::setState(Intake::State::SPIT);
        Misc::cdrift(50,50,800);
        Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Intake::setState(Intake::State::LOCK);
        Misc::cdrift(-55,-55,200);


        // chassis.moveToPointPro(-8,-9,1000,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        // chassis.waitUntilDone();
        // Piston::lift.set_value(true);
        // Intake::setState(Intake::State::SCORE);
        // Misc::cdrift(-10,-10,500);

        chassis.turnHeadingPlus(230, 2, 0, 1.0, 400, false);
        // chassis.turnToPoint(58,41,400,{.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
        chassis.moveToPosePro(-65,-43.5,270,1500,{.forwards=true,.horizontalDrift=2,.lead=0.495,.maxSpeed=80,.minSpeed=0,.earlyExitRange=0}); 
        chassis.waitUntil(14);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        // Misc::cdrift(55,55,110);
        // Misc::cdrift(40,40,670);
        Misc::cdrift(40,40,700);
        chassis.moveToPointPro(-30, -44, 1500,{.forwards=false,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(20.5);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,500);
        Intake::setState(Intake::State::LOCK);

        chassis.movePosePlus(-34, -51, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0.35, 1.0, 1500, false);

        chassis.movePosePlus(-12.5, -51.5, 270, 0, 0, 0.0, -0.85, 1000, -1, true, false, false); // 15 wing
        // chassis.turnHeadingPlus(285, 2, 0.0, 0.25, 2000, false);
        Misc::cdrift(0,-15);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        
        // chassis.turnHeadingPlus(230, 2, 0, 1.0, 400, false);
        // Piston::lift.set_value(false);
        // Intake::setState(Intake::State::LOCK);
        // // chassis.turnToPoint(58,41,400,{.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
        // chassis.moveToPosePro(-65,-47.5,270,1500,{.forwards=true,.horizontalDrift=2,.lead=0.45,.maxSpeed=80,.minSpeed=0,.earlyExitRange=0}); 
        // chassis.waitUntilDone();
        // // Misc::cdrift(55,55,110);
        // // Misc::cdrift(40,40,670);
        // Misc::cdrift(40,40,750);
        // chassis.moveToPointPro(-30, -48, 1500,{.forwards=false,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
        // chassis.waitUntil(20.5);
        // Intake::setState(Intake::State::SCORE);
        // chassis.waitUntilDone();
        // Misc::cdrift(-20,-20,200);
        // Piston::loader.set_value(false);
        // Misc::cdrift(-20,-20,500);
        // Intake::setState(Intake::State::LOCK);

        // chassis.movePosePlus(-34, -53, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        // Piston::hook.set_value(true);
        // chassis.turnHeadingPlus(270, 2, 0.35, 1.0, 1500, false);

        // chassis.movePosePlus(-12.5, -53.5, 270, 0, 0, 0.0, -0.85, 1000, -1, true, false, false); // 15 wing
        // // chassis.turnHeadingPlus(285, 2, 0.0, 0.25, 2000, false);
        // Misc::cdrift(0,-15);
        // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
    void left43mid(){

        chassis.setPose(-49.5, 14.1, 0);
        Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(40,40,500);
        // pros::delay(100);

        chassis.moveToPointPro(-49, 44, 1500,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        Piston::loader.set_value(true);

        chassis.turnPointPlus(-65, 47, 2, 0.0, 1, 1500, false);

        Misc::cdrift(55,55,110);
        Misc::cdrift(40,40,720);

        chassis.moveToPointPro(-30, 47, 1500,{.forwards=false,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(21.5);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,400);
        Intake::setState(Intake::State::LOCK);

        chassis.movePosePlus(-36, 40, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0.35, 1.0, 1500, false);

        chassis.movePosePlus(-15, 39, 270, 0, 0, 0.0, -0.85, 1000, -1, true, false, false); // 15 wing
        chassis.waitUntilDone();
        Piston::hook.set_value(false);

        // chassis.turnHeadingPlus(13, 2, 0, 1.0, 1500, false);
        // chassis.turnPointPlus(0, 56, 2, 0.0, 1.0, 1500, false);
        // chassis.turnHeadingPlus(315, 2, 0, 1.0, 1500, false);
        // chassis.waitUntilDone();
        
        chassis.turnHeadingPlus(305, 1, 0.0, 1.0, 1500, false);
        chassis.turnHeadingPlus(19, 2, 0.0, 1.0, 1500, false);
        Misc::cdrift(50,50,300);
        Piston::loader.set_value(true);
        pros::delay(100);
        chassis.turnHeadingPlus(340, 2, 0.0, 1.0, 1500, false);
        pros::delay(100);
        chassis.turnHeadingPlus(0, 2, 0.0, 1.0, 1500, false);

        // chassis.turnHeadingPlus(0, 2, 0.35, 1.0, 1500, false);
        
        Misc::cdrift(-35,-35,250);
        Piston::loader.set_value(false);
        chassis.turnPointPlus(-27,21,2,0.0,1,1000,false);

        chassis.moveToPointPro(-27,21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(7);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();

        chassis.moveToPointPro(-8.5,6,900,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        Piston::lift.set_value(true);
        Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-10,-10,800);
        Misc::cdrift(40,40,500);
        Piston::lift.set_value(false);
        Piston::descore.set_value(true);
        Intake::setState(Intake::State::LOCK);
        Misc::cdrift(-35,-35,650);
        Misc::cdrift(-10,-10,3000);
    }
    void right43mid(){

        chassis.setPose(-49.5, -14.1, 180);
        Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(40,40,500);
        // pros::delay(100);

        chassis.moveToPointPro(-49, -43, 1500,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        Piston::loader.set_value(true);

        // chassis.turnPointPlus(-65, -46, 2, 0.0, 1, 1500, false);

        chassis.turnHeadingPlus(270, 2, 0.0, 1.0, 900, false);

        Misc::cdrift(55,55,110);
        Misc::cdrift(40,40,720);

        chassis.moveToPointPro(-30, -46, 1500,{.forwards=false,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(21.5);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,400);
        Intake::setState(Intake::State::LOCKBOTTOM);
        Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        Misc::cdrift(40,40,300);

        chassis.moveToPointPro(-24, -25, 1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(28);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        chassis.moveToPointPro(-8.5,-8,900,{.forwards=true,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(1);
        Piston::loader.set_value(false);
        chassis.waitUntilDone();
        chassis.turnHeadingPlus(45, 2, 0, 1.0, 300, false);
        // chassis.turnPointPlus(0, 0, 2, 0.0, 1, 500, false);
        
        // Piston::low.set_value(true);
        // chassis.movePosePlus(-12, -12, 45, 0, 0, 0.0, 0.35, 1500, -1, true, false, false);
        // chassis.moveDistPlus(15, 0.4, 1500, false);
        Intake::setState(Intake::State::SPIT);
        Misc::cdrift(50,50,850);
       

        Misc::cdrift(-45,-45,200);
        Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Intake::setState(Intake::State::LOCK);

        chassis.movePosePlus(-34, -32, 45, 1, 0, 0.35, -0.8, 1300, -1, false, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(90, 2, 0.35, 1.0, 1500, false);

        chassis.movePosePlus(-14.5, -33.5, 90, 0, 0, 0.0, 0.85, 1000, -1, true, false, false); // 15 wing
        chassis.turnHeadingPlus(50, 2, 0.0, 0.25, 2000, false);
        Misc::cdrift(0,15);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

        // chassis.movePosePlus(-34, -53, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        // Piston::hook.set_value(true);
        // chassis.turnHeadingPlus(0, 2, 0.35, 1.0, 1500, false);

        // chassis.movePosePlus(-16, -53.5, 270, 0, 0, 0.0, 0.85, 1000, -1, true, false, false); // 15 wing
        // chassis.turnHeadingPlus(50, 2, 0.0, 0.25, 2000, false);
        // Misc::cdrift(0,15);
        // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
    void halfAWP(){
        chassis.setPose(-49.5, 14.1, 0);
        Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(40,40,500);
        // pros::delay(100);

        chassis.moveToPointPro(-49, 44, 1500,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        Piston::loader.set_value(true);

        chassis.turnPointPlus(-65, 47, 2, 0.0, 1, 1500, false);

        Misc::cdrift(55,55,110);
        Misc::cdrift(40,40,780);

        chassis.moveToPointPro(-30, 47, 1500,{.forwards=false,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(21.5);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,300);
        Intake::setState(Intake::State::LOCK);

        chassis.movePosePlus(-36, 40, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0.35, 1.0, 1500, false);

        chassis.movePosePlus(-15, 39, 270, 0, 0, 0.0, -0.85, 1000, -1, true, false, false); // 15 wing
        chassis.waitUntilDone();
        Piston::hook.set_value(false);

        // chassis.turnHeadingPlus(13, 2, 0, 1.0, 1500, false);
        // chassis.turnPointPlus(0, 56, 2, 0.0, 1.0, 1500, false);
        // chassis.turnHeadingPlus(315, 2, 0, 1.0, 1500, false);
        // chassis.waitUntilDone();
        
        chassis.turnHeadingPlus(305, 1, 0.0, 1.0, 1500, false);
        chassis.turnHeadingPlus(20, 2, 0.0, 1.0, 1500, false);
        Misc::cdrift(50,50,300);
        Piston::loader.set_value(true);
        pros::delay(150);
        chassis.turnHeadingPlus(340, 2, 0.0, 1.0, 1500, false);
        // pros::delay(100);
        chassis.turnHeadingPlus(0, 2, 0.0, 1.0, 1500, false);
        // pros::delay(100);

        // chassis.turnHeadingPlus(0, 2, 0.35, 1.0, 1500, false);
        
        Misc::cdrift(-35,-35,250);
        Piston::loader.set_value(false);
        chassis.turnPointPlus(-27,21,2,0.0,1,500,false);

        chassis.moveToPointPro(-27,21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(7);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();

        chassis.moveToPointPro(-8.5,6,900,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(7);
        Motor::intake1.move(45);
        Piston::lift.set_value(true);
        chassis.waitUntilDone();
        Piston::loader.set_value(false);
        // pros::delay(100);
        // Intake::setState(Intake::State::SCORE);
        Motor::intake1.move(85);
        Misc::cdrift(-10,-10,1000);
        Intake::setState(Intake::State::LOCKBOTTOM);
        Piston::lift.set_value(false);
        // Misc::cdrift(40,40,500);


        chassis.moveToPointPro(-25.5, -22, 1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(28);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        chassis.moveToPointPro(-8.5,-8,900,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(4);
        Piston::loader.set_value(false);
        chassis.waitUntilDone();
        chassis.turnHeadingPlus(45, 2, 0, 1.0, 300, false);
        // chassis.turnPointPlus(0, 0, 2, 0.0, 1, 500, false);
        
        // Piston::low.set_value(true);
        // chassis.movePosePlus(-12, -12, 45, 0, 0, 0.0, 0.35, 1500, -1, true, false, false);
        // chassis.moveDistPlus(15, 0.4, 1500, false);
        Intake::setState(Intake::State::SPIT);
        Misc::cdrift(50,50,250);
        
        Misc::cdrift(15,15,1500);
        // Intake::setState(Intake::State::LG2);
        // Misc::cdrift(15,15,1700);
        // Misc::cdrift(-40,-40,400);
        // Intake::setState(Intake::State::LOCK);
        // Piston::low.set_value(false);

        // chassis.moveToPointPro(-23,-21,900,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        // chassis.waitUntilDone();
        // chassis.turnHeadingPlus(45, 2, 0.35, 1.0, 1500, false);
        // Misc::cdrift(40,40,500);
        

        // Piston::lift.set_value(false);
        // Piston::descore.set_value(true);
        // Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(-35,-35,650);
        // Misc::cdrift(-10,-10,3000);
    }
    void left45(){
        chassis.setPose(-49.5, 14.1, 0);
        Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(40,40,500);
        // pros::delay(100);

        chassis.moveToPointPro(-49, 44, 1500,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        Piston::loader.set_value(true);

        // chassis.turnPointPlus(-65, 47, 2, 0.0, 1, 1500, false);

        chassis.turnHeadingPlus(270, 2, 0, 1.0, 900, false);

        Misc::cdrift(55,55,110);
        Misc::cdrift(40,40,720);

        chassis.moveToPointPro(-30, 47, 1500,{.forwards=false,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(21.5);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,300);
        Intake::setState(Intake::State::LOCK);

        chassis.movePosePlus(-36, 40, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0.35, 1.0, 1500, false);

        chassis.movePosePlus(-15, 39, 270, 0, 0, 0.0, -0.85, 1000, -1, true, false, false); // 15 wing
        chassis.waitUntilDone();
        Piston::hook.set_value(false);

        // chassis.turnHeadingPlus(13, 2, 0, 1.0, 1500, false);
        // chassis.turnPointPlus(0, 56, 2, 0.0, 1.0, 1500, false);
        // chassis.turnHeadingPlus(315, 2, 0, 1.0, 1500, false);
        // chassis.waitUntilDone();
        
        chassis.turnHeadingPlus(305, 1, 0.0, 1.0, 1500, false);
        chassis.turnHeadingPlus(20, 2, 0.0, 1.0, 1500, false);
        Misc::cdrift(50,50,300);
        Piston::loader.set_value(true);
        pros::delay(150);
        chassis.turnHeadingPlus(340, 2, 0.0, 1.0, 1500, false);
        // pros::delay(100);
        chassis.turnHeadingPlus(0, 2, 0.0, 1.0, 1500, false);
        // pros::delay(100);

        // chassis.turnHeadingPlus(0, 2, 0.35, 1.0, 1500, false);
        
        Misc::cdrift(-35,-35,250);
        Piston::loader.set_value(false);
        chassis.turnPointPlus(-27,21,2,0.0,1,500,false);

        chassis.moveToPointPro(-27,21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(7);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();

        chassis.moveToPointPro(-8.5,6,900,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(7);
        Motor::intake1.move(45);
        Piston::lift.set_value(true);
        chassis.waitUntilDone();
        Piston::loader.set_value(false);
        // pros::delay(100);
        Intake::setState(Intake::State::SCORE);
        Motor::intake1.move(85);
        Misc::cdrift(-10,-10,1000);
        Intake::setState(Intake::State::LOCK);
        Piston::lift.set_value(false);
        // Misc::cdrift(40,40,500);

        Misc::cdrift(40,40,500);
        Piston::lift.set_value(false);
        Piston::descore.set_value(true);
        Intake::setState(Intake::State::LOCK);
        Misc::cdrift(-35,-35,650);
        Misc::cdrift(-10,-10,3000);

        // chassis.moveToPointPro(-25, -18.5, 1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        // chassis.waitUntil(28);
        // Piston::loader.set_value(true);
        // chassis.waitUntilDone();
        // chassis.moveToPointPro(-8.5,-6,900,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        // chassis.waitUntil(4);
        // Piston::loader.set_value(false);
        // chassis.waitUntilDone();
        // chassis.turnHeadingPlus(45, 2, 0, 1.0, 300, false);
        // // chassis.turnPointPlus(0, 0, 2, 0.0, 1, 500, false);
        
        // // Piston::low.set_value(true);
        // // chassis.movePosePlus(-12, -12, 45, 0, 0, 0.0, 0.35, 1500, -1, true, false, false);
        // // chassis.moveDistPlus(15, 0.4, 1500, false);
        // Intake::setState(Intake::State::SPIT);
        // Misc::cdrift(50,50,250);
        
        // Misc::cdrift(15,15,1500);
        // // Intake::setState(Intake::State::LG2);
        // // Misc::cdrift(15,15,1700);
        // // Misc::cdrift(-40,-40,400);
        // // Intake::setState(Intake::State::LOCK);
        // // Piston::low.set_value(false);

        // // chassis.moveToPointPro(-23,-21,900,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        // // chassis.waitUntilDone();
        // // chassis.turnHeadingPlus(45, 2, 0.35, 1.0, 1500, false);
        // // Misc::cdrift(40,40,500);
        

        // // Piston::lift.set_value(false);
        // // Piston::descore.set_value(true);
        // // Intake::setState(Intake::State::LOCK);
        // // Misc::cdrift(-35,-35,650);
        // // Misc::cdrift(-10,-10,3000);
    }

    void left43(){
        chassis.setPose(-49.5, 14.1, 0);

        // chassis.moveToPointPro(53,46.2,1200,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1.5});
        // // chassis.moveToPoint(53,45.3,1200,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1.5});
        // chassis.waitUntil(15);
        // Piston::loader.set_value(true);
        // chassis.waitUntilDone();
        // Misc::cdrift(10,-10,50);
        // // chassis.turnToHeading(270,1200,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=1});
        // // chassis.waitUntilDone();
        // // Piston::loader.set_value(true);
        // // pros::delay(100);
        // chassis.turnPointPlus(65,48,2,0.0,1,800,false);
        // // chassis.turnToPoint(65,48,800,{.forwards=true,.maxSpeed=90,.minSpeed=5,.earlyExitRange=1.5});
        // // chassis.waitUntilDone();
        // Misc::cdrift(45,45,110);
        
        // Misc::cdrift(35,35,900);

        Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(40,40,500);
        // pros::delay(100);

        chassis.moveToPointPro(-49, 44.5, 1200,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=4});
        // chassis.waitUntilDone();
        // Piston::loader.set_value(true);
        
        chassis.waitUntil(15);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        Misc::cdrift(10,-10,50);

        chassis.turnPointPlus(-65, 47.5, 2, 0.0, 1, 900, false);

        // chassis.turnHeadingPlus(270, 2, 0, 1.0, 900, false);

        // Misc::cdrift(55,55,110);
        // Misc::cdrift(40,40,780);
        Misc::cdrift(45,45,110);
        
        Misc::cdrift(35,35,900);

        chassis.moveToPointPro(-30, 47, 1500,{.forwards=false,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(21.5);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);

        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,400);
        Intake::setState(Intake::State::LOCK);

        chassis.movePosePlus(-36, 40, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0.35, 1.0, 1500, false);

        chassis.movePosePlus(-15, 39, 270, 0, 0, 0.0, -0.95, 1000, -1, true, false, false); // -0.85
        chassis.waitUntilDone();
        Piston::hook.set_value(false);

        // chassis.turnHeadingPlus(13, 2, 0, 1.0, 1500, false);
        // chassis.turnPointPlus(0, 56, 2, 0.0, 1.0, 1500, false);
        // chassis.turnHeadingPlus(315, 2, 0, 1.0, 1500, false);
        // chassis.waitUntilDone();
        
        // chassis.turnHeadingPlus(305, 1, 0.0, 1.0, 1500, false);
        // chassis.turnHeadingPlus(20, 2, 0.0, 1.0, 1500, false);
        // Misc::cdrift(50,50,300);
        // Piston::loader.set_value(true);
        // pros::delay(150);
        // chassis.turnHeadingPlus(340, 2, 0.0, 1.0, 1500, false);
        // // pros::delay(100);
        // chassis.turnHeadingPlus(0, 2, 0.0, 1.0, 1500, false);
        // // pros::delay(100);

        // // chassis.turnHeadingPlus(0, 2, 0.35, 1.0, 1500, false);
        
        // Misc::cdrift(-35,-35,250);
        // Piston::loader.set_value(false);
        // chassis.turnPointPlus(-27,21,2,0.0,1,500,false);

        chassis.moveToPointPro(-27,21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(4);
        Piston::loader.set_value(true);
        Motor::intake1.move(45);
        Piston::lift.set_value(true);
        chassis.waitUntilDone();

        chassis.moveToPointPro(-9,6.5,900,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        // Piston::lift.set_value(true);
        Piston::loader.set_value(false);
        Motor::intake1.move(90);
        // pros::delay(100);
        // Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-10,-10,1000);
        // Intake::setState(Intake::State::LOCKBOTTOM);
        // Piston::lift.set_value(false);
        // Intake::setState(Intake::State::LOCK);
        // Piston::lift.set_value(false);
        // Misc::cdrift(40,40,500);

        Misc::cdrift(40,40,500);
        Piston::lift.set_value(false);
        Piston::descore.set_value(true);
        Intake::setState(Intake::State::LOCK);
        Misc::cdrift(-35,-35,650);
        Misc::cdrift(-10,-10,3000);
    }

    void left43wingnew(){
        chassis.setPose(-49.5, 14.1, 0);
        Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(40,40,500);
        // pros::delay(100);

        // chassis.moveToPointPro(-49, 44.5, 1200,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=4});
        chassis.moveToPointPro(-49, 43, 1200,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=2});
        // chassis.waitUntilDone();
        // Piston::loader.set_value(true);
        
        chassis.waitUntil(15);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        Misc::cdrift(10,-10,50);

        chassis.turnPointPlus(-65, 48.25, 2, 0.04, 1, 800, false);

        // chassis.turnHeadingPlus(270, 2, 0, 1.0, 900, false);

        // Misc::cdrift(55,55,110);
        // Misc::cdrift(40,40,780);
        Misc::cdrift(45,45,110);
        
        Misc::cdrift(35,35,900);

        chassis.moveToPointPro(-30, 47, 1500,{.forwards=false,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(21.5);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,300);
        Intake::setState(Intake::State::LOCK);

        chassis.movePosePlus(-36, 40, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0.35, 1.0, 1500, false);

        chassis.movePosePlus(-14, 39, 270, 0, 0, 0.0, -0.95, 1000, -1, true, false, false); // -0.85
        chassis.waitUntilDone();
        Misc::cbrake(false);
        pros::delay(1500);
        Misc::cbrake(true);
        Piston::hook.set_value(false);

        // chassis.turnHeadingPlus(13, 2, 0, 1.0, 1500, false);
        // chassis.turnPointPlus(0, 56, 2, 0.0, 1.0, 1500, false);
        // chassis.turnHeadingPlus(315, 2, 0, 1.0, 1500, false);
        // chassis.waitUntilDone();
        
        // chassis.turnHeadingPlus(305, 1, 0.0, 1.0, 1500, false);
        // chassis.turnHeadingPlus(20, 2, 0.0, 1.0, 1500, false);
        // Misc::cdrift(50,50,300);
        // Piston::loader.set_value(true);
        // pros::delay(150);
        // chassis.turnHeadingPlus(340, 2, 0.0, 1.0, 1500, false);
        // // pros::delay(100);
        // chassis.turnHeadingPlus(0, 2, 0.0, 1.0, 1500, false);
        // // pros::delay(100);

        // // chassis.turnHeadingPlus(0, 2, 0.35, 1.0, 1500, false);
        
        // Misc::cdrift(-35,-35,250);
        // Piston::loader.set_value(false);
        // chassis.turnPointPlus(-27,21,2,0.0,1,500,false);

        chassis.moveToPointPro(-27,21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(4);
        Piston::loader.set_value(true);
        Motor::intake1.move(45);
        Piston::lift.set_value(true);
        chassis.waitUntilDone();

        chassis.moveToPointPro(-9,7.5,900,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        // Piston::lift.set_value(true);
        Piston::loader.set_value(false);
        Motor::intake1.move(127);
        // pros::delay(100);
        // Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-10,-10,900);
        
        Misc::cdrift(40,40,500);
        Piston::lift.set_value(false);
        Piston::descore.set_value(true);
        Intake::setState(Intake::State::LOCK);
        Misc::cdrift(-35,-35,650);

        chassis.movePosePlus(-34, 35, 310, 0, 0, 0.0, 0.8, 1300, -1, true, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0.0, 1.0, 900, false);

        chassis.movePosePlus(-11, 36.5, 270, 0, 0, 0.0, -1, 1500, -1, true, false, false); // 15 wing
        // chassis.movePosePlus(-11, 39.5, 270, 0, 0, 0.0, -0.45, 1500, -1, true, false, false); // 15 wing
        // chassis.turnHeadingPlus(285, 2, 0.0, 0.25, 2000, false);
        Misc::cdrift(0,-15);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

        // Misc::cdrift(-10,-10,3000);
    }

    void left43c7(){
        chassis.setPose(-49.5, 14.1, 0);
        Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(40,40,500);
        // pros::delay(100);

        chassis.moveToPointPro(-49, 44, 1200,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=4});
        chassis.waitUntilDone();
        Piston::loader.set_value(true);

        chassis.turnPointPlus(-65, 47, 2, 0.0, 1, 900, false);

        // chassis.turnHeadingPlus(270, 2, 0, 1.0, 900, false);

        Misc::cdrift(55,55,110);
        Misc::cdrift(40,40,780);

        chassis.moveToPointPro(-30, 47, 1500,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(21.5);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,400);
        Intake::setState(Intake::State::LOCK);

        chassis.movePosePlus(-36, 40, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0.35, 1.0, 1500, false);

        chassis.movePosePlus(-15, 39, 270, 0, 0, 0.0, -1.0, 1000, -1, true, false, false); // 15 wing
        chassis.waitUntilDone();
        pros::delay(2000);
        Piston::hook.set_value(false);

        // chassis.turnHeadingPlus(13, 2, 0, 1.0, 1500, false);
        // chassis.turnPointPlus(0, 56, 2, 0.0, 1.0, 1500, false);
        // chassis.turnHeadingPlus(315, 2, 0, 1.0, 1500, false);
        // chassis.waitUntilDone();
        
        // chassis.turnHeadingPlus(305, 1, 0.0, 1.0, 1500, false);
        // chassis.turnHeadingPlus(20, 2, 0.0, 1.0, 1500, false);
        // Misc::cdrift(50,50,300);
        // Piston::loader.set_value(true);
        // pros::delay(150);
        // chassis.turnHeadingPlus(340, 2, 0.0, 1.0, 1500, false);
        // // pros::delay(100);
        // chassis.turnHeadingPlus(0, 2, 0.0, 1.0, 1500, false);
        // // pros::delay(100);

        // // chassis.turnHeadingPlus(0, 2, 0.35, 1.0, 1500, false);
        
        // Misc::cdrift(-35,-35,250);
        // Piston::loader.set_value(false);
        // chassis.turnPointPlus(-27,21,2,0.0,1,500,false);

        chassis.moveToPointPro(-27,21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(4);
        Piston::loader.set_value(true);
        Motor::intake1.move(45);
        Piston::lift.set_value(true);
        chassis.waitUntilDone();

        chassis.moveToPointPro(-8.5,6,900,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        // Piston::lift.set_value(true);
        Piston::loader.set_value(false);
        Motor::intake1.move(127);
        // pros::delay(100);
        // Intake::setState(Intake::State::SCORE);
        // Misc::cdrift(-10,-10,900);

        Misc::cdrift(-10,-10,1000);
        // Intake::setState(Intake::State::LOCK);
        // Piston::lift.set_value(false);
        // Misc::cdrift(40,40,500);

        // Misc::cdrift(40,40,500);
        // Piston::lift.set_value(false);
        // Piston::descore.set_value(true);
        // Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(-35,-35,650);
        // Misc::cdrift(-10,-10,3000);


        chassis.movePosePlus(-36, 33, 310, 0, 0, 0.0, 0.8, 1300, -1, true, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0.0, 1.0, 1500, false);

        chassis.movePosePlus(-11, 33, 270, 0, 0, 0.0, -1, 1500, -1, true, false, false); // 15 wing
        // chassis.movePosePlus(-11, 39.5, 270, 0, 0, 0.0, -0.45, 1500, -1, true, false, false); // 15 wing
        // chassis.turnHeadingPlus(285, 2, 0.0, 0.25, 2000, false);
        Misc::cdrift(0,-15);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    }
    void left433(){
        chassis.setPose(-49.5, 14.1, 0);
        Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(40,40,500);
        // pros::delay(100);

        // chassis.moveToPointPro(-49, 44.5, 1200,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=4});
        chassis.moveToPointPro(-49, 43.5, 1200,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=2});
        // chassis.waitUntilDone();
        // Piston::loader.set_value(true);
        
        chassis.waitUntil(15);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        Misc::cdrift(10,-10,50);

        chassis.turnPointPlus(-65, 48.75, 2, 0.04, 1, 800, false);

        // chassis.turnHeadingPlus(270, 2, 0, 1.0, 900, false);

        // Misc::cdrift(55,55,110);
        // Misc::cdrift(40,40,780);
        Misc::cdrift(45,45,110);
        
        Misc::cdrift(35,35,900);

        chassis.moveToPointPro(-29, 47, 1500,{.forwards=false,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(23.5);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,400);
        Intake::setState(Intake::State::LOCK);

        chassis.movePosePlus(-36, 40, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0.35, 1.0, 1500, false);

        chassis.movePosePlus(-14, 39, 270, 0, 0, 0.0, -0.95, 1000, -1, true, false, false); // -0.85
        chassis.waitUntilDone();
        Misc::cbrake(false);
        pros::delay(1500);
        Misc::cbrake(true);
        Piston::hook.set_value(false);

        // chassis.turnHeadingPlus(13, 2, 0, 1.0, 1500, false);
        // chassis.turnPointPlus(0, 56, 2, 0.0, 1.0, 1500, false);
        // chassis.turnHeadingPlus(315, 2, 0, 1.0, 1500, false);
        // chassis.waitUntilDone();
        
        // chassis.turnHeadingPlus(305, 1, 0.0, 1.0, 1500, false);
        // chassis.turnHeadingPlus(20, 2, 0.0, 1.0, 1500, false);
        // Misc::cdrift(50,50,300);
        // Piston::loader.set_value(true);
        // pros::delay(150);
        // chassis.turnHeadingPlus(340, 2, 0.0, 1.0, 1500, false);
        // // pros::delay(100);
        // chassis.turnHeadingPlus(0, 2, 0.0, 1.0, 1500, false);
        // // pros::delay(100);

        // // chassis.turnHeadingPlus(0, 2, 0.35, 1.0, 1500, false);
        
        // Misc::cdrift(-35,-35,250);
        // Piston::loader.set_value(false);
        // chassis.turnPointPlus(-27,21,2,0.0,1,500,false);

        chassis.moveToPointPro(-27,21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(4);
        Piston::loader.set_value(true);
        Motor::intake1.move(45);
        Piston::lift.set_value(true);
        chassis.waitUntilDone();

        chassis.moveToPointPro(-9,7.5,900,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        // Piston::lift.set_value(true);
        Piston::loader.set_value(false);
        Motor::intake1.move(127);
        // pros::delay(100);
        // Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-10,-10,900);
        Intake::setState(Intake::State::LOCKBOTTOM);
        Piston::lift.set_value(false);
        // Misc::cdrift(40,40,500);


        chassis.moveToPointPro(-25, -23, 1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(27);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        // Misc::cdrift(-50,-50,200);
        Piston::loader.set_value(false);
        pros::delay(200);
        chassis.turnHeadingPlus(45, 2, 0.04, 1.0, 700, false);
        // Piston::loader.set_value(false);

        chassis.moveToPointPro(-8.5,-7.5,1200,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
        // chassis.waitUntil(4);
        // Piston::loader.set_value(false);
        chassis.waitUntilDone();
        // chassis.turnHeadingPlus(45, 2, 0, 1.0, 300, false);
        // chassis.turnPointPlus(0, 0, 2, 0.0, 1, 500, false);
        
        // Piston::low.set_value(true);
        // chassis.movePosePlus(-12, -12, 45, 0, 0, 0.0, 0.35, 1500, -1, true, false, false);
        // chassis.moveDistPlus(15, 0.4, 1500, false);
        Intake::setState(Intake::State::SPIT);
        Motor::intake2.move(-90);
        Misc::cdrift(50,50,250);
        
        Misc::cdrift(15,15,1500);
        // Intake::setState(Intake::State::LG2);
        // Misc::cdrift(15,15,1700);
        // Misc::cdrift(-40,-40,400);
        // Intake::setState(Intake::State::LOCK);
        // Piston::low.set_value(false);

        // chassis.moveToPointPro(-23,-21,900,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        // chassis.waitUntilDone();
        // chassis.turnHeadingPlus(45, 2, 0.35, 1.0, 1500, false);
        // Misc::cdrift(40,40,500);
        

        // Piston::lift.set_value(false);
        // Piston::descore.set_value(true);
        // Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(-35,-35,650);
        // Misc::cdrift(-10,-10,3000);
    }
    void right433(){

        chassis.setPose(-49.5, -14.1, 180);
        Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(40,40,500);
        // pros::delay(100);

        chassis.moveToPointPro(-49, -43, 1500,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        Piston::loader.set_value(true);

        chassis.turnPointPlus(-65, -46, 2, 0.0, 1, 1500, false);

        Misc::cdrift(55,55,110);
        Misc::cdrift(40,40,720);

        chassis.moveToPointPro(-30, -46, 1500,{.forwards=false,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(21.5);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,400);
        Intake::setState(Intake::State::LOCKBOTTOM);
        Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        Misc::cdrift(40,40,300);

        chassis.moveToPointPro(-24, -25, 1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(28);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        chassis.moveToPointPro(-8.5,-8,900,{.forwards=true,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(1);
        Piston::loader.set_value(false);
        chassis.waitUntilDone();
        chassis.turnHeadingPlus(45, 2, 0, 1.0, 300, false);
        // chassis.turnPointPlus(0, 0, 2, 0.0, 1, 500, false);
        
        // Piston::low.set_value(true);
        // chassis.movePosePlus(-12, -12, 45, 0, 0, 0.0, 0.35, 1500, -1, true, false, false);
        // chassis.moveDistPlus(15, 0.4, 1500, false);
        Intake::setState(Intake::State::SPIT);
        Misc::cdrift(50,50,850);
       

        Misc::cdrift(-45,-45,200);
        Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Intake::setState(Intake::State::LOCK);

        chassis.movePosePlus(-34, -32, 45, 1, 0, 0.35, -0.8, 1300, -1, false, false, false);
        Piston::hook.set_value(true);
        chassis.turnHeadingPlus(90, 2, 0.35, 1.0, 1500, false);

        chassis.movePosePlus(-16, -34, 90, 0, 0, 0.0, 0.85, 1000, -1, true, false, false); // 15 wing
        Piston::hook.set_value(false);

        chassis.moveToPointPro(-24,-24,1500,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        chassis.turnHeadingPlus(0, 2, 0.35, 1.0, 1500, false);

        chassis.moveToPointPro(-27,21,1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(27);
        Piston::loader.set_value(true);
        Motor::intake1.move(45);
        Piston::lift.set_value(true);
        chassis.waitUntilDone();

        chassis.moveToPointPro(-8.5,6,900,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        // Piston::lift.set_value(true);
        Piston::loader.set_value(false);
        Motor::intake1.move(127);
        // pros::delay(100);
        // Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-10,-10,900);

        Misc::cdrift(40,40,500);
        Piston::lift.set_value(false);
        Piston::descore.set_value(true);
        Intake::setState(Intake::State::LOCK);
        Misc::cdrift(-35,-35,650);
        Misc::cdrift(-10,-10,3000);



        // chassis.turnHeadingPlus(50, 2, 0.0, 0.25, 2000, false);
        // Misc::cdrift(0,15);
        // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

        // chassis.movePosePlus(-34, -53, 225, 1, 0, 0.35, 0.8, 1300, -1, false, false, false);
        // Piston::hook.set_value(true);
        // chassis.turnHeadingPlus(0, 2, 0.35, 1.0, 1500, false);

        // chassis.movePosePlus(-16, -53.5, 270, 0, 0, 0.0, 0.85, 1000, -1, true, false, false); // 15 wing
        // chassis.turnHeadingPlus(50, 2, 0.0, 0.25, 2000, false);
        // Misc::cdrift(0,15);
        // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }
    void soloSafe(){          
        chassis.setPose(-47.1, -4.1, 0);
        Intake::setState(Intake::State::LOCK);
        Misc::cdrift(40,40,500);
        pros::delay(100);

        chassis.movePosePlus(-49, -42.5, 0, 0, 0, 0.0, -0.8, 1500, -1, true, false, false);

        leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        leftMotors.brake(); 
        rightMotors.brake();

        // pros::delay(200);
        Piston::loader.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0, 1.0, 1500, false);
        leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        Misc::cdrift(55,55,110);
        Misc::cdrift(40,40,670);
        chassis.movePosePlus(-31, -45, 270, 0, 0, 0.0, -1, 1500, -1, true, false, false);

        Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-20,-20,450);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,300);
        Intake::setState(Intake::State::LOCK);
        Misc::cdrift(30,30,100);
        chassis.turnHeadingPlus(5, 2, 0, 1.0, 1500, false);
        chassis.movePosePlus(-24, -25, 0, 2, 0, 0.75, 1, 1500, -1, true, false, false);

        chassis.movePosePlus(-24, 21, 355, 2, 0, 0, 0.9, 1500, -1, true, false, false);

        chassis.movePosePlus(-44, 47, 310, 2, 0, 0.0, 1, 1500, -1, true, false, false);

        chassis.movePosePlus(-31, 48, 270, 0, 0, 0.0, -1, 1000, -1, true, false, false);
        Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-20,-20,600);
        Piston::loader.set_value(true);
        Misc::cdrift(-20,-20,600);
        Intake::setState(Intake::State::LOCK);
        chassis.movePosePlus(-53, 46, 270, 0, 0, 0.0, 0.8, 1500, -1, true, false, false);
        Misc::cdrift(40,40,700);
        chassis.movePosePlus(-13, 10, 315, 2, 0, 0.0, -0.8, 1500, -1, true, false, false);
        pros::delay(100);
        Piston::lift.set_value(true);
        Intake::setState(Intake::State::SCORE);
        pros::delay(500);
        chassis.moveDistPlus(5, 0.55, 1500, true);
        pros::delay(100);
        // Misc::cdrift(55,55,300);
        // pros::delay(100);
        Piston::lift.set_value(false);
        Piston::descore.set_value(true);
        // pros::delay(100);
        chassis.moveDistPlus(-8, 0.6, 1500, false);
        // Misc::cdrift(-40,-40,700);
    }
    void soloNoPush(){          
        chassis.setPose(-49.5, -14.1, 180);
        Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(40,40,500);
        // pros::delay(100);

        chassis.movePosePlus(-49, -45.5, 180, 0, 0, 0.0, 0.7, 1500, -1, true, false, false);
        Piston::loader.set_value(true);

        leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        leftMotors.brake(); 
        rightMotors.brake();

        pros::delay(100);
        // Piston::loader.set_value(true);
        chassis.turnHeadingPlus(270, 2, 0, 1.0, 1500, false);
        leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        Misc::cdrift(55,55,110);
        Misc::cdrift(40,40,670);
        chassis.movePosePlus(-31, -45, 270, 0, 0, 0.0, -1, 1500, -1, true, false, false);

        Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-20,-20,450);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,300);
        Intake::setState(Intake::State::LOCK);
        Misc::cdrift(30,30,50);
        chassis.turnHeadingPlus(5, 2, 0, 1.0, 1500, false);
        chassis.movePosePlus(-24, -25, 0, 2, 0, 0.75, 1, 1500, -1, true, false, true);
        pros::delay(100);
        Piston::loader.set_value(true);
        pros::delay(400);
        Piston::loader.set_value(false);

        chassis.movePosePlus(-24, 21, 355, 2, 0, 0, 0.9, 1500, -1, true, false, false);
        Piston::loader.set_value(true);
        

        chassis.movePosePlus(-44, 47, 310, 2, 0, 0.0, 1, 1500, -1, true, false, false);

        chassis.movePosePlus(-31, 48, 270, 0, 0, 0.0, -1, 1000, -1, true, false, false);
        Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-20,-20,600);
        // Piston::loader.set_value(true);
        Misc::cdrift(-20,-20,600);
        Intake::setState(Intake::State::LOCK);
        chassis.movePosePlus(-53, 46, 270, 0, 0, 0.0, 0.8, 1500, -1, true, false, false);
        Misc::cdrift(40,40,700);
        chassis.movePosePlus(-12, 10, 315, 2, 0, 0.0, -0.8, 1500, -1, true, false, false);
        pros::delay(100);
        Piston::lift.set_value(true);
        Intake::setState(Intake::State::SCORE);
        pros::delay(500);
        // chassis.moveDistPlus(6, 0.7, 1500, true);
        // pros::delay(100);
        Misc::cdrift(60,60,200);
        // pros::delay(100);
        Piston::lift.set_value(false);
        Piston::descore.set_value(true);
        Misc::cdrift(30,30,200);
        // pros::delay(100);
        Misc::cdrift(-60,-60,600);
        // chassis.moveDistPlus(-8, 0.65, 1500, false);
        // Misc::cdrift(-40,-40,700);
    }
    void solo4g(){

        chassis.setPose(-49.5, -14.1, 180);
        Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(40,40,500);
        // pros::delay(100);

        chassis.moveToPointPro(-49, -43, 1500,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        Piston::loader.set_value(true);

        chassis.turnPointPlus(-65, -46, 2, 0.0, 1, 1500, false);

        Misc::cdrift(55,55,110);
        Misc::cdrift(40,40,770);

        chassis.moveToPointPro(-29, -46, 1500,{.forwards=false,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(21);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,400);
        Intake::setState(Intake::State::LOCKBOTTOM);
        // Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        Motor::intake1.move(-10);
        chassis.turnHeadingPlus(0, 2, 0, 1.0, 300, false);


        // Misc::cdrift(40,40,300);

        chassis.moveToPointPro(-25, -25, 1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(10);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        chassis.moveToPointPro(-11,-12,800,{.forwards=true,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(1);
        Piston::loader.set_value(false);
        chassis.waitUntilDone();
        chassis.turnHeadingPlus(45, 2, 0, 1.0, 300, false);
        // chassis.turnPointPlus(0, 0, 2, 0.0, 1, 500, false);
        
        // Piston::low.set_value(true);
        // chassis.movePosePlus(-12, -12, 45, 0, 0, 0.0, 0.35, 1500, -1, true, false, false);
        // chassis.moveDistPlus(15, 0.4, 1500, false);
        // Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Intake::setState(Intake::State::SPIT);
        Misc::cdrift(50,50,700);
       

        Misc::cdrift(-65,-60,300);
        Intake::setState(Intake::State::LOCK);

        chassis.moveToPointPro(-21, 19, 1200,{.forwards=true,.maxSpeed=95,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(30);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        chassis.moveToPointPro(-41, 48, 1200,{.forwards=true,.maxSpeed=95,.minSpeed=0,.earlyExitRange=3});
        chassis.moveToPointPro(-24, 46.15, 800,{.forwards=false,.maxSpeed=95,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();


        // chassis.movePosePlus(-24, 21, 355, 2, 0, 0, 0.9, 1500, -1, true, false, false);
        // Piston::loader.set_value(true);

        // chassis.movePosePlus(-40, 47, 310, 2, 0, 0.0, 1, 1500, -1, true, false, false);

        // chassis.movePosePlus(-26, 48, 270, 0, 0, 0.0, -1, 1000, -1, true, false, false);
        Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-20,-20,600);
        // Piston::loader.set_value(true);
        // Misc::cdrift(-20,-20,600); // tm
        
        


        chassis.moveToPointPro(-51,46,1500,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(6);
        Intake::setState(Intake::State::LOCK);
        // Piston::loader.set_value(false);
        chassis.waitUntilDone();

        // chassis.movePosePlus(-53, 46, 270, 0, 0, 0.0, 0.8, 1500, -1, true, false, false);
        Misc::cdrift(40,40,650);
        chassis.moveToPointPro(-5, 5.5, 1200,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(30);
        Piston::loader.set_value(false);
        chassis.waitUntilDone();
        
        Piston::lift.set_value(true);
        Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-10,-10,800);
        // Misc::cdrift(40,40,500);
        // Piston::lift.set_value(false);
        // Piston::descore.set_value(true);
        // Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(-35,-35,650);
        // Misc::cdrift(-10,-10,3000);
    }

    void solo4gnew(){

        chassis.setPose(-49.5, -14.1, 180);
        Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(40,40,500);
        // pros::delay(100);

        chassis.moveToPointPro(-49, -43, 1500,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        Piston::loader.set_value(true);

        chassis.turnPointPlus(-65, -46, 2, 0.0, 1, 1500, false);

        Misc::cdrift(55,55,110);
        Misc::cdrift(40,40,770);

        chassis.moveToPointPro(-29, -46, 1500,{.forwards=false,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(21);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,400);
        Intake::setState(Intake::State::LOCKBOTTOM);
        // Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        Motor::intake1.move(-10);
        chassis.turnHeadingPlus(0, 2, 0, 1.0, 300, false);


        // Misc::cdrift(40,40,300);

        chassis.moveToPointPro(-25, -25, 1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(10);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        chassis.moveToPointPro(-11,-12,800,{.forwards=true,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(1);
        Piston::loader.set_value(false);
        chassis.waitUntilDone();
        chassis.turnHeadingPlus(45, 2, 0, 1.0, 300, false);
        // chassis.turnPointPlus(0, 0, 2, 0.0, 1, 500, false);
        
        // Piston::low.set_value(true);
        // chassis.movePosePlus(-12, -12, 45, 0, 0, 0.0, 0.35, 1500, -1, true, false, false);
        // chassis.moveDistPlus(15, 0.4, 1500, false);
        // Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        Intake::setState(Intake::State::SPIT);
        Misc::cdrift(50,50,700);
       

        Misc::cdrift(-65,-60,300);
        Intake::setState(Intake::State::LOCK);

        chassis.moveToPointPro(-21, 18.5, 1200,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(26);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        chassis.moveToPointPro(-43, 48, 1200,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        Motor::intake1.move(-127);
        chassis.moveToPointPro(-26, 47, 800,{.forwards=false,.maxSpeed=95,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        Misc::cdrift(-50,-50,300);
        Motor::intake1.move(127); 


        // chassis.movePosePlus(-24, 21, 355, 2, 0, 0, 0.9, 1500, -1, true, false, false);
        // Piston::loader.set_value(true);

        // chassis.movePosePlus(-40, 47, 310, 2, 0, 0.0, 1, 1500, -1, true, false, false);

        // chassis.movePosePlus(-26, 48, 270, 0, 0, 0.0, -1, 1000, -1, true, false, false);
        Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-20,-20,600);
        // Piston::loader.set_value(true);
        Misc::cdrift(-20,-20,700);
        
        


        chassis.moveToPointPro(-51,46,1500,{.forwards=true,.maxSpeed=85,.minSpeed=0,.earlyExitRange=3}); // 46.5 
        chassis.waitUntil(6);
        Intake::setState(Intake::State::LOCK);
        // Piston::loader.set_value(false);
        chassis.waitUntilDone();

        // chassis.movePosePlus(-53, 46, 270, 0, 0, 0.0, 0.8, 1500, -1, true, false, false);
        Motor::intake1.move(30);
        Misc::cdrift(40,40,750);
        chassis.moveToPointPro(-8.5, 8, 2200,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=1});
        chassis.waitUntil(30);
        Motor::intake1.move(-40);
        Piston::loader.set_value(false);
        Piston::lift.set_value(true);
        chassis.waitUntilDone();
        Misc::cdrift(-10,-10);
        // Piston::lift.set_value(true);
        // pros::delay(150);
        
        pros::delay(50);

        Intake::setState(Intake::State::SCORE);
        pros::delay(500);
        Motor::intake1.move(100);
        // Misc::cdrift(40,40,500);
        // Piston::lift.set_value(false);
        // Piston::descore.set_value(true);
        // Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(-35,-35,650);
        Misc::cdrift(-10,-10,2500);
    }


    // void solo463(){

    //     chassis.setPose(-49.5, -14.1, 180);
    //     // Piston::hook.set_value(true);
    //     Intake::setState(Intake::State::LOCK);
    //     // Misc::cdrift(40,40,500);
    //     // pros::delay(100);

    //     chassis.moveToPointPro(-49, -43, 1500,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntilDone();
    //     Piston::loader.set_value(true);

    //     chassis.turnPointPlus(-65, -45, 2, 0.0, 1, 800, false);

    //     Misc::cdrift(55,55,110);
    //     Misc::cdrift(40,40,800);

    //     chassis.moveToPointPro(-29, -46, 1500,{.forwards=false,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntil(25);
    //     Intake::setState(Intake::State::SCORE);
    //     chassis.waitUntilDone();
    //     Misc::cdrift(-20,-20,200);
    //     Piston::loader.set_value(false);
    //     Misc::cdrift(-20,-20,400);
    //     Intake::setState(Intake::State::LOCK);
    //     // Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //     // Motor::intake1.move(-10);
    //     chassis.turnHeadingPlus(0, 2, 0, 1.0, 650, false);


    //     // Misc::cdrift(40,40,300);

    //     chassis.moveToPointPro(-24, -23, 1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntil(10);
    //     Piston::loader.set_value(true);
    //     chassis.waitUntilDone();
    //     Piston::loader.set_value(false);
    //     // chassis.moveToPointPro(-11,-10,800,{.forwards=true,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3});
    //     // chassis.waitUntil(1);
    //     // Piston::loader.set_value(false);
    //     // chassis.waitUntilDone();
    //     // chassis.turnHeadingPlus(45, 2, 0, 1.0, 300, false);
    //     // // chassis.turnPointPlus(0, 0, 2, 0.0, 1, 500, false);
        
    //     // // Piston::low.set_value(true);
    //     // // chassis.movePosePlus(-12, -12, 45, 0, 0, 0.0, 0.35, 1500, -1, true, false, false);
    //     // // chassis.moveDistPlus(15, 0.4, 1500, false);
    //     // // Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    //     // Intake::setState(Intake::State::SPIT);
    //     // Misc::cdrift(50,50,700);
       

    //     // Misc::cdrift(-65,-60,300);
    //     // Intake::setState(Intake::State::LOCK);

    //     chassis.moveToPointPro(-21, 18.5, 1200,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntil(25);
    //     Piston::loader.set_value(true);
    //     chassis.waitUntilDone();
    //     chassis.moveToPointPro(-42, 47, 1200,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntilDone();
    //     Motor::intake1.move(-127);
    //     chassis.moveToPointPro(-26, 45, 800,{.forwards=false,.maxSpeed=95,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntilDone();
    //     Misc::cdrift(-50,-50,300);
    //     Motor::intake1.move(127); 


    //     // chassis.movePosePlus(-24, 21, 355, 2, 0, 0, 0.9, 1500, -1, true, false, false);
    //     // Piston::loader.set_value(true);

    //     // chassis.movePosePlus(-40, 47, 310, 2, 0, 0.0, 1, 1500, -1, true, false, false);

    //     // chassis.movePosePlus(-26, 48, 270, 0, 0, 0.0, -1, 1000, -1, true, false, false);
    //     Intake::setState(Intake::State::SCORE);
    //     Misc::cdrift(-20,-20,600);
    //     // Piston::loader.set_value(true);
    //     Misc::cdrift(-20,-20,700);
        
        


    //     chassis.moveToPointPro(-51,45.1,1500,{.forwards=true,.maxSpeed=85,.minSpeed=0,.earlyExitRange=3}); // 46.5 
    //     chassis.waitUntil(6);
    //     Intake::setState(Intake::State::LOCK);
    //     // Piston::loader.set_value(false);
    //     chassis.waitUntilDone();

    //     // chassis.movePosePlus(-53, 46, 270, 0, 0, 0.0, 0.8, 1500, -1, true, false, false);
    //     Motor::intake1.move(-10);
    //     Misc::cdrift(50,50,150);
    //     Misc::cdrift(40,40,650);
    //     chassis.moveToPointPro(-13, 6, 2000,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=2});
    //     // chassis.moveToPointPro(-8.5, 5, 2000,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=1});
    //     chassis.waitUntil(30);
    //     Motor::intake1.move(30);
    //     Piston::loader.set_value(false);
    //     Piston::lift.set_value(true);
    //     chassis.waitUntilDone();
    //     Misc::cdrift(-10,-10);
    //     // Piston::lift.set_value(true);
    //     // pros::delay(150);
        
    //     pros::delay(50);

    //     Intake::setState(Intake::State::SCORE);
    //     pros::delay(500);
    //     Motor::intake1.move(100);
    //     // Misc::cdrift(40,40,500);
    //     // Piston::lift.set_value(false);
    //     // Piston::descore.set_value(true);
    //     // Intake::setState(Intake::State::LOCK);
    //     // Misc::cdrift(-35,-35,650);
    //     Misc::cdrift(-10,-10,2500);
    // }


    // void solo463(){

    //     chassis.setPose(-49.5, -15, 180);
    //     // Piston::hook.set_value(true);
    //     Intake::setState(Intake::State::LOCK);
    //     // Misc::cdrift(40,40,500);
    //     // pros::delay(100);

    //     chassis.moveToPointPro(-49, -43.1, 1500,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntilDone();
    //     Piston::loader.set_value(true);

    //     chassis.turnPointPlus(-65, -45.1, 2, 0.0, 1, 800, false);

    //     Misc::cdrift(55,55,110);
    //     Misc::cdrift(40,40,800);

    //     chassis.moveToPointPro(-29, -46.1, 1500,{.forwards=false,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntil(25);
    //     Intake::setState(Intake::State::SCORE);
    //     chassis.waitUntilDone();
    //     Misc::cdrift(-20,-20,200);
    //     Piston::loader.set_value(false);
    //     Misc::cdrift(-20,-20,400);
    //     Intake::setState(Intake::State::LOCK);
    //     // Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //     // Motor::intake1.move(-10);
    //     chassis.turnHeadingPlus(0, 2, 0, 1.0, 650, false);


    //     // Misc::cdrift(40,40,300);

    //     chassis.moveToPointPro(-27, -23, 1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntil(8);
    //     Piston::loader.set_value(true);
    //     chassis.waitUntilDone();
    //     Piston::loader.set_value(false);
    //     // chassis.moveToPointPro(-11,-10,800,{.forwards=true,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3});
    //     // chassis.waitUntil(1);
    //     // Piston::loader.set_value(false);
    //     // chassis.waitUntilDone();
    //     // chassis.turnHeadingPlus(45, 2, 0, 1.0, 300, false);
    //     // // chassis.turnPointPlus(0, 0, 2, 0.0, 1, 500, false);
        
    //     // // Piston::low.set_value(true);
    //     // // chassis.movePosePlus(-12, -12, 45, 0, 0, 0.0, 0.35, 1500, -1, true, false, false);
    //     // // chassis.moveDistPlus(15, 0.4, 1500, false);
    //     // // Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    //     // Intake::setState(Intake::State::SPIT);
    //     // Misc::cdrift(50,50,700);
       

    //     // Misc::cdrift(-65,-60,300);
    //     // Intake::setState(Intake::State::LOCK);

    //     chassis.moveToPointPro(-24, 14.5, 1200,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntil(22);
    //     Piston::loader.set_value(true);
    //     chassis.waitUntilDone();
    //     chassis.moveToPointPro(-48, 44, 1200,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntilDone();
    //     Motor::intake1.move(-127);
    //     chassis.moveToPointPro(-29, 41, 800,{.forwards=false,.maxSpeed=95,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntilDone();
    //     Misc::cdrift(-50,-50,300);
    //     Motor::intake1.move(127); 


    //     // chassis.movePosePlus(-24, 21, 355, 2, 0, 0, 0.9, 1500, -1, true, false, false);
    //     // Piston::loader.set_value(true);

    //     // chassis.movePosePlus(-40, 47, 310, 2, 0, 0.0, 1, 1500, -1, true, false, false);

    //     // chassis.movePosePlus(-26, 48, 270, 0, 0, 0.0, -1, 1000, -1, true, false, false);
    //     Intake::setState(Intake::State::SCORE);
    //     Misc::cdrift(-20,-20,600);
    //     // Piston::loader.set_value(true);
    //     Misc::cdrift(-20,-20,700);
        
        


    //     chassis.moveToPointPro(-51,42.1,1500,{.forwards=true,.maxSpeed=85,.minSpeed=0,.earlyExitRange=3}); // 46.5 
    //     chassis.waitUntil(6);
    //     Intake::setState(Intake::State::LOCK);
    //     // Piston::loader.set_value(false);
    //     chassis.waitUntilDone();

    //     // chassis.movePosePlus(-53, 46, 270, 0, 0, 0.0, 0.8, 1500, -1, true, false, false);
    //     Motor::intake1.move(-10);
    //     Misc::cdrift(50,50,150);
    //     Misc::cdrift(40,40,650);
    //     chassis.moveToPointPro(-13, 2, 2000,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=2});
    //     // chassis.moveToPointPro(-8.5, 5, 2000,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=1});
    //     chassis.waitUntil(30);
    //     Motor::intake1.move(30);
    //     Piston::loader.set_value(false);
    //     Piston::lift.set_value(true);
    //     chassis.waitUntilDone();
    //     Misc::cdrift(-10,-10);
    //     // Piston::lift.set_value(true);
    //     // pros::delay(150);
        
    //     pros::delay(50);

    //     Intake::setState(Intake::State::SCORE);
    //     pros::delay(500);
    //     Motor::intake1.move(100);
    //     // Misc::cdrift(40,40,500);
    //     // Piston::lift.set_value(false);
    //     // Piston::descore.set_value(true);
    //     // Intake::setState(Intake::State::LOCK);
    //     // Misc::cdrift(-35,-35,650);
    //     Misc::cdrift(-10,-10,2500);
    // }
    
    void solo463(){

        chassis.setPose(-49.5, -14.1, 180);
        // Piston::hook.set_value(true);
        Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(40,40,500);
        // pros::delay(100);

        chassis.moveToPointPro(-49, -43, 1500,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        Piston::loader.set_value(true);

        chassis.turnPointPlus(-65, -45, 2, 0.0, 1, 800, false);

        Misc::cdrift(55,55,110);
        Misc::cdrift(40,40,800);

        chassis.moveToPointPro(-29, -46, 1500,{.forwards=false,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(25);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,200);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,400);
        Intake::setState(Intake::State::LOCK);
        // Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        // Motor::intake1.move(-10);
        chassis.turnHeadingPlus(0, 2, 0, 1.0, 650, false);


        // Misc::cdrift(40,40,300);

        chassis.moveToPointPro(-24, -23, 1500,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(10);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        Piston::loader.set_value(false);
        // chassis.moveToPointPro(-11,-10,800,{.forwards=true,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3});
        // chassis.waitUntil(1);
        // Piston::loader.set_value(false);
        // chassis.waitUntilDone();
        // chassis.turnHeadingPlus(45, 2, 0, 1.0, 300, false);
        // // chassis.turnPointPlus(0, 0, 2, 0.0, 1, 500, false);
        
        // // Piston::low.set_value(true);
        // // chassis.movePosePlus(-12, -12, 45, 0, 0, 0.0, 0.35, 1500, -1, true, false, false);
        // // chassis.moveDistPlus(15, 0.4, 1500, false);
        // // Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        // Intake::setState(Intake::State::SPIT);
        // Misc::cdrift(50,50,700);
       

        // Misc::cdrift(-65,-60,300);
        // Intake::setState(Intake::State::LOCK);

        chassis.moveToPointPro(-21, 18.5, 1200,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(25);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        chassis.moveToPointPro(-42, 47, 1200,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        Motor::intake1.move(-127);
        chassis.moveToPointPro(-26, 45, 800,{.forwards=false,.maxSpeed=95,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        Misc::cdrift(-50,-50,300);
        Motor::intake1.move(127); 


        // chassis.movePosePlus(-24, 21, 355, 2, 0, 0, 0.9, 1500, -1, true, false, false);
        // Piston::loader.set_value(true);

        // chassis.movePosePlus(-40, 47, 310, 2, 0, 0.0, 1, 1500, -1, true, false, false);

        // chassis.movePosePlus(-26, 48, 270, 0, 0, 0.0, -1, 1000, -1, true, false, false);
        Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-20,-20,600);
        // Piston::loader.set_value(true);
        Misc::cdrift(-20,-20,700);
        
        


        chassis.moveToPointPro(-51,45.1,1500,{.forwards=true,.maxSpeed=85,.minSpeed=0,.earlyExitRange=3}); // 46.5 
        chassis.waitUntil(6);
        Intake::setState(Intake::State::LOCK);
        // Piston::loader.set_value(false);
        chassis.waitUntilDone();

        // chassis.movePosePlus(-53, 46, 270, 0, 0, 0.0, 0.8, 1500, -1, true, false, false);
        Motor::intake1.move(-10);
        Misc::cdrift(50,50,150);
        Misc::cdrift(40,40,650);
        chassis.moveToPointPro(-13, 6, 2000,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=2});
        // chassis.moveToPointPro(-8.5, 5, 2000,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=1});
        chassis.waitUntil(30);
        Motor::intake1.move(30);
        Piston::loader.set_value(false);
        Piston::lift.set_value(true);
        chassis.waitUntilDone();
        Misc::cdrift(-10,-10);
        // Piston::lift.set_value(true);
        // pros::delay(150);
        
        pros::delay(50);

        Intake::setState(Intake::State::SCORE);
        pros::delay(500);
        Motor::intake1.move(100);
        // Misc::cdrift(40,40,500);
        // Piston::lift.set_value(false);
        // Piston::descore.set_value(true);
        // Intake::setState(Intake::State::LOCK);
        // Misc::cdrift(-35,-35,650);
        Misc::cdrift(-10,-10,2500);
    }


    void wingAWPslow(){
        chassis.setPose(53,17.5,0);
        // RclMain.setRclPose(chassis.getPose());
        Intake::setState(Intake::State::LOCK);
        // Piston::hook.set_value(true);
        chassis.moveToPointPro(53,46.2,1200,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1.5});
        // chassis.moveToPoint(53,45.3,1200,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1.5});
        chassis.waitUntil(15);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        Misc::cdrift(10,-10,50);
        // chassis.turnToHeading(270,1200,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=1});
        // chassis.waitUntilDone();
        // Piston::loader.set_value(true);
        // pros::delay(100);
        chassis.turnPointPlus(65,48,2,0.0,1,800,false);
        // chassis.turnToPoint(65,48,800,{.forwards=true,.maxSpeed=90,.minSpeed=5,.earlyExitRange=1.5});
        // chassis.waitUntilDone();
        Misc::cdrift(45,45,170);
        
        Misc::cdrift(35,35,900);
        // Misc::cdrift(-20,-20,200);
        // Misc::cdrift(35,35,450);
        chassis.moveToPointPro(31,48.6,950,{.forwards=false,.maxSpeed=85,.minSpeed=0,.earlyExitRange=3});
        // chassis.moveToPoint(32,49.2,950,{.forwards=false,.maxSpeed=85,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(25.5);
        Intake::setState(Intake::State::SCORE);
        chassis.waitUntilDone();

        // Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-20,-20,500);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,550);
        // Intake::setState(Intake::State::LOCK);
        Misc::cdrift(95,55,200);
        Intake::setState(Intake::State::LOCK);
        Misc::cbrake(false);
        pros::delay(100);
        Misc::cbrake(true);
        chassis.moveToPointPro(29.2,34,1250,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(25);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        Piston::loader.set_value(false);
        // chassis.waitUntilDone();
        // Misc::cbrake(false);
        // pros::delay(1000000);
        Misc::cdrift(-10,80,100);
        chassis.moveToPointPro(22,-12,1300,{.forwards=true,.maxSpeed=90,.minSpeed=15,.earlyExitRange=1.5});
        chassis.waitUntil(37);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        Misc::cdrift(-10,80,50);
        
        // chassis.waitUntil(17);
        // Piston::loader.set_value(true);
        // chassis.waitUntilDone();
        // Piston::loader.set_value(false);



        chassis.moveToPointPro(48,-43.5,1300,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=2});
        chassis.waitUntilDone();
        // Misc::cbrake(false);
        // pros::delay(10000000);
        Misc::cdrift(10,20,50);
        Misc::cdrift(-15,-10,50);

        chassis.moveToPointPro(28,-45.8,1200,{.forwards=false,.maxSpeed=90,.minSpeed=10,.earlyExitRange=3});
        chassis.waitUntil(23);
        Intake::setState(Intake::State::SCORE);
        
        chassis.waitUntilDone();
        Misc::cdrift(-20,-20,600);
        Piston::loader.set_value(true);
        Misc::cdrift(-20,-20,700);
        // Intake::setState(Intake::State::LOCK);

        chassis.moveToPointPro(55,-45.1,1200,{.forwards=true,.maxSpeed=85,.minSpeed=0,.earlyExitRange=2});
        chassis.waitUntil(4);
        Intake::setState(Intake::State::LOCK);
        chassis.waitUntilDone();
        Misc::cdrift(40,40,650);
        


        chassis.moveToPointPro(8,-3.75,1700,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3}); // 7,4.5
        chassis.waitUntil(55);
        Motor::intake1.move(30);
        Piston::loader.set_value(false);
        Piston::lift.set_value(true);
        chassis.waitUntilDone();
        
        // Intake::setState(Intake::State::AUTON);
        Motor::intake1.move(90);
        Misc::cdrift(-35,-35,150);
        Misc::cdrift(-10,-10,900);
        // Misc::cdrift(20,20,200);
        // Intake::setState(Intake::State::SPIT);
        // Motor::intake1.move(0);
        // Motor::intake2.move(0);
        // Misc::cdrift(-20,-20,200);




        // Intake::setState(Intake::State::LOCK);
        // chassis.moveToPoint(35,-29,1250,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=3});
        // chassis.turnToHeading(89,600,{.maxSpeed=127,.minSpeed=10,.earlyExitRange=2});
        // chassis.waitUntilDone();
        // Piston::hook.set_value(false);
        // chassis.moveToPoint(21,-31,1750,{.forwards=false,.maxSpeed=127,.minSpeed=10,.earlyExitRange=2});
        // chassis.turnToHeading(125,800,{.maxSpeed=127,.minSpeed=0,.earlyExitRange=2});
        // chassis.waitUntilDone();
        // Misc::cdrift(0,-15);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    }

    // void solo463(){

    //     chassis.setPose(-49.5, -14.1, 180);
    //     // Piston::hook.set_value(true);
    //     Intake::setState(Intake::State::LOCK);
    //     // Misc::cdrift(40,40,500);
    //     // pros::delay(100);

    //     chassis.moveToPointPro(-49, -43, 1500,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntilDone();
    //     Piston::loader.set_value(true);

    //     chassis.turnPointPlus(-65, -45, 2, 0.0, 1, 800, false);
    //     Motor::intake1.move(-10);

    //     Misc::cdrift(55,55,110);
    //     Misc::cdrift(40,40,800);

    //     chassis.moveToPointPro(-29, -46, 1500,{.forwards=false,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntil(25);
    //     Intake::setState(Intake::State::SCORE);
    //     chassis.waitUntilDone();
    //     Misc::cdrift(-20,-20,200);
    //     Piston::loader.set_value(false);
    //     Misc::cdrift(-20,-20,400);
    //     Intake::setState(Intake::State::LOCK);
    //     // Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //     // Motor::intake1.move(-10);


    //     // chassis.turnHeadingPlus(0, 2, 0, 1.0, 650, false);

    //     Misc::cdrift(80,55,300);

    //     chassis.turnPointPlus(-24, -23, 2, 0.0, 1, 500, false);


    //     // Misc::cdrift(40,40,300);

    //     chassis.moveToPointPro(-24, -23, 1200,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntil(18);
    //     Piston::loader.set_value(true);
    //     chassis.waitUntilDone();
    //     Piston::loader.set_value(false);

    //     chassis.turnPointPlus(-21, -18.5, 2, 0.0, 1, 400, false);
    //     // chassis.moveToPointPro(-11,-10,800,{.forwards=true,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3});
    //     // chassis.waitUntil(1);
    //     // Piston::loader.set_value(false);
    //     // chassis.waitUntilDone();
    //     // chassis.turnHeadingPlus(45, 2, 0, 1.0, 300, false);
    //     // // chassis.turnPointPlus(0, 0, 2, 0.0, 1, 500, false);
        
    //     // // Piston::low.set_value(true);
    //     // // chassis.movePosePlus(-12, -12, 45, 0, 0, 0.0, 0.35, 1500, -1, true, false, false);
    //     // // chassis.moveDistPlus(15, 0.4, 1500, false);
    //     // // Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    //     // Intake::setState(Intake::State::SPIT);
    //     // Misc::cdrift(50,50,700);
       

    //     // Misc::cdrift(-65,-60,300);
    //     // Intake::setState(Intake::State::LOCK);

    //     chassis.moveToPointPro(-21, 18.5, 1200,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=3});
    //     chassis.waitUntil(25);
    //     Piston::loader.set_value(true);
    //     chassis.waitUntilDone();
    //     chassis.moveToPointPro(-42, 47, 1200,{.forwards=true,.maxSpeed=100,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntilDone();
    //     Motor::intake1.move(-127);
    //     chassis.moveToPointPro(-26, 45, 800,{.forwards=false,.maxSpeed=95,.minSpeed=0,.earlyExitRange=3});
    //     chassis.waitUntilDone();
    //     Misc::cdrift(-50,-50,300);
    //     Motor::intake1.move(127); 


    //     // chassis.movePosePlus(-24, 21, 355, 2, 0, 0, 0.9, 1500, -1, true, false, false);
    //     // Piston::loader.set_value(true);

    //     // chassis.movePosePlus(-40, 47, 310, 2, 0, 0.0, 1, 1500, -1, true, false, false);

    //     // chassis.movePosePlus(-26, 48, 270, 0, 0, 0.0, -1, 1000, -1, true, false, false);
    //     Intake::setState(Intake::State::SCORE);
    //     Misc::cdrift(-20,-20,600);
    //     // Piston::loader.set_value(true);
    //     Misc::cdrift(-20,-20,500);
        
        


    //     chassis.moveToPointPro(-51,45.1,1500,{.forwards=true,.maxSpeed=85,.minSpeed=0,.earlyExitRange=3}); // 46.5 
    //     chassis.waitUntil(6);
    //     Intake::setState(Intake::State::LOCK);
    //     // Piston::loader.set_value(false);
    //     chassis.waitUntilDone();

    //     // chassis.movePosePlus(-53, 46, 270, 0, 0, 0.0, 0.8, 1500, -1, true, false, false);
    //     Motor::intake1.move(-10);
    //     Misc::cdrift(50,50,150);
    //     Misc::cdrift(40,40,650);
    //     chassis.moveToPointPro(-12, 7, 2000,{.forwards=false,.maxSpeed=95,.minSpeed=0,.earlyExitRange=2});
    //     // chassis.moveToPointPro(-8.5, 5, 2000,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=1});
    //     chassis.waitUntil(30);
    //     Motor::intake1.move(30);
    //     Piston::loader.set_value(false);
    //     Piston::lift.set_value(true);
    //     chassis.waitUntilDone();
    //     Misc::cdrift(-10,-10);
    //     // Piston::lift.set_value(true);
    //     // pros::delay(150);
        
    //     pros::delay(50);

    //     Intake::setState(Intake::State::SCORE);
    //     pros::delay(500);
    //     Motor::intake1.move(100);
    //     // Misc::cdrift(40,40,500);
    //     // Piston::lift.set_value(false);
    //     // Piston::descore.set_value(true);
    //     // Intake::setState(Intake::State::LOCK);
    //     // Misc::cdrift(-35,-35,650);
    //     Misc::cdrift(-10,-10,2500);
    // }
    void skills() { 
        chassis.setPose(-64, 18, 180);
        Piston::hook.set_value(false);
        Piston::odom.set_value(true);
        Intake::setState(Intake::State::LOCK);
        // Piston::loader.set_value(true);
        // pros::delay(300);
        // Piston::loader.set_value(false);   
        // pros::delay(100);   
        
        // // 1
        // Misc::cdrift(75,75,200); // 45 45 600
        // Misc::cdrift(63,63,750);
        // Misc::cdrift(70,70,550);
        // // Piston::loader.set_value(true);
        // Misc::cdrift(50,50,250);
        // // Piston::loader.set_value(false);
        // Misc::cdrift(50,50,600);

        // 2
        Misc::cdrift(80,80,200); // 45 45 600
        Misc::cdrift(70,63,750);
        Misc::cdrift(70,63,600);
        // Piston::loader.set_value(true);
        Misc::cdrift(53,50,250);
        // Piston::loader.set_value(false);
        Misc::cdrift(50,50,600);


        chassis.turnHeadingPlus(180, 2, 0, 1.0, 1500, false);
        Piston::odom.set_value(false);
        pros::delay(500);

        Misc::reset(4,4);
        Misc::reset(3,3);
        leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

        // pros::delay(10000000);


        // Misc::cdrift(65,65,675);
        // pros::delay(500);
        // Misc::cdrift(45,45,750);
        // Misc::cdrift(-20,20,200);
        // Misc::cdrift(20,-20,200);
        // Misc::cdrift(-20,20,200);
        // Misc::cdrift(20,-20,200);

        // chassis.turnHeadingPlus(270, 2, 0, 1.0, 500, false);


        // Misc::cdrift(-65,-65,850);
        // Piston::odom.set_value(false);
        // chassis.turnHeadingPlus(270, 2, 0, 1.0, 500, false);
        // Misc::reset(3,1);
        // Misc::reset(4,2);
        // leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        // rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        chassis.moveToPointPro(-63, -22, 1500,{.forwards=false,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        // Misc::cdrift(-60,-60,500);
        chassis.turnHeadingPlus(90, 2, 0, 1.0, 1500, false);
        // chassis.movePosePlus(-30, -21, 95, 0, 0, 0.0, 0.6, 2000, -1, true, false, false); // 35
        // chassis.movePosePlus(-24, -22, 95, 0, 0, 0.0, 0.6, 1000, -1, true, false, false);
        chassis.moveToPointPro(-20, -22, 1500,{.forwards=true,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3}); // -22 -22
        chassis.waitUntilDone();
        chassis.turnHeadingPlus(45, 2, 0, 1.0, 1500, false);
        Piston::low.set_value(true);
        // chassis.movePosePlus(-12, -12, 45, 0, 0, 0.0, 0.35, 1500, -1, true, false, false);
        // chassis.moveDistPlus(15, 0.4, 1500, false);
        Misc::cdrift(35,35,400);
        Intake::setState(Intake::State::LG1);
        Misc::cdrift(15,15,1500);
        Intake::setState(Intake::State::LG2);
        Misc::cdrift(15,15,1700);
        Misc::cdrift(-45,-45,420);
        Intake::setState(Intake::State::SCORE);
        Piston::low.set_value(false);

        chassis.turnHeadingPlus(0, 2, 0, 1.0, 500, false);
        chassis.moveToPointPro(-24, 20, 1500,{.forwards=true,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(23.5);
        Intake::setState(Intake::State::LOCK);
        // Piston::loader.set_value(true);
        chassis.waitUntilDone();
        // Piston::loader.set_value(false);
        // Piston::loader.set_value(true);
        // chassis.movePosePlus(-24, 22, 355, 0, 0, 0.0, 0.8, 2000, -1, true, false, false);
        chassis.moveToPointPro(-47, 41, 1500,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
        // chassis.movePosePlus(-46, 48, 315, 4, 0, 0.0, 0.8, 2000, -1, true, false, false);

        // leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        // rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        // leftMotors.brake(); 
        // rightMotors.brake();

        // pros::delay(200);
        // Piston::loader.set_value(true);
        chassis.waitUntilDone();
        Piston::loader.set_value(true);
        // chassis.turnHeadingPlus(270, 2, 0, 1.0, 1500, false);
        chassis.turnPointPlus(-65, 42.5, 2, 0.0, 1, 1500, false);
        // chassis.turnToPoint(-65,47,1250,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});
        // leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        // rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        
        Misc::cdrift(40,40,450);
        // Misc::cdrift(55,55,100);
        Misc::cdrift(30,30,1400);


        chassis.movePosePlus(-52, 55, 225, 4, 0, 0.0, -0.8, 2000, -1, false, false, false);
        chassis.movePosePlus(20, 55, 90, 4, 0, 0.0, 1, 2000, -1, false, false, false);
        chassis.movePosePlus(40, 42, 120, 4, 0, 0.0, 1, 1000, -1, true, false, false);
        chassis.turnHeadingPlus(90, 2, 0, 1.0, 1000, false);
        Misc::reset(2,2);

        chassis.moveToPointPro(24, 48.75, 1000,{.forwards=false,.maxSpeed=85,.minSpeed=0,.earlyExitRange=2});

        // chassis.movePosePlus(30, 48, 90, 4, 0, 0.0, -0.8, 2000, -1, true, false, false);
        chassis.waitUntilDone();
        Piston::loader.set_value(false);
        // Motor::intake1.move(-100);
        // pros::delay(100);
        // Motor::intake1.move(127);
        Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-20,-20,200);
        Motor::intake2.move(-127);
        pros::delay(100);
        Motor::intake2.move(127);
        // Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,1500);
        Intake::setState(Intake::State::LOCK);
        Piston::loader.set_value(true);




        // Misc::cdrift(-35,-35,450);
        // chassis.turnHeadingPlus(90, 1, 0, 1.0, 600, false);
        // chassis.turnHeadingPlus(90, 2, 0, 1.0, 300, true);
        // Misc::reset(2,2);

        // // chassis.movePosePlus(30, 49, 90, 4, 0, 0.0, -0.8, 2000, -1, true, false, false);
        // // chassis.turnPointPlus(65, 49, 2, 0.0, 1, 1500, false);
        // // chassis.turnHeadingPlus(90, 1, 0, 1.0, 1500, false);
        // Motor::intake1.move(-100);
        // Misc::cdrift(-40,-40,100);
        // Motor::intake1.move(127);
        // Misc::cdrift(-40,-40,150);

        // // chassis.movePosePlus(-52, 55, 225, 4, 0, 0.0, -0.8, 2000, -1, false, false, false);
        // // chassis.movePosePlus(28.5, 55, 90, 4, 0, 0.0, 1, 2000, -1, true, false, false);
        // // chassis.turnHeadingPlus(45, 2, 0, 1.0, 1500, false);
        // // Misc::cdrift(-35,-35,450);
        // // chassis.turnHeadingPlus(90, 1, 0, 1.0, 600, false);
        // // chassis.turnHeadingPlus(90, 2, 0, 1.0, 300, true);
        // // Misc::reset(2,2);

        // // // chassis.movePosePlus(30, 49, 90, 4, 0, 0.0, -0.8, 2000, -1, true, false, false);
        // // // chassis.turnPointPlus(65, 49, 2, 0.0, 1, 1500, false);
        // // // chassis.turnHeadingPlus(90, 1, 0, 1.0, 1500, false);
        // // Motor::intake1.move(-100);
        // // Misc::cdrift(-40,-40,100);
        // // Motor::intake1.move(127);
        // // Misc::cdrift(-40,-40,150);
        

        // Intake::setState(Intake::State::SCORE);
        // Misc::cdrift(-20,-20,450);
        // // Piston::loader.set_value(false);
        // Misc::cdrift(-20,-20,1200);
        // Intake::setState(Intake::State::LOCK);
        // // chassis.movePosePlus(30, 48, 90, 4, 0, 0.0, -0.8, 2000, -1, true, false, false);
        chassis.moveToPointPro(48, 48.5, 1500,{.forwards=true,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();

        // chassis.movePosePlus(55, 46, 90, 0, 0, 0.0, 0.8, 1500, -1, true, false, false);
        Misc::cdrift(40,40,1900); // 1500
        chassis.moveToPointPro(25, 48.5, 1500,{.forwards=false,.maxSpeed=85,.minSpeed=0,.earlyExitRange=2});

        // chassis.movePosePlus(30, 48, 90, 4, 0, 0.0, -0.8, 2000, -1, true, false, false);
        chassis.waitUntilDone();
        Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-20,-20,450);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,300);
        Motor::intake1.move(70);
        Misc::cdrift(-20,-20,800);
        // Intake::setState(Intake::State::LOCK);

        chassis.moveToPointPro(50, 30, 1200,{.forwards=true,.maxSpeed=127,.minSpeed=30,.earlyExitRange=3});
        chassis.waitUntilDone();
        // chassis.
        // chassis.movePosePlus(65, 26, 180, 0, 0, 0.0, 0.8, 2000, -1, false, false, false);

        // chassis.movePosePlus(60, 25, 180, 4, 0, 0.0, 1, 2000, -1, false, false, false);
        Intake::setState(Intake::State::LOCK);
        chassis.turnHeadingPlus(179, 0, 0, 1.0, 800, false);


        // Misc::cdrift(30,30,400);
        // chassis.turnHeadingPlus(180, 2, 0, 1.0, 1500, false);
        // chassis.moveToPointPro(27, 35, 1500,{.forwards=true,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3}); // 31.5
        // chassis.waitUntilDone();
        // Intake::setState(Intake::State::LOCK);
        // chassis.turnHeadingPlus(90, 2, 0, 1.0, 1500, false);
        // Misc::reset(1,1);
        // chassis.moveToPointPro(55.5, 30, 1200,{.forwards=true,.maxSpeed=127,.minSpeed=30,.earlyExitRange=3});
        // // chassis.
        // // chassis.movePosePlus(65, 26, 180, 0, 0, 0.0, 0.8, 2000, -1, false, false, false);

        // // chassis.movePosePlus(60, 25, 180, 4, 0, 0.0, 1, 2000, -1, false, false, false);
        // chassis.turnHeadingPlus(179, 0, 0, 1.0, 1500, false);

        pros::delay(500);
        Piston::odom.set_value(true);

        // or
        // Misc::cdrift(80,80,200); // 45 45 600
        // Misc::cdrift(63,63,750);
        // Misc::cdrift(70,70,550);
        // // Piston::loader.set_value(true);
        // Misc::cdrift(50,50,250);
        // // Piston::loader.set_value(false);
        // Misc::cdrift(50,50,600);

        Misc::cdrift(80,80,200); // 45 45 600
        Misc::cdrift(63,65,750);
        Misc::cdrift(63,65,600);
        // Piston::loader.set_value(true);
        Misc::cdrift(50,52,250);
        // Piston::loader.set_value(false);
        Misc::cdrift(50,50,600);


        // Misc::cdrift(80,80,200); // 45 45 600
        // Misc::cdrift(70,63,750);
        // Misc::cdrift(70,63,600);
        // // Piston::loader.set_value(true);
        // Misc::cdrift(53,50,250);
        // // Piston::loader.set_value(false);
        // Misc::cdrift(50,50,600);

        chassis.turnHeadingPlus(180, 2, 0, 1.0, 1500, false);
        Piston::odom.set_value(false);
        pros::delay(500);

        Misc::reset(4,1);
        Misc::reset(1,2);
        leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

        chassis.moveToPointPro(63, -22, 1500,{.forwards=false,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        // Misc::cdrift(-60,-60,500);
        chassis.turnHeadingPlus(270, 2, 0, 1.0, 1500, false);
        // chassis.movePosePlus(-30, -21, 95, 0, 0, 0.0, 0.6, 2000, -1, true, false, false); // 35
        // chassis.movePosePlus(-24, -22, 95, 0, 0, 0.0, 0.6, 1000, -1, true, false, false);
        Misc::cdrift(80,35,650);
        pros::delay(100);
        chassis.turnHeadingPlus(225, 2, 0, 1.0, 1000, false);
        chassis.moveToPointPro(28.5, -20.5, 1500,{.forwards=true,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntil(8);
        Piston::loader.set_value(true);
        chassis.waitUntilDone();
        // Misc::cdrift(25,25,200);
        

        chassis.turnHeadingPlus(135, 2, 0, 1.0, 1500, false);
        
        Misc::cdrift(-55,-55,800);

        Piston::lift.set_value(true);
        pros::delay(100);
        Motor::intake1.move(-50);
        Misc::cdrift(20,20,210);
        Motor::intake2.move(-100);
        Misc::cdrift(20,20,100);
        
        // Misc::cdrift(-15,-15);
        // pros::delay(200);
        Intake::setState(Intake::State::SKILLS);
        pros::delay(1000);
        Intake::indexUntilDetected(2000);
        Motor::intake1.move(35);
        pros::delay(200);
        
        Motor::intake1.move(-127);
        Misc::cdrift(40,40,400);
        Piston::lift.set_value(false);
        


        // Piston::lift.set_value(true);
        // Intake::setState(Intake::State::SKILLS);
        // Misc::cdrift(-15,-15); // 3200
        // Intake::indexUntilDetected(3500);
        // Misc::cdrift(40,40,400);
        // Intake::setState(Intake::State::LOCK);
        // Piston::lift.set_value(false);


        // 2/2

        chassis.moveToPointPro(48, -47, 1500,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();
        Intake::setState(Intake::State::LOCK);
        Piston::loader.set_value(true);
        chassis.turnPointPlus(65, -49, 2, 0.0, 1, 1500, false);
        Misc::cdrift(45,45,600);
        // Misc::cdrift(55,55,100);
        Misc::cdrift(40,40,1200);
        chassis.movePosePlus(53, -62, 45, 4, 0, 0.0, -0.8, 2000, -1, false, false, false); // 60.5


        chassis.movePosePlus(-20, -61.75, 270, 4, 0, 0.0, 1, 2000, -1, false, false, false);
        chassis.movePosePlus(-39, -48.5, 300, 4, 0, 0.0, 0.7, 1000, -1, true, false, false);
        chassis.turnHeadingPlus(270, 2, 0, 1.0, 1000, false);
        Misc::reset(4,2);

        chassis.moveToPointPro(-23, -48.1, 1000,{.forwards=false,.maxSpeed=85,.minSpeed=0,.earlyExitRange=2});

        // chassis.movePosePlus(30, 48, 90, 4, 0, 0.0, -0.8, 2000, -1, true, false, false);
        chassis.waitUntilDone();
        Piston::loader.set_value(false);
        Motor::intake1.move(-127);
        pros::delay(100);
        Motor::intake1.move(127);

        Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-20,-20,200);
        Motor::intake2.move(-127);
        pros::delay(100);
        Motor::intake2.move(127);
        // Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,1500);
        Intake::setState(Intake::State::LOCK);
        Piston::loader.set_value(true);

        // Intake::setState(Intake::State::SCORE);
        // Misc::cdrift(-20,-20,450);
        // // Piston::loader.set_value(false);
        // Misc::cdrift(-20,-20,1350);
        // Intake::setState(Intake::State::LOCK);




        // chassis.movePosePlus(-29, -61.5, 270, 4, 0, 0.0, 1, 2000, -1, true, false, false); // 60
        // chassis.turnHeadingPlus(225, 2, 0, 1.0, 1500, false);
        // Misc::cdrift(-40,-40,500);
        
        // chassis.turnHeadingPlus(270, 1, 0, 1.0, 600, false);
        // chassis.turnHeadingPlus(270, 2, 0, 1.0, 300, true);
        // Misc::reset(4,2); // n

        // // Misc::cdrift(-40,-40,250);
        // Motor::intake1.move(-100);
        // Misc::cdrift(-60,-60,100);
        // Motor::intake1.move(127);
        // Misc::cdrift(-40,-40,150);

        // Intake::setState(Intake::State::SCORE);
        // Misc::cdrift(-20,-20,450);
        // // Piston::loader.set_value(false);
        // Misc::cdrift(-20,-20,1200);
        // Intake::setState(Intake::State::LOCK);

        // // Intake::setState(Intake::State::SCORE);
        // // Misc::cdrift(-20,-20,450);
        // // // Piston::loader.set_value(false);
        // // Misc::cdrift(-20,-20,600);
        // // Intake::setState(Intake::State::LOCK);
        // // chassis.movePosePlus(30, 48, 90, 4, 0, 0.0, -0.8, 2000, -1, true, false, false);
        chassis.moveToPointPro(-48, -48, 1500,{.forwards=true,.maxSpeed=80,.minSpeed=0,.earlyExitRange=3});
        chassis.waitUntilDone();

        // chassis.movePosePlus(55, 46, 90, 0, 0, 0.0, 0.8, 1500, -1, true, false, false);
        Misc::cdrift(35,35,1700);
        chassis.moveToPointPro(-24, -48.5, 1500,{.forwards=false,.maxSpeed=85,.minSpeed=0,.earlyExitRange=2});

        // chassis.movePosePlus(30, 48, 90, 4, 0, 0.0, -0.8, 2000, -1, true, false, false);
        chassis.waitUntilDone();
        Intake::setState(Intake::State::SCORE);
        Misc::cdrift(-20,-20,450);
        Piston::loader.set_value(false);
        // Misc::cdrift(-20,-20,600);
        Misc::cdrift(-20,-20,500);
        Motor::intake1.move(70);
        Piston::loader.set_value(false);
        Misc::cdrift(-20,-20,650);

        // Misc::cdrift(40,40,200);
        
        // Intake::setState(Intake::State::LOCK);

        chassis.moveToPointPro(-48, -32, 1200,{.forwards=true,.maxSpeed=127,.minSpeed=30,.earlyExitRange=3});
        chassis.waitUntilDone();

        // chassis.movePosePlus(65, 26, 180, 0, 0, 0.0, 0.8, 2000, -1, false, false, false);

        // chassis.movePosePlus(60, 25, 180, 4, 0, 0.0, 1, 2000, -1, false, false, false);
        Intake::setState(Intake::State::LOCK);
        chassis.turnHeadingPlus(359, 0, 0, 1.0, 800, false);
        // pros::delay(500);
        // Piston::odom.set_value(true);

        // Misc::cdrift(45,45,500);
        Misc::cdrift(80,80,600);

        // chassis.movePosePlus(-60, -26, 0, 3, 0, 0.0, 1, 2000, -1, false, false, false);
        // chassis.turnHeadingPlus(355, 2, 0, 1.0, 1000, false);
        // Piston::odom.set_value(true);
        // Misc::cdrift(70,70,750);
        // // Misc::cdrift(30,30,400);
    }

    void forward(){
        Misc::cdrift(20,20);
    }
} // namespace Auton

// <-------------------------------------------------------------- Driver Code ----------------------------------------------------------->
namespace Driver{
    bool b_loader = false, b_clamp = false, b_aligner = false, b_hook = false, b_driver = false, b_middle = false, b_lift = false, b_odom = false, b_descore = false;
    double curveVal = 7.0;
    // void joystick(){
    //     while(1){
    //         const int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    //         const int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    //         leftMotors.move(leftY + rightX);
    //         rightMotors.move(leftY - rightX);

    //         pros::delay(10);

    //         // if(TaskHandler::driver) {
    //         //     int leftY = Misc::curve(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), curveVal, false); 
    //         //     int rightX = Misc::curve(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), curveVal, false);
    //         //     leftMotors.move(leftY+rightX); 
    //         //     rightMotors.move(leftY-rightX);
    //         // }
    //         // pros::delay(Misc::DELAY);
    //     }
    // }

    void intake(){
        while (1){
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) Intake::setState(Intake::State::LOCK);
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) Intake::setState(Intake::State::SCORE);
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) Intake::setState(Intake::State::SPIT);
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) Intake::setState(Intake::State::LG2);
             else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) Intake::setState(Intake::State::SKILLS);
            else {
                Motor::intake1.brake();
                Motor::intake2.brake();
                Piston::hood.set_value(false);
                Piston::low.set_value(false);
            }
            pros::delay(Misc::DELAY);
        }
    }

    void piston(){
        while(1){
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) { Misc::togglePiston(Piston::loader, b_loader); }
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { Misc::togglePiston(Piston::hook, b_hook); }
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { Misc::togglePiston(Piston::lift, b_lift); }
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { Misc::togglePiston(Piston::descore, b_descore); }
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { Misc::togglePiston(Piston::odom, b_odom); }
            (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) ? Piston::low.set_value(true) : Piston::low.set_value(false);

            pros::delay(Misc::DELAY);
        }
    }
} // namespace Driver

// <-------------------------------------------------------------- Auton ----------------------------------------------------------->

using AutonFunc = void(*)();
std::vector<std::pair<std::string, AutonFunc>> autonRoutines = {
    {"Default Auton", Auton::left433}, // wingAWPslow is the new solo 4+6+3. left433
    {"Solo 4+6+3", Auton::wingAWPslow},
    // {"Default Auton", Auton::solo463},
    // {"Back Left 7 wing", Auton::leftsevenwingback},
    // {"Back Right 7 wing", Auton::rightsevenwingback},
    {"Wing Left 4 + 3 ", Auton::left43wing},
    {"Middle Left 4 + 3 ", Auton::left43mid},
    {"Wing Right 4 + 3 ", Auton::right43wing},
    {"Middle Right 4 + 3 ", Auton::right43mid},
    {"Half AWP 4 + 5 + 3 ", Auton::halfAWP},
    {"Half AWP 4 + 3 + 3", Auton::left433},
    {"4 + 3", Auton::left43},
    {"Counter 7 ball  4 + 3", Auton::left43c7},
    // {"Back Left 7 wing", Auton::leftsevenend},
    // {"Back Left 7 wing", Auton::rightsevenend},
    {"Back Left 7 wing", Auton::leftsevenwingback},
    {"Front Left 7 wing", Auton::leftsevenwingfront},
    {"Front Right 7 wing", Auton::rightsevenwingfront},
    {"Solo 4 Goals", Auton::solo4g},
    {"Skills", Auton::skills},
    // Right 10W descore
    // Left 80001X descore but on inside
    // 10B 6 + 3
    // 10B variation 3 + 6
    // Arpit 4
    // Regular 4
    // Joshua 4 + 5
    // Left to Right Solo
    // Wing AWP
};

int autonState = 0;

void autonSwitch() {
    // if(TaskHandler::autonSelect) {    
    pros::delay(Misc::DELAY);
    if (Sensor::autonSwitch.get_new_press()) { autonState++; if (autonState == autonRoutines.size()) autonState = 0; }
    pros::lcd::set_text(4, autonRoutines[autonState].first);
    // }
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
    Piston::hook.set_value(true);
    MotionPlusTuning::apply();
    // chassis.setPose(0, 0, 0);
    chassis.setPose(-24, -24, 90);
    chassis.calibrate(); 
    // RclMain.setRclPose(chassis.getPose());
    // RclMain.startTracking();
    Sensor::o_colorSort.set_led_pwm(100);
    Sensor::o_colorSort.set_integration_time(5);
    Sensor::o_crossed.set_led_pwm(100);
    Sensor::o_crossed.set_integration_time(5);
    Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intake2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    pros::Task autonSelect([]{ while(1){ autonSwitch(); pros::delay(Misc::DELAY); }});
    pros::Task ledTask([] { while (1) { Sensor::o_colorSort.set_led_pwm(100); pros::delay(10); }});

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
            // horizontal.getDistanceTraveled()/chassis.getPose(true).theta;
            // pros::lcd::print(0, "X: %f", chassis.getPose().x);
            // pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            // pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);

            // Misc::reset(2,1);
            // Misc::reset(3,2);

            // Misc::reset(4,4);
            // Misc::reset(3,3); 
            // -64 -36

            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            // pros::lcd::print(5, "H Offset: %f", horizontal.getDistanceTraveled()/chassis.getPose(true).theta);
            // pros::lcd::print(6, "V Offset: %f", vertical.getDistanceTraveled()/chassis.getPose(true).theta);
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
    // pros::Task stopIntake([]{ while(1){ Jam::intake(); pros::delay(Misc::DELAY); }});
    // pros::Task screenC([]{ while (1) { Screen::update(); pros::delay(100); }});
}
void disabled() {}
void competition_initialize() {}
ASSET(example_txt); // PP

// <------------------------------------------------------------- Auton ------------------------------------------------------------->
void autonomous() {
    // Misc::cdrift(20,20);
    // pros::delay(500);
    Piston::hook.set_value(false);
    // pros::delay(100000000);
    // chassis.setPose(0,0,0);
    // chassis.turnToHeading(90, 1500, {.maxSpeed=127, .minSpeed=0, .earlyExitRange=0});
    // chassis.turnToHeading(180, 1500, {.maxSpeed=127, .minSpeed=0, .earlyExitRange=0});
    // chassis.turnToHeading(270, 1500, {.maxSpeed=127, .minSpeed=0, .earlyExitRange=0});
    // chassis.turnToHeading(0, 1500, {.maxSpeed=127, .minSpeed=0, .earlyExitRange=0});

    // Auton::solo();
    // Auton::left43mid();
    // Auton::halfAWP();
    // Auton::right();
    // Auton::skills();
    // Auton::right43wing();
    // Auton::right43mid();
    // Auton::solo4g();
    // Auton::solo463();


    // Auton::leftsevenwingfront();
    // Auton::leftsevenwingback();
    // Auton::rightsevenwingback();


    // Skills
    // Piston::lift.set_value(true);
    // pros::delay(100);
    // // Intake::setState(Intake::State::LOCK);
    // Motor::intake1.move(-50);
    // Misc::cdrift(20,20,250);
    
    // // Misc::cdrift(-15,-15);
    // // pros::delay(200);
    // Intake::setState(Intake::State::SKILLS);
    // Intake::indexUntilDetected(4000);
    // Misc::cdrift(40,40,400);
    // Intake::setState(Intake::State::LOCK);
    // Piston::lift.set_value(false);



    // Piston::lift.set_value(true);
    // Intake::setState(Intake::State::LOCK);

    

    // chassis.setPose(0,0,0);
    // chassis.moveToPoint(0,24,100000);
    // chassis.moveToPointPro(0,24,100000);
    // Auton::soloNoPush();
    // Auton::soloSafe();
    // Auton::skills();
    // chassis.setPose(53,-18,180);
    // Intake::setState(Intake::State::LOCK);
    // Piston::hook.set_value(true);
    // chassis.moveToPoint(53,-48.8,1500,{.forwards=true,.maxSpeed=70,.minSpeed=0,.earlyExitRange=0});
    // chassis.waitUntil(15);
    // Piston::loader.set_value(true);
    // chassis.waitUntilDone();
    // Misc::cdrift(10,-10,50);
    // // chassis.turnToHeading(270,1200,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=1});
    // // chassis.waitUntilDone();
    // // Piston::loader.set_value(true);
    // // pros::delay(100);
    // // chassis.turnToPoint(65,-48,2000,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});
    // chassis.turnToHeading(90,2000,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});
    // chassis.waitUntilDone();
    // Misc::cdrift(45,45,110);
    
    // Misc::cdrift(35,35,850);
    // // Misc::cdrift(-20,-20,200);
    // // Misc::cdrift(35,35,450);

    // chassis.moveToPoint(32,-53,2000,{.forwards=false,.maxSpeed=75,.minSpeed=0,.earlyExitRange=0});
    // chassis.waitUntil(20);
    // Intake::setState(Intake::State::SCORE);
    // chassis.waitUntilDone();

    // // Intake::setState(Intake::State::SCORE);
    // Misc::cdrift(-20,-20,500);
    // Piston::loader.set_value(false);
    // Misc::cdrift(-20,-20,550);
    // Intake::setState(Intake::State::LOCK);
    // // Misc::cdrift(20,20,500);
    // // Misc::cdrift(0,0);
    // // Auton::left();
    // pros::delay(1000000);
    // chassis.setPose(-47.5, -3.85, 0);
    // chassis.movePointPlus(24, 24, 0.08, 1, 2000, false, false, false);
    // chassis.movePointPlus(0, 0, 0.00, 1, 2000, true, false, false);
    // chassis.movePointPlus(0, 24, 0, 1, 2000, false, false, false);
    // chassis.movePointPlus(0, 36, 0, 1, 2000, false, false, false);

    // chassis.movePosePlus(24, 24, 90, 1.0, 0.0, 0.0, 0.8, 1500, -1, true, false, false);
    // chassis.turnHeadingPlus(90, 2, 0.0, 1.00, 100000, false);
    // chassis.movePointPlus(24, 0, 0, 1.0, 1750, true, false, false);
    // pros::delay(100000000);

    // chassis.setPose(47.5, 14.8, 270);
    // Piston::hook.set_value(true);
    // Intake::setState(Intake::State::LOCK);

    // chassis.movePointPlus(22, 22.5, 0.08, 1.00, 1750, true, false, false);
    // // pros::delay(400);
    // chassis.waitUntil(7.1);
    // Piston::loader.set_value(true);
    // // chassis.waitUntilDone();

    // // chassis.turnPointPlus(58, 41, 2, 0.08, 1.00, 400, false);
    // chassis.turnHeadingPlus(55, 2, 0.16, 1.00, 1000, false);

    // // No exact 1:1 mapping from Genesis lead/horizontalDrift to Plus.
    // // Good starting approximation:
    // // chassis.turnHeadingPlus(125, 2, 0.16, 1.00, 400, false);
    // chassis.movePointPlus(55, 47.5, 0.08, 1.00, 1750, true, false, false);
    // // chassis.movePosePlus(64, 47.5, 90, 10.0, 1.0, 0.0, 0.8, 1500, -1, true, false, false);
    // // chassis.waitUntilDone();

    // chassis.turnHeadingPlus(90, 2, 0.16, 1.00, 1000, false);

    // Misc::cdrift(30, 30, 500);

    // chassis.movePointPlus(25, 47.5, 0.0, -0.71, 1150, true, false, false);
    // chassis.waitUntil(16);
    // Intake::setState(Intake::State::SCORE);
    // chassis.waitUntilDone();

    // Misc::cdrift(-20, -20, 850);
    // Piston::loader.set_value(false);
    // Misc::cdrift(-20, -20, 700);
    // Intake::setState(Intake::State::LOCK);

    // Misc::cdrift(70, 70, 390);

    // chassis.turnHeadingPlus(125, 2, 0.16, 1.00, 700, false);
    // chassis.waitUntilDone();

    // Piston::hook.set_value(false);

    // chassis.movePosePlus(13, 57.5, 90, 10.0, 0.30, 0.0, -0.63, 1500, -1, true, false, false);
    // chassis.waitUntilDone();

    // Misc::cdrift(0, -15);
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // chassis.moveToPoint(22,22.5,1750,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
    // chassis.waitUntil(7.1);
    // Piston::loader.set_value(true);
    // chassis.waitUntilDone();
    // chassis.turnToPoint(58,41,400,{.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
    // // chassis.moveToPoint(63,-45,400,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
    // chassis.moveToPose(64,47.5,90,1500,{.forwards=true,.horizontalDrift=8,.lead=0.45,.maxSpeed=80,.minSpeed=0,.earlyExitRange=0}); // 65 47
    // chassis.waitUntilDone();
    // Misc::cdrift(30,30,500);
    // // Misc::cdrift(-20,-20,200);
    // // Misc::cdrift(45,45,590);
    // chassis.moveToPoint(25,47.5,1150,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
    // chassis.waitUntil(16);
    // Intake::setState(Intake::State::SCORE);
    // chassis.waitUntilDone();
    // Misc::cdrift(-20,-20,850);
    // Piston::loader.set_value(false);
    // Misc::cdrift(-20,-20,700);
    // Intake::setState(Intake::State::LOCK);
    // // Motor::intake2.brake();
    

    // Misc::cdrift(70,70,390);
    // chassis.turnToHeading(125,700,{.maxSpeed=127,.minSpeed=20,.earlyExitRange=3});
    // chassis.waitUntilDone();
    // Piston::hook.set_value(false);
    // chassis.moveToPose(11,59,90,1500,{.forwards=false,.horizontalDrift=8,.lead=0.45,.maxSpeed=80,.minSpeed=0,.earlyExitRange=0}); 
    // chassis.waitUntilDone();
    // Misc::cdrift(0,-15);
    // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // -10, -23.5 
    // chassis.movePointPlus(0, 24, 0.08, 0.7, 2000, false, false, false);
    // chassis.movePointPlus(0, 0, 0, -0.7, 2000, true, false, false);
    // chassis.turnHeadingPlus(90, 2, 0, 1.0, 1500, false);
    // chassis.turnHeadingPlus(0, 2, 0, 1.0, 1500, false);
    // chassis.turnHeadingPlus(90, 1, 0, 1.0, 1500, false);
    // chassis.turnHeadingPlus(180, 0, 0, 1.0, 1500, false);
    

    // chassis.movePointPlus(24, 24, 0.08, 1, 2000, false, false, false);
    // chassis.movePointPlus(0, 0, 0.00, 1, 2000, true, false, false);
    // chassis.movePosePlus(24, 24, 90, 9, 0, 0, 1, 1200, -1, true);


    // chassis.movePosePlus(-22.2, 46.2, -40, 12.6, 0, 0.1, 1, 60, -1, false, false);
    // //     frontRightWing = false;
    // chassis.movePosePlus(-22.2, 46.2, -40, 10.2, 0, 0.1, 1, 1500, -1, true, true);
    // chassis.movePosePlus(0, 24, 0, 0, 0, 0.0, 1, 1500, -1, true, false);
    // chassis.movePosePlus(0, 0, 180, 1, 0, 0.0, 1, 1500, -1, true, false);
    // chassis.movePosePlus(24, 24, 235, 2, 0, 0.0, -1, 1500, -1, true, false);
    // chassis.movePosePlus(0, 0, 0, 0, 4, 0.0, -1, 1500, -1, true, false);
    // chassis.turnHeadingPlus(0, 2, 0, 1.0, 1500, false);


    // leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    // rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    // leftMotors.brake(); 
    // rightMotors.brake();
    // chassis.movePosePlus(0, 0, 0, 0, 0, -1, -1, 1000, -1, true);
    // chassis.movePosePlus(1.5, 7.3, 0, 0, 0, -1, -1, 1000, -1, true);
    // chassis.movePointPlus(0, 24, 0, 1, 2000, false, false, false);
    // chassis.movePointPlus(0, 36, 0, 1, 2000, false, false, false);


    // chassis.turnHeadingPlus(10, 2, 0, 1.0, 1500, false);
    // chassis.turnHeadingPlus(0, 2, 0, 1.0, 1500, false);
    // chassis.turnHeadingPlus(20, 2, 0, 1.0, 1500, false);
    // chassis.turnHeadingPlus(0, 2, 0, 1.0, 1500, false);
    // chassis.turnHeadingPlus(45, 2, 0, 1.0, 1500, false);
    // chassis.turnHeadingPlus(0, 2, 0, 1.0, 1500, false);
    // chassis.turnHeadingPlus(60, 2, 0, 1.0, 1500, false);
    // chassis.turnHeadingPlus(0, 2, 0, 1.0, 1500, false);
    // chassis.turnHeadingPlus(90, 2, 0, 1.0, 1500, false);
    // chassis.turnHeadingPlus(0, 2, 0, 1.0, 1500, false);
    // chassis.turnHeadingPlus(120, 2, 0, 1.0, 1500, false);
    // chassis.turnHeadingPlus(0, 2, 0, 1.0, 1500, false);
    // chassis.turnHeadingPlus(180, 2, 0, 1.0, 1500, false);
    // // chassis.movePointPlus(0, 0, 0, 0.8, 2000, true, false, false);
    // chassis.turnHeadingPlus(0, 2, 0, 1.0, 1500, false);

    // chassis.turnHeadingPlus(0, 2, 0, 1.0, 1200, false);
    // pros::delay(300);
    // chassis.turnHeadingPlus(0, 2, 0, 1.0, 1200, false);

    
    // chassis.turnToHeading(90,100000);
    // Auton::left45();
    // Auton::solo4g();
    // Auton::left433();
    // Auton::skills();
    // pros::delay(10000000);
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
// void opcontrol() {
//     // pros::lcd::shutdown();
//     // lv_init();
//     // lv_img_set_src(sKiss, &gay);
// 	// lv_obj_set_pos(sKiss, 0, 0);

//     // lv_img_set_src(sixSeven, &sixseven);
// 	// lv_obj_set_pos(sixSeven, 140, 0);

//     do{
//         Auton::skills();
//     }
//     while(!(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)));

//     pros::Task intakeTask(Driver::intake);
//     pros::Task driverTask(Driver::joystick);
//     pros::Task pistonTask(Driver::piston);
//     // TaskHandler::autonSelect = false;
//     TaskHandler::colorSort = false;
//     TaskHandler::antiJam = false;
//     Piston::odom.set_value(true);
//     Piston::lift.set_value(false);
//     Piston::loader.set_value(false);

// 	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST); rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
//     Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);  Motor::intake2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

//     while(1) {
//         // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) OdomTune::run(1);
//         // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) OdomTune::run(-1);
//         pros::delay(Misc::DELAY);
//     }
// }

void opcontrol() {

    // pros::Task macroTask([] {
    //     Auton::skills();
    // });

    // while (!(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
    //          controller.get_digital(pros::E_CONTROLLER_DIGITAL_A))) {
    //     pros::delay(Misc::DELAY);
    // }

    // chassis.cancelAllMotions();
    // macroTask.remove();

    // -------- // 

    TaskHandler::colorSort = false;
    TaskHandler::antiJam = false;
    Piston::odom.set_value(true);
    Piston::lift.set_value(false);
    // Piston::loader.set_value(false);

    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    Motor::intake1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    Motor::intake2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    leftMotors.brake();
    rightMotors.brake();
    Motor::intake1.brake();
    Motor::intake2.brake();

    pros::Task intakeTask(Driver::intake);
    // pros::Task driverTask(Driver::joystick);
    pros::Task pistonTask(Driver::piston);

    while (true) {
        const int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // leftMotors.move(leftY + rightX);
        // rightMotors.move(leftY - rightX);

        leftMotors.move(leftY);
        rightMotors.move(rightY);

        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) OdomTune::run(1);
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) OdomTune::run(-1);

        pros::delay(10);
    }
}
