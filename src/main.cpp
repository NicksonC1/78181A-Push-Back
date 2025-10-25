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
#include "brainScreenLVGL.h"
#include "config.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

genesis::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 11.5 inch track width
                              genesis::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              10 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

genesis::ControllerSettings linearController (8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              6, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

genesis::ControllerSettings angularController(2.85, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              16, // derivative gain (kD) 
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

genesis::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
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
    void led(){
        while(1){
            Sensor::o_colorSort.set_integration_time(5);
            Sensor::o_colorSort.set_led_pwm(100);
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
    // void reset(){
    //     constexpr double field = 144.0;
    //     constexpr double offsetF = 10.0; 
    //     constexpr double offsetR = 4.0;
    //     double theta = (90 - s_imu.get_heading()) * M_PI / 180.0;
    //     double d_front = Sensor::d_front.get_distance() / 25.4;
    //     double d_right = Sensor::d_right.get_distance() / 25.4;
    //     double x = (field - d_right) - (offsetR * cos(theta)) - (offsetF * sin(theta));
    //     double y = (field - d_front) - (offsetF * cos(theta)) + (offsetR * sin(theta));
    //     chassis.setPose(x, y, s_imu.get_heading());
    // }
    // void reset() {
    //     constexpr double field = 144.0;  // field size in inches
    //     constexpr double halfField = field / 2.0;
    //     constexpr double offsetF = 10.0; // inches (front sensor)
    //     constexpr double offsetR = 4.0;  // inches (right sensor)

    //     double heading = s_imu.get_heading();  // degrees
    //     double theta = heading * M_PI / 180.0; // radians

    //     // Convert mm → inches
    //     double d_front = Sensor::d_front.get_distance() / 25.4;
    //     double d_right = Sensor::d_right.get_distance() / 25.4;

    //     // Compute position relative to field center
    //     double x = halfField - d_right - (offsetR * cos(theta)) - (offsetF * sin(theta));
    //     double y = halfField - d_front - (offsetF * cos(theta)) + (offsetR * sin(theta));

    //     // Convert so origin = center
    //     x -= halfField;
    //     y -= halfField;

    //     chassis.setPose(x, y, heading);
    // }
    void reset() {
        constexpr double field = 144.0;
        constexpr double halfField = field / 2.0;
        constexpr double offsetF = 10.0;
        constexpr double offsetR = -4.0;

        double heading = s_imu.get_heading();
        double theta = heading * M_PI / 180.0;

        double d_front = Sensor::d_front.get_distance() / 25.4; // mm → in
        double d_right = Sensor::d_right.get_distance() / 25.4;

        double x = (d_right - halfField) - (offsetR * cos(theta)) - (offsetF * sin(theta));
        double y = (halfField - d_front) - (offsetF * cos(theta)) + (offsetR * sin(theta));

        chassis.setPose(x, y, heading);

        printf("Pose -> X: %.2f, Y: %.2f, Heading: %.2f\n", x, y, heading);
    }

    int curve(int input, double t = 5, bool activated = true) {
        if(!activated) return input;
        val = (std::exp(-t/10)) + std::exp((std::abs(input)-100)/10)*(1-std::exp(-t/10)) * input;
        return val;
    }
} // namespace Misc


// <-------------------------------------------------------------- Anti Jam ----------------------------------------------------------->
namespace Jam{
    int counter = 0;
    int counter1 = 0;
    bool stuck = false;
    void antiJam(){
        if(TaskHandler::antiJam){
            counter+=Misc::DELAY;
            if(Motor::intakeF.get_actual_velocity() == 0 && counter > 300) stuck = true;
            if (stuck == true) {
                // TaskHandler::colorSort = false;
                Motor::intakeF.move(-127);
                pros::delay(100);
                Motor::intakeF.move(127);
                stuck = false;
                counter = 0;  
            }
        }
        if(TaskHandler::antiJam2){
            counter+=Misc::DELAY;
            if(Motor::intakeM.get_actual_velocity() == 0 && counter > 300) stuck = true;
            if (stuck == true) {
                // TaskHandler::colorSort = false;
                Motor::intakeM.move(-127);
                pros::delay(100);
                Motor::intakeM.move(127);
                stuck = false;
                counter = 0;  
            }
        }
        if(TaskHandler::antiJam3){
            counter+=Misc::DELAY;
            if(Motor::intakeU.get_actual_velocity() == 0 && counter > 300) stuck = true;
            if (stuck == true) {
                // TaskHandler::colorSort = false;
                Motor::intakeU.move(-127);
                pros::delay(100);
                Motor::intakeU.move(127);
                stuck = false;
                counter = 0;  
            }
        }
    }
    void intake(){
        if(Sensor::d_filled.get_distance() < 25) {
            counter1+=Misc::DELAY;
            if(counter1 > 1000) TaskHandler::filled = true;
        }
        // counter1 = 0;
    }
}

namespace Color {
    enum class colorVals { NONE, BLUE, RED };
    colorVals state = colorVals::NONE;
    bool isDone = false, isC = false, extend_once = false;
    constexpr double rLow = 5.0, rHigh = 38.0, bLow = 190.0, bHigh = 220.0, minProx = 95; 
    inline bool isRed(double h, double low, double max) { return h > low && h < max; }
    inline bool isBlue(double h, double low, double max) { return h > low && h < max; }
    inline bool withinProx(int input, double max) { return (input > max); }
    colorVals colorConvertor(colorVals input) { return (input == colorVals::BLUE) ? colorVals::RED : colorVals::BLUE; }
    void colorSort(colorVals input) {
        colorVals lastColor = colorVals::NONE;
        if(TaskHandler::colorSort){
            if(input == colorVals::RED && isRed(Sensor::o_colorSort.get_hue(),rLow,rHigh) && withinProx(Sensor::o_colorSort.get_proximity(),minProx)){
                Piston::miniHood.set_value(true);
                pros::delay(200);
                extend_once = true;
            }
            else if(input == colorVals::BLUE && isBlue(Sensor::o_colorSort.get_hue(),bLow,bHigh) && withinProx(Sensor::o_colorSort.get_proximity(),minProx)){
                Piston::miniHood.set_value(true);
                pros::delay(200);
                extend_once = true;
            }
            else { Piston::miniHood.set_value(false); }
            extend_once = false;
        }
    }
} // namespace Color

// <-------------------------------------------------------------- Auto Routes ----------------------------------------------------------->
namespace Auton{
    int state = 0;
    namespace Test{
        void main() { 
            // Color::state = Color::colorVals::BLUE;
            // // Lift::setState(0);
            // TaskHandler::antiJam = false;
            // TaskHandler::colorSort = true;
            // Piston::mogo.set_value(true);
            // pros::delay(100);
            // Motor::intake.move(127);
            // TaskHandler::autoIntake1 = true;
            Misc::cdrift(55,55,550);
        }
    } // namespace Test
    namespace Template{
        void left(){
            chassis.moveToPoint(17,-22,2000,{.forwards=false,.maxSpeed=80,.minSpeed=10,.earlyExitRange=1});
            chassis.turnToPoint(0,-3.5,1500,{.forwards=true,.maxSpeed=80,.minSpeed=10,.earlyExitRange=1});
            // chassis.turnToHeading(315,1200,{.direction=genesis::AngularDirection::AUTO,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            TaskHandler::colorSort = false;
            Misc::cdrift(30,30,350);
            Piston::miniHood.set_value(true);
            TaskHandler::antiJam = true;
            TaskHandler::antiJam2 = true;
            TaskHandler::antiJam3 = true;
            Motor::intakeF.move(127); Motor::intakeM.move(127); Motor::intakeU.move(85);
            Misc::cdrift(25,25,750);
            pros::delay(600);
            Motor::intakeF.brake(); Motor::intakeM.brake(); Motor::intakeU.brake();
            chassis.moveToPoint(48,-49,2000,{.forwards=false,.maxSpeed=85,.minSpeed=10,.earlyExitRange=0});
            chassis.waitUntilDone();
            Piston::loader.set_value(true);
            TaskHandler::antiJam2 = false;
            TaskHandler::antiJam3 = false;
            TaskHandler::colorSort = true;
            Piston::miniHood.set_value(false);
            chassis.turnToPoint(68,-46,1500,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1});
            // chassis.turnToHeading(90,1500,{.direction=genesis::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
            chassis.waitUntilDone();
            Motor::intakeF.move(127); Motor::intakeM.move(127); Motor::intakeU.move(127);
            Misc::cdrift(55,55,300);
            Misc::cdrift(15,15,1450);
            Piston::loader.set_value(false);

            chassis.moveToPoint(25.5,-51.5,1200,{.forwards=false,.maxSpeed=75,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Misc::cdrift(-10,-10);
            Piston::hood.set_value(true);
            TaskHandler::antiJam2 = true;
            TaskHandler::antiJam3 = true;
            pros::delay(900);
            TaskHandler::antiJam2 = false;
            TaskHandler::antiJam3 = false;
            pros::delay(1000);
            Misc::cdrift(45,45,450);
            Piston::hood.set_value(false);
            Misc::cdrift(0,0,250);
            Misc::cdrift(-75,-75,500);
        }  
        void right(){
            chassis.moveToPoint(20,23,2000,{.forwards=false,.maxSpeed=80,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Piston::loader.set_value(false);
            chassis.turnToPoint(0,1.5,1500,{.forwards=true,.maxSpeed=80,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Misc::cdrift(25,25,150);
            TaskHandler::antiJam = false;
            Motor::intakeF.move(-127); Motor::intakeM.move(-127); 
            Motor::intakeU.move(-127);
            Misc::cdrift(25,25,300);
            Motor::intakeU.brake();
            Misc::cdrift(25,25,700);
            pros::delay(500);
            Motor::intakeF.brake(); Motor::intakeM.brake(); Motor::intakeU.brake();

            chassis.moveToPoint(48,45,2000,{.forwards=false,.maxSpeed=85,.minSpeed=10,.earlyExitRange=0});
            chassis.waitUntilDone();
            Piston::loader.set_value(true);
            TaskHandler::antiJam = true;
            TaskHandler::antiJam2 = false;
            TaskHandler::antiJam3 = false;
            TaskHandler::colorSort = true;
            Piston::miniHood.set_value(false);
            chassis.turnToPoint(68,44,1500,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1});
            // chassis.turnToHeading(90,1500,{.direction=genesis::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
            chassis.waitUntilDone();
            Motor::intakeF.move(127); Motor::intakeM.move(127); Motor::intakeU.move(127);
            Misc::cdrift(55,55,300);
            Misc::cdrift(15,15,1450);
            Piston::loader.set_value(false);

            chassis.moveToPoint(25.5,47,1200,{.forwards=false,.maxSpeed=75,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Misc::cdrift(-10,-10);
            Piston::hood.set_value(true);
            TaskHandler::antiJam2 = true;
            TaskHandler::antiJam3 = true;
            pros::delay(900);
            TaskHandler::antiJam2 = false;
            TaskHandler::antiJam3 = false;
            pros::delay(1200);
            Misc::cdrift(45,45,450);
            Piston::hood.set_value(false);
            Misc::cdrift(0,0,350);
            Misc::cdrift(-75,-75,500);
        }  
        void solo(){          
            TaskHandler::colorSort = false;
            chassis.setPose(47.28, -7.67, 237.5);
            TaskHandler::antiJam = true;
            Motor::intakeM.move(127); Motor::intakeF.move(127);
            chassis.moveToPose(9,-42,180,2100,{.forwards=true,.horizontalDrift=12,.lead=0.38,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            pros::delay(250);
            Piston::loader.set_value(true);
            
            chassis.moveToPoint(17,-22,2000,{.forwards=false,.maxSpeed=80,.minSpeed=10,.earlyExitRange=1});
            chassis.turnToPoint(0,-3.5,1500,{.forwards=true,.maxSpeed=80,.minSpeed=10,.earlyExitRange=1});
            // chassis.turnToHeading(315,1200,{.direction=genesis::AngularDirection::AUTO,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            TaskHandler::colorSort = false;
            Misc::cdrift(30,30,350);
            Piston::miniHood.set_value(true);
            TaskHandler::antiJam = true;
            TaskHandler::antiJam2 = true;
            TaskHandler::antiJam3 = true;
            Motor::intakeF.move(127); Motor::intakeM.move(127); Motor::intakeU.move(85);
            Misc::cdrift(25,25,450);
            pros::delay(600);
            Motor::intakeF.brake(); Motor::intakeM.brake(); Motor::intakeU.brake();
            chassis.moveToPoint(48,-49,2000,{.forwards=false,.maxSpeed=85,.minSpeed=10,.earlyExitRange=0});
            chassis.waitUntil(6);
            Piston::miniHood.set_value(false);
            chassis.waitUntilDone();
            Piston::loader.set_value(true);
            TaskHandler::antiJam2 = false;
            TaskHandler::antiJam3 = false;
            TaskHandler::colorSort = true;
            Piston::miniHood.set_value(false);
            chassis.turnToPoint(68,-46,1500,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1});
            // chassis.turnToHeading(90,1500,{.direction=genesis::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
            chassis.waitUntilDone();
            Motor::intakeF.move(127); Motor::intakeM.move(127); Motor::intakeU.move(127);
            Misc::cdrift(55,55,300);
            Misc::cdrift(15,15,900);
            Piston::loader.set_value(false);

            chassis.moveToPoint(25.5,-51.5,1200,{.forwards=false,.maxSpeed=75,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Misc::cdrift(-10,-10);
            Piston::hood.set_value(true);
            TaskHandler::antiJam2 = true;
            TaskHandler::antiJam3 = true;
            pros::delay(900);
            TaskHandler::antiJam2 = false;
            TaskHandler::antiJam3 = false;

            Piston::hood.set_value(false);
            Piston::loader.set_value(false);


            chassis.moveToPoint(37.5,-10,3000,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPoint(20,16,3000,{.forwards=true,.maxSpeed=75,.minSpeed=10,.earlyExitRange=1});
            // chassis.moveToPoint(20,23,3000,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Misc::cdrift(75,-75,150);
            chassis.moveToPoint(41.5,43,1750,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Piston::loader.set_value(true);
            chassis.turnToPoint(68,44.5,1250,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Motor::intakeF.move(127); Motor::intakeM.move(127); Motor::intakeU.move(127);
            // Misc::cdrift(50,50,1500);
            Misc::cdrift(55,55,300);
            Misc::cdrift(15,15,900);
            chassis.moveToPoint(20,46,1200,{.forwards=false,.maxSpeed=80,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Piston::hood.set_value(true);
            // Piston::loader.set_value(true);
            TaskHandler::antiJam2 = true;
            TaskHandler::antiJam3 = true;
            // pros::delay(1500);
            // TaskHandler::antiJam2 = false;
            // TaskHandler::antiJam3 = false;
        }
    }
    namespace Qual{
        void leftB(){
            TaskHandler::colorSort = true;
            Color::state = Color::colorVals::RED;
            chassis.setPose(47.28, -7.67, 237.5);
            TaskHandler::antiJam = true;
            Motor::intakeM.move(127); Motor::intakeF.move(127);
            chassis.moveToPose(9,-42,180,3000,{.forwards=true,.horizontalDrift=12,.lead=0.38,.maxSpeed=60,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            pros::delay(250);
            Piston::loader.set_value(true);
            Template::left();
        }
        void rightB(){
            TaskHandler::colorSort = true;
            Color::state = Color::colorVals::RED; // sort blue, red alliance
            chassis.setPose(47.28, 7.67, 302.5);
            // chassis.setPose(50, 18, 270);
            TaskHandler::antiJam = true;
            Motor::intakeU.move(127); Motor::intakeM.move(127); Motor::intakeF.move(127);
            // chassis.moveToPose(9,42.5,0,2100,{.forwards=true,.horizontalDrift=12,.lead=0.5,.maxSpeed=60,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPose(9,42.5,0,2100,{.forwards=true,.horizontalDrift=12,.lead=0.38,.maxSpeed=60,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            pros::delay(250);
            Piston::loader.set_value(true);
            Template::right();
        }
        void soloB(){
            Color::state = Color::colorVals::RED;
            Template::solo();
		}



        void leftR(){
            TaskHandler::colorSort = true;
            Color::state = Color::colorVals::BLUE;
            chassis.setPose(47.28, -7.67, 237.5);
            TaskHandler::antiJam = true;
            Motor::intakeM.move(127); Motor::intakeF.move(127);
            chassis.moveToPose(9,-42,180,3000,{.forwards=true,.horizontalDrift=12,.lead=0.38,.maxSpeed=60,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            pros::delay(250);
            Piston::loader.set_value(true);
            Template::left();
        }
        void rightR(){
            TaskHandler::colorSort = true;
            Color::state = Color::colorVals::BLUE; // sort blue, red alliance
            chassis.setPose(47.28, 7.67, 302.5);
            // chassis.setPose(50, 16, 270);
            TaskHandler::antiJam = true;
            Motor::intakeU.move(127); Motor::intakeM.move(127); Motor::intakeF.move(127);
            // chassis.moveToPose(8,42.5,0,2100,{.forwards=true,.horizontalDrift=12,.lead=0.6,.maxSpeed=60,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPose(9,42.5,0,2100,{.forwards=true,.horizontalDrift=12,.lead=0.38,.maxSpeed=60,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            pros::delay(250);
            Piston::loader.set_value(true);
            Template::right();
        }
        void soloR(){
            Color::state = Color::colorVals::BLUE;
            Template::solo();
		}
        void halfAWP(){ 
            TaskHandler::colorSort = false;
            chassis.setPose(47.28, -7.67, 237.5);
            TaskHandler::antiJam = true;
            Motor::intakeM.move(127); Motor::intakeF.move(127);
            chassis.moveToPose(9,-42,180,2100,{.forwards=true,.horizontalDrift=12,.lead=0.38,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            pros::delay(250);
            Piston::loader.set_value(true);
            
            chassis.moveToPoint(17,-22,2000,{.forwards=false,.maxSpeed=80,.minSpeed=10,.earlyExitRange=1});
            chassis.turnToPoint(0,-3.5,1500,{.forwards=true,.maxSpeed=80,.minSpeed=10,.earlyExitRange=1});
            // chassis.turnToHeading(315,1200,{.direction=genesis::AngularDirection::AUTO,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            TaskHandler::colorSort = false;
            Misc::cdrift(30,30,350);
            Piston::miniHood.set_value(true);
            TaskHandler::antiJam = true;
            TaskHandler::antiJam2 = true;
            TaskHandler::antiJam3 = true;
            Motor::intakeF.move(127); Motor::intakeM.move(127); Motor::intakeU.move(85);
            Misc::cdrift(25,25,450);
            pros::delay(600);
            Motor::intakeF.brake(); Motor::intakeM.brake(); Motor::intakeU.brake();
            chassis.moveToPoint(48,-49,2000,{.forwards=false,.maxSpeed=85,.minSpeed=10,.earlyExitRange=0});
            chassis.waitUntil(6);
            Piston::miniHood.set_value(false);
            chassis.waitUntilDone();
            Piston::loader.set_value(true);
            TaskHandler::antiJam2 = false;
            TaskHandler::antiJam3 = false;
            TaskHandler::colorSort = true;
            Piston::miniHood.set_value(false);
            chassis.turnToPoint(68,-46,1500,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1});
            // chassis.turnToHeading(90,1500,{.direction=genesis::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
            chassis.waitUntilDone();
            Motor::intakeF.move(127); Motor::intakeM.move(127); Motor::intakeU.move(127);
            Misc::cdrift(55,55,300);
            Misc::cdrift(15,15,900);
            Piston::loader.set_value(false);

            chassis.moveToPoint(25.5,-51.5,1200,{.forwards=false,.maxSpeed=75,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Misc::cdrift(-10,-10);
            Piston::hood.set_value(true);
            TaskHandler::antiJam2 = true;
            TaskHandler::antiJam3 = true;
            pros::delay(900);
            TaskHandler::antiJam2 = false;
            TaskHandler::antiJam3 = false;

            Piston::hood.set_value(false);
            Piston::loader.set_value(false);


            chassis.moveToPoint(40,8,3000,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.turnToHeading(302.5,1200,{.direction=genesis::AngularDirection::AUTO,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1});
            // chassis.turnToPoint(20,16,3000,{.forwards=true,.maxSpeed=75,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPose(5,42.5,0,2100,{.forwards=true,.horizontalDrift=12,.lead=0.55,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            pros::delay(250);
            Piston::loader.set_value(true);
            
            chassis.moveToPoint(16,22,2000,{.forwards=false,.maxSpeed=80,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Piston::loader.set_value(false);
            chassis.turnToPoint(0,3,1500,{.forwards=true,.maxSpeed=80,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Misc::cdrift(25,25,150);
            TaskHandler::antiJam = false;
            Motor::intakeF.move(-127); Motor::intakeM.move(-127); 
            Motor::intakeU.move(-127);
            // Misc::cdrift(25,25,300);
            // Motor::intakeU.brake();
            Misc::cdrift(25,25,1000);
            pros::delay(500);
            Motor::intakeF.brake(); Motor::intakeM.brake(); Motor::intakeU.brake();
            chassis.moveToPoint(40,-45,2000,{.forwards=false,.maxSpeed=127,.minSpeed=10,.earlyExitRange=0});
            chassis.turnToPoint(68,44,1500,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1});
            Piston::loader.set_value(true);
        }
        } // namespace Qual
        namespace Elim{
            void left(){

			}
			void right(){

			}
			void solo(){

			}
        } // namespace Elim
    namespace Skills{
        void main(){
            // From pov of red, left = 0;
            // TaskHandler::colorSort = false;
            // Color::state = Color::colorVals::BLUE;
            // chassis.setPose(-65, -18, 0);
            // TaskHandler::antiJam = true;
            // Motor::intakeM.move(127); Motor::intakeF.move(127); Motor::intakeU.move(127);
            // Piston::loader.set_value(true);

            // Misc::cdrift(60,62,2750);
            // Misc::cdrift(90,45,500);
            // Misc::cdrift(40,90,250);
            // chassis.turnToHeading(0,1200,{.direction=genesis::AngularDirection::AUTO,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1});
            // chassis.waitUntilDone();
            // pros::delay(250);
            // Misc::reset();

            TaskHandler::colorSort = false;
            Color::state = Color::colorVals::BLUE;
            TaskHandler::antiJam = true;
            chassis.setPose(47, -16, 180);
            Motor::intakeU.move(127); Motor::intakeM.move(127); Motor::intakeF.move(127);
            chassis.moveToPoint(48,-46,2000,{.forwards=true,.maxSpeed=80,.minSpeed=10,.earlyExitRange=0});
            chassis.waitUntilDone();
            Piston::loader.set_value(true);
            chassis.turnToPoint(68,-47.5,1250,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
            // chassis.turnToHeading(90,1500,{.direction=genesis::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
            chassis.waitUntilDone();
            Motor::intakeF.move(127); Motor::intakeM.move(127); Motor::intakeU.move(127);
            // Piston::loader.set_value(true);
            Misc::cdrift(65,65,375);
            Misc::cdrift(15,15,2200);
            chassis.moveToPoint(25.5,-47.5,1200,{.forwards=false,.maxSpeed=75,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Piston::hood.set_value(true);
            Misc::cdrift(-10,-10);
            TaskHandler::antiJam2 = true;
            TaskHandler::antiJam3 = true;
            pros::delay(2200);
            TaskHandler::antiJam2 = false;
            TaskHandler::antiJam3 = false;
            Motor::intakeF.brake(); Motor::intakeM.brake(); Motor::intakeU.brake();
            // Piston::hood.set_value(false);
            Piston::loader.set_value(false);
            Misc::cdrift(55,50,500);


            // 2
            chassis.swingToHeading(270,genesis::DriveSide::RIGHT,1500,{.direction=genesis::AngularDirection::CW_CLOCKWISE,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Piston::hood.set_value(false);
            chassis.moveToPoint(-36,-60,2000,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Piston::loader.set_value(true);
            chassis.moveToPoint(-48,-47,1500,{.forwards=true,.maxSpeed=80,.minSpeed=10,.earlyExitRange=2});
            
            chassis.turnToPoint(-68,-47,1500,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
            // chassis.turnToHeading(90,1500,{.direction=genesis::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
            chassis.waitUntilDone();
            Motor::intakeF.move(127); Motor::intakeM.move(127); Motor::intakeU.move(127);
            Misc::cdrift(65,65,375);
            Misc::cdrift(15,15,2000);

            Piston::loader.set_value(false);

            chassis.moveToPoint(-25.5,-50,1200,{.forwards=false,.maxSpeed=75,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Misc::cdrift(-10,-10);
            Piston::hood.set_value(true);
            TaskHandler::antiJam2 = true;
            TaskHandler::antiJam3 = true;
            pros::delay(2200);
            TaskHandler::antiJam2 = false;
            TaskHandler::antiJam3 = false;


            // 3 

            chassis.moveToPoint(-44,41,3000,{.forwards=true,.maxSpeed=95,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Piston::hood.set_value(false);
            Piston::loader.set_value(true);
            chassis.turnToPoint(-68,40,1500,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
            // chassis.turnToHeading(90,1500,{.direction=genesis::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
            chassis.waitUntilDone();
            Motor::intakeF.move(127); Motor::intakeM.move(127); Motor::intakeU.move(127);
            Misc::cdrift(65,65,550);
            Misc::cdrift(15,15,2000);

            Piston::loader.set_value(false);

            chassis.moveToPoint(-25.5,41,1200,{.forwards=false,.maxSpeed=75,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Misc::cdrift(-10,-10);
            Piston::hood.set_value(true);
            TaskHandler::antiJam2 = true;
            TaskHandler::antiJam3 = true;
            pros::delay(2200);
            TaskHandler::antiJam2 = false;
            TaskHandler::antiJam3 = false;
            Misc::cdrift(50,45,500);


            // 4
            chassis.swingToHeading(90,genesis::DriveSide::RIGHT,1500,{.direction=genesis::AngularDirection::CW_CLOCKWISE,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Piston::hood.set_value(false);
            chassis.moveToPoint(33,53.5,2000,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Piston::loader.set_value(true);
            chassis.moveToPoint(39,39.5,1500,{.forwards=true,.maxSpeed=80,.minSpeed=10,.earlyExitRange=1});

            
            chassis.turnToPoint(59,39.5,1500,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
            // chassis.turnToHeading(90,1500,{.direction=genesis::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
            chassis.waitUntilDone();
            // Piston::loader.set_value(true);
            Motor::intakeF.move(127); Motor::intakeM.move(127); Motor::intakeU.move(127);
            Misc::cdrift(65,65,400);
            Misc::cdrift(15,15,2200);

            Piston::loader.set_value(false);

            chassis.moveToPoint(16.5,43.5,1200,{.forwards=false,.maxSpeed=75,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Misc::cdrift(-10,-10);
            Piston::hood.set_value(true);
            TaskHandler::antiJam2 = true;
            TaskHandler::antiJam3 = true;
            pros::delay(2200);
            TaskHandler::antiJam2 = false;
            TaskHandler::antiJam3 = false;
            chassis.turnToHeading(90,1500,{.direction=genesis::AngularDirection::AUTO,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();

            Misc::cdrift(95,63,910);
            chassis.turnToHeading(180,1500,{.direction=genesis::AngularDirection::AUTO,.maxSpeed=90,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntilDone();
            Piston::loader.set_value(true);
            Misc::cdrift(70,74,1300);
            // chassis.moveToPose(68,24,180,3000,{.forwards=true,.horizontalDrift=12,.lead=0.3,.maxSpeed=75,.minSpeed=10,.earlyExitRange=1});

        }
    } // namespace Skills
} // namespace Auton

// <-------------------------------------------------------------- Driver Code ----------------------------------------------------------->
namespace Driver{
    bool b_loader = false, b_hood = false, b_minihood = false, b_intake = false, b_driver = false;
    int saberC = 0;
    double curveVal = 7.0;
    void joystick(){
        while(1){
            if(TaskHandler::driver) {
                // int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
                // int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
                int leftY = Misc::curve(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), curveVal, false); 
                int rightX = Misc::curve(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), curveVal, false);
                // leftMotors.move(leftY+rightX);
                // rightMotors.move(leftY-rightX);
                // leftMotors.move(leftY+rightX*0.9);
                // rightMotors.move(leftY-rightX*0.9);
                if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) { b_driver =! b_driver; }
                if(b_driver) {leftMotors.move(leftY*0.4+rightX*0.4); rightMotors.move(leftY*0.4-rightX*0.4); }
                else { leftMotors.move(leftY+rightX*0.9); rightMotors.move(leftY-rightX*0.9);}
            }
            pros::delay(Misc::DELAY);
        }
    }
    void intake(){
        while(1){
            if(TaskHandler::intake){
                // if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && TaskHandler::filled) { Motor::intakeF.move(127); Motor::intakeM.move(127); }
                // else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { Motor::intakeU.move(127); Motor::intakeF.move(127); Motor::intakeM.move(127); }
                if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { TaskHandler::intakeSpin = !TaskHandler::intakeSpin; }
                if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && TaskHandler::intakeSpin == false) { Motor::intakeU.move(127); Motor::intakeF.move(127); Motor::intakeM.move(127); }
                else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && TaskHandler::intakeSpin == true) { Motor::intakeU.brake(); Motor::intakeF.move(127); Motor::intakeM.move(127); }
                else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { TaskHandler::filled = false; Jam::counter1 = 0; Motor::intakeF.move(-127); Motor::intakeM.move(-127); Motor::intakeU.move(-127);}
                else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { TaskHandler::filled = false; Jam::counter1 = 0; Motor::intakeF.move(-60); Motor::intakeM.move(-80); Motor::intakeU.move(-80);}
                else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { TaskHandler::filled = false; Jam::counter1 = 0; Piston::hood.set_value(true); Motor::intakeF.move(127); Motor::intakeM.move(127); Motor::intakeU.move(127);}
                else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { TaskHandler::intakeSpin = false; TaskHandler::filled = false; Jam::counter1 = 0; Piston::miniHood.set_value(true); pros::delay(250); Motor::intakeF.move(127); Motor::intakeM.move(127); Motor::intakeU.move(127);}
                else{ Motor::intakeF.brake(); Motor::intakeM.brake(); Motor::intakeU.brake(); Piston::hood.set_value(false); Piston::miniHood.set_value(false); }
            }
            pros::delay(Misc::DELAY);
        }
    }
    void piston(){
        while(1){
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) { Misc::togglePiston(Piston::loader, b_loader); }
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { Misc::togglePiston(Piston::intake, b_intake); }
            pros::delay(Misc::DELAY);
        }
    }
} // namespace Driver

// <-------------------------------------------------------------- Auton ----------------------------------------------------------->
namespace Screen {
    void update() {
        controller.clear();  
        pros::delay(500);
        // controller.set_text(0, 0, "1: " + std::to_string(Sensor::o_colorSort.get_proximity()));
        // controller.set_text(0, 0, "1: " + std::to_string(Motor::intakeU.get_torque()));
        // controller.set_text(0, 0, "Mode: " + std::to_string(Motor::lbL.get_brake_mode()));
        // controller.set_text(0, 0, "Mode: " + Motor::lbL.get_brake_mode());
        // controller.set_text(0, 0, "Pos: " + std::to_string(Motor::lbR.get_position()));
        // controller.set_text(0, 0, "Pos: " + std::to_string(Sensor::d_filled.get_distance()));
        controller.set_text(0, 0, "State: " + TaskHandler::intakeSpin ? "Normal" : "Slow");

        // controller.set_text(0, 0, "Dist: " + std::to_string(Sensor::d_colorSort.get_distance()));
        // printf("%d\n",Sensor::d_colorSort.get_distance());
        // controller.set_text(0, 0, "Run Time: " + std::to_string(pros::millis() / 1000) + "s");
        // controller.set_text(0, 0, "X: " + std::to_string(chassis.getPose().x) + "\nY: " + std::to_string(chassis.getPose().y));
        // controller.set_text(1, 0, "Test Text 1");
        // controller.set_text(2, 0, "Test Text 2");
        pros::delay(500);
    }
}

using AutonFunc = void(*)();
std::vector<std::pair<std::string, AutonFunc>> autonRoutines = {
    {"Default Auton", Auton::Test::main},
    
    {"Blue Left Qual", Auton::Qual::leftB},
    {"Red Left Qual", Auton::Qual::leftR},

    {"Blue Right Qual", Auton::Qual::rightB},
    {"Red Right Qual", Auton::Qual::rightR},

    {"Blue Solo Qual", Auton::Qual::soloB},
    {"Red Solo Qual", Auton::Qual::soloR},

    {"Half AWP (Doesn't rly work)", Auton::Qual::halfAWP},

    {"Skills", Auton::Skills::main},
};


void autonSwitch() {
    if(TaskHandler::autonSelect) {    
        pros::delay(Misc::DELAY);
        if (Sensor::autonSwitch.get_new_press()) { autonState++; if (autonState == autonRoutines.size()) autonState = 0; }
    }
    pros::lcd::set_text(4, autonRoutines[autonState].first);
}

// LV_IMG_DECLARE(tdbg);
// LV_IMG_DECLARE(WORLDS_logo);
LV_IMG_DECLARE(WO_logo);
LV_IMG_DECLARE(Final_log);
LV_IMG_DECLARE(screen);
// lv_obj_t * sbg = lv_img_create(lv_scr_act());
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
    Sensor::o_colorSort.set_led_pwm(100);
    Sensor::o_colorSort.set_integration_time(5);
    Motor::intakeF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intakeM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intakeU.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // lv_init();
    // set_up();
    // pros::Task LVGL_upd([&]() { screen_upd(); });

	// lv_img_set_src(slogo, &Final_log);
	// lv_obj_set_pos(slogo, 20, 15);

    // lv_img_set_src(Slogo, &screen);
	// lv_obj_set_pos(Slogo, 0, 0);

    pros::Task screenTask([&]() {
        while (1) {
            // pros::lcd::print(3, "Pos: %d", Sensor::lbR.get_position());
            // pros::lcd::print(3, "Pos: %f", Motor::lb.get_position());
            // Misc::reset();
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
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
    // TaskHandler::antiJam = true;
    pros::Task antiJam([&](){ while(1) { Jam::antiJam(); pros::delay(Misc::DELAY); }});
    // Piston::loader.set_value(true);
    pros::Task colorTask(Misc::led);
    // TaskHandler::colorSort = true;
    TaskHandler::colorSort = false;
    // Color::state = Color::colorVals::RED;
    pros::Task sorterC([&](){ while(1) { Color::colorSort(Color::state);  pros::delay(5); }});
    // Motor::intakeF.move(127);
    // Motor::intakeM.move(127);
    // Motor::intakeU.move(127);
    // pros::delay(1000000);
    // chassis.turnToHeading(90,1000);
    // chassis.turnToHeading(180,1000);
    // chassis.turnToHeading(270,1000);
    // chassis.turnToHeading(0,1000);
    // chassis.moveToPoint(0, 24, 100000);
    // chassis.turnToHeading(180,1000);
    // chassis.moveToPoint(0, 0, 100000);

    // Auton::Blue::Qual::right(); // jammed while outtaking the rings on lower goal, matchloader hardware problem
    // Auton::Qual::soloR();
    // Auton::Qual::halfAWP();
    // Auton::Skills::main();
    // pros::delay(1000000);
    // Color::state = Color::colorVals::BLUE;
    // TaskHandler::antiJam = true;
    // pros::Task sorterC([&](){ while(1) { Color::colorSort(Color::state);  pros::delay(5); }});
    // pros::Task antiJam([&](){ while(1) { Jam::antiJam(); pros::delay(Misc::DELAY); }});
    // Sensor::o_colorSort.set_led_pwm(100);
    // Sensor::o_colorSort.set_integration_time(5);
    
    (autonState < autonRoutines.size()) ? autonRoutines[autonState].second() : Auton::Test::main();
}

// <--------------------------------------------------------------- Driver --------------------------------------------------------------->
void opcontrol() {
    pros::lcd::shutdown();
    lv_init();
    lv_img_set_src(Slogo, &screen);
	lv_obj_set_pos(Slogo, 0, 0);

    pros::Task intakeTask(Driver::intake);
    pros::Task driverTask(Driver::joystick);
    pros::Task pistonTask(Driver::piston);
    // pros::Task hangTask(Driver::hang);
    TaskHandler::colorSort = false;
    TaskHandler::antiJam = false;
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST); rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    Motor::intakeF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intakeM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intakeU.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // Lift::setState(0);
    // Piston::tipper.set_value(false);
    // pros::lcd::clear();
    // lv_img_set_src(sbg, &tdbg);
	// lv_obj_set_pos(sbg,0,0);
	// lv_img_set_src(slogo, &logo);
	// lv_obj_set_pos(slogo,105,-15);
    Piston::hood.set_value(false);
    Piston::miniHood.set_value(false);
    while(1) {
        // Sensor::o_colorSort.set_led_pwm(100);
        // Color::colorSort(Color::colorVals::BLUE);
        
        // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        //     TaskHandler::isDriver = false;
        //     Misc::cdrift(30,30,230,true);
        //     TaskHandler::isDriver = true;
        // }
        // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) Driver::release();
        // right = goal tip
        // down = left doinker
        // b = right do
        // 
        pros::delay(Misc::DELAY);
    }
}