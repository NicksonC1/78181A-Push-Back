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

genesis::ControllerSettings linearController (7.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              6, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

genesis::ControllerSettings angularController(2.75, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              17.5, // derivative gain (kD) 
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
    void reset() {
        constexpr double field = 144.0;
        constexpr double halfField = field / 2.0;
        constexpr double offsetF = 10.0;
        constexpr double offsetR = -4.0;

        double heading = s_imu.get_heading();
        double theta = heading * M_PI / 180.0;

        double d_front = Sensor::d_front.get_distance() / 25.4;
        double d_left = Sensor::d_left.get_distance() / 25.4;

        double x = (d_left - halfField) - (offsetR * cos(theta)) - (offsetF * sin(theta));
        double y = (halfField - d_front) - (offsetF * cos(theta)) + (offsetR * sin(theta));

        chassis.setPose(x, y, heading);

        printf("Pose -> X: %.2f, Y: %.2f, Heading: %.2f\n", x, y, heading);
    }
    void reset2(int sign) {
        constexpr double offsetR = -13.0;
        double d_left = Sensor::d_left.get_distance() / 25.4;
        double x = chassis.getPose().x;
        double heading = chassis.getPose().theta;
        double y = sign * ((72.0 - d_left) + offsetR);
        chassis.setPose(x, y, heading);
    }

    void resetB1() {
        constexpr double field = 144.0;
        constexpr double halfField = field / 2.0;

        constexpr double offsetF = 3.0;  // forward from robot center
        constexpr double offsetR = 6.0;  // right from robot center

        // Distance sensors (inches)
        double d_back  = Sensor::d_front.get_distance() / 25.4;
        double d_right = Sensor::d_left.get_distance() / 25.4;

        // Heading is assumed to be 0 degrees (robot squared to walls)
        double heading = s_imu.get_heading();

        double x = (halfField - d_right) - offsetR;
        double y = (halfField - d_back)  - offsetF;

        chassis.setPose(x, y, heading);

        printf("Pose -> X: %.2f, Y: %.2f, Heading: %.2f\n", x, y, heading);
    }


    void resetB2() {
        constexpr double field = 144.0;
        constexpr double halfField = field / 2.0;

        constexpr double offsetF = -3.0; // sensor behind center
        constexpr double offsetR = -6.0; // sensor left of center

        // Distance sensors (inches)
        double d_back  = Sensor::d_front.get_distance() / 25.4;
        double d_right = Sensor::d_left.get_distance() / 25.4;

        // Assumed squared to walls
        double heading = s_imu.get_heading();

        double x = (-halfField + d_right) - offsetR;
        double y = (-halfField + d_back)  - offsetF;

        chassis.setPose(x, y, heading);

        printf("Pose -> X: %.2f, Y: %.2f, Heading: %.2f\n", x, y, heading);
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
                // Piston::miniHood.set_value(true);
                pros::delay(200);
                extend_once = true;
            }
            else if(input == colorVals::BLUE && isBlue(Sensor::o_colorSort.get_hue(),bLow,bHigh) && withinProx(Sensor::o_colorSort.get_proximity(),minProx)){
                // Piston::miniHood.set_value(true);
                pros::delay(200);
                extend_once = true;
            }
            // else { Piston::miniHood.set_value(false); }
            extend_once = false;
        }
    }
} // namespace Color

// <-------------------------------------------------------------- Auto Routes ----------------------------------------------------------->
namespace Auton{
    int state = 0;
    namespace Test{
        void main() { 
            Misc::cdrift(55,55,550);
        }
    } // namespace Test
    namespace Template{
        void left(){

        }
        void right(){
            
        }  
        void solo(){          
            
        }
        void leftseven(){
            chassis.setPose(48,-15,270);
            Piston::hook.set_value(true);
            Motor::intakeF.move(127); 
            chassis.moveToPoint(22,-22.5,1750,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntil(6);
            Piston::loader.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToPoint(66,-42,400,{.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
            // chassis.moveToPoint(63,-45,400,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPose(65,-44.5,90,1250,{.forwards=true,.horizontalDrift=8,.lead=0.41,.maxSpeed=127,.minSpeed=0,.earlyExitRange=0});
            chassis.waitUntilDone();
            Misc::cdrift(45,45,350);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(45,45,590);
            chassis.moveToPoint(20,-46,1250,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
            chassis.waitUntilDone();
            Motor::intakeF.move(-127); 
            Motor::intakeU.move(-127);
            pros::delay(40);
            Motor::intakeF.move(127); 
            Motor::intakeU.move(127);
            Misc::cdrift(-20,-20,1500);
            Motor::intakeU.brake();
            Piston::loader.set_value(false);

            chassis.moveToPoint(42,-32,1250,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Piston::hook.set_value(false);
            chassis.moveToPoint(14,-36,1750,{.forwards=false,.maxSpeed=85,.minSpeed=0,.earlyExitRange=0});
            chassis.waitUntilDone();
            Misc::cdrift(0,-15);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        }
        void rightseven(){
            chassis.setPose(48,15,270);
            Piston::hook.set_value(true);
            Motor::intakeF.move(127); 
            chassis.moveToPoint(22,22.5,1750,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntil(7.1);
            Piston::loader.set_value(true);
            chassis.waitUntilDone();
            chassis.turnToPoint(58,41,400,{.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
            // chassis.moveToPoint(63,-45,400,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPose(65,47,90,1500,{.forwards=true,.horizontalDrift=8,.lead=0.44,.maxSpeed=85,.minSpeed=0,.earlyExitRange=0});
            chassis.waitUntilDone();
            Misc::cdrift(30,30,350);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(45,45,590);
            chassis.moveToPoint(20,47.5,1250,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
            chassis.waitUntilDone();
            Motor::intakeF.move(-127); 
            Motor::intakeU.move(-127);
            pros::delay(100);
            Motor::intakeF.move(127);
            Motor::intakeU.move(127);
            Misc::cdrift(-20,-20,1500);
            Motor::intakeU.brake();
            Piston::loader.set_value(false);

            chassis.moveToPoint(42,63,1250,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Piston::hook.set_value(false);
            chassis.turnToHeading(95,800,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=2});
            chassis.moveToPoint(11,63,1750,{.forwards=false,.maxSpeed=85,.minSpeed=0,.earlyExitRange=0});
            chassis.waitUntilDone();
            Misc::cdrift(0,-15);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

            // chassis.moveToPoint(36,32,1250,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=2});
            // chassis.waitUntilDone();
            // Piston::hook.set_value(false);
            // chassis.turnToHeading(280,1000,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=2});
            // chassis.moveToPoint(10,35,1750,{.forwards=true,.maxSpeed=85,.minSpeed=0,.earlyExitRange=0});
            // chassis.waitUntilDone();
            // Misc::cdrift(0,15);
            // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        }
        void rushAWP(){
            chassis.setPose(48,-15,270);
            Piston::hook.set_value(true);
            Motor::intakeF.move(127); 

            chassis.moveToPose(8.5,-46.5,180,1250,{.forwards=true,.horizontalDrift=11,.lead=0.44,.maxSpeed=127,.minSpeed=0,.earlyExitRange=1});
            chassis.waitUntilDone();
            
            chassis.moveToPoint(29,-36,1250,{.forwards=false,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
            chassis.moveToPoint(39,-48.5,1250,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=1});
            chassis.turnToHeading(90,650,{.maxSpeed=127,.minSpeed=0,.earlyExitRange=1});
            
            // chassis.moveToPoint(20,-50,1250,{.forwards=false,.maxSpeed=75,.minSpeed=10,.earlyExitRange=3});

            // chassis.moveToPose(25,-47.5,90,1500,{.forwards=false,.horizontalDrift=8,.lead=0.3,.maxSpeed=127,.minSpeed=0,.earlyExitRange=1});
            chassis.waitUntilDone();
            Misc::cdrift(-60,-60,450);
            Motor::intakeU.move(127);
            Misc::cdrift(-20,-20,500);
            Piston::loader.set_value(true);
            Misc::cdrift(-20,-20,800);
            Motor::intakeU.brake();
            chassis.moveToPoint(52.5,-49.5,1250,{.forwards=true,.maxSpeed=95,.minSpeed=0,.earlyExitRange=3});
            chassis.waitUntilDone();
            Misc::cdrift(35,35,350);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,590);

            chassis.moveToPoint(9,-12,1250,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
            chassis.waitUntil(5);
            Piston::loader.set_value(false);
            chassis.waitUntilDone();
            // Piston::loader.set_value(false);
            // Misc::cdrift(-40,-40,150);
            Motor::intakeU.move(55);
            Piston::middle.set_value(true);
            Misc::cdrift(-30,-30,900);
            Misc::cdrift(20,85,350);
            Piston::middle.set_value(false);
            Motor::intakeU.brake();

            chassis.moveToPoint(20,22,1250,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.waitUntil(19);

            // chassis.waitUntilDone();

            //new
            Piston::loader.set_value(true);
            chassis.waitUntilDone();
            chassis.moveToPoint(54,42,200,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
            // chassis.turnToPoint(56,41,400,{.maxSpeed=90,.minSpeed=10,.earlyExitRange=0});
            // chassis.moveToPoint(63,-45,400,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=1});
            chassis.moveToPose(60,49,90,1250,{.forwards=true,.horizontalDrift=7,.lead=0.54,.maxSpeed=127,.minSpeed=0,.earlyExitRange=0});
            chassis.waitUntilDone();
            Misc::cdrift(30,30,250);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(45,45,400);
            chassis.moveToPoint(20,49.5,950,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
            chassis.waitUntilDone();
            Motor::intakeU.move(127);
            Misc::cdrift(-20,-20,1500);
            Motor::intakeU.brake();
            Piston::loader.set_value(false);
        }

        void safeAWP(){
            chassis.setPose(50,8,180);
            Motor::intakeF.move(127);
            Piston::hook.set_value(true);
            Misc::cdrift(50,50,500);
            pros::delay(150);

            chassis.moveToPoint(48,46.5,1220,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=1.5});
            chassis.waitUntilDone();
            // chassis.turnToHeading(270,1200,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=1});
            // chassis.waitUntilDone();
            Piston::loader.set_value(true);
            pros::delay(100);
            chassis.turnToPoint(65,49,750,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=1.5});
            chassis.waitUntilDone();
            
            Misc::cdrift(45,45,550);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,600);

            chassis.moveToPoint(30,49.5,950,{.forwards=false,.maxSpeed=85,.minSpeed=0,.earlyExitRange=3});
            chassis.waitUntilDone();
            Motor::intakeF.move(-127); 
            Motor::intakeU.move(-127);
            pros::delay(40);
            Motor::intakeF.move(127); 
            Motor::intakeU.move(127);
            Misc::cdrift(-20,-20,500);
            Piston::loader.set_value(false);
            Misc::cdrift(-20,-20,600);
            Motor::intakeU.brake();
            Misc::cdrift(90,45,200);
            chassis.moveToPoint(26,26,1250,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=3});
            chassis.waitUntilDone();
            Misc::cdrift(-10,80,150);
            chassis.moveToPoint(24,-25,1150,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=1});
            chassis.waitUntil(17);
            Piston::loader.set_value(true);
            chassis.waitUntilDone();
            Piston::loader.set_value(false);


            chassis.moveToPoint(10,-6.2,750,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
            chassis.waitUntilDone();
            Motor::intakeF.move(-127);
            Motor::intakeU.move(-127);
            pros::delay(20);
            Motor::intakeF.move(127);
            Motor::intakeU.move(55);
            Piston::middle.set_value(true);
            // Piston::loader.set_value(false);
            // Misc::cdrift(-40,-40,150);
            // Motor::intakeF.move(127);
            // Motor::intakeU.move(55);
            // Piston::middle.set_value(true);
            Misc::cdrift(-10,-10,600);
            // Misc::cdrift(20,85,350);
            Piston::middle.set_value(false);
            Motor::intakeU.brake();


            chassis.moveToPoint(48,-46.2,1100,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=1});
            chassis.waitUntilDone();
            // chassis.turnToHeading(270,1200,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=1});
            // chassis.waitUntilDone();
            Piston::loader.set_value(true);
            pros::delay(100);
            chassis.turnToPoint(65,-47.2,700,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=2});
            chassis.waitUntilDone();
            
            Misc::cdrift(50,50,550);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,590);

            chassis.moveToPoint(30,-48,950,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
            chassis.waitUntilDone();
            Motor::intakeF.move(-127); 
            Motor::intakeU.move(-127);
            pros::delay(40);
            Motor::intakeF.move(127); 
            Motor::intakeU.move(127);
            Misc::cdrift(-20,-20,500);
            Piston::loader.set_value(false);
            Misc::cdrift(-20,-20,500);
            // Motor::intakeU.brake();
        }
        void leftMiddle(){
            chassis.setPose(48,-15,270);
            Piston::hook.set_value(true);
            Motor::intakeF.move(127); 

            chassis.moveToPose(8.5,-46.5,180,1250,{.forwards=true,.horizontalDrift=11,.lead=0.44,.maxSpeed=127,.minSpeed=0,.earlyExitRange=1});
            chassis.waitUntilDone();
            
            chassis.moveToPoint(29,-36,1250,{.forwards=false,.maxSpeed=127,.minSpeed=20,.earlyExitRange=2});
            chassis.moveToPoint(39,-48.5,1250,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=1});
            chassis.turnToHeading(90,650,{.maxSpeed=127,.minSpeed=0,.earlyExitRange=1});
            
            // chassis.moveToPoint(20,-50,1250,{.forwards=false,.maxSpeed=75,.minSpeed=10,.earlyExitRange=3});

            // chassis.moveToPose(25,-47.5,90,1500,{.forwards=false,.horizontalDrift=8,.lead=0.3,.maxSpeed=127,.minSpeed=0,.earlyExitRange=1});
            chassis.waitUntilDone();
            Misc::cdrift(-60,-60,450);
            Motor::intakeU.move(127);
            Misc::cdrift(-20,-20,500);
            Piston::loader.set_value(true);
            Misc::cdrift(-20,-20,800);
            Motor::intakeU.brake();
            chassis.moveToPoint(52.5,-49.5,1250,{.forwards=true,.maxSpeed=95,.minSpeed=0,.earlyExitRange=3});
            chassis.waitUntilDone();
            Misc::cdrift(35,35,350);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,590);

            chassis.moveToPoint(9,-12,1250,{.forwards=false,.maxSpeed=127,.minSpeed=0,.earlyExitRange=3});
            chassis.waitUntil(5);
            Piston::loader.set_value(false);
            chassis.waitUntilDone();
            // Piston::loader.set_value(false);
            // Misc::cdrift(-40,-40,150);
            Motor::intakeU.move(55);
            Piston::middle.set_value(true);
            Misc::cdrift(-30,-30,900);
            Misc::cdrift(20,85,350);
            Piston::middle.set_value(false);
            Motor::intakeU.brake();
            
            chassis.moveToPoint(38,-25,1250,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Piston::hook.set_value(false);
            chassis.turnToHeading(90,800,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=2});
            chassis.moveToPoint(14,-28,1750,{.forwards=false,.maxSpeed=85,.minSpeed=0,.earlyExitRange=0});
            chassis.waitUntilDone();
            Misc::cdrift(0,-15);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

        }
    }

    namespace Qual{
        void leftB(){
            TaskHandler::colorSort = false;
            Color::state = Color::colorVals::RED;
            Template::left();
        }
        void rightB(){
            TaskHandler::colorSort = false;
            Color::state = Color::colorVals::RED;
            Template::right();
        }
        void soloB(){
            TaskHandler::colorSort = false;
            Color::state = Color::colorVals::RED;
            Template::solo();
		}
        void leftR(){
            TaskHandler::colorSort = false;
            Color::state = Color::colorVals::BLUE;
            Template::left();
        }
        void rightR(){
            TaskHandler::colorSort = false;
            Color::state = Color::colorVals::BLUE;
            Template::right();
        }
        void soloR(){
            TaskHandler::colorSort = false;
            Color::state = Color::colorVals::BLUE;
            Template::solo();
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
            chassis.setPose(-50,17.5,0);
            Motor::intakeF.move(127);
            Piston::hook.set_value(true);
            chassis.moveToPoint(-48,46,1250,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});
            chassis.waitUntilDone();
            // chassis.turnToHeading(270,1200,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=1});
            // chassis.waitUntilDone();
            Piston::loader.set_value(true);
            pros::delay(100);
            chassis.turnToPoint(-65,47,1250,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});
            chassis.waitUntilDone();
            
            Misc::cdrift(35,35,700);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,800);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,800);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,800);

            // chassis.moveToPoint(-20,-49,1250,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
            // chassis.waitUntilDone();
            // Motor::intakeU.move(127);
            // Misc::cdrift(-20,-20,2500);
            // Motor::intakeU.brake();
            // Piston::loader.set_value(false);

            chassis.moveToPoint(-42,64,1250,{.forwards=false,.maxSpeed=90,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            chassis.turnToHeading(270,800,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});
            chassis.waitUntilDone();
            Piston::loader.set_value(false);
            // Misc::reset2(-1);
            chassis.moveToPoint(30,64,1250,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=1});
            chassis.waitUntilDone();
            // chassis.turnToHeading(270,800,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});
            // chassis.waitUntilDone();
            // pros::delay(500);
            // Misc::resetB1();
            // pros::delay(300);
            // pros::delay(1000000000);
            chassis.moveToPoint(42,51,1250,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});
            chassis.waitUntilDone();
            // pros::delay(1000000000);
            chassis.turnToHeading(90,800,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});
            chassis.moveToPoint(20,49.5,1250,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});
            chassis.waitUntilDone();
            Motor::intakeF.move(-127);
            Motor::intakeU.move(-127);
            pros::delay(150);
            Motor::intakeF.move(127);
            Motor::intakeU.move(127);
            Misc::cdrift(-20,-20,1000);
            Piston::loader.set_value(true);
            Misc::cdrift(-20,-20,1500);
            Motor::intakeU.brake();

            chassis.moveToPoint(45,49,1250,{.forwards=true,.maxSpeed=95,.minSpeed=0,.earlyExitRange=3});
            chassis.waitUntilDone();
            Misc::cdrift(35,35,700);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,800);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,800);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,800);
            // Misc::cdrift(-20,-20,200);
            // Misc::cdrift(35,35,800);

            chassis.moveToPoint(25,49,1250,{.forwards=false,.maxSpeed=85,.minSpeed=0,.earlyExitRange=1});
            chassis.waitUntilDone();
            Motor::intakeF.move(-127);
            Motor::intakeU.move(-127);
            pros::delay(150);
            Motor::intakeF.move(127);
            Motor::intakeU.move(127);
            Misc::cdrift(-20,-20,1000);
            Piston::loader.set_value(false);
            Misc::cdrift(-20,-20,1500);
            Motor::intakeU.brake();

            Misc::cdrift(90,45,200);
            // chassis.moveToPoint(20,24,1500,{.forwards=true,.maxSpeed=70,.minSpeed=10,.earlyExitRange=3});
            // chassis.waitUntilDone();
            // Misc::cdrift(-10,80,150);
            // chassis.moveToPoint(19,-25,2500,{.forwards=true,.maxSpeed=55,.minSpeed=0,.earlyExitRange=1});
            // chassis.waitUntilDone();
            pros::delay(200);

            chassis.moveToPoint(17,26,1500,{.forwards=true,.maxSpeed=70,.minSpeed=10,.earlyExitRange=3});
            chassis.waitUntilDone();
            Misc::cdrift(-10,80,150);
            chassis.moveToPoint(17.5,-25,2500,{.forwards=true,.maxSpeed=55,.minSpeed=0,.earlyExitRange=1});
            chassis.waitUntilDone();
            pros::delay(200);
            // chassis.waitUntil(17);
            // Piston::loader.set_value(true);
            // chassis.waitUntilDone();
            // Piston::loader.set_value(false);


            chassis.moveToPoint(9,-13,750,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=3});
            chassis.waitUntilDone();
            Motor::intakeF.move(-127);
            Motor::intakeU.move(-127);
            pros::delay(150);
            Motor::intakeF.move(127);
            Motor::intakeU.move(55);
            Piston::middle.set_value(true);
            Misc::cdrift(-30,-30,2000);
            Piston::middle.set_value(false);
            Motor::intakeU.brake();


            chassis.moveToPoint(37,-45,1100,{.forwards=true,.maxSpeed=127,.minSpeed=0,.earlyExitRange=1});
            chassis.waitUntilDone();
            // chassis.turnToHeading(270,1200,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=1});
            // chassis.waitUntilDone();
            Piston::loader.set_value(true);
            pros::delay(100);
            chassis.turnToPoint(65,-45,700,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=2});
            chassis.waitUntilDone();
            
            Misc::cdrift(50,50,550);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,590);
            Misc::cdrift(50,50,550);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,590);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,590);

            chassis.moveToPoint(36,-62,1250,{.forwards=false,.maxSpeed=90,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            Piston::loader.set_value(false);
            chassis.turnToHeading(268,800,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});
            chassis.waitUntilDone();
            // Piston::loader.set_value(false);
            // Misc::reset2(-1);
            chassis.moveToPoint(-24,-63.5,1250,{.forwards=true,.maxSpeed=90,.minSpeed=10,.earlyExitRange=2});
            chassis.waitUntilDone();
            // pros::delay(500);
            // Misc::resetB2();
            // pros::delay(100);
            chassis.moveToPoint(-51,-49,1250,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});
            chassis.turnToHeading(270,800,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});
            chassis.moveToPoint(-20,-49,1250,{.forwards=false,.maxSpeed=85,.minSpeed=0,.earlyExitRange=2});
            chassis.waitUntilDone();
            Motor::intakeF.move(-127);
            Motor::intakeU.move(-127);
            pros::delay(150);
            Motor::intakeF.move(127);
            Motor::intakeU.move(127);
            Misc::cdrift(-20,-20,1000);
            Piston::loader.set_value(true);
            Misc::cdrift(-20,-20,1500);
            Motor::intakeU.brake();

            chassis.moveToPoint(-55,-49,1500,{.forwards=true,.maxSpeed=70,.minSpeed=0,.earlyExitRange=3});
            chassis.waitUntilDone();
            Misc::cdrift(35,35,700);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,800);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,800);
            Misc::cdrift(-20,-20,200);
            Misc::cdrift(35,35,800);

            chassis.moveToPoint(-32,-49,1250,{.forwards=false,.maxSpeed=85,.minSpeed=0,.earlyExitRange=1});
            chassis.waitUntilDone();
            Motor::intakeF.move(-127);
            Motor::intakeU.move(-127);
            pros::delay(150);
            Motor::intakeF.move(127);
            Motor::intakeU.move(127);
            Misc::cdrift(-20,-20,1000);
            Piston::loader.set_value(false);
            Misc::cdrift(-20,-20,1500);
            Motor::intakeU.brake();
            Misc::cdrift(80,20,500);
            // chassis.moveToPoint(-48,-53,1250,{.forwards=true,.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});

            chassis.moveToPoint(-71,-28,1500,{.forwards=false,.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});
            chassis.turnToHeading(351,800,{.maxSpeed=90,.minSpeed=0,.earlyExitRange=0});
            chassis.waitUntilDone();
            // Misc::cdrift(65,65,2500);
            // Misc::park(65,65,2500);
            // Misc::park(65,65,2000);
            // Misc::cdrift(40,40,700);
            Motor::intakeF.move(-127);
            Misc::cdrift(30,33,1300);
            Piston::loader.set_value(true);
            Misc::cdrift(-20,-20,150);
            pros::delay(400);
            Misc::park(70,70,1000);
            Misc::cdrift(25,25);
        }
    } // namespace Skills
} // namespace Auton

// <-------------------------------------------------------------- Driver Code ----------------------------------------------------------->
namespace Driver{
    bool b_loader = false, b_clamp = false, b_aligner = false, b_hook = false, b_driver = false, b_middle = false;
    int saberC = 0;
    double curveVal = 7.0;
    void joystick(){
        while(1){
            if(TaskHandler::driver) {
                int leftY = Misc::curve(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), curveVal, false); 
                int rightX = Misc::curve(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), curveVal, false);
                // leftMotors.move(leftY+rightX*0.9);
                // rightMotors.move(leftY-rightX*0.9);
                if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) { b_driver =! b_driver; }
                if(b_driver) {leftMotors.move(leftY*0.45+rightX*0.45); rightMotors.move(leftY*0.45-rightX*0.45); }
                else { leftMotors.move(leftY+rightX*0.9); rightMotors.move(leftY-rightX*0.9);}
            }
            pros::delay(Misc::DELAY);
        }
    }
    void intake(){
        while(1){
            if(TaskHandler::intake){
                if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { Motor::intakeF.move(127); }
                else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { Motor::intakeF.move(127); Motor::intakeU.move(127); }
                else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { Motor::intakeF.move(-127); Motor::intakeU.move(-127); }
                else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) { Piston::middle.set_value(true); Motor::intakeF.move(127); Motor::intakeU.move(50);}
                else{ Motor::intakeF.brake();  Motor::intakeU.brake(); Piston::middle.set_value(false); }
            }
            pros::delay(Misc::DELAY);
        }
    }
    void piston(){
        while(1){
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) { Misc::togglePiston(Piston::loader, b_loader); }
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { Misc::togglePiston(Piston::hook, b_hook); }
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
    {"Default Auton", Auton::Template::safeAWP},
    
    {"Left", Auton::Template::leftseven},
    {"Right", Auton::Template::rightseven},
    {"Solo", Auton::Template::safeAWP},

    {"Left Middle", Auton::Template::leftMiddle},

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
LV_IMG_DECLARE(sixseven);
LV_IMG_DECLARE(gay);
// lv_obj_t * sbg = lv_img_create(lv_scr_act());
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
    Sensor::o_colorSort.set_led_pwm(100);
    Sensor::o_colorSort.set_integration_time(5);
    Sensor::o_crossed.set_led_pwm(100);
    Sensor::o_crossed.set_integration_time(5);
    Motor::intakeF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intakeU.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

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
            // Misc::resetB1();
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
    // // TaskHandler::antiJam = true;
    // pros::Task antiJam([&](){ while(1) { Jam::antiJam(); pros::delay(Misc::DELAY); }});
    // // Piston::loader.set_value(true);
    // pros::Task colorTask(Misc::led);
    // // TaskHandler::colorSort = true;
    // TaskHandler::colorSort = false;
    // // Color::state = Color::colorVals::RED;
    // pros::Task sorterC([&](){ while(1) { Color::colorSort(Color::state);  pros::delay(5); }});
    // // Motor::intakeF.move(127);
    // // Motor::intakeM.move(127);
    // // Motor::intakeU.move(127);
    // // pros::delay(1000000);
    // chassis.turnToHeading(90,1000);
    // chassis.turnToHeading(180,1000);
    // chassis.turnToHeading(270,1000);
    // chassis.turnToHeading(0,1000);
    // chassis.moveToPoint(0, 24, 1500);
    // chassis.turnToHeading(180,1000);
    // chassis.moveToPoint(0, 0, 1500);
    // chassis.turnToHeading(0,1000);
    // chassis.moveToPoint(-24,24,1750,{.forwards=true,.maxSpeed=127,.minSpeed=10,.earlyExitRange=0});
    // Auton::Qual::leftB();
    // Auton::Template::leftseven();
    // Auton::Template::rightseven();
    // Auton::Template::rushAWP();
    // Auton::Template::safeAWP();
    // Auton::Skills::main();
    // Motor::intakeF.move(-127);
    // Misc::cdrift(30,33,1300);
    // Piston::loader.set_value(true);
    // Misc::cdrift(-20,-20,150);
    // pros::delay(400);
    // Misc::park(70,70,1000);
    // Misc::cdrift(25,25);
    // pros::delay(1000000);
    

    // Auton::Blue::Qual::right(); // jammed while outtaking the rings on lower goal, matchloader hardware problem
    // Auton::Qual::soloR();
    // Auton::Qual::halfAWP();
    // Auton::Skills::main();
    // Auton::Qual::soloR();
    // Piston::miniHood.set_value(true);
    // Auton::Qual::leftB();
    // Auton::Qual::leftR();
    // chassis.turnToHeading(90,100000);
    // Auton::Skills::main();
    // // Misc::park(65,65,2000);
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
    // pros::lcd::shutdown();
    // lv_init();
    // lv_img_set_src(sKiss, &gay);
	// lv_obj_set_pos(sKiss, 0, 0);

    // lv_img_set_src(sixSeven, &sixseven);
	// lv_obj_set_pos(sixSeven, 140, 0);

    pros::Task intakeTask(Driver::intake);
    pros::Task driverTask(Driver::joystick);
    pros::Task pistonTask(Driver::piston);
    TaskHandler::colorSort = false;
    TaskHandler::antiJam = false;
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST); rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    Motor::intakeF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);  Motor::intakeU.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // Lift::setState(0);
    // Piston::tipper.set_value(false);
    // pros::lcd::clear();
    // lv_img_set_src(sbg, &tdbg);
	// lv_obj_set_pos(sbg,0,0);
	// lv_img_set_src(slogo, &logo);
	// lv_obj_set_pos(slogo,105,-15);
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