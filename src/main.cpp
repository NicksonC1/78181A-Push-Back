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

// genesis::ControllerSettings linearController (8, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               6, // derivative gain (kD)
//                                               0, // anti windup
//                                               1, // small error range, in inches
//                                               100, // small error range timeout, in milliseconds
//                                               3, // large error range, in inches
//                                               500, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

// genesis::ControllerSettings angularController(2.85, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               16, // derivative gain (kD) 
//                                               0, // anti windup
//                                               1, // small error range, in inches
//                                               100, // small error range timeout, in milliseconds
//                                               3, // large error range, in inches
//                                               500, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

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

genesis::ControllerSettings angularController(0, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD) 
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
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
    void reset() {
        constexpr double field = 144.0;
        constexpr double halfField = field / 2.0;
        constexpr double offsetF = 10.0;
        constexpr double offsetR = -4.0;

        double heading = s_imu.get_heading();
        double theta = heading * M_PI / 180.0;

        double d_front = Sensor::d_front.get_distance() / 25.4;
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
        }
    } // namespace Skills
} // namespace Auton

// <-------------------------------------------------------------- Driver Code ----------------------------------------------------------->
namespace Driver{
    bool b_loader = false, b_clamp = false, b_park = false, b_driver = false, b_goal = false;
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
                if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { Motor::intakeF.move(127); }
                else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { Motor::intakeF.move(127); Motor::intakeU.move(127); }
                else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { Motor::intakeF.move(-127); Motor::intakeU.move(-127); }
                else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { Misc::togglePiston(Piston::goal,b_goal); Motor::intakeF.move(127); }
                else{ Motor::intakeF.brake();  Motor::intakeU.brake(); }
            }
            pros::delay(Misc::DELAY);
        }
    }
    void piston(){
        while(1){
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) { Misc::togglePiston(Piston::loader, b_loader); }
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { Misc::togglePiston(Piston::park, b_park); }
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { Misc::togglePiston(Piston::clamp, b_clamp); }
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
    Motor::intakeF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);  Motor::intakeU.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

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
    // Auton::Qual::soloR();
    // Piston::miniHood.set_value(true);
    // Auton::Qual::leftB();
    // Auton::Qual::leftR();
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