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

std::vector<std::pair<float, float>> points;

namespace TaskHandler {
    bool antiJam = false;
    bool autonSelect = true;
    bool colorSort = true;
    bool isShared = !colorSort;
    bool lbD = true;
    bool autoIntake = false;
    bool autoIntake1 = false;
    bool autoIntake2 = false;
    bool autoIntake3 = false;
    int sharedSpeed = 127;
    bool isDriver = true;
    bool dIntake = true;
} // namespace TaskHandler

// <------------------------------------------------------------ Miscellaneous ------------------------------------------------------------>
namespace Misc{
    constexpr int DELAY = 10;
    constexpr double X = -7.0;
    pros::motor_brake_mode_e_t brakeState = pros::E_MOTOR_BRAKE_HOLD;
    pros::motor_brake_mode_e_t brakeStateI = pros::E_MOTOR_BRAKE_COAST;
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
    };
} // namespace Misc

// <-------------------------------------------------------------- Auto Routes ----------------------------------------------------------->
namespace Auton{
    int state = 0;
    namespace Test{
        void main() { 
            // Color::state = Color::colorVals::BLUE;
            // // Lift::setState(0);
            // TaskHandler::antiJam = false;
            // TaskHandler::colorSort = true;
            // TaskHandler::autoIntake = false;
            // TaskHandler::autoIntake1 = false;
            // TaskHandler::autoIntake2 = false;
            // Piston::mogo.set_value(true);
            // pros::delay(100);
            // Motor::intake.move(127);
            // TaskHandler::autoIntake1 = true;
        }
        void main2() { 
            // Color::state = Color::colorVals::NONE;
            // Motor::intake.move(-127);
        }
        void coords(){ 
            points.emplace_back(-24,24);
            points.emplace_back(-7,41);
            points.emplace_back(24,48);
            Misc::chain(points); // vec, angular timeout, lateral timeout
        }
        void linear(){
            Misc::linear(24,2000,{.forwards = true,.maxSpeed = 127,.minSpeed = 10,.earlyExitRange = 2});
        }

    } // namespace Test
    namespace Red{
        namespace Qual{
            void left(){

			}
			void right(){

			}
			void solo(){

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
    } // namespace Red
    namespace Blue{
        namespace Qual{
            void left(){

			}
			void right(){

			}
			void solo(){

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
    } // namespace Blue   
    namespace Skills{
        void main(){
            
        }
    } // namespace Skills
} // namespace Auton

// <-------------------------------------------------------------- Driver Code ----------------------------------------------------------->
namespace Driver{
    bool b_loader = false, b_hood = false;
    int saberC = 0;
    void joystick(){
        while(1){
            if(TaskHandler::isDriver) {
                int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
                int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
                leftMotors.move(leftY+rightX);
                rightMotors.move(leftY-rightX);
            }
            // chassis.arcade(leftY, rightX);
            pros::delay(Misc::DELAY);
        }
    }
    void intake(){
        while(1){
            if(TaskHandler::dIntake){
                if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { Motor::intakeF.move(127); Motor::intakeM.move(127); Motor::intakeU.move(127);}
                else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { Motor::intakeF.move(-127); Motor::intakeM.move(-127); Motor::intakeU.move(-127);}
                else{ Motor::intakeF.brake(); Motor::intakeM.brake(); Motor::intakeU.brake();}
            }
            pros::delay(Misc::DELAY);
        }
    }
    void piston(){
        while(1){
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { Misc::togglePiston(Piston::loader, b_loader); }
            if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { Misc::togglePiston(Piston::miniHood, b_hood); }
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
        // controller.set_text(0, 0, "Mode: " + std::to_string(Motor::lbL.get_brake_mode()));
        // controller.set_text(0, 0, "Mode: " + Motor::lbL.get_brake_mode());
        // controller.set_text(0, 0, "Pos: " + std::to_string(Motor::lbR.get_position()));
        // controller.set_text(0, 0, "Pos: " + std::to_string(Sensor::lbD.get_distance()));

        // controller.set_text(0, 0, "Dist: " + std::to_string(Sensor::d_colorSort.get_distance()));
        // printf("%d\n",Sensor::d_colorSort.get_distance());
        // controller.set_text(0, 0, "Run Time: " + std::to_string(pros::millis() / 1000) + "s");
        // controller.set_text(0, 0, "X: " + std::to_string(chassis.getPose().x) + "\nY: " + std::to_string(chassis.getPose().y));
        controller.set_text(1, 0, "Test Text 1");
        controller.set_text(2, 0, "Test Text 2");
        // controller.set_text(3, 0, "Drive Left Temp: " + std::to_string(leftMotors.get_temperature()) + "C");
        // controller.set_text(4, 0, "Drive Right Temp: " + std::to_string(rightMotors.get_temperature()) + "C");
        // controller.set_text(5, 0, "Lift Motor Temp: " + std::to_string((Motor::lbL.get_temperature()+Motor::lbR.get_temperature())/2) + "C");
        // controller.set_text(6, 0, "Intake Motor Temp: " + std::to_string(Motor::intake.get_temperature()) + "C");
        // controller.set_text(7, 0, "Inertial Yaw: " + std::to_string(s_imu.get_rotation()));
        // controller.set_text(8, 0, "Battery: " + std::to_string(pros::battery::get_capacity()) + "%");
        pros::delay(500);
    }
}

using AutonFunc = void(*)();
std::vector<std::pair<std::string, AutonFunc>> autonRoutines = {
    // {"Default Auton", Auton::Blue::Elim::negCloseWall},

    // {"Red Positive Elims", Auton::Red::Elim::posfive},
    // {"Red Negative Elims", Auton::Red::Elim::negCloseWall},

    // {"Blue Positive Elims", Auton::Blue::Elim::posfive},
    // {"Blue Negative Elims", Auton::Blue::Elim::negCloseWall},
};


void autonSwitch() {
    if(TaskHandler::autonSelect) {    
        pros::delay(Misc::DELAY);
        if (Sensor::autonSwitch.get_new_press()) { autonState++; if (autonState == autonRoutines.size()) autonState = 0; }
    }
    // pros::lcd::set_text(4, autonRoutines[autonState].first);
}

// LV_IMG_DECLARE(tdbg);
// LV_IMG_DECLARE(WORLDS_logo);
LV_IMG_DECLARE(WO_logo);
LV_IMG_DECLARE(Final_log);

// lv_obj_t * sbg = lv_img_create(lv_scr_act());
lv_obj_t * slogo = lv_img_create(lv_scr_act());
lv_obj_t * Wlogo = lv_img_create(lv_scr_act());

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
    Motor::intakeF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intakeM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intakeU.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // lv_init();
    // set_up();
    // pros::Task LVGL_upd([&]() { screen_upd(); });

	// lv_img_set_src(slogo, &Final_log);
	// lv_obj_set_pos(slogo, 20, 15);

    pros::Task screenTask([&]() {
        while (1) {
            // pros::lcd::print(3, "Pos: %d", Sensor::lbR.get_position());
            // pros::lcd::print(3, "Pos: %f", Motor::lb.get_position());
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::delay(50);
        }
    });

    pros::Task autonSelect([]{ while(1){ autonSwitch(); pros::delay(Misc::DELAY); }});
    // pros::Task screenC([]{ while (1) { Screen::update(); pros::delay(100); }});
}
void disabled() {}
void competition_initialize() {}
ASSET(example_txt); // PP

// <------------------------------------------------------------- autonom ------------------------------------------------------------->
void autonomous() {
    // Color::state = Color::colorVals::BLUE;
    // TaskHandler::antiJam = true;
    // pros::Task sorterC([&](){ while(1) { Color::colorSort(Color::state);  pros::delay(5); }});
    // pros::Task toPosC([&](){ while(1) { Color::toPos(Color::colorConvertor(Color::state)); pros::delay(5); }});
    // pros::Task toPosC1([&](){ while(1) { Color::toPos1(Color::colorConvertor(Color::state)); pros::delay(10); }});
    // pros::Task toPosC2([&](){ while(1) { Color::toPos2(); pros::delay(10); }});

    // pros::Task toPosC3([&](){ while(1) { Color::toPos3(); pros::delay(10); }});
    // pros::Task antiJam([&](){ while(1) { Jam::antiJam(); pros::delay(Misc::DELAY); }});
    pros::Task colorTask(Misc::led);
    // Sensor::o_colorSort.set_led_pwm(100);
    // Sensor::o_colorSort.set_integration_time(5);
    
    (autonState < autonRoutines.size()) ? autonRoutines[autonState].second() : Auton::Test::main();

    // (autonState < autonRoutines.size()) ? autonRoutines[autonState].second() : Auton::Test::main();
}

// <--------------------------------------------------------------- Driver --------------------------------------------------------------->
void opcontrol() {
    pros::Task intakeTask(Driver::intake);
    pros::Task driverTask(Driver::joystick);
    pros::Task pistonTask(Driver::piston);
    // pros::Task hangTask(Driver::hang);
    TaskHandler::colorSort = false;
    TaskHandler::antiJam = false;
    TaskHandler::autoIntake = false;
    TaskHandler::autoIntake1 = false;
    TaskHandler::autoIntake2 = false;
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST); rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    Motor::intakeF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intakeM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); Motor::intakeU.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
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