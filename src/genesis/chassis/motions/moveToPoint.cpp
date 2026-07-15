#include <cmath>
#include "genesis/chassis/chassis.hpp"
#include "genesis/logger/logger.hpp"
#include "genesis/timer.hpp"
#include "genesis/util.hpp"
#include "pros/misc.hpp"

void genesis::Chassis::moveToPoint(float x, float y, int timeout, MoveToPointParams params, bool async) {
    params.earlyExitRange = fabs(params.earlyExitRange);
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { moveToPoint(x, y, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    // reset PIDs and exit conditions
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();

    // initialize vars used between iterations
    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    bool close = false;
    float prevLateralOut = 0; // previous lateral power
    float prevAngularOut = 0; // previous angular power
    const int compState = pros::competition::get_status();
    std::optional<bool> prevSide = std::nullopt;

    // calculate target pose in standard form
    Pose target(x, y);
    target.theta = lastPose.angle(target);

    // main loop
    while (!timer.isDone() && ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()) || !close) &&
           this->motionRunning) {
        // update position
        const Pose pose = getPose(true, true);

        // update distance traveled
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        // calculate distance to the target point
        const float distTarget = pose.distance(target);

        // check if the robot is close enough to the target to start settling
        if (distTarget < 7.5 && close == false) {
            close = true;
            params.maxSpeed = fmax(fabs(prevLateralOut), 60);
        }

        // motion chaining
        const bool side =
            (pose.y - target.y) * -sin(target.theta) <= (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
        if (prevSide == std::nullopt) prevSide = side;
        const bool sameSide = side == prevSide;
        // exit if close
        if (!sameSide && params.minSpeed != 0) break;
        prevSide = side;

        // calculate error
        const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float angularError = angleError(adjustedRobotTheta, pose.angle(target));
        float lateralError = pose.distance(target) * cos(angleError(pose.theta, pose.angle(target)));

        // update exit conditions
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

        // get output from PIDs
        float lateralOut = lateralPID.update(lateralError);
        float angularOut = angularPID.update(radToDeg(angularError));
        if (close) angularOut = 0;

        // apply restrictions on angular speed
        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);
        angularOut = slew(angularOut, prevAngularOut, angularSettings.slew);

        // apply restrictions on lateral speed
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);
        // constrain lateral output by max accel
        // but not for decelerating, since that would interfere with settling
        if (!close) lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

        // prevent moving in the wrong direction
        if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
        else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);

        // constrain lateral output by the minimum speed
        if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0) lateralOut = fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);

        // update previous output
        prevAngularOut = angularOut;
        prevLateralOut = lateralOut;

        infoSink()->debug("Angular Out: {}, Lateral Out: {}", angularOut, lateralOut);

        // ratio the speeds to respect the max speed
        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        // move the drivetrain
        drivetrain.leftMotors->move(leftPower);
        drivetrain.rightMotors->move(rightPower);

        // delay to save resources
        pros::delay(10);
    }

    // stop the drivetrain
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}

void genesis::Chassis::moveToPointPro(float x, float y, int timeout, MoveToPointParams params, bool async) {
    params.earlyExitRange = fabs(params.earlyExitRange);
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([=]() { moveToPointPro(x, y, timeout, params, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    lateralLargeExit.reset();
    lateralSmallExit.reset();

    Pose lastPose = getPose(true, true);
    distTraveled = 0;
    Timer timer(timeout);
    bool close = false;
    float prevLateralOut = 0;
    float prevAngularOut = 0;
    std::optional<bool> prevSide = std::nullopt;

    Pose target(x, y);
    target.theta = lastPose.angle(target);

    const float initialAdjustedTheta = params.forwards ? lastPose.theta : lastPose.theta + M_PI;
    const float initialAngularErrorDeg = radToDeg(angleError(initialAdjustedTheta, lastPose.angle(target)));
    const float initialLateralErrorFeet = lastPose.distance(target) / 12.0f;
    lateralControllerPlus.reset(initialLateralErrorFeet);
    lateralControllerPlus.setKp(initialLateralErrorFeet);
    turnControllerPlus.reset(initialAngularErrorDeg);
    turnControllerPlus.setKp(initialAngularErrorDeg);

    const float maxVoltageScale = 12000.0f / 127.0f;
    const float lateralSlewVoltage = lateralSettings.slew > 0 ? lateralSettings.slew * maxVoltageScale : 0.0f;
    float allowedVoltage = std::fabs(params.maxSpeed) * maxVoltageScale;
    const float minVoltage = std::fabs(params.minSpeed) * maxVoltageScale;
    const float closeVoltage = 60.0f * maxVoltageScale;

    while (!timer.isDone() && ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()) || !close) &&
           this->motionRunning) {
        const Pose pose = getPose(true, true);

        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        const float distTarget = pose.distance(target);
        if (distTarget < 7.5f && !close) {
            close = true;
            allowedVoltage = std::max(std::fabs(prevLateralOut), closeVoltage);
        }

        const bool side =
            (pose.y - target.y) * -sin(target.theta) <= (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
        if (prevSide == std::nullopt) prevSide = side;
        const bool sameSide = side == prevSide;
        if (!sameSide && params.minSpeed != 0) break;
        prevSide = side;

        const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float angularErrorDeg = radToDeg(angleError(adjustedRobotTheta, pose.angle(target)));
        float lateralError = pose.distance(target) * cos(angleError(pose.theta, pose.angle(target)));

        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

        float lateralOut = lateralControllerPlus.tick(lateralError / 12.0f);
        float angularOut = turnControllerPlus.tick(angularErrorDeg);
        if (close) angularOut = 0;

        angularOut = std::clamp(angularOut, -allowedVoltage, allowedVoltage);
        angularOut = slew(angularOut, prevAngularOut, angularSettings.slew > 0 ? angularSettings.slew * maxVoltageScale : 0.0f);

        lateralOut = std::clamp(lateralOut, -allowedVoltage, allowedVoltage);
        if (!close && lateralSlewVoltage > 0) lateralOut = slew(lateralOut, prevLateralOut, lateralSlewVoltage);

        if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0.0f);
        else if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0.0f);

        if (params.forwards && lateralOut < minVoltage && lateralOut > 0) lateralOut = minVoltage;
        if (!params.forwards && -lateralOut < minVoltage && lateralOut < 0) lateralOut = -minVoltage;

        prevAngularOut = angularOut;
        prevLateralOut = lateralOut;

        float leftPower = lateralOut + angularOut;
        float rightPower = lateralOut - angularOut;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / allowedVoltage;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        drivetrain.leftMotors->move_voltage(static_cast<int>(std::lround(leftPower)));
        drivetrain.rightMotors->move_voltage(static_cast<int>(std::lround(rightPower)));
        pros::delay(10);
    }

    drivetrain.leftMotors->move_voltage(0);
    drivetrain.rightMotors->move_voltage(0);
    distTraveled = -1;
    this->endMotion();
}
