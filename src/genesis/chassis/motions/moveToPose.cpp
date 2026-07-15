#include <cmath>
#include "genesis/chassis/chassis.hpp"
#include "genesis/logger/logger.hpp"
#include "genesis/timer.hpp"
#include "genesis/util.hpp"
#include "pros/misc.hpp"

void genesis::Chassis::moveToPose(float x, float y, float theta, int timeout, MoveToPoseParams params, bool async) {
    // take the mutex
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { moveToPose(x, y, theta, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    // reset PIDs and exit conditions
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();
    angularLargeExit.reset();
    angularSmallExit.reset();

    // calculate target pose in standard form
    Pose target(x, y, M_PI_2 - degToRad(theta));
    if (!params.forwards) target.theta = fmod(target.theta + M_PI, 2 * M_PI); // backwards movement

    // use global horizontalDrift is horizontalDrift is 0
    if (params.horizontalDrift == 0) params.horizontalDrift = drivetrain.horizontalDrift;

    // initialize vars used between iterations
    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    bool close = false;
    bool lateralSettled = false;
    bool prevSameSide = false;
    float prevLateralOut = 0; // previous lateral power
    float prevAngularOut = 0; // previous angular power
    const int compState = pros::competition::get_status();

    // main loop
    while (!timer.isDone() &&
           ((!lateralSettled || (!angularLargeExit.getExit() && !angularSmallExit.getExit())) || !close) &&
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

        // check if the lateral controller has settled
        if (lateralLargeExit.getExit() && lateralSmallExit.getExit()) lateralSettled = true;

        // calculate the carrot point
        Pose carrot = target - Pose(cos(target.theta), sin(target.theta)) * params.lead * distTarget;
        if (close) carrot = target; // settling behavior

        // calculate if the robot is on the same side as the carrot point
        const bool robotSide =
            (pose.y - target.y) * -sin(target.theta) <= (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
        const bool carrotSide = (carrot.y - target.y) * -sin(target.theta) <=
                                (carrot.x - target.x) * cos(target.theta) + params.earlyExitRange;
        const bool sameSide = robotSide == carrotSide;
        // exit if close
        if (!sameSide && prevSameSide && close && params.minSpeed != 0) break;
        prevSameSide = sameSide;

        // calculate error
        const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float angularError =
            close ? angleError(adjustedRobotTheta, target.theta) : angleError(adjustedRobotTheta, pose.angle(carrot));
        float lateralError = pose.distance(carrot);
        // only use cos when settling
        // otherwise just multiply by the sign of cos
        // maxSlipSpeed takes care of lateralOut
        if (close) lateralError *= cos(angleError(pose.theta, pose.angle(carrot)));
        else lateralError *= sgn(cos(angleError(pose.theta, pose.angle(carrot))));

        // update exit conditions
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);
        angularSmallExit.update(radToDeg(angularError));
        angularLargeExit.update(radToDeg(angularError));

        // get output from PIDs
        float lateralOut = lateralPID.update(lateralError);
        float angularOut = angularPID.update(radToDeg(angularError));

        // apply restrictions on angular speed
        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);

        // apply restrictions on lateral speed
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);

        // constrain lateral output by max accel
        if (!close) lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

        // constrain lateral output by the max speed it can travel at without
        // slipping
        const float radius = 1 / fabs(getCurvature(pose, carrot));
        const float maxSlipSpeed(sqrt(params.horizontalDrift * radius * 9.8));
        lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);
        // prioritize angular movement over lateral movement
        const float overturn = fabs(angularOut) + fabs(lateralOut) - params.maxSpeed;
        if (overturn > 0) lateralOut -= lateralOut > 0 ? overturn : -overturn;

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

        infoSink()->debug("lateralOut: {} angularOut: {}", lateralOut, angularOut);

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

void genesis::Chassis::moveToPosePro(float x, float y, float theta, int timeout, MoveToPoseParams params, bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([=]() { moveToPosePro(x, y, theta, timeout, params, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularLargeExit.reset();
    angularSmallExit.reset();

    Pose target(x, y, M_PI_2 - degToRad(theta));
    if (!params.forwards) target.theta = fmod(target.theta + M_PI, 2 * M_PI);

    if (params.horizontalDrift == 0) params.horizontalDrift = drivetrain.horizontalDrift;

    Pose lastPose = getPose(true, true);
    distTraveled = 0;
    Timer timer(timeout);
    bool close = false;
    bool lateralSettled = false;
    bool prevSameSide = false;
    float prevLateralOut = 0;
    float prevAngularOut = 0;

    const float initialDistTarget = lastPose.distance(target);
    Pose initialCarrot = target - Pose(cos(target.theta), sin(target.theta)) * params.lead * initialDistTarget;
    const float initialAdjustedTheta = params.forwards ? lastPose.theta : lastPose.theta + M_PI;
    const float initialAngularErrorDeg =
        radToDeg(angleError(initialAdjustedTheta, lastPose.angle(initialCarrot)));
    float initialLateralError = lastPose.distance(initialCarrot);
    initialLateralError *= std::copysign(1.0f, cos(angleError(lastPose.theta, lastPose.angle(initialCarrot))));
    lateralControllerPlus.reset(initialLateralError / 12.0f);
    lateralControllerPlus.setKp(initialLateralError / 12.0f);
    turnControllerPlus.reset(initialAngularErrorDeg);
    turnControllerPlus.setKp(initialAngularErrorDeg);

    const float voltageScale = 12000.0f / 127.0f;
    float allowedVoltage = std::fabs(params.maxSpeed) * voltageScale;
    const float minVoltage = std::fabs(params.minSpeed) * voltageScale;
    const float closeVoltage = 60.0f * voltageScale;
    const float lateralSlewVoltage = lateralSettings.slew > 0 ? lateralSettings.slew * voltageScale : 0.0f;
    const float angularSlewVoltage = angularSettings.slew > 0 ? angularSettings.slew * voltageScale : 0.0f;

    while (!timer.isDone() &&
           ((!lateralSettled || (!angularLargeExit.getExit() && !angularSmallExit.getExit())) || !close) &&
           this->motionRunning) {
        const Pose pose = getPose(true, true);

        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        const float distTarget = pose.distance(target);
        if (distTarget < 7.5f && !close) {
            close = true;
            allowedVoltage = std::max(std::fabs(prevLateralOut), closeVoltage);
        }

        if (lateralLargeExit.getExit() && lateralSmallExit.getExit()) lateralSettled = true;

        Pose carrot = target - Pose(cos(target.theta), sin(target.theta)) * params.lead * distTarget;
        if (close) carrot = target;

        const bool robotSide =
            (pose.y - target.y) * -sin(target.theta) <= (pose.x - target.x) * cos(target.theta) + params.earlyExitRange;
        const bool carrotSide = (carrot.y - target.y) * -sin(target.theta) <=
                                (carrot.x - target.x) * cos(target.theta) + params.earlyExitRange;
        const bool sameSide = robotSide == carrotSide;
        if (!sameSide && prevSameSide && close && params.minSpeed != 0) break;
        prevSameSide = sameSide;

        const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float angularErrorDeg = radToDeg(
            close ? angleError(adjustedRobotTheta, target.theta) : angleError(adjustedRobotTheta, pose.angle(carrot)));
        float lateralError = pose.distance(carrot);
        if (close) lateralError *= cos(angleError(pose.theta, pose.angle(carrot)));
        else lateralError *= std::copysign(1.0f, cos(angleError(pose.theta, pose.angle(carrot))));

        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);
        angularSmallExit.update(angularErrorDeg);
        angularLargeExit.update(angularErrorDeg);

        float lateralOut = lateralControllerPlus.tick(lateralError / 12.0f);
        float angularOut = turnControllerPlus.tick(angularErrorDeg);

        angularOut = std::clamp(angularOut, -allowedVoltage, allowedVoltage);
        if (angularSlewVoltage > 0) angularOut = slew(angularOut, prevAngularOut, angularSlewVoltage);

        lateralOut = std::clamp(lateralOut, -allowedVoltage, allowedVoltage);
        if (!close && lateralSlewVoltage > 0) lateralOut = slew(lateralOut, prevLateralOut, lateralSlewVoltage);

        const float radius = 1.0f / fabs(getCurvature(pose, carrot));
        const float maxSlipSpeed = sqrt(params.horizontalDrift * radius * 9.8f);
        const float maxSlipVoltage = maxSlipSpeed * voltageScale;
        lateralOut = std::clamp(lateralOut, -maxSlipVoltage, maxSlipVoltage);

        const float overturn = fabs(angularOut) + fabs(lateralOut) - allowedVoltage;
        if (overturn > 0) lateralOut -= lateralOut > 0 ? overturn : -overturn;

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
