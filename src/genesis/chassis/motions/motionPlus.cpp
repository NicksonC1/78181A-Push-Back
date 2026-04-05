#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include "genesis/chassis/chassis.hpp"
#include "genesis/chassis/odom.hpp"
#include "genesis/util.hpp"
#include "pros/misc.hpp"

namespace {

constexpr float kInchesPerFootPlus = 12.0f;
constexpr float kFeetPerInchPlus = 1.0f / kInchesPerFootPlus;

// Keep the original foot-based motion formulas, but convert the public inch API at the boundaries.
// constexpr float kMotionExitSpeedFeet = 0.0f;
constexpr float kMotionExitSpeedFeet = 0.5f;
constexpr float kPushExitSpeedFeet = 0.05f;
constexpr float kMoveDistExitErrorFeet = 0.05f;
// constexpr float kMotionChainToleranceFeet = 0.0f;
constexpr float kMotionChainToleranceFeet = 0.2f;
// constexpr float kMovePointSettleDistanceFeet = 0.0f;
constexpr float kMovePointSettleDistanceFeet = 0.7f;
// constexpr float kMovePointAngularStopDistanceFeet = 0.0f;
constexpr float kMovePointAngularStopDistanceFeet = 0.5f;
// constexpr float kMovePointExitErrorFeet = 0.0f;
constexpr float kMovePointExitErrorFeet = 0.6f;
constexpr float kEndIntakeExitErrorFeet = 0.5f;
constexpr float kMovePoseCloseDistanceFeet = 0.7f;
constexpr float kMovePoseGhostDistancePerSpeedFeet = 2.0f;
constexpr float kApsAngularDenominatorScaleFeet = 25.0f;
constexpr float kApsCrosstrackScaleFeet = 10.0f;
constexpr float kApsLookaheadFeet = 1.8f;

float inchesToFeetPlus(float inches) { return inches * kFeetPerInchPlus; }

genesis::Pose poseToFeetPlus(genesis::Pose pose) {
    pose.x = inchesToFeetPlus(pose.x);
    pose.y = inchesToFeetPlus(pose.y);
    return pose;
}

genesis::Pose currentPoseFeetPlus(genesis::Chassis& chassis, bool reverse = false) {
    genesis::Pose pose = poseToFeetPlus(chassis.getPose());
    if (reverse) {
        pose.theta += 180.0f;
        pose.theta = pose.theta - 360.0f * std::floor((pose.theta + 180.0f) / 360.0f);
    }
    return pose;
}

genesis::PathPlus pathToFeetPlus(const genesis::PathPlus& path) {
    std::vector<genesis::Pose> waypoints;
    waypoints.reserve(path.size());
    for (std::size_t i = 0; i < path.size(); i++) {
        genesis::Pose point = path.at(static_cast<int>(i));
        point.x = inchesToFeetPlus(point.x);
        point.y = inchesToFeetPlus(point.y);
        waypoints.push_back(point);
    }
    return genesis::PathPlus(std::move(waypoints));
}

float clampPlus(float input, float min, float max) {
    if (input < min) return min;
    if (input > max) return max;
    return input;
}

float reduceAnglePlus(float theta) { return theta - 360.0f * std::floor((theta + 180.0f) / 360.0f); }

float angleErrorPlus(float target, float theta) { return reduceAnglePlus(target - theta); }

float facePlus(const genesis::Pose& pose, const genesis::Pose& other) {
    return reduceAnglePlus(genesis::radToDeg(std::atan2(other.x - pose.x, other.y - pose.y)) - pose.theta);
}

float anglePlus(const genesis::Pose& pose, const genesis::Pose& other) {
    return reduceAnglePlus(genesis::radToDeg(std::atan2(other.x - pose.x, other.y - pose.y)));
}

float parallelPlus(const genesis::Pose& pose, const genesis::Pose& other) { return reduceAnglePlus(other.theta - pose.theta); }

float parallelPlus(const genesis::Pose& pose, float target) { return reduceAnglePlus(target - pose.theta); }

void setPosePlus(genesis::Pose& pose, float x, float y, float theta) {
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
}

float dotProductPlus(const genesis::Pose& a, const genesis::Pose& b) { return a.x * b.x + a.y * b.y; }

std::pair<float, float> scaleToRatioPlus(float factor, float num1, float num2) {
    const float peak = std::max(std::fabs(num1), std::fabs(num2));
    if (peak == 0) return {0, 0};
    const float ratio = factor / peak;
    return {num1 * ratio, num2 * ratio};
}

std::pair<float, float> reduceRatioPlus(float max, float num1, float num2) {
    if (std::fabs(num1) > std::fabs(max) && std::fabs(num1) >= std::fabs(num2)) {
        num2 *= max / std::fabs(num1);
        num1 = std::copysign(max, num1);
    } else if (std::fabs(num2) > std::fabs(max) && std::fabs(num2) >= std::fabs(num1)) {
        num1 *= max / std::fabs(num2);
        num2 = std::copysign(max, num2);
    }
    return {num1, num2};
}

float getRadiusPlus(const genesis::Pose& p1, const genesis::Pose& p2) {
    const float angle = genesis::degToRad(facePlus(p1, p2));
    const float denom = std::sqrt(std::max(0.0f, 2.0f - 2.0f * std::cos(2.0f * angle)));
    if (denom == 0) return std::numeric_limits<float>::infinity();
    return p1.distance(p2) / denom;
}

float getRadiusPlus(const genesis::Pose& p1, const genesis::Pose& p2, const genesis::Pose& p3) {
    const float A = p1.x * (p2.y - p3.y) - p1.y * (p2.x - p3.x) + p2.x * p3.y - p3.x * p2.y;
    const float B = (p1.x * p1.x + p1.y * p1.y) * (p3.y - p2.y) + (p2.x * p2.x + p2.y * p2.y) * (p1.y - p3.y) +
                    (p3.x * p3.x + p3.y * p3.y) * (p2.y - p1.y);
    const float C = (p1.x * p1.x + p1.y * p1.y) * (p2.x - p3.x) + (p2.x * p2.x + p2.y * p2.y) * (p3.x - p1.x) +
                    (p3.x * p3.x + p3.y * p3.y) * (p1.x - p2.x);
    const float D = (p1.x * p1.x + p1.y * p1.y) * (p3.x * p2.y - p2.x * p3.y) +
                    (p2.x * p2.x + p2.y * p2.y) * (p1.x * p3.y - p3.x * p1.y) +
                    (p3.x * p3.x + p3.y * p3.y) * (p2.x * p1.y - p1.x * p2.y);
    if (A == 0) return std::numeric_limits<float>::infinity();
    return std::sqrt((B * B + C * C - 4.0f * A * D) / (4.0f * A * A));
}

float getCurvaturePlus(const genesis::Pose& p1, const genesis::Pose& p2) {
    const float side =
        std::copysign(1.0f, std::cos(genesis::degToRad(p1.theta)) * (p2.x - p1.x) -
                                 std::sin(genesis::degToRad(p1.theta)) * (p2.y - p1.y));
    const float radius = getRadiusPlus(p1, p2);
    return radius != 0 ? side / radius : std::numeric_limits<float>::max();
}

float getCurvaturePlus(const genesis::Pose& p1, const genesis::Pose& p2, float heading) {
    const float side =
        std::copysign(1.0f, std::cos(genesis::degToRad(heading)) * (p2.x - p1.x) -
                                 std::sin(genesis::degToRad(heading)) * (p2.y - p1.y));
    const float radius = getRadiusPlus(p1, p2);
    return radius != 0 ? side / radius : std::numeric_limits<float>::max();
}

float circleIntersectPlus(const genesis::Pose& linep1, const genesis::Pose& linep2, const genesis::Pose& center,
                          float radius) {
    const genesis::Pose d = linep2 - linep1;
    const genesis::Pose f = linep1 - center;
    const float a = dotProductPlus(d, d);
    const float b = 2.0f * dotProductPlus(f, d);
    const float c = dotProductPlus(f, f) - radius * radius;
    float discriminant = b * b - 4.0f * a * c;

    if (discriminant >= 0) {
        discriminant = std::sqrt(discriminant);
        const float t1 = (-b - discriminant) / (2.0f * a);
        const float t2 = (-b + discriminant) / (2.0f * a);
        if (t1 >= 0 && t1 <= 1) return t1;
        if (t2 >= 0 && t2 <= 1) return t2;
    }

    return -1;
}

float scalarSpeedPlus() {
    const genesis::Pose speed = genesis::getSpeed(false);
    return inchesToFeetPlus(std::hypot(speed.x, speed.y));
}

float angularVelocityPlus() { return std::fabs(genesis::getSpeed(false).theta); }

float currentHeadingPlus(genesis::Chassis& chassis, bool reverse = false) {
    genesis::Pose pose = chassis.getPose();
    if (reverse) pose.theta = reduceAnglePlus(pose.theta + 180.0f);
    return pose.theta;
}

genesis::Pose currentPosePlus(genesis::Chassis& chassis, bool reverse = false) {
    genesis::Pose pose = chassis.getPose();
    if (reverse) pose.theta = reduceAnglePlus(pose.theta + 180.0f);
    return pose;
}

float currentRollPlus(pros::Imu* imu) { return imu != nullptr ? imu->get_roll() : 0.0f; }

float averagePositionDegrees(const pros::MotorGroup* motors) {
    const std::vector<double> positions = motors->get_position_all();
    if (positions.empty()) return 0;
    double total = 0;
    for (double position : positions) total += position;
    return total / positions.size();
}

float degreesToUnits(float degrees, float wheelDiameter, float unit) { return degrees * wheelDiameter * M_PI / 360.0f * unit; }

void runDrivetrainPlus(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, float left, float right, bool reverse) {
    if (reverse) {
        leftMotors->move_voltage(static_cast<int>(std::lround(-right)));
        rightMotors->move_voltage(static_cast<int>(std::lround(-left)));
    } else {
        leftMotors->move_voltage(static_cast<int>(std::lround(left)));
        rightMotors->move_voltage(static_cast<int>(std::lround(right)));
    }
}

void stopDrivetrainPlus(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors) {
    leftMotors->move_voltage(0);
    rightMotors->move_voltage(0);
}

} // namespace

void genesis::Chassis::setUnitPlus(float unit) { this->unitPlus = unit; }

void genesis::Chassis::setLateralPIDPlus(float kPInitial, float kPFinal, float kPKnee, float kPPower, float kI,
                                         float kD, float integralDeadband, bool integralSignReset) {
    lateralKpPlus.configure(kPInitial, kPFinal, kPKnee, kPPower);
    lateralControllerPlus.configure(kI, kD, integralDeadband, integralSignReset);
}

void genesis::Chassis::setTurnPIDPlus(float kPInitial, float kPFinal, float kPKnee, float kPPower, float kI,
                                      float kD, float integralDeadband, bool integralSignReset) {
    turnKpPlus.configure(kPInitial, kPFinal, kPKnee, kPPower);
    turnControllerPlus.configure(kI, kD, integralDeadband, integralSignReset);
}

void genesis::Chassis::setHeadingCorrectPIDPlus(float kPInitial, float kPFinal, float kPKnee, float kPPower,
                                                float kI, float kD, float integralDeadband, bool integralSignReset) {
    headingCorrectKpPlus.configure(kPInitial, kPFinal, kPKnee, kPPower);
    headingCorrectControllerPlus.configure(kI, kD, integralDeadband, integralSignReset);
}

void genesis::Chassis::setPursuitSettingsPlus(float minLookahead, float maxLookahead, float pursuitVeloConst,
                                              float pursuitCurvConst, float pursuitCTEConst, float radiusLookahead) {
    this->minLookaheadPlus = minLookahead;
    this->maxLookaheadPlus = maxLookahead;
    this->pursuitVeloConstPlus = pursuitVeloConst;
    this->pursuitCurvConstPlus = pursuitCurvConst;
    this->pursuitCTEConstPlus = pursuitCTEConst;
    this->radiusLookaheadPlus = radiusLookahead;
}

void genesis::Chassis::setIntakeExitSupplierPlus(std::function<float()> supplier, float crossedVelocity,
                                                 float settledVelocity) {
    this->intakeExitSupplierPlus = std::move(supplier);
    this->intakeCrossedVelocityPlus = crossedVelocity;
    this->intakeSettledVelocityPlus = settledVelocity;
    ExitPlus::resetIntaked();
}

void genesis::Chassis::clearIntakeExitSupplierPlus() {
    this->intakeExitSupplierPlus = {};
    ExitPlus::resetIntaked();
}

void genesis::Chassis::moveDistPlus(float target, float maxSpeed, int timeout, bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([=] { this->moveDistPlus(target, maxSpeed, timeout, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    if (!headingTargetInitializedPlus) {
        headingTargetPlus = currentHeadingPlus(*this);
        headingTargetInitializedPlus = true;
    }
    const float targetFeet = inchesToFeetPlus(target);
    lateralControllerPlus.reset(targetFeet);
    lateralControllerPlus.setKp(targetFeet);
    headingCorrectControllerPlus.reset(angleErrorPlus(headingTargetPlus, currentHeadingPlus(*this)));
    headingCorrectControllerPlus.setKp(angleErrorPlus(headingTargetPlus, currentHeadingPlus(*this)));

    const float startDistance = degreesToUnits(
        0.5f * (averagePositionDegrees(drivetrain.leftMotors) + averagePositionDegrees(drivetrain.rightMotors)),
        drivetrain.wheelDiameter, footPlus);
    float adjustedTargetFeet = targetFeet + startDistance;

    int timer = 0;
    float error = 0;
    Pose lastPose = getPose(true);
    distTraveled = 0;

    do {
        const Pose pose = getPose(true);
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        const float distance = degreesToUnits(
            0.5f * (averagePositionDegrees(drivetrain.leftMotors) + averagePositionDegrees(drivetrain.rightMotors)),
            drivetrain.wheelDiameter, footPlus);
        error = adjustedTargetFeet - distance;

        const float lateralOutput =
            clampPlus(lateralControllerPlus.tick(error), -std::fabs(maxSpeed) * 12000.0f, std::fabs(maxSpeed) * 12000.0f);
        const float headingOutput = clampPlus(headingCorrectControllerPlus.tick(angleErrorPlus(headingTargetPlus, currentHeadingPlus(*this))),
                                              -std::fabs(maxSpeed) * 12000.0f, std::fabs(maxSpeed) * 12000.0f);
        const auto outputs = reduceRatioPlus(std::fabs(maxSpeed) * 12000.0f, lateralOutput + headingOutput,
                                             lateralOutput - headingOutput);
        runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, outputs.first, outputs.second, false);

        timer += 10;
        pros::delay(10);
    } while ((ExitPlus::error(error, kMoveDistExitErrorFeet) || ExitPlus::velo(scalarSpeedPlus(), kMotionExitSpeedFeet)) &&
             ExitPlus::time(timer, timeout) && this->motionRunning);

    stopDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors);
    distTraveled = -1;
    this->endMotion();
}

void genesis::Chassis::pushPlus(float speed, int timeout, bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([=] { this->pushPlus(speed, timeout, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    if (!headingTargetInitializedPlus) {
        headingTargetPlus = currentHeadingPlus(*this);
        headingTargetInitializedPlus = true;
    }
    headingCorrectControllerPlus.reset(angleErrorPlus(headingTargetPlus, currentHeadingPlus(*this)));
    headingCorrectControllerPlus.setKp(angleErrorPlus(headingTargetPlus, currentHeadingPlus(*this)));

    int timer = 0;
    Pose lastPose = getPose(true);
    distTraveled = 0;

    do {
        const Pose pose = getPose(true);
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        const float lateralOutput = speed * 12000.0f;
        const float headingOutput = headingCorrectControllerPlus.tick(angleErrorPlus(headingTargetPlus, currentHeadingPlus(*this)));
        const auto outputs = reduceRatioPlus(std::fabs(speed) * 12000.0f, lateralOutput + headingOutput,
                                             lateralOutput - headingOutput);
        runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, outputs.first, outputs.second, false);

        timer += 10;
        pros::delay(10);
    } while ((ExitPlus::velo(scalarSpeedPlus(), kPushExitSpeedFeet) && std::fabs(currentRollPlus(sensors.imu)) < 10.0f) &&
             ExitPlus::time(timer, timeout) && this->motionRunning);

    stopDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors);
    distTraveled = -1;
    this->endMotion();
}

void genesis::Chassis::pushPlus(float coord, bool isY, float speed, int timeout, bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([=] { this->pushPlus(coord, isY, speed, timeout, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    if (!headingTargetInitializedPlus) {
        headingTargetPlus = currentHeadingPlus(*this);
        headingTargetInitializedPlus = true;
    }
    const float coordFeet = inchesToFeetPlus(coord);
    bool start = isY ? currentPoseFeetPlus(*this).y > coordFeet : currentPoseFeetPlus(*this).x > coordFeet;
    bool cur = start;
    int timer = 0;
    Pose lastPose = getPose(true);
    distTraveled = 0;

    headingCorrectControllerPlus.reset(angleErrorPlus(headingTargetPlus, currentHeadingPlus(*this)));
    headingCorrectControllerPlus.setKp(angleErrorPlus(headingTargetPlus, currentHeadingPlus(*this)));

    do {
        const Pose rawPose = getPose(true);
        distTraveled += rawPose.distance(lastPose);
        lastPose = rawPose;

        const Pose pose = currentPoseFeetPlus(*this);
        cur = isY ? pose.y > coordFeet : pose.x > coordFeet;

        const float lateralOutput = speed * 12000.0f;
        const float headingOutput = headingCorrectControllerPlus.tick(angleErrorPlus(headingTargetPlus, currentHeadingPlus(*this)));
        const auto outputs = reduceRatioPlus(std::fabs(speed) * 12000.0f, lateralOutput + headingOutput,
                                             lateralOutput - headingOutput);
        runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, outputs.first, outputs.second, false);

        timer += 10;
        pros::delay(10);
    } while (((ExitPlus::velo(scalarSpeedPlus(), kMotionExitSpeedFeet) && std::fabs(currentRollPlus(sensors.imu)) < 10.0f) || start == cur) &&
             ExitPlus::time(timer, timeout) && this->motionRunning);

    stopDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors);
    distTraveled = -1;
    this->endMotion();
}

void genesis::Chassis::crossBarrierPlus(bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([=] { this->crossBarrierPlus(false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    if (sensors.imu == nullptr) {
        distTraveled = -1;
        this->endMotion();
        return;
    }

    if (!headingTargetInitializedPlus) {
        headingTargetPlus = currentHeadingPlus(*this);
        headingTargetInitializedPlus = true;
    }
    bool reachedMinPitch = false;
    bool reachedMaxPitch = false;
    Pose lastPose = getPose(true);
    distTraveled = 0;

    headingCorrectControllerPlus.reset(angleErrorPlus(headingTargetPlus, currentHeadingPlus(*this)));
    headingCorrectControllerPlus.setKp(angleErrorPlus(headingTargetPlus, currentHeadingPlus(*this)));

    do {
        const Pose pose = getPose(true);
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        const float roll = currentRollPlus(sensors.imu);
        if (roll < -10.0f) reachedMinPitch = true;
        if (roll > 10.0f) reachedMaxPitch = true;

        const float lateralOutput = 12000.0f;
        const float headingOutput = headingCorrectControllerPlus.tick(angleErrorPlus(headingTargetPlus, currentHeadingPlus(*this)));
        const auto outputs = reduceRatioPlus(12000.0f, lateralOutput + headingOutput, lateralOutput - headingOutput);
        runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, outputs.first, outputs.second, false);

        pros::delay(10);
    } while ((!reachedMinPitch || !reachedMaxPitch || std::fabs(currentRollPlus(sensors.imu) + 5.0f) > 3.0f) &&
             this->motionRunning);

    stopDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors);
    distTraveled = -1;
    this->endMotion();
}

void genesis::Chassis::turnHeadingPlus(float target, int sides, float minSpeed, float maxSpeed, int timeout, bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([=] { this->turnHeadingPlus(target, sides, minSpeed, maxSpeed, timeout, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    turnControllerPlus.reset(angleErrorPlus(target, currentHeadingPlus(*this)));
    turnControllerPlus.setKp(sides == 2 ? angleErrorPlus(target, currentHeadingPlus(*this))
                                        : angleErrorPlus(target, currentHeadingPlus(*this)) / 2.5f);

    int timer = 0;
    float lastHeading = currentHeadingPlus(*this);
    distTraveled = 0;

    do {
        const float currentHeading = currentHeadingPlus(*this);
        distTraveled += std::fabs(angleErrorPlus(currentHeading, lastHeading));
        lastHeading = currentHeading;

        float output = clampPlus(turnControllerPlus.tick(angleErrorPlus(target, currentHeading)), -maxSpeed * 12000.0f,
                                 maxSpeed * 12000.0f);
        if (std::fabs(output) < minSpeed * 12000.0f) output = std::copysign(minSpeed * 12000.0f, output);

        if (sides == 0) runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, output, 0, false);
        else if (sides == 1) runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, 0, -output, false);
        else runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, output, -output, false);

        timer += 10;
        pros::delay(10);
    } while (((minSpeed == 0 && ExitPlus::velo(angularVelocityPlus(), 3.0f)) ||
              ExitPlus::error(angleErrorPlus(target, currentHeadingPlus(*this)), 4.0f)) &&
             timer < timeout && this->motionRunning);

    headingTargetPlus = target;
    headingTargetInitializedPlus = true;
    stopDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors);
    distTraveled = -1;
    this->endMotion();
}

void genesis::Chassis::turnPointPlus(float x, float y, int sides, float minSpeed, float maxSpeed, int timeout,
                                     bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([=] { this->turnPointPlus(x, y, sides, minSpeed, maxSpeed, timeout, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    const Pose end(x, y, -1);
    Pose pose = currentPosePlus(*this);

    turnControllerPlus.reset(facePlus(pose, end));
    turnControllerPlus.setKp(sides == 2 ? facePlus(pose, end) : facePlus(pose, end) / 2.5f);

    int timer = 0;
    float lastHeading = pose.theta;
    distTraveled = 0;

    do {
        pose = currentPosePlus(*this);
        distTraveled += std::fabs(angleErrorPlus(pose.theta, lastHeading));
        lastHeading = pose.theta;

        float output = clampPlus(turnControllerPlus.tick(facePlus(pose, end)), -maxSpeed * 12000.0f, maxSpeed * 12000.0f);
        if (std::fabs(output) < minSpeed * 12000.0f) output = std::copysign(minSpeed * 12000.0f, output);

        if (sides == 0) runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, output, 0, false);
        else if (sides == 1) runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, 0, -output, false);
        else runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, output, -output, false);

        timer += 10;
        pros::delay(10);
    } while (((minSpeed == 0 && ExitPlus::velo(angularVelocityPlus(), 1.0f)) || ExitPlus::error(facePlus(pose, end), 1.0f)) &&
             timer < timeout && this->motionRunning);

    headingTargetPlus = currentHeadingPlus(*this);
    headingTargetInitializedPlus = true;
    stopDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors);
    distTraveled = -1;
    this->endMotion();
}

void genesis::Chassis::turnPointOneIterPlus(float x, float y, int sides, float minSpeed, float maxSpeed) {
    const Pose end(x, y, -1);
    const Pose pose = currentPosePlus(*this);

    float output = clampPlus(turnControllerPlus.tick(facePlus(pose, end)), -maxSpeed * 12000.0f, maxSpeed * 12000.0f);
    if (std::fabs(output) < minSpeed * 12000.0f) output = std::copysign(minSpeed * 12000.0f, output);

    if (sides == 0) runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, output, 0, false);
    else if (sides == 1) runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, 0, -output, false);
    else runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, output, -output, false);

    headingTargetPlus = currentHeadingPlus(*this);
    headingTargetInitializedPlus = true;
}

void genesis::Chassis::movePointPlus(float x, float y, float minSpeed, float maxSpeed, int timeout, bool settle,
                                     bool endIntake, bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([=] { this->movePointPlus(x, y, minSpeed, maxSpeed, timeout, settle, endIntake, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    const bool reverse = maxSpeed < 0;
    const float tolerance = minSpeed == 0 ? kMotionChainToleranceFeet : 0.0f;
    bool exitStatus = false;
    bool lastExitStatus = false;
    bool crossedOnce = false;

    const Pose end(inchesToFeetPlus(x), inchesToFeetPlus(y), -1);
    Pose pose = currentPoseFeetPlus(*this, reverse);

    lateralControllerPlus.reset(pose.distance(end));
    lateralControllerPlus.setKp(1);
    turnControllerPlus.reset(facePlus(pose, end));
    turnControllerPlus.setKp(180);

    ExitPlus::resetIntaked();

    int timer = 0;
    Pose lastPose = getPose(true);
    distTraveled = 0;

    do {
        lastExitStatus = ExitPlus::hc(pose, end, currentHeadingPlus(*this, reverse), tolerance);

        const Pose odomPose = getPose(true);
        distTraveled += odomPose.distance(lastPose);
        lastPose = odomPose;

        pose = currentPoseFeetPlus(*this, reverse);
        exitStatus = ExitPlus::hc(pose, end, currentHeadingPlus(*this, reverse), tolerance);
        if (!exitStatus && lastExitStatus) crossedOnce = true;

        float distanceOutput = lateralControllerPlus.tick(pose.distance(end));
        if (settle && pose.distance(end) < kMovePointSettleDistanceFeet) distanceOutput *= std::cos(degToRad(facePlus(pose, end)));
        distanceOutput =
            std::copysign(clampPlus(std::fabs(distanceOutput), std::fabs(minSpeed) * 12000.0f, 12000.0f), distanceOutput);

        float angularOutput = turnControllerPlus.tick(facePlus(pose, end));
        if (pose.distance(end) < kMovePointAngularStopDistanceFeet) angularOutput = 0;
        angularOutput = clampPlus(angularOutput, -12000.0f, 12000.0f);

        const float rescale = std::fabs(distanceOutput) + std::fabs(angularOutput) - 12000.0f;
        if (rescale > 0) distanceOutput -= std::copysign(rescale, distanceOutput);

        const auto outputs = reduceRatioPlus(std::fabs(maxSpeed) * 12000.0f, distanceOutput + angularOutput,
                                             distanceOutput - angularOutput);
        runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, outputs.first, outputs.second, reverse);

        timer += 10;
        pros::delay(10);
    } while ((((minSpeed == 0 && ExitPlus::velo(scalarSpeedPlus(), kMotionExitSpeedFeet)) ||
               ExitPlus::error(pose.distance(end), kMovePointExitErrorFeet) ||
               !crossedOnce) &&
              (!endIntake ||
               ((intakeExitSupplierPlus &&
                 ExitPlus::intaked(intakeExitSupplierPlus(), intakeCrossedVelocityPlus, intakeSettledVelocityPlus)) ||
                ExitPlus::error(pose.distance(end), kEndIntakeExitErrorFeet))) &&
              ExitPlus::time(timer, timeout) && this->motionRunning));

    headingTargetPlus = currentHeadingPlus(*this);
    headingTargetInitializedPlus = true;

    if (minSpeed == 0) stopDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors);
    distTraveled = -1;
    this->endMotion();
}

void genesis::Chassis::movePosePlus(float x, float y, float theta, float dLead, float gLead, float minSpeed,
                                    float maxSpeed, int timeout, float chasePower, bool settle, bool endIntake,
                                    bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([=] {
            this->movePosePlus(x, y, theta, dLead, gLead, minSpeed, maxSpeed, timeout, chasePower, settle, endIntake,
                               false);
        });
        this->endMotion();
        pros::delay(10);
        return;
    }

    const bool reverse = maxSpeed < 0;
    const float tolerance = minSpeed == 0 ? kMotionChainToleranceFeet : 0.0f;
    bool closeEnd = false;
    bool closeGhost = false;
    bool exitStatus = false;
    bool lastExitStatus = false;
    bool crossedOnce = false;

    if (reverse) theta += 180.0f;

    const float xFeet = inchesToFeetPlus(x);
    const float yFeet = inchesToFeetPlus(y);
    const float dLeadFeet = inchesToFeetPlus(dLead);
    const Pose end(xFeet, yFeet, theta);
    Pose pose = currentPoseFeetPlus(*this, reverse);
    Pose carrot(xFeet - std::sin(degToRad(theta)) * dLeadFeet, yFeet - std::cos(degToRad(theta)) * dLeadFeet, -1);
    const Pose initialCarrot(carrot.x, carrot.y, -1);
    Pose target(0, 0, 0);

    float minCarrotDist = pose.distance(carrot);
    const float initCarrotDist = pose.distance(carrot);

    lateralControllerPlus.reset(pose.distance(carrot));
    lateralControllerPlus.setKp(1);
    turnControllerPlus.reset(facePlus(pose, carrot));
    turnControllerPlus.setKp(180);

    ExitPlus::resetIntaked();

    int timer = 0;
    Pose lastPose = getPose(true);
    distTraveled = 0;

    do {
        lastExitStatus = ExitPlus::hc(pose, end, theta, tolerance);

        const Pose odomPose = getPose(true);
        distTraveled += odomPose.distance(lastPose);
        lastPose = odomPose;

        pose = currentPoseFeetPlus(*this, reverse);

        if (pose.distance(carrot) < minCarrotDist) minCarrotDist = pose.distance(carrot);
        const float carrotScale = initCarrotDist == 0 ? 0 : minCarrotDist / initCarrotDist;
        setPosePlus(carrot, xFeet - carrotScale * std::sin(degToRad(theta)) * dLeadFeet,
                    yFeet - carrotScale * std::cos(degToRad(theta)) * dLeadFeet, -1);
        const Pose ghost(initialCarrot.x + (carrot.x - initialCarrot.x) * (1 - gLead),
                         initialCarrot.y + (carrot.y - initialCarrot.y) * (1 - gLead), -1);

        exitStatus = ExitPlus::hc(pose, end, theta, tolerance);
        if (!exitStatus && lastExitStatus) crossedOnce = true;

        float angleError = 0;
        if (pose.distance(carrot) < kMovePoseCloseDistanceFeet || closeEnd) {
            closeEnd = true;
            if (pose.distance(end) < kMovePoseCloseDistanceFeet) angleError = parallelPlus(pose, end);
            else angleError = facePlus(pose, end);
            setPosePlus(target, end.x, end.y, end.theta);
        } else if (pose.distance(ghost) < kMovePoseGhostDistancePerSpeedFeet * std::fabs(maxSpeed) || closeGhost) {
            closeGhost = true;
            angleError = facePlus(pose, carrot);
            setPosePlus(target, carrot.x, carrot.y, carrot.theta);
        } else {
            angleError = facePlus(pose, ghost);
            setPosePlus(target, ghost.x, ghost.y, ghost.theta);
        }

        float angularOutput = clampPlus(turnControllerPlus.tick(angleError), -12000.0f, 12000.0f);
        float distanceOutput =
            (closeEnd || pose.distance(end) > pose.distance(carrot)) ? lateralControllerPlus.tick(pose.distance(end))
                                                                     : lateralControllerPlus.tick(pose.distance(carrot));
        if (settle && closeEnd && pose.distance(end) < kMovePoseCloseDistanceFeet) distanceOutput *= std::cos(degToRad(facePlus(pose, target)));
        distanceOutput =
            std::copysign(clampPlus(std::fabs(distanceOutput), std::fabs(minSpeed) * 12000.0f, 12000.0f), distanceOutput);

        const float radius = getRadiusPlus(pose, target) / std::fmin(pose.distance(target), 1.0f);
        const float maxSlipSpeed = std::sqrt(chasePower * radius * 1000000.0f);
        if (chasePower != -1 && (!closeEnd || pose.distance(end) >= kMovePoseCloseDistanceFeet))
            distanceOutput = clampPlus(distanceOutput, -maxSlipSpeed, maxSlipSpeed);

        const float rescale = std::fabs(distanceOutput) + std::fabs(angularOutput) - 12000.0f;
        if (rescale > 0) distanceOutput -= std::copysign(rescale, distanceOutput);

        const auto outputs = reduceRatioPlus(std::fabs(maxSpeed) * 12000.0f, distanceOutput + angularOutput,
                                             distanceOutput - angularOutput);
        runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, outputs.first, outputs.second, reverse);

        timer += 10;
        pros::delay(10);
    } while (((minSpeed == 0 && ExitPlus::velo(scalarSpeedPlus(), kMotionExitSpeedFeet)) || !crossedOnce) &&
             (!endIntake ||
              ((intakeExitSupplierPlus &&
                ExitPlus::intaked(intakeExitSupplierPlus(), intakeCrossedVelocityPlus, intakeSettledVelocityPlus)) ||
               ExitPlus::error(pose.distance(end), kEndIntakeExitErrorFeet))) &&
             ExitPlus::time(timer, timeout) && this->motionRunning);

    headingTargetPlus = reverse ? theta - 180.0f : theta;
    headingTargetInitializedPlus = true;

    if (minSpeed == 0) stopDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors);
    distTraveled = -1;
    this->endMotion();
}

void genesis::Chassis::followPursuitPlus(PathPlus path, int timeout, float chasePower, bool reverse, bool endIntake,
                                         bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([=] { this->followPursuitPlus(path, timeout, chasePower, reverse, endIntake, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    if (path.empty()) {
        distTraveled = -1;
        this->endMotion();
        return;
    }

    const PathPlus pathFeet = pathToFeetPlus(path);
    const float minLookaheadFeet = inchesToFeetPlus(minLookaheadPlus);
    const float maxLookaheadFeet = inchesToFeetPlus(maxLookaheadPlus);
    const float trackWidthFeet = inchesToFeetPlus(drivetrain.trackWidth);

    int timer = 0;
    int pathIndex = 0;
    Pose pose = currentPoseFeetPlus(*this, reverse);
    Pose lastPose = getPose(true);
    distTraveled = 0;
    ExitPlus::resetIntaked();

    do {
        const Pose odomPose = getPose(true);
        distTraveled += odomPose.distance(lastPose);
        lastPose = odomPose;

        pose = currentPoseFeetPlus(*this, reverse);
        pathIndex = pathFeet.closestIndex(pose);

        const float crosstrackError = pose.distance(pathFeet.at(pathIndex));
        const int radiusIndex = pathIndex + static_cast<int>(radiusLookaheadPlus);
        const float lookahead = clampPlus(scalarSpeedPlus() * pursuitVeloConstPlus +
                                              getRadiusPlus(pose, pathFeet.at(radiusIndex)) * pursuitCurvConstPlus -
                                              crosstrackError * pursuitCTEConstPlus,
                                          minLookaheadFeet, maxLookaheadFeet);
        const Pose lookaheadPoint = pathFeet.lookaheadPoint(pose, pathIndex, lookahead);

        const float turnError = facePlus(pose, lookaheadPoint);
        if (std::fabs(turnError) >= 90.0f) {
            const float dir = std::copysign(1.0f, turnError);
            runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, pathFeet.at(pathIndex).theta * dir,
                              pathFeet.at(pathIndex).theta * -dir, reverse);
            timer += 10;
            pros::delay(10);
            continue;
        }

        float voltageScale = pathFeet.at(pathIndex).theta;
        if (chasePower != -1) {
            const float radius = getRadiusPlus(pose, lookaheadPoint);
            voltageScale = std::fmin(std::sqrt(chasePower * radius / lookahead * 1000000.0f), voltageScale);
        }

        const float curvature = getCurvaturePlus(pose, lookaheadPoint);
        const float leftRatio = 2.0f + curvature * trackWidthFeet;
        const float rightRatio = 2.0f - curvature * trackWidthFeet;
        const auto outputs = scaleToRatioPlus(voltageScale, leftRatio, rightRatio);
        runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, outputs.first, outputs.second, reverse);

        timer += 10;
        pros::delay(10);
    } while ((((pathFeet.lastPoint().theta == 0 && ExitPlus::velo(scalarSpeedPlus(), kMotionExitSpeedFeet)) ||
               pathIndex != pathFeet.lastIndex()) &&
              (!endIntake || !intakeExitSupplierPlus ||
               ExitPlus::intaked(intakeExitSupplierPlus(), intakeCrossedVelocityPlus, intakeSettledVelocityPlus)) &&
              ExitPlus::time(timer, timeout) && this->motionRunning));

    headingTargetPlus = currentHeadingPlus(*this);
    headingTargetInitializedPlus = true;
    stopDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors);
    distTraveled = -1;
    this->endMotion();
}

void genesis::Chassis::followStanleyPlus(PathPlus path, int timeout, float k, float chasePower, bool reverse,
                                         bool endIntake, bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([=] { this->followStanleyPlus(path, timeout, k, chasePower, reverse, endIntake, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    if (path.empty()) {
        distTraveled = -1;
        this->endMotion();
        return;
    }

    const PathPlus pathFeet = pathToFeetPlus(path);
    const float trackWidthFeet = inchesToFeetPlus(drivetrain.trackWidth);

    int timer = 0;
    int pathIndex = 0;
    int furthestIndex = 0;
    Pose pose = currentPoseFeetPlus(*this, reverse);
    Pose lastPose = getPose(true);
    distTraveled = 0;
    ExitPlus::resetIntaked();

    do {
        const Pose odomPose = getPose(true);
        distTraveled += odomPose.distance(lastPose);
        lastPose = odomPose;

        pose = currentPoseFeetPlus(*this, reverse);
        pathIndex = pathFeet.closestIndex(pose);
        if (pathIndex > furthestIndex) furthestIndex = pathIndex;
        else if (pathIndex < furthestIndex) pathIndex = furthestIndex;

        const Pose curPoint = pathFeet.at(pathIndex);
        const Pose nextPoint = pathFeet.at(pathIndex + 1);
        const Pose prevPoint = pathFeet.at(pathIndex - 1);

        const float pathHeading =
            pathIndex == pathFeet.lastIndex() ? anglePlus(prevPoint, curPoint) : anglePlus(curPoint, nextPoint);
        const float crosstrackError = std::copysign(pose.distance(curPoint), -getCurvaturePlus(curPoint, pose, pathHeading));
        const float headingError = parallelPlus(pose, pathHeading);
        const float speed = scalarSpeedPlus();
        const float crosstrackScale =
            radToDeg(std::atan(speed == 0 ? std::copysign(std::numeric_limits<float>::infinity(), k * crosstrackError)
                                          : k * crosstrackError / speed));
        const float steering = 5.0f * headingError + crosstrackScale;

        if (std::fabs(steering) >= 90.0f) {
            const float dir = std::copysign(1.0f, steering);
            runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, pathFeet.at(pathIndex).theta * dir,
                              pathFeet.at(pathIndex).theta * -dir, reverse);
            timer += 10;
            pros::delay(10);
            continue;
        }

        float voltageScale = pathFeet.at(pathIndex).theta;
        if (chasePower != -1) {
            const float radius = trackWidthFeet / std::tan(degToRad(std::fabs(steering)));
            voltageScale = std::fmin(std::sqrt(chasePower * radius * 1000000.0f), voltageScale);
        }

        const float leftRatio = 2.0f + std::tan(degToRad(steering)) / 2.0f;
        const float rightRatio = 2.0f - std::tan(degToRad(steering)) / 2.0f;
        const auto outputs = scaleToRatioPlus(voltageScale, leftRatio, rightRatio);
        runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, outputs.first, outputs.second, reverse);

        timer += 10;
        pros::delay(10);
    } while ((((pathFeet.lastPoint().theta == 0 && ExitPlus::velo(scalarSpeedPlus(), kMotionExitSpeedFeet)) ||
               pathIndex != pathFeet.lastIndex()) &&
              (!endIntake || !intakeExitSupplierPlus ||
               ExitPlus::intaked(intakeExitSupplierPlus(), intakeCrossedVelocityPlus, intakeSettledVelocityPlus)) &&
              ExitPlus::time(timer, timeout) && this->motionRunning));

    headingTargetPlus = currentHeadingPlus(*this);
    headingTargetInitializedPlus = true;
    stopDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors);
    distTraveled = -1;
    this->endMotion();
}

void genesis::Chassis::followAPSPlus(PathPlus path, int timeout, float maxSpeed, float chasePower, bool reverse,
                                     bool endIntake, bool async) {
    this->requestMotionStart();
    if (!this->motionRunning) return;
    if (async) {
        pros::Task task([=] { this->followAPSPlus(path, timeout, maxSpeed, chasePower, reverse, endIntake, false); });
        this->endMotion();
        pros::delay(10);
        return;
    }

    if (path.empty()) {
        distTraveled = -1;
        this->endMotion();
        return;
    }

    const PathPlus pathFeet = pathToFeetPlus(path);

    int timer = 0;
    int pathIndex = 0;
    int furthestIndex = 0;
    Pose pose = currentPoseFeetPlus(*this, reverse);
    Pose lastPose = getPose(true);
    distTraveled = 0;
    ExitPlus::resetIntaked();

    turnControllerPlus.reset(0);
    turnControllerPlus.setKp(0);

    do {
        const Pose odomPose = getPose(true);
        distTraveled += odomPose.distance(lastPose);
        lastPose = odomPose;

        pose = currentPoseFeetPlus(*this, reverse);
        pathIndex = pathFeet.closestIndex(pose);
        if (pathIndex > furthestIndex) furthestIndex = pathIndex;
        else if (pathIndex < furthestIndex) pathIndex = furthestIndex;

        const Pose curPoint = pathFeet.at(pathIndex);
        const Pose nextPoint = pathFeet.at(pathIndex + 1);
        const Pose prevPoint = pathFeet.at(pathIndex - 1);

        float pathHeading = 0;
        if (pathIndex == 0) pathHeading = anglePlus(curPoint, nextPoint);
        else if (pathIndex == pathFeet.lastIndex()) pathHeading = anglePlus(prevPoint, curPoint);
        else pathHeading = 0.5f * (anglePlus(curPoint, nextPoint) + anglePlus(prevPoint, curPoint));

        const float crosstrackError = pose.distance(curPoint);
        float angularTerm = parallelPlus(pose, pathHeading);
        const float angularDenom = std::pow(kApsAngularDenominatorScaleFeet * crosstrackError, 2.0f);
        const float scaledAngular = angularDenom == 0 ? std::fabs(angularTerm) : std::fabs(angularTerm / angularDenom);
        angularTerm = std::copysign(std::fmin(scaledAngular, std::fabs(angularTerm)), angularTerm);
        const float crosstrackTerm = !pathFeet.nearEnd(pathIndex)
                                         ? kApsCrosstrackScaleFeet * crosstrackError * degToRad(facePlus(pose, curPoint))
                                         : 0.0f;
        const float headingOutput = clampPlus(turnControllerPlus.tick(angularTerm + crosstrackTerm), -12000.0f, 12000.0f);

        float lateralOutput = curPoint.theta;
        if (chasePower != -1) {
            const Pose lookaheadPoint = pathFeet.lookaheadPoint(pose, pathIndex, kApsLookaheadFeet);
            const float radius = getRadiusPlus(pose, lookaheadPoint);
            lateralOutput = std::fmin(std::sqrt(chasePower * radius * 1000000.0f), lateralOutput);
        }

        const float rescale = std::fabs(lateralOutput) + std::fabs(headingOutput) - 12000.0f;
        if (rescale > 0) lateralOutput -= std::copysign(rescale, lateralOutput);

        const auto outputs =
            reduceRatioPlus(maxSpeed * 12000.0f, lateralOutput + headingOutput, lateralOutput - headingOutput);
        runDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors, outputs.first, outputs.second, reverse);

        timer += 10;
        pros::delay(10);
    } while ((((pathFeet.lastPoint().theta == 0 && ExitPlus::velo(scalarSpeedPlus(), kMotionExitSpeedFeet)) ||
               pathIndex != pathFeet.lastIndex()) &&
              (!endIntake || !intakeExitSupplierPlus ||
               ExitPlus::intaked(intakeExitSupplierPlus(), intakeCrossedVelocityPlus, intakeSettledVelocityPlus)) &&
              ExitPlus::time(timer, timeout) && this->motionRunning));

    headingTargetPlus = currentHeadingPlus(*this);
    headingTargetInitializedPlus = true;
    stopDrivetrainPlus(drivetrain.leftMotors, drivetrain.rightMotors);
    distTraveled = -1;
    this->endMotion();
}
