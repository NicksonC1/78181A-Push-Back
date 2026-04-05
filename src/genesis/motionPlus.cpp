#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include "genesis/motionPlus.hpp"
#include "genesis/util.hpp"

namespace genesis {

AsymptoticGainsPlus::AsymptoticGainsPlus(float i, float f, float k, float p)
    : i(i),
      f(f),
      k(k),
      p(p) {}

void AsymptoticGainsPlus::configure(float i, float f, float k, float p) {
    this->i = i;
    this->f = f;
    this->k = k;
    this->p = p;
}

void AsymptoticGainsPlus::setGain(float setpoint) { this->setpoint = setpoint; }

float AsymptoticGainsPlus::getGain() const {
    return (f - i) * std::pow(std::fabs(setpoint), p) /
               (std::pow(std::fabs(setpoint), p) + std::pow(k, p)) +
           i;
}

PIDPlus::PIDPlus(AsymptoticGainsPlus& kP, float kI, float kD, float integralDeadband, bool integralSignReset)
    : kP(kP),
      kI(kI),
      kD(kD),
      integralDeadband(integralDeadband),
      integralSignReset(integralSignReset) {}

void PIDPlus::configure(float kI, float kD, float integralDeadband, bool integralSignReset) {
    this->kI = kI;
    this->kD = kD;
    this->integralDeadband = integralDeadband;
    this->integralSignReset = integralSignReset;
}

float PIDPlus::tick(float error) {
    this->error = error;
    if (std::fabs(error) > integralDeadband) totalError = 0;
    else if (integralSignReset && std::signbit(error) != std::signbit(lastError)) totalError = 0;
    else totalError += error * 0.01f;
    derivative = (error - lastError) * 100.0f;
    lastError = error;

    return kP.getGain() * error + kI * totalError + kD * derivative;
}

void PIDPlus::reset(float initialError) {
    lastError = initialError;
    totalError = 0;
    derivative = 0;
    error = initialError;
}

void PIDPlus::setKp(float setpoint) { kP.setGain(setpoint); }

float PIDPlus::getError() const { return error; }

bool ExitPlus::crossed = false;

bool ExitPlus::error(float error, float minError) { return std::fabs(error) > minError; }

bool ExitPlus::velo(float velocity, float minVelocity) { return velocity > minVelocity; }

bool ExitPlus::hc(Pose pose, Pose target, float theta, float tolerance) {
    const float heading = degToRad(theta);
    return (pose.y - target.y) * -std::cos(heading) >= std::sin(heading) * (pose.x - target.x) + tolerance;
}

bool ExitPlus::time(int elapsed, int timeout) { return elapsed < timeout; }

bool ExitPlus::intaked(float intakeVelocity, float crossedVelocity, float settledVelocity) {
    if (!crossed && intakeVelocity > crossedVelocity) crossed = true;
    if (intakeVelocity < settledVelocity && crossed) {
        crossed = false;
        return false;
    }
    return true;
}

void ExitPlus::resetIntaked() { crossed = false; }

PathPlus::PathPlus(std::vector<Pose> waypoints)
    : waypoints(std::move(waypoints)) {}

int PathPlus::closestIndex(Pose pose) const {
    if (waypoints.empty()) return 0;
    int closestIndex = 0;
    float closestDist = std::numeric_limits<float>::max();
    for (int i = 0; i < static_cast<int>(waypoints.size()); i++) {
        const float dist = pose.distance(waypoints.at(i));
        if (dist < closestDist) {
            closestIndex = i;
            closestDist = dist;
        }
    }
    return closestIndex;
}

int PathPlus::lookaheadIndex(Pose pose, int startIndex, float lookahead) const {
    if (waypoints.empty()) return 0;
    startIndex = std::clamp(startIndex, 0, lastIndex());
    for (int i = startIndex; i < static_cast<int>(waypoints.size()); i++) {
        if (pose.distance(waypoints.at(i)) >= lookahead) return i;
    }
    return lastIndex();
}

int PathPlus::lastIndex() const { return waypoints.empty() ? 0 : static_cast<int>(waypoints.size()) - 1; }

Pose PathPlus::closestPoint(Pose pose) const { return at(closestIndex(pose)); }

Pose PathPlus::lookaheadPoint(Pose pose, int startIndex, float lookahead) const { return at(lookaheadIndex(pose, startIndex, lookahead)); }

Pose PathPlus::lastPoint() const { return at(lastIndex()); }

Pose PathPlus::at(int index) const {
    if (waypoints.empty()) return Pose(0, 0, 0);
    return waypoints.at(std::clamp(index, 0, lastIndex()));
}

bool PathPlus::nearEnd(int index) const { return lastIndex() - index <= 5; }

bool PathPlus::empty() const { return waypoints.empty(); }

std::size_t PathPlus::size() const { return waypoints.size(); }

} // namespace genesis
