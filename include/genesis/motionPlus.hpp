#pragma once

#include <vector>
#include "genesis/pose.hpp"

namespace genesis {

inline constexpr float inchPlus = 1.0f;
inline constexpr float footPlus = 1.0f / 12.0f;
inline constexpr float yardPlus = 1.0f / 36.0f;
inline constexpr float cmPlus = 2.54f;
inline constexpr float meterPlus = 0.0254f;

class AsymptoticGainsPlus {
    public:
        AsymptoticGainsPlus(float i, float f, float k, float p);
        void configure(float i, float f, float k, float p);
        void setGain(float setpoint);
        float getGain() const;

    private:
        float i;
        float f;
        float k;
        float p;
        float setpoint = 0;
};

class PIDPlus {
    public:
        PIDPlus(AsymptoticGainsPlus& kP, float kI = 0, float kD = 0, float integralDeadband = 0,
                bool integralSignReset = false);
        void configure(float kI, float kD, float integralDeadband = 0, bool integralSignReset = false);
        float tick(float error);
        void reset(float initialError);
        void setKp(float setpoint);
        float getError() const;

    private:
        AsymptoticGainsPlus& kP;
        float kI;
        float kD;
        float integralDeadband;
        float error = 0;
        float totalError = 0;
        float lastError = 0;
        float derivative = 0;
        bool integralSignReset;
};

class ExitPlus {
    public:
        static bool error(float error, float minError);
        static bool velo(float velocity, float minVelocity);
        static bool hc(Pose pose, Pose target, float theta, float tolerance = 0);
        static bool time(int elapsed, int timeout);
        static bool intaked(float intakeVelocity, float crossedVelocity = 675.0f, float settledVelocity = 600.0f);
        static void resetIntaked();

    private:
        static bool crossed;
};

class PathPlus {
    public:
        explicit PathPlus(std::vector<Pose> waypoints = {});

        int closestIndex(Pose pose) const;
        int lookaheadIndex(Pose pose, int startIndex, float lookahead) const;
        int lastIndex() const;
        Pose closestPoint(Pose pose) const;
        Pose lookaheadPoint(Pose pose, int startIndex, float lookahead) const;
        Pose lastPoint() const;
        Pose at(int index) const;
        bool nearEnd(int index) const;
        bool empty() const;
        std::size_t size() const;

    private:
        std::vector<Pose> waypoints;
};

} // namespace genesis
