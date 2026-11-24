#pragma once

#include "protocol.h"
#include <Eigen/Geometry>
#include <mutex>

namespace spacecal {

struct Pose_t {
    Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
};

// the world pose a driver pose stands for
Eigen::Affine3d getWorldFromDriverPose(const vr::DriverPose_t& pose);

// how AppliedCalibration follows what the poses say (below)
struct AppliedCalibrationSettings {
    double followSeconds = 2.0; // the time constant of each of the two stages
    double turnRateScaleDeg = 60.0; // a pair counts 1 / (1 + (turning rate / this)^2): at 60 degrees a second, half
    double disagreeTurnDeg = 10.0; // the two devices' turns disagree by more than this, degrees a second,
    double disagreeTurnFraction = 0.4; // plus this fraction of the slower one's rate (a device that jumps is not the measure)
    double disagreeMoveMeters = 0.08; // their motions at the mount by more than this, metres a second,
    double disagreeMoveFraction = 0.5; // plus this fraction of the slower one's speed: a pair that does is not counted
    double brokenFactor = 5.0; // and one that disagrees by this many times that is a device that has lost its place
    double stoppedTurnDeg = 0.5; // a device turning slower than this (degrees a second) and moving slower than
    double stoppedMoveMeters = 0.002; // this (metres a second) has stopped, which it cannot have
    double otherTurningDeg = 3.0; // while the other turns faster than this
    double agreeingMotionSeconds = 0.5; // after that, or a stopped device, the pairs count again once the two have moved together this long
    double movingTurnDeg = 5.0; // moving together: turning faster than this, degrees a second,
    double movingMeters = 0.05; // or moving faster than this, metres a second
    double motionBaselineSeconds = 0.04; // the two devices' motions are compared over at least this long
    double catchUpMeters = 0.04; // a difference at the feet larger than this,
    double catchUpEvidenceWeight = 0.8; // shown by pairs that count this much or more (the head turning slower than 30 degrees a second),
    double catchUpEvidenceSeconds = 0.5; // on average over this long,
    double catchUpAfterSeconds = 0.4; // for this long, is caught up with,
    double catchUpRate = 4.0; // this much of the way a second in both stages,
    double caughtUpMeters = 0.01; // until it is down to this
    double catchUpLeastWeight = 0.25; // and while catching up a pair counts at least this much
    double startSeconds = 1.0; // a calibration begins caught up with for this long
    double feetBelowMeters = 1.5; // where a difference is judged: this far below the target device
};

// The world calibration the driver applies to the devices of the tracking system a calibration is for, and how it gets from one
// value to the next without a step anyone can see.
//
// Under relative calibration the overlay hands the driver the mount L, the target device's pose in the reference device's frame,
// and every new pose of the reference paired with the target's latest gives the world calibration C = H L T^-1. Measured over
// recorded sessions: one pair is off by a millimetre or two at the feet while the head is still and by centimetres while it turns
// fast (the two systems' poses are of slightly different moments, and the headset's is predicted), and what the two spaces need
// wanders by a few millimetres over tens of seconds as the user moves about the room. Followed pair by pair, as it was, the
// trackers moved with every turn of the head. So the calibration follows the pairs slowly, through two stages of a two second
// time constant (its speed never jumps); each pair counts for less the faster the head turns, and not at all while the two
// devices' own motions disagree: a headset whose tracking has stopped while the tracker on it moves still says its pose is
// valid. A large difference that well-agreeing pairs keep showing is the two spaces having really moved against each other (a
// recentre, the mount found anew) and is caught up with in a second or two. A step of either system's world, which shows in its
// devices' world-from-driver, is taken into the calibration at once, so nothing moves.
//
// With relative calibration off the overlay hands it the world calibration itself, and with a continuous calibration the driver
// goes to each new one through the same two stages instead of at once (the overlay's came as a step every half second, of 3 mm
// at the feet and up to 11), and to one well away from what is applied, at the catch-up's rate.
//
// One of these serves one calibration, whichever devices it is applied to, and is safe to call from the drivers' threads. The
// time is handed in.
class AppliedCalibration {
public:
    using Settings = AppliedCalibrationSettings;

    explicit AppliedCalibration(const Settings& settings = {})
        : m_settings(settings)
    {
    }
    AppliedCalibration(const AppliedCalibration&) = delete;
    AppliedCalibration& operator=(const AppliedCalibration&) = delete;

    // Relative calibration: the two devices' poses as their drivers gave them, the reference's new since the last call (the driver
    // calls this from the target's pose updates once the reference has a new pose), and the mount
    void addPair(double now, const vr::DriverPose_t& reference, const vr::DriverPose_t& target, const Pose_t& mount);

    // A world calibration from the overlay: gone to through the two stages when `glide` (a continuous calibration), at once if not
    void setWorld(double now, const Pose_t& calibration, bool glide);

    // Under a world calibration the target's (and the reference's) world-from-driver is watched here for a step, and the target's
    // position is where the stages turn about
    void watchFrames(const vr::DriverPose_t& target, const vr::DriverPose_t* reference);

    // What to apply to the devices now. false before the first calibration
    bool current(double now, Pose_t& calibration);

    // the calibration ended: the next one begins from its own first calibration
    void reset();

private:
    struct Pair {
        double time = 0.0;
        Eigen::Affine3d reference = Eigen::Affine3d::Identity();
        Eigen::Affine3d target = Eigen::Affine3d::Identity();
    };
    struct Frames {
        bool known = false;
        Eigen::Affine3d target = Eigen::Affine3d::Identity();
        Eigen::Affine3d reference = Eigen::Affine3d::Identity();
    };

    void begin(double now, const Pose_t& calibration);
    void followFrames(const vr::DriverPose_t& target, const vr::DriverPose_t* reference);
    void advanceWorld(double now);
    [[nodiscard]] double difference(const Pose_t& one, const Pose_t& other, const Eigen::Vector3d& at) const;
    enum class Agreement {
        Agree,
        Disagree, // the pair is not counted
        Broken, // nor any until the two have moved together again
    };
    [[nodiscard]] Agreement agreement(const Pair& before, const Pair& now, const Pose_t& mount, double& turnRate, bool& moving) const;

    const Settings m_settings;
    mutable std::mutex m_mutex;

    bool m_hasCalibration = false;
    bool m_relative = false;
    Pose_t m_applied; // what is applied
    Pose_t m_following; // the first stage, which m_applied follows
    Pose_t m_world; // under a world calibration, the overlay's, which m_following follows
    bool m_worldGlide = false;
    double m_time = 0.0; // the stages were moved on to this time
    double m_beganTime = 0.0;
    Eigen::Vector3d m_pivot = Eigen::Vector3d::Zero(); // the target device in its world, where the stages turn about
    Frames m_frames;

    // the pairs of the last tenth of a second or so, for the two devices' motions
    static constexpr size_t k_PAIRS = 16;
    Pair m_pairs[k_PAIRS];
    size_t m_pairCount = 0;
    size_t m_pairNext = 0;

    Eigen::Vector3d m_lastReferencePosition = Eigen::Vector3d::Zero(); // the reference's pose of the last pair, as its driver gave it
    Eigen::Quaterniond m_lastReferenceRotation = Eigen::Quaterniond::Identity();

    bool m_holding = false;
    double m_agreeingSeconds = 0.0;
    double m_differingSeconds = 0.0;
    double m_meanDifference = 0.0; // between the pairs and what is applied, at the feet, by how much each pair counts
    bool m_catchingUp = false;
};

} // namespace spacecal
