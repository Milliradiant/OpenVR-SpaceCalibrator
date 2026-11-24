#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <vector>

namespace spacecal {

// One motion of two devices between the same two moments, each in its own frame at the first of them: H_a^-1 * H_b for the
// reference, T_a^-1 * T_b for the target. A turn is a rotation vector (axis times angle, radians), a move is in metres.
struct MountMotion {
    Eigen::Vector3d referenceTurn = Eigen::Vector3d::Zero();
    Eigen::Vector3d referenceMove = Eigen::Vector3d::Zero();
    Eigen::Vector3d targetTurn = Eigen::Vector3d::Zero();
    Eigen::Vector3d targetMove = Eigen::Vector3d::Zero();
};

// the motion from one pose of a device to a later one, in its own frame at the first
void motionBetween(const Eigen::Quaterniond& rotationA, const Eigen::Vector3d& positionA, const Eigen::Quaterniond& rotationB, const Eigen::Vector3d& positionB, Eigen::Vector3d& turn, Eigen::Vector3d& move);

// Where a target device sits on a reference device it is fixed to (the mount: the target's pose in the reference's frame), from
// how the two move together.
//
// If L is the mount, the two devices' motions between any two moments differ by it alone: A * L = L * B, whatever either tracking
// system's world is and wherever the pair is in the room. So every motion with some turn in it says something about the one
// constant L, and they can all be added up: over a session, and from one session to the next. The turns give the mount's rotation
// (the two turning axes are the same axis seen from each device), and with that the moves give its translation.
//
// What is kept is a few sums, not the motions: the estimate has the same cost and size after an hour as after a minute, and the
// sums are what is stored between sessions. A motion that does not fit (a tracking glitch, a device held still while the other
// is not) is left out, and if for a while hardly any fit, the target has been moved on the reference and the estimate starts over.
// A target moved by a few degrees (bumped, or put back a little differently) still fits most motions loosely: that shows as the
// latest motions, solved on their own, agreeing with each other and not with the estimate, and the estimate becomes theirs.
class MountEstimator {
public:
    struct Options {
        double minTurn = 0.07; // a motion that turned less than this (4 degrees) says too little: radians
        double maxTurnMismatch = 0.15; // the two devices' turning angles may differ by this fraction (and always by a degree)
        double gate = 3.0; // a motion is left out when it is this many times further off the estimate than motions are typically
        double minTurnGate = 0.012; // ... but never for less than this, radians (0.7 degrees)
        double minMoveGate = 0.006; // or this, metres
        double minWeight = 150.0; // motions before there is an estimate
        double maxWeight = 30000.0; // what the sums hold at most: older motions fade as new ones come once it is reached
        double minTurnCondition = 0.08; // the turns must be about more than one axis: second over first singular value
        double minMoveCondition = 0.04; // and the moves' equations must hold the translation in every direction
        // the mount is taken to have changed when nearly none of the latest motions fit the estimate: this share of them, as a
        // running mean over about this many
        double remountWindow = 150.0;
        double remountRejected = 0.85;
        // or when it has changed by a few degrees, which the gate lets most motions through: the latest motions are also added up
        // in blocks of this many, each solved on its own, and when two blocks in a row agree with each other and not with the
        // estimate, by more than this, the two blocks are the estimate from then on. A block of 300 (half a minute or so of
        // play) is within 0.5 degrees of a settled estimate nine times in ten and was never 1.4 degrees off it in hours
        double blockWeight = 300.0;
        double remountAngle = 0.026; // radians (1.5 degrees)
        double remountDistance = 0.02; // metres
    };

    enum class Added {
        Taken,
        TooLittleTurn, // not a motion to judge anything by
        Rejected, // does not fit two devices fixed to each other with this mount
        StartedOver, // rejected, and with it the estimate was dropped: the mount has changed
        Remounted, // the mount has changed by a little: the latest motions' estimate is the estimate from now on
    };

    MountEstimator() { reset(); }
    explicit MountEstimator(const Options& options)
        : m_options(options)
    {
        reset();
    }

    void reset();
    Added add(const MountMotion& motion);

    // there is an estimate to use: enough motions, about enough axes
    [[nodiscard]] bool ready() const { return m_sums.ready; }
    // and it is good enough to calibrate by: this many motions, or two blocks that agreed when the mount changed
    [[nodiscard]] bool settled(double minWeight) const { return m_sums.ready && (m_sums.weight >= minWeight || m_fromBlocks); }
    [[nodiscard]] const Eigen::Quaterniond& rotation() const { return m_sums.rotation; }
    [[nodiscard]] const Eigen::Vector3d& translation() const { return m_sums.translation; }
    [[nodiscard]] double weight() const { return m_sums.weight; }
    // how far off the estimate a motion typically is: radians of its turn, metres in its move's equation
    [[nodiscard]] double turnResidual() const { return std::sqrt(m_sums.turnResidual2); }
    [[nodiscard]] double moveResidual() const { return std::sqrt(m_sums.moveResidual2); }
    [[nodiscard]] size_t startedOver() const { return m_startedOver; }
    [[nodiscard]] size_t remounted() const { return m_remounted; }
    // the share of the latest motions that did not fit the estimate (a running mean)
    [[nodiscard]] double rejectedShare() const { return m_rejectedShare; }

    // the sums, to keep between sessions. restore() takes what state() gave and nothing else: false, and an empty estimate, if not
    static constexpr size_t k_STATE_SIZE = 52;
    [[nodiscard]] std::vector<double> state() const;
    bool restore(const std::vector<double>& state);

private:
    // the sums of some motions, and the mount they give
    struct Sums {
        double weight = 0.0;
        Eigen::Matrix3d turns = Eigen::Matrix3d::Zero(); // sum of referenceTurn * targetTurn^T
        Eigen::Matrix3d moveNormal = Eigen::Matrix3d::Zero(); // sum of M^T M, with M = R_A - I
        double moveByRotation[3][3][3] = {}; // sum of M^T(i,j) * targetMove(k): the right side's part that goes with the rotation
        Eigen::Vector3d moveConstant = Eigen::Vector3d::Zero(); // sum of M^T * referenceMove
        double turnResidual2 = 0.0, moveResidual2 = 0.0; // running mean squares over the motions taken

        bool ready = false;
        Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d translation = Eigen::Vector3d::Zero();

        void clear() { *this = Sums {}; }
        // whether a motion that turned alike on both devices fits these sums' mount, and how far off it is
        bool fits(const MountMotion& motion, const Eigen::Matrix3d& m, const Options& options, double& turnResidual2, double& moveResidual2) const;
        void add(const MountMotion& motion, const Eigen::Matrix3d& m, double turnResidual2, double moveResidual2, double maxWeight);
        void addSums(const Sums& other);
        void solve(const Options& options);
    };

    Options m_options;
    Sums m_sums; // every motion that fitted, the oldest fading once there are maxWeight of them
    Sums m_block; // the latest motions, block by block
    Sums m_lastBlock;
    bool m_lastBlockDisagreed = false;
    bool m_fromBlocks = false; // the estimate is two blocks' since the mount last changed

    double m_rejectedShare = 0.0;
    size_t m_startedOver = 0;
    size_t m_remounted = 0;
};

} // namespace spacecal
