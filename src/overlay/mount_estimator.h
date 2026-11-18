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
    };

    enum class Added {
        Taken,
        TooLittleTurn, // not a motion to judge anything by
        Rejected, // does not fit two devices fixed to each other with this mount
        StartedOver, // rejected, and with it the estimate was dropped: the mount has changed
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
    [[nodiscard]] bool ready() const { return m_ready; }
    [[nodiscard]] const Eigen::Quaterniond& rotation() const { return m_rotation; }
    [[nodiscard]] const Eigen::Vector3d& translation() const { return m_translation; }
    [[nodiscard]] double weight() const { return m_weight; }
    // how far off the estimate a motion typically is: radians of its turn, metres in its move's equation
    [[nodiscard]] double turnResidual() const { return std::sqrt(m_turnResidual2); }
    [[nodiscard]] double moveResidual() const { return std::sqrt(m_moveResidual2); }
    [[nodiscard]] size_t startedOver() const { return m_startedOver; }
    // the share of the latest motions that did not fit the estimate (a running mean)
    [[nodiscard]] double rejectedShare() const { return m_rejectedShare; }

    // the sums, to keep between sessions. restore() takes what state() gave and nothing else: false, and an empty estimate, if not
    static constexpr size_t k_STATE_SIZE = 52;
    [[nodiscard]] std::vector<double> state() const;
    bool restore(const std::vector<double>& state);

private:
    void solve();

    Options m_options;
    double m_weight = 0.0;
    Eigen::Matrix3d m_turns; // sum of referenceTurn * targetTurn^T
    Eigen::Matrix3d m_moveNormal; // sum of M^T M, with M = R_A - I
    double m_moveByRotation[3][3][3]; // sum of M^T(i,j) * targetMove(k): the right side's part that goes with the rotation
    Eigen::Vector3d m_moveConstant; // sum of M^T * referenceMove
    double m_turnResidual2 = 0.0, m_moveResidual2 = 0.0; // running mean squares over the motions taken

    bool m_ready = false;
    Eigen::Quaterniond m_rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d m_translation = Eigen::Vector3d::Zero();

    double m_rejectedShare = 0.0;
    size_t m_startedOver = 0;
};

} // namespace spacecal
