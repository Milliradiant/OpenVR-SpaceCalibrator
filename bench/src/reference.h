#pragma once

#include "session.h"

namespace bench {

// The reference a calibration is judged against, worked out from a whole recording with hindsight.
//
// The target device is fixed to the reference device, so there is one constant pose of the target in the reference's frame, the
// mount L, and one time offset between the two devices' streams. With H(t) the reference's world pose and T(t) the target's:
//
//     H(t) * L = C(t) * T(t + offset)
//
// where C(t), the world calibration, is what Space Calibrator is after. L and the offset come from how the two devices move between
// pairs of moments (H_a^-1 H_b = L (T_a^-1 T_b) L^-1), which holds whatever C is and so is untouched by the reference's space
// drifting or being recentred. C(t) then follows from every pose pair, and its slow part, averaged over a window, is the reference
// world calibration: where a perfect calibration would have put the target's tracking system at that time.
struct ReferenceOptions {
    double maxLag = 0.15; // the time offset is searched within this, seconds either way
    double pairSpacing = 0.3; // between the two moments of a motion pair, seconds
    double pairStep = 0.05; // from one pair to the next, seconds
    double minPairAngle = 4.0 / k_DEG; // a pair that turned less than this says too little about the mount
    double smoothWindow = 8.0; // the reference world calibration is the mean over this many seconds either way
    double jumpThreshold = 0.03; // a step of the world calibration this large at the feet, between the 2 s before and after, is a jump
    double feetBelowTarget = 1.5; // where an error is judged: this far below the target device, metres
    double feetSpread = 0.5; // and this far to each side of that point: a calibration turned about the vertical moves nothing right under it
    bool usePoseTimeOffset = false; // take a pose to be of the time the driver said (its poseTimeOffset), not of when it came in
};

struct MountFit {
    bool ok = false;
    Pose mount;
    size_t pairs = 0;
    double rotationResidualDeg = 0.0; // median angle between the two devices' turning axes, after the fit
    double translationResidualMm = 0.0; // median residual of the translation equations
    double translationCondition = 0.0; // smallest over largest eigenvalue of the translation's normal matrix: 0 is degenerate
};

struct Reference {
    bool ok = false;
    std::string error;
    ReferenceOptions options;

    double timeOffset = 0.0; // the target's stream is this much behind the reference's, seconds
    double timeOffsetCorrelation = 0.0;
    std::vector<double> windowOffsets; // the offset of each 20 s window that moved enough to say

    MountFit mount;
    MountFit firstHalf, secondHalf; // the mount from each half of the recording alone: how well a session pins it

    // the world calibration, one entry per reference pose used
    std::vector<double> times;
    std::vector<Pose> instant; // from that one pose pair
    std::vector<Pose> smooth; // the reference
    std::vector<Eigen::Vector3d> targetPositions; // the target device in its own tracking system's world, at that time
    std::vector<int> segment; // which stretch between jumps the entry belongs to
    std::vector<double> jumps; // times the world calibration stepped

    // the reference world calibration and the point errors are judged at, at a time. false where the recording has none
    [[nodiscard]] bool at(double time, Pose& calibration, Eigen::Vector3d& feet) const;
    // how far a world calibration puts the feet from where the reference does, metres. negative where there is no reference
    [[nodiscard]] double feetError(double time, const Pose& calibration) const;
    // how far two calibrations put the feet apart: the worst of the point under the target and four around it
    [[nodiscard]] double feetDistance(const Pose& one, const Pose& other, const Eigen::Vector3d& feet) const;
    // how far a mount (a relative calibration) puts the feet from where the reference mount does, with the head as it was at a
    // time. negative where there is no reference
    [[nodiscard]] double mountFeetError(const Session& session, double time, const Pose& mount) const;
};

Reference solveReference(const Session& session, const ReferenceOptions& options);

// the time a sample is of, by the options
double sampleTime(const Sample& sample, const ReferenceOptions& options);

} // namespace bench
