#pragma once

#include "applied_calibration.h"
#include "replay.h"

namespace bench {

// a moment at which to ask what the driver would start a relative calibration from, with this mount
struct DriverProbe {
    double time = 0.0;
    Pose mount;
};

// what the driver applied, with where the target device was in its world then
struct AppliedSample {
    double time = 0.0;
    Pose calibration;
    Eigen::Vector3d target = Eigen::Vector3d::Zero();
    spacecal::AppliedCalibration::Status status;
};

struct DriverReplayResult {
    std::vector<AppliedEvent> applied; // what the driver applies to the target, ten times a second as a recording has it
    std::vector<AppliedSample> series; // the same at every new pose of the reference, for how still it holds
    std::vector<Pose> probes; // for each probe, the calibration a relative session beginning at that pose starts from
    double relativeSeconds = 0.0; // under relative calibration, with both devices tracking
    double holdingSeconds = 0.0; // of that, with the two devices' motions disagreeing (no pair counted)
    double catchingUpSeconds = 0.0;
    size_t catchUps = 0; // times it began to catch up, its first calibrations' start included
};

// Runs what the driver applies (src/driver/applied_calibration.cpp, built into the bench as it is) over a recording: every pose of
// the reference and the target goes through it in the order and at the times they came in, as the driver's pose hook takes them,
// with the calibrations in `events` in force from their times on, as the overlay set them live or as a replay of the overlay's code
// would.
DriverReplayResult replayDriver(const Session& session, const std::vector<CalibrationEvent>& events, const std::vector<DriverProbe>& probes = {}, const spacecal::AppliedCalibration::Settings& settings = {});

} // namespace bench
