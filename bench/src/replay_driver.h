#pragma once

#include "replay.h"

namespace bench {

// a moment at which to ask what the driver would start a relative calibration from, with this mount
struct DriverProbe {
    double time = 0.0;
    Pose mount;
};

struct DriverReplayResult {
    std::vector<AppliedEvent> applied; // what the driver applies to the target, ten times a second as a recording has it
    std::vector<Pose> probes; // for each probe, the calibration a relative session beginning at that pose starts from
};

// Runs the driver's own relative calibration (src/driver/relative_calibration.cpp, built into the bench as it is) over a
// recording: every pose of the reference and the target goes through it in the order and at the times they came in, with the
// calibrations in `events` in force from their times on, as the overlay set them live or as a replay of the overlay's code would.
DriverReplayResult replayDriver(const Session& session, const std::vector<CalibrationEvent>& events, const std::vector<DriverProbe>& probes = {});

} // namespace bench
