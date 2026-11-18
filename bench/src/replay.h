#pragma once

#include "session.h"
#include <map>

namespace bench {

// a calibration handed to the driver for the target device, live or in a replay
struct CalibrationEvent {
    double time = 0.0;
    bool enabled = false;
    bool relative = false; // the calibration is the target's pose in the reference's frame (the mount), not a world calibration
    Pose calibration;
    std::string note;
};

struct ReplayOptions {
    bool relative = true; // with relative calibration on
    int samples = 100; // the overlay's calibration speed: 100 (fast), 250 (slow) or 500 (very slow) samples a calibration
    double tickRate = 90.0; // how often the overlay's loop comes round, a second. it samples 20 times a second within that
    bool verbose = false; // print what the calibration logs
};

struct ReplayResult {
    bool ok = false;
    std::string error;
    std::vector<CalibrationEvent> events;
    size_t computed = 0;
    std::map<std::string, size_t> rejections; // the overlay's own reason -> how many calibrations it turned down for it
};

// Runs the overlay's own calibration (src/overlay/calibration.cpp, unchanged, built into the bench) over a recording as a session
// started fresh in continuous calibration, feeding it the two devices' latest poses at each turn of its loop as the driver's shared
// memory would. One replay a process: the overlay's calibration manager is a singleton.
ReplayResult replayOverlay(const Session& session, const ReplayOptions& options);

} // namespace bench
