#pragma once

#include <filesystem>
#include <string>

namespace bench {

// A made-up session with a known answer, written through the driver's own recorder: a headset looking about and holding still by
// turns, a tracker fixed to it, each device's poses at its own rate, latency and noise. For checking the bench itself.
struct SimulateOptions {
    double duration = 300.0;
    unsigned seed = 1;
    double referenceRate = 90.0, targetRate = 250.0; // poses a second
    double referenceLatency = 0.004, targetLatency = 0.016; // how old a pose is when the driver gets it, seconds
    double reportedTargetAge = 0.009; // what the target's driver says of that, in its poseTimeOffset
    double referencePositionNoise = 0.0005, referenceRotationNoise = 0.0008; // metres, radians
    double targetPositionNoise = 0.0003, targetRotationNoise = 0.0005;
    double driftMetres = 0.02, driftDegrees = 0.3; // how far the reference's space wanders against the target's
    double jumpTime = -1.0; // the reference's space steps at this time (a recentre), if positive
};

bool simulate(const std::filesystem::path& file, const SimulateOptions& options, std::string& error);

} // namespace bench
