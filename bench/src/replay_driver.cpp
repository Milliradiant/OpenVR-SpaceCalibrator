#include "replay_driver.h"
#include "driver_pose.h"

// the driver's own unit, built for the bench with the overlay's view of OpenVR's types (protocol.h has them for both)
#include "applied_calibration.h"
#include <memory>

namespace bench {

namespace {

    spacecal::Pose_t poseOf(const Pose& pose)
    {
        return spacecal::Pose_t { .rot = pose.q, .pos = pose.p };
    }

} // namespace

DriverReplayResult replayDriver(const Session& session, const std::vector<CalibrationEvent>& events, const std::vector<DriverProbe>& probes, const spacecal::AppliedCalibration::Settings& settings)
{
    DriverReplayResult result;
    const Device* reference = session.reference();
    const Device* target = session.target();
    if (!reference || !target) {
        return result;
    }

    // the one the session runs through, and one asked at the probes' poses alone
    auto driver = std::make_unique<spacecal::AppliedCalibration>(settings);
    auto probing = std::make_unique<spacecal::AppliedCalibration>(settings);

    vr::DriverPose_t referencePose = {};
    vr::DriverPose_t targetPose = {};
    result.probes.resize(probes.size());
    size_t nextProbe = 0;
    size_t event = 0;
    bool hasEvent = false;
    double lastAppliedTime = -1e9;
    double lastTime = 0.0;
    bool referenceNew = false;
    bool wasCatchingUp = false;

    // the two devices' poses, in the order they came in
    size_t r = 0, t = 0;
    while (r < reference->samples.size() || t < target->samples.size()) {
        const bool takeReference = t >= target->samples.size() || (r < reference->samples.size() && reference->samples[r].time <= target->samples[t].time);
        const Sample& sample = takeReference ? reference->samples[r++] : target->samples[t++];
        if (takeReference) {
            referencePose = driverPoseOf(sample);
            referenceNew = true;
            continue;
        }
        targetPose = driverPoseOf(sample);
        const double seconds = sample.time - lastTime;
        lastTime = sample.time;

        while (event < events.size() && events[event].time <= sample.time) {
            hasEvent = true;
            event++;
        }
        spacecal::Pose_t calibration;

        // a probe is answered at the target's last pose before its time: the pose the live driver took its first calibration at
        while (nextProbe < probes.size() && (t >= target->samples.size() || target->samples[t].time > probes[nextProbe].time)) {
            probing->reset();
            probing->addPair(sample.time, referencePose, targetPose, poseOf(probes[nextProbe].mount));
            if (probing->current(sample.time, calibration)) {
                result.probes[nextProbe] = Pose { calibration.rot, calibration.pos };
            }
            nextProbe++;
        }

        if (!hasEvent) {
            continue;
        }
        const CalibrationEvent& current = events[event - 1];
        if (!current.enabled) {
            driver->reset();
            continue;
        }
        if (current.relative) {
            driver->addPair(sample.time, referencePose, targetPose, poseOf(current.calibration));
        } else if (current.lerp) {
            driver->setWorld(sample.time, poseOf(current.calibration), true);
            driver->watchFrames(targetPose, &referencePose);
        } else {
            driver->setWorld(sample.time, poseOf(current.calibration), false);
        }
        if (!driver->current(sample.time, calibration)) {
            continue;
        }

        const spacecal::AppliedCalibration::Status status = driver->status();
        if (status.relative && sample.valid && seconds < 0.1) {
            result.relativeSeconds += seconds;
            result.holdingSeconds += status.holding ? seconds : 0.0;
            result.catchingUpSeconds += status.catchingUp ? seconds : 0.0;
        }
        result.catchUps += status.catchingUp && !wasCatchingUp;
        wasCatchingUp = status.catchingUp;

        if (sample.time - lastAppliedTime >= 0.1) {
            lastAppliedTime = sample.time;
            result.applied.push_back({ sample.time, Pose { calibration.rot, calibration.pos } });
        }
        if (referenceNew && sample.valid) {
            referenceNew = false;
            result.series.push_back({ sample.time, Pose { calibration.rot, calibration.pos }, sample.world.p, status });
        }
    }
    return result;
}

} // namespace bench
