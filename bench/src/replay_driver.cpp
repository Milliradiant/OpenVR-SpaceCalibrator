#include "replay_driver.h"
#include "driver_pose.h"

// the driver's own unit, built for the bench with the overlay's view of OpenVR's types (protocol.h has them for both)
#include "relative_calibration.h"
#include <memory>

namespace bench {

namespace {

    ipc::protocol::Command_SetDeviceTransform_t transformOf(const Session& session, const Pose& mount)
    {
        ipc::protocol::Command_SetDeviceTransform_t transform = {};
        transform.enabled(true);
        transform.relativeCoordSystem(true);
        transform.unTargetOpenVrDeviceId = static_cast<vr::TrackedDeviceIndex_t>(session.targetIndex);
        transform.unRelativeReferenceOpenvrDeviceId = static_cast<vr::TrackedDeviceIndex_t>(session.referenceIndex);
        transform.unRelativeTargetOpenvrDeviceId = static_cast<vr::TrackedDeviceIndex_t>(session.targetIndex);
        transform.translation = { { mount.p.x(), mount.p.y(), mount.p.z() } };
        transform.rotation = { mount.q.w(), mount.q.x(), mount.q.y(), mount.q.z() };
        return transform;
    }

} // namespace

DriverReplayResult replayDriver(const Session& session, const std::vector<CalibrationEvent>& events, const std::vector<DriverProbe>& probes)
{
    DriverReplayResult result;
    const Device* reference = session.reference();
    const Device* target = session.target();
    if (!reference || !target) {
        return result;
    }

    // the one the session runs through, and one asked at the probes' poses alone
    auto driver = std::make_unique<spacecal::RelativeCalibration>();
    auto probing = std::make_unique<spacecal::RelativeCalibration>();

    static vr::DriverPose_t poses[vr::k_unMaxTrackedDeviceCount];
    for (vr::DriverPose_t& pose : poses) {
        pose = {};
    }

    const vr::TrackedDeviceIndex_t targetIndex = static_cast<vr::TrackedDeviceIndex_t>(session.targetIndex);
    const vr::TrackedDeviceIndex_t referenceIndex = static_cast<vr::TrackedDeviceIndex_t>(session.referenceIndex);
    result.probes.resize(probes.size());
    size_t nextProbe = 0;
    size_t event = 0;
    bool hasEvent = false;
    double lastAppliedTime = -1e9;

    // the two devices' poses, in the order they came in
    size_t r = 0, t = 0;
    while (r < reference->samples.size() || t < target->samples.size()) {
        const bool takeReference = t >= target->samples.size() || (r < reference->samples.size() && reference->samples[r].time <= target->samples[t].time);
        const Sample& sample = takeReference ? reference->samples[r++] : target->samples[t++];
        const vr::TrackedDeviceIndex_t index = takeReference ? referenceIndex : targetIndex;

        poses[index] = driverPoseOf(sample);
        if (takeReference) {
            continue;
        }

        while (event < events.size() && events[event].time <= sample.time) {
            hasEvent = true;
            event++;
        }
        spacecal::Pose_t calibration;

        // a probe is answered at the target's last pose before its time: the pose the live driver took its first calibration at
        while (nextProbe < probes.size() && (t >= target->samples.size() || target->samples[t].time > probes[nextProbe].time)) {
            probing->drop(targetIndex);
            const spacecal::RelativeCalibration::Update update = probing->update(targetIndex, transformOf(session, probes[nextProbe].mount), poses, sample.time, calibration);
            if (update.hasCalibration) {
                result.probes[nextProbe] = Pose { calibration.rot, calibration.pos };
            }
            nextProbe++;
        }

        if (!hasEvent) {
            continue;
        }
        const CalibrationEvent& current = events[event - 1];
        if (!current.enabled || !current.relative) {
            driver->drop(targetIndex);
            continue;
        }
        const spacecal::RelativeCalibration::Update update = driver->update(targetIndex, transformOf(session, current.calibration), poses, sample.time, calibration);
        if (update.hasCalibration && sample.time - lastAppliedTime >= 0.1) {
            lastAppliedTime = sample.time;
            result.applied.push_back({ sample.time, Pose { calibration.rot, calibration.pos } });
        }
    }
    return result;
}

} // namespace bench
