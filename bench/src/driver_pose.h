#pragma once

// A recorded pose as the vr::DriverPose_t its driver gave: what the overlay's and the driver's own code take.

#include "protocol.h"
#include "session.h"

namespace bench {

inline vr::DriverPose_t driverPoseOf(const Sample& sample)
{
    vr::DriverPose_t pose = {};
    const spacecal::recording::PoseRecord& record = sample.record;
    const spacecal::recording::FrameRecord& frame = sample.frame;
    pose.poseTimeOffset = record.poseTimeOffset;
    pose.qWorldFromDriverRotation = { frame.worldFromDriverRotation[0], frame.worldFromDriverRotation[1], frame.worldFromDriverRotation[2], frame.worldFromDriverRotation[3] };
    pose.qDriverFromHeadRotation = { frame.driverFromHeadRotation[0], frame.driverFromHeadRotation[1], frame.driverFromHeadRotation[2], frame.driverFromHeadRotation[3] };
    pose.qRotation = { record.rotation[0], record.rotation[1], record.rotation[2], record.rotation[3] };
    for (int i = 0; i < 3; i++) {
        pose.vecWorldFromDriverTranslation[i] = frame.worldFromDriverTranslation[i];
        pose.vecDriverFromHeadTranslation[i] = frame.driverFromHeadTranslation[i];
        pose.vecPosition[i] = record.position[i];
        pose.vecVelocity[i] = record.velocity[i];
        pose.vecAcceleration[i] = record.acceleration[i];
        pose.vecAngularVelocity[i] = record.angularVelocity[i];
        pose.vecAngularAcceleration[i] = record.angularAcceleration[i];
    }
    pose.result = static_cast<vr::ETrackingResult>(record.result);
    pose.poseIsValid = record.flags & spacecal::recording::PoseFlag_Valid;
    pose.deviceIsConnected = record.flags & spacecal::recording::PoseFlag_Connected;
    pose.willDriftInYaw = record.flags & spacecal::recording::PoseFlag_WillDriftInYaw;
    pose.shouldApplyHeadModel = record.flags & spacecal::recording::PoseFlag_ShouldApplyHeadModel;
    return pose;
}

} // namespace bench
