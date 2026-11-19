#pragma once

#include "protocol.h"
#include <Eigen/Geometry>

namespace spacecal {

enum class DeltaSize {
    TINY,
    SMALL,
    LARGE
};

struct Pose_t {
    Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
};

struct DeviceCalibration_t {
    Pose_t pose;
    DeltaSize eDeltaSize = DeltaSize::TINY;
    bool hasCalibration = false;
    double lastUpdateSeconds = 0.0;
};

// the world pose a driver pose stands for
Eigen::Affine3d getWorldFromDriverPose(const vr::DriverPose_t& pose);

// The driver's side of a relative calibration. The overlay hands it the mount (the target device's pose in the reference device's
// frame); at every pose of a calibrated device this works out the world calibration the two devices' poses of now give with that
// mount, C' = H' * C_L * T'^-1, and moves the calibration it applies toward it, quickly enough to follow the two tracking spaces as
// they shift against each other and slowly enough to average out what one pose pair is off by.
//
// It is a unit of its own, with the time handed in, so that the calibration's bench can run it over a recorded session exactly as
// the driver does (bench/src/replay_driver.cpp).
class RelativeCalibration {
public:
    struct Update {
        bool hasCalibration = false; // `calibration` is one to apply
        bool computed = false; // both devices had a valid pose: the calibration was moved on
    };

    RelativeCalibration();

    void setAlignmentParams(const ipc::protocol::Command_SetAlignmentSpeedParams_t& params) { m_alignmentParams = params; }

    // the device's transform is not a relative one (any more): what was blended for it belongs to the session that ended
    void drop(vr::TrackedDeviceIndex_t deviceIndex);

    // `reference` and `target` are the two devices' latest poses as their drivers gave them, nullptr where a device has none.
    // `calibration` is what to apply to the device's pose now
    Update update(vr::TrackedDeviceIndex_t deviceIndex, const ipc::protocol::Command_SetDeviceTransform_t& transform, const vr::DriverPose_t* reference, const vr::DriverPose_t* target, double nowSeconds, Pose_t& calibration);
    // the same, with `poses` the latest pose of every device
    Update update(vr::TrackedDeviceIndex_t deviceIndex, const ipc::protocol::Command_SetDeviceTransform_t& transform, const vr::DriverPose_t* poses, double nowSeconds, Pose_t& calibration);

private:
    [[nodiscard]] double getTransformRate(DeltaSize delta) const;
    [[nodiscard]] DeltaSize getTransformDeltaSize(DeltaSize priorDelta, const Eigen::Vector3d& deviceWorldPos, const Pose_t& current, const Pose_t& target) const;

    ipc::protocol::Command_SetAlignmentSpeedParams_t m_alignmentParams = {};
    DeviceCalibration_t m_cachedCalibrations[vr::k_unMaxTrackedDeviceCount] = {}; // cache of calibrations for relative calibration
};

} // namespace spacecal
