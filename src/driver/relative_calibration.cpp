#include "relative_calibration.h"

#include <algorithm>

namespace spacecal {

namespace {

    Eigen::Quaterniond eigenFromHmdQuat(const vr::HmdQuaternion_t& quat)
    {
        return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
    }
    Eigen::Translation3d eigenTransFromHmdVec(const double pos[3])
    {
        return Eigen::Translation3d(pos[0], pos[1], pos[2]);
    }
    Eigen::Vector3d eigenVecFromHmdVec(const double pos[3])
    {
        return Eigen::Vector3d(pos[0], pos[1], pos[2]);
    }

    bool isDeviceIndexValid(const vr::TrackedDeviceIndex_t index)
    {
        return index < vr::k_unMaxTrackedDeviceCount && index != vr::k_unTrackedDeviceIndexInvalid && index != vr::k_unTrackedDeviceIndexOther;
    }
    bool isPoseValid(const vr::DriverPose_t& pose)
    {
        return pose.deviceIsConnected && pose.poseIsValid && pose.result == vr::ETrackingResult::TrackingResult_Running_OK;
    }

    constexpr double k_MM = 0.001;
    constexpr double k_DEGREES = EIGEN_PI / 180.0;

} // namespace

static DeltaSize maxDelta(DeltaSize a, DeltaSize b)
{
    return (static_cast<int>(a) > static_cast<int>(b)) ? a : b;
}

DeltaSize RelativeCalibration::getTransformDeltaSize(DeltaSize priorDelta, const Eigen::Vector3d& deviceWorldPos, const Pose_t& current, const Pose_t& target) const
{
    const Eigen::Vector3d srcPos = current.pos + current.rot * deviceWorldPos;
    const Eigen::Vector3d targetPos = target.pos + target.rot * deviceWorldPos;

    const double transDelta = (srcPos - targetPos).squaredNorm();
    const double rotDelta = current.rot.angularDistance(target.rot);

    DeltaSize transLevel, rotLevel;

    if (transDelta > m_alignmentParams.thr_trans_large)
        transLevel = DeltaSize::LARGE;
    else if (transDelta > m_alignmentParams.thr_trans_small)
        transLevel = DeltaSize::SMALL;
    else
        transLevel = DeltaSize::TINY;

    if (rotDelta > m_alignmentParams.thr_rot_large)
        rotLevel = DeltaSize::LARGE;
    else if (rotDelta > m_alignmentParams.thr_rot_small)
        rotLevel = DeltaSize::SMALL;
    else
        rotLevel = DeltaSize::TINY;

    if (transLevel == DeltaSize::TINY && rotLevel == DeltaSize::TINY)
        return DeltaSize::TINY;

    return maxDelta(priorDelta, maxDelta(transLevel, rotLevel));
}

double RelativeCalibration::getTransformRate(DeltaSize delta) const
{
    switch (delta) {
    case DeltaSize::TINY:
        return m_alignmentParams.align_speed_tiny;
    case DeltaSize::SMALL:
        return m_alignmentParams.align_speed_small;
    default:
        return m_alignmentParams.align_speed_large;
    }
}

// lerps between two calibrations by lerp factor % around local point
static Pose_t interpolatePoseAround(double lerp, const Eigen::Vector3d& localPoint, const Pose_t& current, const Pose_t& target)
{
    const Eigen::Vector3d initialWorld = current.pos + current.rot * localPoint;
    const Eigen::Vector3d targetWorld = target.pos + target.rot * localPoint;
    const Eigen::Vector3d finalWorld = initialWorld * (1.0 - lerp) + targetWorld * lerp;

    Eigen::Quaterniond rot = current.rot.slerp(lerp, target.rot);
    Eigen::Vector3d pos = finalWorld - rot * localPoint;

    return Pose_t { .rot = rot, .pos = pos };
}

Eigen::Affine3d getWorldFromDriverPose(const vr::DriverPose_t& pose)
{
    const Eigen::Quaterniond worldFromDriverRot = eigenFromHmdQuat(pose.qWorldFromDriverRotation);
    const Eigen::Vector3d worldFromDriverPos = eigenVecFromHmdVec(pose.vecWorldFromDriverTranslation);

    const Eigen::Quaterniond rot = worldFromDriverRot * eigenFromHmdQuat(pose.qRotation);
    const Eigen::Vector3d pos = worldFromDriverPos + worldFromDriverRot * eigenVecFromHmdVec(pose.vecPosition);

    return Eigen::Translation3d(pos) * rot;
}

RelativeCalibration::RelativeCalibration()
{
    // How fast the applied calibration follows the one the two devices' poses give, by how far apart the two are (the translation
    // thresholds are squared distances at the device). Measured over recorded sessions with the calibration's bench: one pose pair
    // gives the calibration to about 3 mm at the feet, and a blend with a time constant of half a second to a second halves that
    // and still follows the two tracking spaces as they move against each other; at the 20 s there was here before, it lags them
    // by more than a pose pair's noise.
    m_alignmentParams.thr_rot_tiny = 0.2 * k_DEGREES;
    m_alignmentParams.thr_rot_small = 1.5 * k_DEGREES;
    m_alignmentParams.thr_rot_large = 4.0 * k_DEGREES;

    m_alignmentParams.thr_trans_tiny = (5.0 * k_MM) * (5.0 * k_MM);
    m_alignmentParams.thr_trans_small = (25.0 * k_MM) * (25.0 * k_MM);
    m_alignmentParams.thr_trans_large = (60.0 * k_MM) * (60.0 * k_MM);

    m_alignmentParams.align_speed_tiny = 1.5;
    m_alignmentParams.align_speed_small = 3.0;
    m_alignmentParams.align_speed_large = 10.0;
}

void RelativeCalibration::drop(vr::TrackedDeviceIndex_t deviceIndex)
{
    if (deviceIndex < vr::k_unMaxTrackedDeviceCount) {
        m_cachedCalibrations[deviceIndex].hasCalibration = false;
    }
}

RelativeCalibration::Update RelativeCalibration::update(vr::TrackedDeviceIndex_t deviceIndex, const ipc::protocol::Command_SetDeviceTransform_t& transform, const vr::DriverPose_t* poses, double nowSeconds, Pose_t& calibration)
{
    const vr::DriverPose_t* reference = isDeviceIndexValid(transform.unRelativeReferenceOpenvrDeviceId) ? &poses[transform.unRelativeReferenceOpenvrDeviceId] : nullptr;
    const vr::DriverPose_t* target = isDeviceIndexValid(transform.unRelativeTargetOpenvrDeviceId) ? &poses[transform.unRelativeTargetOpenvrDeviceId] : nullptr;
    return update(deviceIndex, transform, reference, target, nowSeconds, calibration);
}

RelativeCalibration::Update RelativeCalibration::update(vr::TrackedDeviceIndex_t deviceIndex, const ipc::protocol::Command_SetDeviceTransform_t& transform, const vr::DriverPose_t* reference, const vr::DriverPose_t* target, double nowSeconds, Pose_t& calibration)
{
    Update result;
    if (deviceIndex >= vr::k_unMaxTrackedDeviceCount) {
        return result;
    }
    DeviceCalibration_t& cached = m_cachedCalibrations[deviceIndex];

    if (reference && isPoseValid(*reference) && target && isPoseValid(*target)) {
        // The two devices' latest poses, as their drivers gave them (H' and T'). Neither is predicted and no latency between them
        // is made up for. Both were, and over recorded sessions both made the result worse: the prediction turned a pose by its
        // turning rate about the driver's axes, where drivers give the rate in the device's own, and the latency estimate was
        // either never there (above some 300 poses a second it never had enough of both devices' samples) or, made to work, noisier
        // than the few milliseconds it stood for. A world calibration holds whatever the poses' time, so nothing is lost by taking
        // the pair as it is; what the pair is off by, the blend below averages out.
        const Eigen::Affine3d refWorldNow = getWorldFromDriverPose(*reference);
        const Eigen::Affine3d targetWorldNow = getWorldFromDriverPose(*target);

        // local-space calibration transform (C_L)
        const Eigen::Affine3d calibrationLocal = eigenTransFromHmdVec(transform.translation.v) * eigenFromHmdQuat(transform.rotation);

        // where the target is expected with the mount applied, T_H = H' * C_L, and the calibration that takes its pose there:
        // C' = T_H * (T')^-1
        const Eigen::Affine3d worldCalib = refWorldNow * calibrationLocal * targetWorldNow.inverse();

        const Pose_t targetCalib = {
            .rot = Eigen::Quaterniond(worldCalib.rotation()).normalized(),
            .pos = worldCalib.translation(),
        };

        if (!cached.hasCalibration) {
            cached = {
                .pose = targetCalib,
                .eDeltaSize = DeltaSize::TINY,
                .hasCalibration = true,
                .lastUpdateSeconds = nowSeconds,
            };
        } else {
            const double deltaTime = nowSeconds - cached.lastUpdateSeconds;
            cached.lastUpdateSeconds = nowSeconds;

            const Eigen::Vector3d deviceWorldPos = targetWorldNow.translation();
            cached.eDeltaSize = getTransformDeltaSize(cached.eDeltaSize, deviceWorldPos, cached.pose, targetCalib);

            // the blend goes at its rate whatever else: it was multiplied by the latency estimate's confidence, and where that
            // was zero the calibration of the first pose pair was applied for as long as the session lasted
            const double lerp = std::clamp(deltaTime * getTransformRate(cached.eDeltaSize), 0.0, 1.0);
            cached.pose = interpolatePoseAround(lerp, deviceWorldPos, cached.pose, targetCalib);
        }
        result.computed = true;
    }

    // the calibration is kept, so that it goes on being applied while either device has lost tracking
    result.hasCalibration = cached.hasCalibration;
    calibration = cached.pose;
    return result;
}

} // namespace spacecal
