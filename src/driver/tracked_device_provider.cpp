#include "tracked_device_provider.h"

#include "interface_hook_injector.h"
#include "log.h"
#include "util.h"
#include "virtual_desktop.h"
#include "vrmath.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <openvr_driver.h>

namespace spacecal {
vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext* pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

    util::init();
    logging::Init(/* isOverlay */ false);
    hmd::initVD();

    LOG_OPENVR_INFO("Starting SpaceCalibrator-Nova. Compiled against OpenVR driver API {}.{}.{}", vr::k_nSteamVRVersionMajor, vr::k_nSteamVRVersionMinor, vr::k_nSteamVRVersionBuild);

    m_sessionRecorder.start(util::getSpaceCalibratorConfigDir() / "recordings");

    m_ipcServer.Connect(this);
    hooking::InjectHooks(this, pDriverContext);

    return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Cleanup()
{
    LOG_OPENVR_INFO("Shutting down SpaceCalibrator-Nova...");

    hooking::DisableHooks();
    m_ipcServer.Shutdown();
    m_sessionRecorder.stop();
    VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

const char* const* ServerTrackedDeviceProvider::GetInterfaceVersions()
{
    return vr::k_InterfaceVersions;
}
void ServerTrackedDeviceProvider::RunFrame()
{
    // we query hmd refresh rate for prediction
    vr::PropertyContainerHandle_t hHmdContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(vr::k_unTrackedDeviceIndex_Hmd);
    const double displayFrequency = vr::VRProperties()->GetFloatProperty(hHmdContainer, vr::Prop_DisplayFrequency_Float);
    m_fVsyncPredictionTime = 1.0 / displayFrequency;

    // device state may get desynced so we update it here
    vr::TrackedDevicePose_t rawPoses[vr::k_unMaxTrackedDeviceCount] = {};
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses((float)m_fVsyncPredictionTime, rawPoses, vr::k_unMaxTrackedDeviceCount);

    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        vr::DriverPose_t latest;
        if (!IsPoseValid(rawPoses[i]) && m_ipcServer.GetPose(i, latest) && IsPoseValid(latest)) {
            vr::DriverPose_t corrected = latest;
            corrected.deviceIsConnected = rawPoses[i].bDeviceIsConnected;
            corrected.poseIsValid = rawPoses[i].bPoseIsValid;
            corrected.result = rawPoses[i].eTrackingResult;
            m_ipcServer.UpdatePose(i, corrected);
        }
    }

    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        m_ipcServer.GetTransformIfChanged(i, m_frameTransformSequences[i], m_frameTransforms[i]);
    }
    m_sessionRecorder.onFrame(m_frameTransforms, displayFrequency);
}
bool ServerTrackedDeviceProvider::ShouldBlockStandbyMode()
{
    return false;
}

// @TODO: should spacecal inform the overlay to ignore calibrations during standby?
void ServerTrackedDeviceProvider::EnterStandby() { }
void ServerTrackedDeviceProvider::LeaveStandby() { }

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double (&vector)[3])
{
    vr::HmdQuaternion_t vectorQuat = { 0.0, vector[0], vector[1], vector[2] };
    auto rotatedVectorQuat = quat * vectorQuat * -quat;
    return { rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z };
}
inline void copyVec3(double dest[3], const vr::HmdVector3d_t& src)
{
    ASSERT(dest != nullptr, "dest was not a valid memory address!");
    dest[0] = src.v[0];
    dest[1] = src.v[1];
    dest[2] = src.v[2];
}

inline vr::HmdQuaternion_t hmdQuatFromEigen(const Eigen::Quaterniond& quat)
{
    return vr::HmdQuaternion_t { quat.w(), quat.x(), quat.y(), quat.z() };
}
inline vr::HmdVector3d_t hmdVecFromEigenVec(const Eigen::Vector3d& pos)
{
    return vr::HmdVector3d_t { .v = { pos.x(), pos.y(), pos.z() } };
}
inline vr::HmdVector3d_t hmdVecFromEigenVec(const Eigen::Translation3d& pos)
{
    return vr::HmdVector3d_t { .v = { pos.x(), pos.y(), pos.z() } };
}

inline Eigen::Quaterniond eigenFromHmdQuat(const vr::HmdQuaternion_t& quat)
{
    return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
}
inline Eigen::Translation3d eigenTransFromHmdVec(const double pos[3])
{
    return Eigen::Translation3d(pos[0], pos[1], pos[2]);
}
inline Eigen::Vector3d eigenVecFromHmdVec(const double pos[3])
{
    return Eigen::Vector3d(pos[0], pos[1], pos[2]);
}

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(vr::TrackedDeviceIndex_t unWhichDevice, vr::DriverPose_t& newPose)
{
    // bounds check, as sometimes the id is invalid?
    if (!IsDeviceIndexValid(unWhichDevice))
        return true;

    // the pose as the tracking driver gave it, before anything below changes it
    m_sessionRecorder.onPose(unWhichDevice, newPose);

    m_ipcServer.UpdatePose(unWhichDevice, newPose);

    vr::DriverPose_t& modifiedPose = newPose;

    const ipc::protocol::Command_SetDeviceTransform_t transform = m_ipcServer.GetTransform(unWhichDevice);

    // the cached calibration belongs to the relative session it was blended in. drop it once that session ends, so the next one
    // starts from its own first calibration instead of blending its way out of a stale one
    if (!transform.enabled() || !transform.relativeCoordSystem()) {
        m_relativeCalibration.drop(unWhichDevice);
    }

    if (transform.hideContinuousTracker()) {
        newPose.vecPosition[0] = -newPose.vecWorldFromDriverTranslation[0];
        newPose.vecPosition[1] = -newPose.vecWorldFromDriverTranslation[1] + 9001; // put it 9001m above the origin
        newPose.vecPosition[2] = -newPose.vecWorldFromDriverTranslation[2];
    } else if (transform.enabled()) {
        HandleQuirks(transform.quirks, modifiedPose);

        if (transform.relativeCoordSystem()) {
            Pose_t calibration;
            const double nowSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            // the two devices' latest poses as their drivers gave them (this one's is the pose above, before HandleQuirks)
            vr::DriverPose_t referencePose, targetPose;
            const bool hasReference = IsDeviceIndexValid(transform.unRelativeReferenceOpenvrDeviceId) && m_ipcServer.GetPose(transform.unRelativeReferenceOpenvrDeviceId, referencePose);
            const bool hasTarget = IsDeviceIndexValid(transform.unRelativeTargetOpenvrDeviceId) && m_ipcServer.GetPose(transform.unRelativeTargetOpenvrDeviceId, targetPose);
            const RelativeCalibration::Update update = m_relativeCalibration.update(unWhichDevice, transform, hasReference ? &referencePose : nullptr, hasTarget ? &targetPose : nullptr, nowSeconds, calibration);
            if (update.hasCalibration) {
                m_sessionRecorder.onApplied(unWhichDevice, calibration.rot, calibration.pos);
            }

            applyCalibrationToPose(
                modifiedPose,
                hmdQuatFromEigen(calibration.rot),
                hmdVecFromEigenVec(calibration.pos),
                transform.scale,
                transform.calibrateMotionVecs());
        } else {
            // classic world-space application
            applyCalibrationToPose(modifiedPose, transform.rotation, transform.translation, transform.scale, transform.calibrateMotionVecs());
        }
    }

    return true;
}

void ServerTrackedDeviceProvider::applyCalibrationToPose(vr::DriverPose_t& pose, vr::HmdQuaternion_t rotation, vr::HmdVector3d_t pos, double scale, bool calibrateMotionVecs)
{
    // apply like classic world-space calibration
    pose.qWorldFromDriverRotation = rotation * pose.qWorldFromDriverRotation;

    pose.vecPosition[0] *= scale;
    pose.vecPosition[1] *= scale;
    pose.vecPosition[2] *= scale;

    vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(rotation, pose.vecWorldFromDriverTranslation);
    pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + pos.v[0];
    pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + pos.v[1];
    pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + pos.v[2];

    // @NOTE: remove branch in future update once i determine this works well
    if (calibrateMotionVecs) {
        // velocity
        pose.vecVelocity[0] *= scale;
        pose.vecVelocity[1] *= scale;
        pose.vecVelocity[2] *= scale;

        // acceleration
        pose.vecAcceleration[0] *= scale;
        pose.vecAcceleration[1] *= scale;
        pose.vecAcceleration[2] *= scale;
    }
}

void ServerTrackedDeviceProvider::HandleQuirks(ipc::protocol::DeviceQuirks_t quirks, vr::DriverPose_t& pose)
{
    // @TODO: implement quirks handling

    if (ENUM_HAS_FLAGS(quirks, ipc::protocol::DeviceQuirks_t::QUIRK_SCALE)) {
        // m_poses->vecAngularVelocity
    }

    if (ENUM_HAS_FLAGS(quirks, ipc::protocol::DeviceQuirks_t::QUIRK_RECOMPUTE_VELOCITY)) {
        // m_poses->vecAngularVelocity
    }

    if (ENUM_HAS_FLAGS(quirks, ipc::protocol::DeviceQuirks_t::QUIRK_RECOMPUTE_ANGULAR_VELOCITY)) {
        // m_poses->vecAngularVelocity
    }

    // this improves pose prediction on some vendors' devices (eg Pico)
    if (ENUM_HAS_FLAGS(quirks, ipc::protocol::DeviceQuirks_t::QUIRK_HAS_WORLDSPACE_ANGULAR_VELOCITY)) {
        // convert quat from world space to local space
        vr::HmdQuaternion_t qInverse = -pose.qRotation; // invert quaternion, i should probably make this an explicit function bc i dont like operators being used like this
        vr::HmdVector3d_t localAngularVel = quaternionRotateVector(qInverse, pose.vecAngularVelocity);
        copyVec3(pose.vecAngularVelocity, localAngularVel);
    }
}

void ServerTrackedDeviceProvider::ResetCalibration()
{
    // @TODO: idk what would be more correct than "random fucking offsets"
}

void ServerTrackedDeviceProvider::RequestVirtualDesktopProps()
{
    // this simply states whether or not the vd driver is loaded. it doesnt tell us if the active hmd IS vd
    m_hmdMetaData.isVirtualDesktopAvailable = vr::VRDriverManager()->GetDriverHandle("virtualdesktop") != vr::k_ulInvalidDriverHandle;
    // NOTE: DO NOT INVOKE IF VD IS NOT THE ACTIVE HMD! WILL CAUSE CRASH!!
    if (m_hmdMetaData.isVirtualDesktopAvailable) {
        m_hmdMetaData.VD_hmdModel = hmd::VD_getHmdModel();
        m_hmdMetaData.VD_stageTrackingEnabled = hmd::VD_isStageTrackingEnabled();
    }
    m_ipcServer.UpdateHmdMetadata(m_hmdMetaData);
}

void ServerTrackedDeviceProvider::SetAlignmentSpeedParams(ipc::protocol::Command_SetAlignmentSpeedParams_t& params)
{
    m_relativeCalibration.setAlignmentParams(params);
}
}
