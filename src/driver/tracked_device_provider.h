#pragma once

#include "ipc_server.h"
#include "applied_calibration.h"
#include <Eigen/Geometry>
#include <chrono>
#include <openvr_driver.h>

namespace spacecal {

class ServerTrackedDeviceProvider : public vr::IServerTrackedDeviceProvider {
public:
    vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override;
    void Cleanup() override;
    const char* const* GetInterfaceVersions() override;
    void RunFrame() override;
    bool ShouldBlockStandbyMode() override;
    void EnterStandby() override;
    void LeaveStandby() override;

    bool HandleDevicePoseUpdated(vr::TrackedDeviceIndex_t unWhichDevice, vr::DriverPose_t& newPose);
    void ResetCalibration();
    void RequestVirtualDesktopProps();
    void SetAlignmentSpeedParams(ipc::protocol::Command_SetAlignmentSpeedParams_t& params);
    void HandleQuirks(ipc::protocol::DeviceQuirks_t quirks, vr::DriverPose_t& pose);

    [[nodiscard]] inline bool IsDeviceIndexValid(const vr::TrackedDeviceIndex_t index) const
    {
        return index < vr::k_unMaxTrackedDeviceCount && index != vr::k_unTrackedDeviceIndexInvalid && index != vr::k_unTrackedDeviceIndexOther;
    }

    [[nodiscard]] inline bool IsPoseValid(const vr::DriverPose_t pose) const
    {
        return pose.deviceIsConnected && pose.poseIsValid && pose.result == vr::ETrackingResult::TrackingResult_Running_OK;
    }

    [[nodiscard]] inline bool IsPoseValid(const vr::TrackedDevicePose_t pose) const
    {
        return pose.bDeviceIsConnected && pose.bPoseIsValid && pose.eTrackingResult == vr::ETrackingResult::TrackingResult_Running_OK;
    }

private:
    void applyCalibrationToPose(vr::DriverPose_t& pose, vr::HmdQuaternion_t rotation, vr::HmdVector3d_t pos, double scale, bool calibrateMotionVecs);

private:
    ipc::Server m_ipcServer;
    double m_fVsyncPredictionTime = 0.0;
    ipc::protocol::SharedData_HmdMetadata m_hmdMetaData = {};

    // what is applied for each continuous calibration, by the calibration's own target device
    AppliedCalibration m_appliedCalibrations[vr::k_unMaxTrackedDeviceCount];
    friend class ipc::Server;
};
}
