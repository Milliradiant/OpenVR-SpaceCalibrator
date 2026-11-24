#pragma once

// clang-format off
#include "protocol.h"
#include "shared_state.h"
#include "ipc.h"
// clang-format on

namespace spacecal {
class ServerTrackedDeviceProvider;
}

namespace ipc {
class Server {
public:
    bool Connect(spacecal::ServerTrackedDeviceProvider* driver);
    void Shutdown();

    // the device's pose as its tracking driver gave it, for the overlay and for the driver's own relative calibration. any thread
    void UpdatePose(vr::TrackedDeviceIndex_t unWhichDevice, const vr::DriverPose_t& pose);
    // the latest of them. false for an index that is no device, or before the device's first pose. any thread
    bool GetPose(vr::TrackedDeviceIndex_t unWhichDevice, vr::DriverPose_t& pose) const;
    // what the overlay has the driver do with the device (disabled until it says otherwise). any thread
    protocol::Command_SetDeviceTransform_t GetTransform(vr::TrackedDeviceIndex_t unWhichDevice) const;
    // the headset's metadata, for the overlay
    void UpdateHmdMetadata(const protocol::SharedData_HmdMetadata& metadata);

private:
    static void Callback_Handshake(::IpcCommandType_t cmdType, ::IpcHandle_t hIpcServer, void* pArguments, void* userdata);
    static void Callback_SetAlignmentSpeedParams(::IpcCommandType_t cmdType, ::IpcHandle_t hIpcServer, void* pArguments, void* userdata);
    static void Callback_ResetCalibration(::IpcCommandType_t cmdType, ::IpcHandle_t hIpcServer, void* pArguments, void* userdata);
    static void Callback_RequestVirtualDesktopProps(::IpcCommandType_t cmdType, ::IpcHandle_t hIpcServer, void* pArguments, void* userdata);

private:
    ::IpcHandle_t m_hIpc = k_hInvalidIpcHandle;
    spacecal::ServerTrackedDeviceProvider* m_driver = nullptr;

    // the shared memory's, or m_localState where there is none (the IPC did not start, or has shut down), so that the poses and
    // the relative calibration go on working all the same
    std::atomic<protocol::SharedState_t*> m_state = &m_localState;
    protocol::SharedState_t m_localState = {};
    // the devices that have had a pose: a slot's zeros before its first are no pose
    std::atomic<uint64_t> m_posedDevices = 0;

    static const ::IpcFunction_t m_funcs[];
};
}
