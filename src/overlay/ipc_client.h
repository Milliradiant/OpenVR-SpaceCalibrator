#pragma once

// clang-format off
#include "protocol.h"
#include "shared_state.h"
#include "ipc.h"
// clang-format on

namespace ipc {
class IpcClient {
public:
    bool Connect();
    void Shutdown();

    inline const bool IsConnected() const { return m_connected; }

    void SetDeviceTransform(protocol::Command_SetDeviceTransform_t deviceTransform);
    void SetAlignmentSpeed(protocol::Command_SetAlignmentSpeedParams_t alignmentParams);
    void ResetCalibration();
    void RequestVirtualDesktopProps();
    void PollPoses();

private:
    bool m_connected = false;
    bool m_reportedIncompatibleDriver = false;
    ::IpcHandle_t m_hIpc = k_hInvalidIpcHandle;
    protocol::SharedState_t* m_state = nullptr; // the driver's shared memory, once it is there and of this protocol

    static const IpcFunction_t m_funcs[];

    friend class VRState; // for m_poses
};
}
