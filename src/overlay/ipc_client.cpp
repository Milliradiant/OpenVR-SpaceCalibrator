#include "ipc_client.h"
#include "calibration.h"
#include "log.h"
#include "vr_core.h"


namespace ipc {
const IpcFunction_t IpcClient::m_funcs[] = {
    {
        .commandType = protocol::IPC_COMMAND_HANDSHAKE,
        .szFunctionName = "SpaceCalibratorNova_Handshake",
    },
    {
        .commandType = protocol::IPC_COMMAND_SET_ALIGNMENT_SPEED_PARAMS,
        .szFunctionName = "SpaceCalibratorNova_SetAlignmentSpeedParams",
    },
    {
        .commandType = protocol::IPC_COMMAND_RESET_CALIBRATION,
        .szFunctionName = "SpaceCalibratorNova_ResetCalibration",
    },
    {
        .commandType = protocol::IPC_COMMAND_REQUEST_VIRTUAL_DESKTOP_PROPS,
        .szFunctionName = "SpaceCalibratorNova_RequestVirtualDesktopProps",
    },
};

bool IpcClient::Connect()
{
    m_hIpc = ipc_client_init({
        .szSharedMemoryName = protocol::k_szIpcIdentifier,
        .dwSharedBufferSizeBytes = sizeof(protocol::SharedState_t),
        .aFunctions = m_funcs,
        .dwFunctionCount = IPC_COUNT_OF(m_funcs),
        .aOperations = nullptr,
        .dwOperationCount = 0,
        .userData = this,
    });

    if (m_hIpc == k_hInvalidIpcHandle) {
        LOG_IPC_ERROR("Failed to initialise IPC client!");
        m_connected = false;
        return false;
    }

    // send handshake to driver (it logs a protocol other than its own)
    protocol::Command_Handshake_t handshakeArgs;
    ipc_client_dispatch_function(m_hIpc, protocol::IPC_COMMAND_HANDSHAKE, &handshakeArgs, sizeof(handshakeArgs));

    // the poses, the transforms and the headset's metadata are shared through the driver's own memory layout: one of another
    // protocol cannot be read, and is not written to
    size_t bufferSize = 0;
    void* buffer = ipc_get_streaming_buffer(m_hIpc, &bufferSize);
    if (!protocol::hasSharedState(buffer, bufferSize)) {
        if (!m_reportedIncompatibleDriver) {
            m_reportedIncompatibleDriver = true;
            LOG_IPC_ERROR("The driver's shared memory is not of IPC protocol {} ({} bytes): a driver of another version, or one still starting", (uint32_t)protocol::IPC_PROTOCOL_CURRENT, bufferSize);
        }
        ipc_client_shutdown(&m_hIpc);
        m_connected = false;
        return false;
    }
    m_state = static_cast<protocol::SharedState_t*>(buffer);
    m_connected = true;

    LOG_IPC_INFO("IPC connection established: {}", m_connected);

    return m_connected;
}

void IpcClient::Shutdown()
{
    m_state = nullptr;
    m_connected = false;
    ipc_client_shutdown(&m_hIpc);
}

void IpcClient::SetDeviceTransform(protocol::Command_SetDeviceTransform_t deviceTransform)
{
    if (!m_state || deviceTransform.unTargetOpenVrDeviceId >= vr::k_unMaxTrackedDeviceCount) {
        LOG_IPC_ERROR("Tried updating the transform of device {}, but the driver's shared memory is not there", deviceTransform.unTargetOpenVrDeviceId);
        return;
    }
    protocol::storeSlot(m_state->transforms[deviceTransform.unTargetOpenVrDeviceId], deviceTransform);
}
void IpcClient::SetAlignmentSpeed(protocol::Command_SetAlignmentSpeedParams_t alignmentParams)
{
    ipc_client_dispatch_function(m_hIpc, protocol::IPC_COMMAND_SET_ALIGNMENT_SPEED_PARAMS, &alignmentParams, sizeof(alignmentParams));
}
void IpcClient::ResetCalibration()
{
    ipc_client_dispatch_function(m_hIpc, protocol::IPC_COMMAND_RESET_CALIBRATION, nullptr, 0);
}
void IpcClient::RequestVirtualDesktopProps()
{
    if (spacecal::VRState::getInstance()->isHmdVirtualDesktop()) {
        ipc_client_dispatch_function(m_hIpc, protocol::IPC_COMMAND_REQUEST_VIRTUAL_DESKTOP_PROPS, nullptr, 0);
    }
}
// each device's latest pose and the headset's metadata. a slot that could not be read keeps what it had
void IpcClient::PollPoses()
{
    if (!m_state) {
        return;
    }
    vr::DriverPose_t* poses = spacecal::CalibrationManager::getInstance()->m_poses;
    for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        protocol::loadSlot(m_state->poses[i], poses[i]);
    }
    protocol::loadSlot(m_state->hmdMetadata, spacecal::VRState::getInstance()->m_hmdMetadata);
}
}
