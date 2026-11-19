#include "openvr_driver.h"
#define IPC_IMPLEMENTATION
#include "ipc_server.h"
#include "log.h"
#include "platform.h"
#include "tracked_device_provider.h"

namespace ipc {
const ::IpcFunction_t Server::m_funcs[] = {
    {
        .commandType = protocol::IPC_COMMAND_HANDSHAKE,
        .szFunctionName = "SpaceCalibratorNova_Handshake",
        .callback = &Server::Callback_Handshake,
    },
    {
        .commandType = protocol::IPC_COMMAND_SET_ALIGNMENT_SPEED_PARAMS,
        .szFunctionName = "SpaceCalibratorNova_SetAlignmentSpeedParams",
        .callback = &Server::Callback_SetAlignmentSpeedParams,
    },
    {
        .commandType = protocol::IPC_COMMAND_RESET_CALIBRATION,
        .szFunctionName = "SpaceCalibratorNova_ResetCalibration",
        .callback = &Server::Callback_ResetCalibration,
    },
    {
        .commandType = protocol::IPC_COMMAND_REQUEST_VIRTUAL_DESKTOP_PROPS,
        .szFunctionName = "SpaceCalibratorNova_RequestVirtualDesktopProps",
        .callback = &Server::Callback_RequestVirtualDesktopProps,
    },
};

void Server::Callback_Handshake(::IpcCommandType_t cmdType, ::IpcHandle_t hIpcServer, void* pArguments, void* userdata)
{
    if (pArguments) {
        protocol::Command_Handshake_t* pHandshakeParams = reinterpret_cast<protocol::Command_Handshake_t*>(pArguments);
        Server* pThis = reinterpret_cast<Server*>(userdata);

        platform::setThreadName("IPC Thread");

        if (pHandshakeParams->version != protocol::IPC_PROTOCOL_CURRENT) {
            if (pHandshakeParams->version < protocol::IPC_PROTOCOL_CURRENT) {
                // overlay too old
                LOG_IPC_CRITICAL("The overlay is older than the driver! Expected IPC version {}, got {}.", protocol::IPC_PROTOCOL_CURRENT, pHandshakeParams->version);
            } else {
                // overlay too new
                LOG_IPC_CRITICAL("The overlay is newer than the driver! Expected IPC version {}, got {}.", protocol::IPC_PROTOCOL_CURRENT, pHandshakeParams->version);
            }
            // @TODO: invoke function in client telling it server ipc version to verify version and refuse interop
        } else {
            LOG_IPC_INFO("Overlay connected. Using IPC {}", pHandshakeParams->version);
        }
    }
}

void Server::Callback_SetAlignmentSpeedParams(::IpcCommandType_t cmdType, ::IpcHandle_t hIpcServer, void* pArguments, void* userdata)
{
    if (pArguments) {
        protocol::Command_SetAlignmentSpeedParams_t* pAlignmentParams = reinterpret_cast<protocol::Command_SetAlignmentSpeedParams_t*>(pArguments);
        Server* pThis = reinterpret_cast<Server*>(userdata);
        if (pThis && pAlignmentParams) {
            pThis->m_driver->SetAlignmentSpeedParams(*pAlignmentParams);
        } else {
            LOG_IPC_ERROR("Server::Callback_SetAlignmentSpeedParams failed due to nullptr");
        }
    }
}

void Server::Callback_ResetCalibration(::IpcCommandType_t cmdType, ::IpcHandle_t hIpcServer, void* pArguments, void* userdata)
{
    if (pArguments) {
        Server* pThis = reinterpret_cast<Server*>(userdata);
        if (pThis) {
            pThis->m_driver->ResetCalibration();
        }

        // LOG_IPC_INFO("Server::Callback_ResetCalibration");
    }
}

void Server::Callback_RequestVirtualDesktopProps(::IpcCommandType_t cmdType, ::IpcHandle_t hIpcServer, void* pArguments, void* userdata)
{
    if (pArguments) {
        Server* pThis = reinterpret_cast<Server*>(userdata);
        if (pThis) {
            pThis->m_driver->RequestVirtualDesktopProps();
        }

        // LOG_IPC_INFO("Server::Callback_RequestVirtualDesktopProps");
    }
}

void Server::UpdatePose(vr::TrackedDeviceIndex_t unWhichDevice, const vr::DriverPose_t& pose)
{
    if (unWhichDevice >= vr::k_unMaxTrackedDeviceCount) {
        LOG_IPC_FATAL("Attempted to update shared pose buffer, but got invalid device index {}...", unWhichDevice);
        return;
    }
    // only this device's pose is written: no other pose is touched and no lock is taken that anything else could hold
    protocol::storeSlotShared(m_state.load(std::memory_order_acquire)->poses[unWhichDevice], pose);
    m_posedDevices.fetch_or(uint64_t(1) << unWhichDevice, std::memory_order_relaxed);
}

bool Server::GetPose(vr::TrackedDeviceIndex_t unWhichDevice, vr::DriverPose_t& pose) const
{
    if (unWhichDevice >= vr::k_unMaxTrackedDeviceCount || !(m_posedDevices.load(std::memory_order_relaxed) & (uint64_t(1) << unWhichDevice))) {
        return false;
    }
    return protocol::loadSlot(m_state.load(std::memory_order_acquire)->poses[unWhichDevice], pose);
}

protocol::Command_SetDeviceTransform_t Server::GetTransform(vr::TrackedDeviceIndex_t unWhichDevice) const
{
    // the last transform this thread had for each device: should a read ever not come through (writes kept starting on the
    // copy being read, where the overlay writes a device's transform a few times a second at most), the device goes on with it
    thread_local protocol::Command_SetDeviceTransform_t t_lastTransforms[vr::k_unMaxTrackedDeviceCount] = {};
    if (unWhichDevice >= vr::k_unMaxTrackedDeviceCount) {
        return {};
    }
    protocol::Command_SetDeviceTransform_t& last = t_lastTransforms[unWhichDevice];
    protocol::loadSlot(m_state.load(std::memory_order_acquire)->transforms[unWhichDevice], last);
    return last;
}

bool Server::GetTransformIfChanged(vr::TrackedDeviceIndex_t unWhichDevice, uint32_t& sequence, protocol::Command_SetDeviceTransform_t& transform) const
{
    if (unWhichDevice >= vr::k_unMaxTrackedDeviceCount) {
        return false;
    }
    const protocol::SharedSlot_t<protocol::Command_SetDeviceTransform_t>& slot = m_state.load(std::memory_order_acquire)->transforms[unWhichDevice];
    const uint32_t current = protocol::shared::loadSequence(slot.sequence, std::memory_order_relaxed);
    if (current == sequence || !protocol::loadSlot(slot, transform)) {
        return false;
    }
    sequence = current;
    return true;
}

void Server::UpdateHmdMetadata(const protocol::SharedData_HmdMetadata& metadata)
{
    protocol::storeSlot(m_state.load(std::memory_order_acquire)->hmdMetadata, metadata);
}

bool Server::Connect(spacecal::ServerTrackedDeviceProvider* driver)
{
    m_driver = driver;
    m_state.store(&m_localState, std::memory_order_release);

    m_hIpc = ::ipc_server_init({
        .szSharedMemoryName = protocol::k_szIpcIdentifier,
        .dwSharedBufferSizeBytes = sizeof(protocol::SharedState_t),
        .aFunctions = m_funcs,
        .dwFunctionCount = IPC_COUNT_OF(m_funcs),
        .aOperations = nullptr,
        .dwOperationCount = 0,
        .userData = this,
    });

    if (m_hIpc == k_hInvalidIpcHandle) {
        LOG_IPC_ERROR("Failed to initialise IPC server!");
        return false;
    }

    size_t bufferSize = 0;
    void* buffer = ::ipc_get_streaming_buffer(m_hIpc, &bufferSize);
    if (!buffer || bufferSize < sizeof(protocol::SharedState_t)) {
        LOG_IPC_ERROR("The IPC's shared memory is too small for the shared state ({} bytes, {} needed)", bufferSize, sizeof(protocol::SharedState_t));
        return false;
    }

    // the IPC has zeroed it: every slot empty, every transform disabled. the magic goes in last, for the overlay to see it whole
    protocol::SharedState_t* state = static_cast<protocol::SharedState_t*>(buffer);
    std::atomic_ref<uint64_t>(state->magic).store(protocol::k_SHARED_STATE_MAGIC, std::memory_order_release);
    m_state.store(state, std::memory_order_release);
    return true;
}

void Server::Shutdown()
{
    // what is shared goes back to this process's own memory before the shared memory goes
    m_state.store(&m_localState, std::memory_order_release);
    ::ipc_server_shutdown(&m_hIpc);
}
}
