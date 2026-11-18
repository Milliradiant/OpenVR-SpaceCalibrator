#include "replay.h"

// the overlay's own headers, with bench/stubs/log.h in the place of its logging
#include "calibration.h"
#include "configuration.h"
#include "vr_core.h"
#include <cstdio>
#include <cstring>

// What the replay stands in for. The overlay's calibration.cpp reaches SteamVR through four things: the VR state (who the devices
// are), the IPC client (the poses in, the calibration out), the configuration (its settings) and its log. Each is given here with
// the overlay's own declarations, backed by the recording. Nothing in calibration.cpp is changed for it.
namespace bench {
namespace {

    struct ReplayState {
        const Session* session = nullptr;
        ReplayOptions options;
        ReplayResult* result = nullptr;
        double time = 0.0;
        size_t cursor[spacecal::recording::k_MAX_DEVICES] = {};
        spacecal::VRDevice_t devices[vr::k_unMaxTrackedDeviceCount] = {};
        std::string pendingNote;
    };
    ReplayState g_replay;

    vr::DriverPose_t driverPoseOf(const Sample& sample)
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

} // namespace

void replayLog(std::string&& line)
{
    if (g_replay.options.verbose) {
        std::printf("    %9.2f  %s\n", g_replay.time, line.c_str());
    }
    if (!g_replay.result) {
        return;
    }
    static const char* rejected = "Rejecting calibration due to ";
    if (line.rfind(rejected, 0) == 0) {
        const size_t end = line.find(';');
        g_replay.result->rejections[line.substr(std::strlen(rejected), end == std::string::npos ? end : end - std::strlen(rejected))]++;
        g_replay.result->computed++;
    } else if (line.rfind("Using new calibration", 0) == 0) {
        g_replay.result->computed++;
        g_replay.pendingNote += line.substr(std::strlen("Using new calibration "));
    } else if (line.rfind("Forcibly applying", 0) == 0) {
        g_replay.pendingNote += "forced, ";
    } else if (line.rfind("Detected tracking discontinuity", 0) == 0) {
        g_replay.pendingNote += "a device's frame stepped, ";
    }
}

} // namespace bench

// --- the VR state: the devices of the recording -------------------------------------------------------------------------------

namespace spacecal {

VRState* VRState::s_instance = nullptr;

bool VRState::init()
{
    s_instance = this;
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        m_aDevices[i] = bench::g_replay.devices[i];
    }
    return true;
}

// as the overlay's own (vr_core.cpp), which cannot be built without SteamVR
const VRDevice_t VRState::findVrDevice(const std::string& trackingSystem, const std::string& model, const std::string& serial) const
{
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        const auto& device = m_aDevices[i];
        if (!device.bIsConnected) {
            continue;
        }
        if (model.empty() || serial.empty() || trackingSystem.empty() || device.szModel.empty() || device.szSerial.empty() || device.szTrackingSystemId.empty()) {
            continue;
        }
        const int matches = (device.szModel == model ? 1 : 0) + (device.szSerial == serial ? 1 : 0);
        const bool isHmd = device.eDeviceClass == vr::TrackedDeviceClass::TrackedDeviceClass_HMD;
        if (device.szTrackingSystemId == trackingSystem && ((matches == 2 && !isHmd) || (matches >= 1 && isHmd))) {
            return device;
        }
    }
    return {};
}

const VRDevice_t VRState::getVrDevice(const size_t index) const
{
    if (index < vr::k_unMaxTrackedDeviceCount) {
        return m_aDevices[index];
    }
    return {};
}

void VRState::debugListDevices() const { }

// --- the configuration: a session set to calibrate continuously between the recording's two devices -----------------------------

ConfigurationManager* ConfigurationManager::s_instance = nullptr;

ConfigurationError ConfigurationManager::init()
{
    s_instance = this;
    return ConfigurationError::Ok;
}

ConfigurationError ConfigurationManager::saveConfiguration() const
{
    return ConfigurationError::Ok;
}

} // namespace spacecal

// --- the IPC client: poses in from the recording, the calibration out to the replay's result -------------------------------------

namespace ipc {

bool IpcClient::Connect()
{
    m_connected = true;
    return true;
}

void IpcClient::Shutdown() { }
void IpcClient::SetAlignmentSpeed(protocol::Command_SetAlignmentSpeedParams_t) { }
void IpcClient::ResetCalibration() { }
void IpcClient::RequestVirtualDesktopProps() { }

// the driver's shared memory holds each device's latest pose: here, the last one the recording has at this time
void IpcClient::PollPoses()
{
    using namespace bench;
    vr::DriverPose_t* poses = spacecal::CalibrationManager::getInstance()->m_poses;
    for (const auto& [index, device] : g_replay.session->devices) {
        size_t& cursor = g_replay.cursor[index];
        while (cursor + 1 < device.samples.size() && device.samples[cursor + 1].time <= g_replay.time) {
            cursor++;
        }
        if (!device.samples.empty() && device.samples[cursor].time <= g_replay.time) {
            poses[index] = driverPoseOf(device.samples[cursor]);
        }
    }
}

void IpcClient::SetDeviceTransform(protocol::Command_SetDeviceTransform_t transform)
{
    using namespace bench;
    if (static_cast<int>(transform.unTargetOpenVrDeviceId) != g_replay.session->targetIndex || !g_replay.result) {
        return;
    }
    CalibrationEvent event;
    event.time = g_replay.time;
    event.enabled = transform.enabled();
    event.relative = transform.relativeCoordSystem();
    event.calibration = Pose {
        Eigen::Quaterniond(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z).normalized(),
        Eigen::Vector3d(transform.translation.v[0], transform.translation.v[1], transform.translation.v[2]),
    };
    event.note = g_replay.pendingNote;
    g_replay.pendingNote.clear();

    // the overlay sends what it has again at every chance. only a change is a new calibration
    std::vector<CalibrationEvent>& events = g_replay.result->events;
    if (!events.empty()) {
        const CalibrationEvent& last = events.back();
        if (last.enabled == event.enabled && last.relative == event.relative && last.calibration.p == event.calibration.p && last.calibration.q.coeffs() == event.calibration.q.coeffs()) {
            return;
        }
    }
    events.push_back(event);
}

} // namespace ipc

// --- OpenVR itself: the calibration only asks whether it is there -----------------------------------------------------------------

extern "C" {
void* VR_GetGenericInterface(const char*, vr::EVRInitError* error)
{
    static int present = 0;
    if (error) {
        *error = vr::VRInitError_None;
    }
    return &present;
}
uint32_t VR_GetInitToken()
{
    return 1;
}
}

namespace bench {

ReplayResult replayOverlay(const Session& session, const ReplayOptions& options)
{
    ReplayResult result;
    const Device* reference = session.reference();
    const Device* target = session.target();
    if (!reference || !target || reference->samples.empty() || target->samples.empty()) {
        result.error = "the recording does not say which two devices the calibration is between";
        return result;
    }
    if (spacecal::CalibrationManager::getInstance() != nullptr) {
        result.error = "one replay a process";
        return result;
    }

    g_replay = ReplayState {};
    g_replay.session = &session;
    g_replay.options = options;
    g_replay.result = &result;

    for (const auto& [index, device] : session.devices) {
        if (index < 0 || index >= static_cast<int>(vr::k_unMaxTrackedDeviceCount) || device.samples.empty()) {
            continue;
        }
        spacecal::VRDevice_t& entry = g_replay.devices[index];
        entry.bIsConnected = true;
        entry.dwDeviceIndex = static_cast<vr::TrackedDeviceIndex_t>(index);
        entry.eDeviceClass = static_cast<vr::TrackedDeviceClass>(device.deviceClass);
        // a device the recording has no name for still has to be found by one
        entry.szTrackingSystemId = device.trackingSystem.empty() ? "system-" + std::to_string(index) : device.trackingSystem;
        entry.szModel = device.model.empty() ? "model-" + std::to_string(index) : device.model;
        entry.szSerial = device.serial.empty() ? "serial-" + std::to_string(index) : device.serial;
    }

    static spacecal::VRState vrState;
    vrState.init();

    static spacecal::ConfigurationManager configuration;
    configuration.init();
    spacecal::Configuration::Calibration_t calibration;
    calibration.is_active = false; // nothing stored: the session calibrates from scratch
    calibration.anchor_mode = options.relative ? spacecal::Configuration::AnchorMode::HmdRelative : spacecal::Configuration::AnchorMode::FixedWorld;
    calibration.continuous.is_active = true;
    calibration.attempt_auto_fix_playspace_jumps = true;
    calibration.calibration_speed = static_cast<uint64_t>(options.samples);
    const auto deviceOf = [&](int index) {
        const spacecal::VRDevice_t& device = g_replay.devices[index];
        return spacecal::Configuration::TrackingDevice { device.szModel, device.szSerial, device.szTrackingSystemId };
    };
    calibration.reference_device = deviceOf(session.referenceIndex);
    calibration.target_device = deviceOf(session.targetIndex);
    configuration.getConfiguration()->calibrations = { calibration };

    static spacecal::CalibrationManager manager;

    // the overlay's loop, from the first moment both devices have a pose
    const double begin = std::max(reference->samples.front().time, target->samples.front().time);
    g_replay.time = begin;
    manager.init();
    const double step = 1.0 / options.tickRate;
    for (double time = begin; time <= session.duration; time += step) {
        g_replay.time = time;
        // the overlay's clock starts at its own launch. a second in, so that no time it keeps is zero
        manager.calibrationTick(time - begin + 1.0);
    }

    g_replay.result = nullptr;
    result.ok = true;
    return result;
}

} // namespace bench
