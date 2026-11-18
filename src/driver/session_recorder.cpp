#include "session_recorder.h"

#include "constants.h"
#include "latency_estimator.h"
#include "log.h"
#include <cstring>
#include <ctime>

namespace spacecal {

static uint64_t deviceBit(vr::TrackedDeviceIndex_t deviceIndex)
{
    return deviceIndex < recording::k_MAX_DEVICES ? (uint64_t(1) << deviceIndex) : 0;
}

double SessionRecorder::now() const
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - m_start).count();
}

void SessionRecorder::start(const std::filesystem::path& directory)
{
    stop();

    vr::EVRSettingsError settingsError = vr::VRSettingsError_None;
    bool enabled = vr::VRSettings()->GetBool(k_SETTINGS_SECTION, k_SETTINGS_RECORD_POSES, &settingsError);
    if (settingsError != vr::VRSettingsError_None) {
        enabled = true;
    }
    if (!enabled) {
        LOG_OPENVR_INFO("Pose recording is switched off ({}.{})", k_SETTINGS_SECTION, k_SETTINGS_RECORD_POSES);
        return;
    }

    settingsError = vr::VRSettingsError_None;
    int32_t maxMegabytes = vr::VRSettings()->GetInt32(k_SETTINGS_SECTION, k_SETTINGS_RECORDINGS_MAX_MB, &settingsError);
    if (settingsError != vr::VRSettingsError_None || maxMegabytes <= 0) {
        maxMegabytes = k_RECORDINGS_DEFAULT_MAX_MB;
    }

    m_start = std::chrono::steady_clock::now();
    if (!m_writer.open(directory, uint64_t(maxMegabytes) * 1024 * 1024)) {
        LOG_OPENVR_WARN("Not recording poses: {}", m_writer.error());
        return;
    }

    const std::time_t wallClock = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    char utc[32] = {};
    std::tm utcTime = {};
#if defined(_WIN32)
    gmtime_s(&utcTime, &wallClock);
#else
    gmtime_r(&wallClock, &utcTime);
#endif
    std::strftime(utc, sizeof(utc), "%Y-%m-%dT%H:%M:%SZ", &utcTime);

    std::string session = "format=1\n";
    session += "program=Space Calibrator driver " SPACECAL_VERSION_STRING "\n";
    session += std::string("started_utc=") + utc + "\n";
    session += "started_unix=" + std::to_string(static_cast<long long>(wallClock)) + "\n";
    session += "thinned_interval=" + std::to_string(k_RECORD_THINNED_INTERVAL_SEC) + "\n";
    m_writer.session(0.0, session);
    m_writer.flush(true);

    {
        std::lock_guard<std::mutex> lock(m_writerMutex);
        m_stopWriter = false;
    }
    m_reportedStopped = false;
    m_reportedDropped = 0;
    m_writerThread = std::thread([this] { writerLoop(); });

    LOG_OPENVR_INFO("Recording this session's poses to \"{}\" (the recordings together are kept under {} MB)", m_writer.path().string(), maxMegabytes);
}

void SessionRecorder::stop()
{
    if (m_writerThread.joinable()) {
        {
            std::lock_guard<std::mutex> lock(m_writerMutex);
            m_stopWriter = true;
        }
        m_writerWake.notify_all();
        m_writerThread.join();
    }

    if (m_writer.isOpen()) {
        m_writer.note(now(), "end of session");
    }
    m_writer.close();
}

SessionRecorder::~SessionRecorder()
{
    // the driver stops the recording when SteamVR cleans it up. if the process is going down without that, the thread is already
    // gone with it and waiting for it here would hang the exit: let it go, and the file keeps what was written
    if (m_writerThread.joinable()) {
        m_writerThread.detach();
    }
}

void SessionRecorder::writerLoop()
{
    double lastSyncTime = now();
    std::unique_lock<std::mutex> lock(m_writerMutex);
    while (!m_stopWriter) {
        m_writerWake.wait_for(lock, std::chrono::duration<double>(k_RECORD_WRITE_INTERVAL_SEC));
        lock.unlock();

        const double time = now();
        const bool sync = time - lastSyncTime >= k_RECORD_SYNC_INTERVAL_SEC;
        if (sync) {
            lastSyncTime = time;
        }
        m_writer.flush(sync);

        lock.lock();
    }
}

// nothing a recording does may reach the tracking it sits in: these three are called from SteamVR's own threads, and whatever goes
// wrong in them (memory, most likely) ends the recording's part of that call and nothing else
void SessionRecorder::onPose(vr::TrackedDeviceIndex_t deviceIndex, const vr::DriverPose_t& pose) noexcept
try {
    if (!m_writer.isOpen() || deviceIndex >= recording::k_MAX_DEVICES) {
        return;
    }

    const double time = now();
    const uint64_t bit = deviceBit(deviceIndex);
    m_seenDevices.fetch_or(bit, std::memory_order_relaxed);

    recording::FrameRecord frame;
    frame.worldFromDriverRotation[0] = pose.qWorldFromDriverRotation.w;
    frame.worldFromDriverRotation[1] = pose.qWorldFromDriverRotation.x;
    frame.worldFromDriverRotation[2] = pose.qWorldFromDriverRotation.y;
    frame.worldFromDriverRotation[3] = pose.qWorldFromDriverRotation.z;
    frame.driverFromHeadRotation[0] = pose.qDriverFromHeadRotation.w;
    frame.driverFromHeadRotation[1] = pose.qDriverFromHeadRotation.x;
    frame.driverFromHeadRotation[2] = pose.qDriverFromHeadRotation.y;
    frame.driverFromHeadRotation[3] = pose.qDriverFromHeadRotation.z;
    std::memcpy(frame.worldFromDriverTranslation, pose.vecWorldFromDriverTranslation, sizeof(frame.worldFromDriverTranslation));
    std::memcpy(frame.driverFromHeadTranslation, pose.vecDriverFromHeadTranslation, sizeof(frame.driverFromHeadTranslation));

    bool frameChanged = false;
    {
        std::lock_guard<std::mutex> lock(m_poseStateMutex);
        if (!(m_fullRateDevices.load(std::memory_order_relaxed) & bit)) {
            if (time - m_lastThinnedPoseTime[deviceIndex] < k_RECORD_THINNED_INTERVAL_SEC) {
                return;
            }
            m_lastThinnedPoseTime[deviceIndex] = time;
        }
        if (!m_hasFrame[deviceIndex] || !(m_frames[deviceIndex] == frame)) {
            m_hasFrame[deviceIndex] = true;
            m_frames[deviceIndex] = frame;
            frameChanged = true;
        }
    }

    if (frameChanged) {
        m_writer.frame(time, static_cast<uint8_t>(deviceIndex), frame);
    }

    recording::PoseRecord record;
    record.flags = (pose.poseIsValid ? recording::PoseFlag_Valid : 0)
        | (pose.deviceIsConnected ? recording::PoseFlag_Connected : 0)
        | (pose.willDriftInYaw ? recording::PoseFlag_WillDriftInYaw : 0)
        | (pose.shouldApplyHeadModel ? recording::PoseFlag_ShouldApplyHeadModel : 0);
    record.result = static_cast<int32_t>(pose.result);
    record.poseTimeOffset = pose.poseTimeOffset;
    record.rotation[0] = pose.qRotation.w;
    record.rotation[1] = pose.qRotation.x;
    record.rotation[2] = pose.qRotation.y;
    record.rotation[3] = pose.qRotation.z;
    std::memcpy(record.position, pose.vecPosition, sizeof(record.position));
    std::memcpy(record.velocity, pose.vecVelocity, sizeof(record.velocity));
    std::memcpy(record.angularVelocity, pose.vecAngularVelocity, sizeof(record.angularVelocity));
    std::memcpy(record.acceleration, pose.vecAcceleration, sizeof(record.acceleration));
    std::memcpy(record.angularAcceleration, pose.vecAngularAcceleration, sizeof(record.angularAcceleration));
    for (int i = 0; i < 3; i++) {
        if (pose.vecAcceleration[i] != 0.0 || pose.vecAngularAcceleration[i] != 0.0) {
            record.flags |= recording::PoseFlag_HasAcceleration;
        }
    }
    m_writer.pose(time, static_cast<uint8_t>(deviceIndex), record);
} catch (...) {
}

void SessionRecorder::onApplied(vr::TrackedDeviceIndex_t deviceIndex, const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation) noexcept
try {
    if (!m_writer.isOpen() || deviceIndex >= recording::k_MAX_DEVICES) {
        return;
    }

    const double time = now();
    const double interval = deviceIndex == m_targetDevice.load(std::memory_order_relaxed) ? k_RECORD_APPLIED_TARGET_INTERVAL_SEC : k_RECORD_APPLIED_OTHER_INTERVAL_SEC;
    {
        std::lock_guard<std::mutex> lock(m_poseStateMutex);
        if (time - m_lastAppliedTime[deviceIndex] < interval) {
            return;
        }
        m_lastAppliedTime[deviceIndex] = time;
    }

    recording::AppliedRecord record;
    record.rotation[0] = rotation.w();
    record.rotation[1] = rotation.x();
    record.rotation[2] = rotation.y();
    record.rotation[3] = rotation.z();
    record.translation[0] = translation.x();
    record.translation[1] = translation.y();
    record.translation[2] = translation.z();
    m_writer.applied(time, static_cast<uint8_t>(deviceIndex), record);
} catch (...) {
}

void SessionRecorder::onFrame(const ipc::protocol::Command_SetDeviceTransform_t* transforms, LatencyEstimator& latencyEstimator, double displayFrequency) noexcept
try {
    if (!m_writer.isOpen()) {
        if (m_writerThread.joinable() && !m_reportedStopped) {
            m_reportedStopped = true;
            LOG_OPENVR_WARN("Stopped recording poses: {}", m_writer.error());
        }
        return;
    }
    const double time = now();

    // what the overlay asks the driver to do with each device, whenever that changes: every calibration it accepts shows here
    uint64_t fullRateDevices = 1;
    vr::TrackedDeviceIndex_t referenceDevice = vr::k_unTrackedDeviceIndexInvalid;
    vr::TrackedDeviceIndex_t targetDevice = vr::k_unTrackedDeviceIndexInvalid;
    for (uint32_t i = 0; i < recording::k_MAX_DEVICES; i++) {
        const ipc::protocol::Command_SetDeviceTransform_t& transform = transforms[i];

        recording::TransformRecord record;
        record.flags = (transform.enabled() ? recording::TransformFlag_Enabled : 0)
            | (transform.lerpCalibrations() ? recording::TransformFlag_LerpCalibrations : 0)
            | (transform.hideContinuousTracker() ? recording::TransformFlag_HideContinuousTracker : 0)
            | (transform.relativeCoordSystem() ? recording::TransformFlag_RelativeCoordSystem : 0)
            | (transform.calibrateMotionVecs() ? recording::TransformFlag_CalibrateMotionVecs : 0);
        record.quirks = static_cast<uint16_t>(transform.quirks);
        record.referenceDevice = transform.unRelativeReferenceOpenvrDeviceId;
        record.targetDevice = transform.unRelativeTargetOpenvrDeviceId;
        std::memcpy(record.translation, transform.translation.v, sizeof(record.translation));
        record.rotation[0] = transform.rotation.w;
        record.rotation[1] = transform.rotation.x;
        record.rotation[2] = transform.rotation.y;
        record.rotation[3] = transform.rotation.z;
        record.scale = transform.scale;

        // a device the overlay never touched has the same zeroes as the record this starts from, and writes nothing
        const bool untouched = record.flags == 0 && record.referenceDevice == 0 && record.targetDevice == 0 && record.scale == 0.0;
        if (!untouched && !(record == m_transforms[i])) {
            m_transforms[i] = record;
            m_writer.transform(time, static_cast<uint8_t>(i), record);
        }

        if (transform.enabled()) {
            if (transform.unRelativeReferenceOpenvrDeviceId < recording::k_MAX_DEVICES) {
                referenceDevice = transform.unRelativeReferenceOpenvrDeviceId;
            }
            if (transform.unRelativeTargetOpenvrDeviceId < recording::k_MAX_DEVICES) {
                targetDevice = transform.unRelativeTargetOpenvrDeviceId;
            }
        }
    }
    fullRateDevices |= deviceBit(referenceDevice) | deviceBit(targetDevice);
    m_fullRateDevices.store(fullRateDevices, std::memory_order_relaxed);
    m_targetDevice.store(targetDevice, std::memory_order_relaxed);

    // who each device is, once SteamVR knows
    if (time - m_lastDeviceScanTime >= k_RECORD_DEVICE_SCAN_INTERVAL_SEC) {
        m_lastDeviceScanTime = time;
        const uint64_t seenDevices = m_seenDevices.load(std::memory_order_relaxed);
        for (uint32_t i = 0; i < recording::k_MAX_DEVICES; i++) {
            if (!(seenDevices & deviceBit(i))) {
                continue;
            }
            const vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(i);
            if (container == vr::k_ulInvalidPropertyContainer) {
                continue;
            }
            const std::string serial = vr::VRProperties()->GetStringProperty(container, vr::Prop_SerialNumber_String);
            const std::string model = vr::VRProperties()->GetStringProperty(container, vr::Prop_ModelNumber_String);
            const std::string trackingSystem = vr::VRProperties()->GetStringProperty(container, vr::Prop_TrackingSystemName_String);
            const int32_t deviceClass = vr::VRProperties()->GetInt32Property(container, vr::Prop_DeviceClass_Int32);

            const std::string text = serial + "\t" + model + "\t" + trackingSystem + "\t" + std::to_string(deviceClass);
            if (text != m_deviceTexts[i]) {
                m_deviceTexts[i] = text;
                m_writer.device(time, static_cast<uint8_t>(i), serial, model, trackingSystem, deviceClass);
            }
        }
    }

    if (referenceDevice < recording::k_MAX_DEVICES && targetDevice < recording::k_MAX_DEVICES && time - m_lastLatencyTime >= k_RECORD_LATENCY_INTERVAL_SEC) {
        m_lastLatencyTime = time;
        recording::LatencyRecord record;
        record.referenceDevice = static_cast<uint8_t>(referenceDevice);
        record.latency = latencyEstimator.get_latency(referenceDevice, targetDevice, record.confidence);
        m_writer.latency(time, static_cast<uint8_t>(targetDevice), record);
    }

    if (displayFrequency != m_displayFrequency && displayFrequency > 0.0) {
        m_displayFrequency = displayFrequency;
        m_writer.note(time, "display_frequency=" + std::to_string(displayFrequency));
    }

    const uint64_t dropped = m_writer.droppedRecords();
    if (dropped != m_reportedDropped) {
        m_reportedDropped = dropped;
        LOG_OPENVR_WARN("The pose recording dropped records ({} so far): the disk is not keeping up", dropped);
    }

} catch (...) {
}

} // namespace spacecal
