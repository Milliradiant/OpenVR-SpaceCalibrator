#pragma once

#include "pose_recording.h"
#include "protocol.h"
#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <openvr_driver.h>
#include <string>
#include <thread>

namespace spacecal {

class LatencyEstimator;

constexpr const char* k_SETTINGS_SECTION = "driver_01spacecalibrator";
constexpr const char* k_SETTINGS_RECORD_POSES = "recordPoses"; // whether a session's poses are recorded. on unless set to false
constexpr const char* k_SETTINGS_RECORDINGS_MAX_MB = "recordingsMaxMB"; // what the recordings may take together before the oldest go
constexpr int32_t k_RECORDINGS_DEFAULT_MAX_MB = 1024;
constexpr double k_RECORD_THINNED_INTERVAL_SEC = 0.2; // a device the calibration does not use is recorded this often
constexpr double k_RECORD_APPLIED_TARGET_INTERVAL_SEC = 0.1; // the calibration applied to the target device
constexpr double k_RECORD_APPLIED_OTHER_INTERVAL_SEC = 1.0; // and to the tracking system's other devices
constexpr double k_RECORD_LATENCY_INTERVAL_SEC = 0.2;
constexpr double k_RECORD_DEVICE_SCAN_INTERVAL_SEC = 2.0;
constexpr double k_RECORD_WRITE_INTERVAL_SEC = 0.25; // how often what was recorded goes to the file
constexpr double k_RECORD_SYNC_INTERVAL_SEC = 2.0; // how much of a recording a crash of SteamVR can cost

// Records a SteamVR session for the calibration's bench (pose_recording.h has the format): the raw poses of the reference and the
// target device and of the headset at the rate they come in, the other devices thinned, and what the overlay had the driver apply.
// It only reads what passes through the driver, and changes no pose. The file is written by a thread of its own, so that no thread
// of SteamVR's ever waits for the disk.
class SessionRecorder {
public:
    ~SessionRecorder();

    // reads the settings and begins a recording in `directory`. does nothing if recording is switched off or the file cannot be made
    void start(const std::filesystem::path& directory);
    void stop();

    // a pose as the tracking driver gave it, before any calibration. any thread
    void onPose(vr::TrackedDeviceIndex_t deviceIndex, const vr::DriverPose_t& pose) noexcept;
    // the world calibration just applied to a device under relative calibration. any thread
    void onApplied(vr::TrackedDeviceIndex_t deviceIndex, const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation) noexcept;
    // once per driver frame, from the thread that runs it: what the overlay asks for and who the devices are
    void onFrame(const ipc::protocol::Command_SetDeviceTransform_t* transforms, LatencyEstimator& latencyEstimator, double displayFrequency) noexcept;

private:
    [[nodiscard]] double now() const;
    void writerLoop();

    recording::Writer m_writer;
    std::chrono::steady_clock::time_point m_start {};

    std::thread m_writerThread;
    std::mutex m_writerMutex;
    std::condition_variable m_writerWake;
    bool m_stopWriter = false; // under m_writerMutex

    std::atomic<uint64_t> m_fullRateDevices = 1; // bit per device index. the headset always is
    std::atomic<uint64_t> m_seenDevices = 0;
    std::atomic<uint32_t> m_targetDevice = vr::k_unTrackedDeviceIndexInvalid;

    // per device, for the pose threads
    std::mutex m_poseStateMutex;
    double m_lastThinnedPoseTime[recording::k_MAX_DEVICES] = {};
    double m_lastAppliedTime[recording::k_MAX_DEVICES] = {};
    bool m_hasFrame[recording::k_MAX_DEVICES] = {};
    recording::FrameRecord m_frames[recording::k_MAX_DEVICES] = {};

    // for the frame thread alone
    recording::TransformRecord m_transforms[recording::k_MAX_DEVICES] = {};
    std::string m_deviceTexts[recording::k_MAX_DEVICES];
    double m_lastDeviceScanTime = -1e9;
    double m_lastLatencyTime = -1e9;
    double m_displayFrequency = 0.0;
    uint64_t m_reportedDropped = 0;
    bool m_reportedStopped = false;
};

} // namespace spacecal
