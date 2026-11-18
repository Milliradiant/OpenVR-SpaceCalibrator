#pragma once

#include "geometry.h"
#include "pose_recording.h"
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace bench {

// one recorded pose update of a device
struct Sample {
    double time = 0.0; // when the driver received it, seconds since the recording began
    bool valid = false; // connected, pose valid, tracking result Running_OK
    double poseTimeOffset = 0.0;
    Pose raw; // the driver pose, in the driver's space
    Pose world; // world-from-driver * the driver pose: what the calibration works with
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); // as reported, driver space
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero(); // as reported
    spacecal::recording::PoseRecord record;
    spacecal::recording::FrameRecord frame;
};

struct TransformEvent {
    double time = 0.0;
    spacecal::recording::TransformRecord record;
};

struct AppliedEvent {
    double time = 0.0;
    Pose calibration;
};

struct LatencyEvent {
    double time = 0.0;
    double latency = 0.0;
    double confidence = 0.0;
};

struct Device {
    int index = -1;
    std::string serial, model, trackingSystem;
    int deviceClass = 0;
    std::vector<Sample> samples;
    std::vector<double> frameChanges; // times its world-from-driver or driver-from-head changed
    std::vector<TransformEvent> transforms;
    std::vector<AppliedEvent> applied;
    std::vector<LatencyEvent> latency;

    [[nodiscard]] std::string name() const;
    // the device's world pose at a time, between the two samples around it. false outside the samples, across a gap longer than
    // maxGap, or where either sample is not valid
    [[nodiscard]] bool worldAt(double time, Pose& pose, double maxGap = 0.1) const;
};

struct Session {
    std::filesystem::path path;
    std::map<std::string, std::string> info; // the Session record's key=value lines
    std::vector<std::pair<double, std::string>> notes;
    std::map<int, Device> devices;
    double duration = 0.0;
    uint64_t records = 0;
    uint64_t skippedRecords = 0;
    int referenceIndex = -1;
    int targetIndex = -1;

    bool load(const std::filesystem::path& file, std::string& error);
    [[nodiscard]] const Device* reference() const;
    [[nodiscard]] const Device* target() const;
};

} // namespace bench
