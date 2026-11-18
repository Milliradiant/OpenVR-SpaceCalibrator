// The driver's recorder (src/driver/session_recorder.cpp) as the driver builds it, run against a stand-in for SteamVR: its
// settings and its device properties. What it is called with, from several threads at once, has to be what its file holds.

#include "session_recorder.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <vector>

using namespace spacecal;
using namespace spacecal::recording;
namespace fs = std::filesystem;

static int g_failures = 0;
static std::vector<std::string> g_log;
static std::mutex g_logMutex;

// where the driver's log lines go in this build (bench/stubs/log.h)
namespace bench {
void replayLog(std::string&& line)
{
    std::lock_guard<std::mutex> lock(g_logMutex);
    g_log.push_back(std::move(line));
}
}

#define CHECK(condition)                                                    \
    do {                                                                    \
        if (!(condition)) {                                                 \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            g_failures++;                                                   \
        }                                                                   \
    } while (0)

static bool logged(const char* start)
{
    std::lock_guard<std::mutex> lock(g_logMutex);
    for (const std::string& line : g_log) {
        if (line.rfind(start, 0) == 0) {
            return true;
        }
    }
    return false;
}

// --- SteamVR, as far as the recorder asks -----------------------------------------------------------------------------------------

struct Settings : vr::IVRSettings {
    std::map<std::string, bool> bools;
    std::map<std::string, int32_t> ints;

    const char* GetSettingsErrorNameFromEnum(vr::EVRSettingsError) override { return ""; }
    void SetBool(const char*, const char*, bool, vr::EVRSettingsError*) override { }
    void SetInt32(const char*, const char*, int32_t, vr::EVRSettingsError*) override { }
    void SetFloat(const char*, const char*, float, vr::EVRSettingsError*) override { }
    void SetString(const char*, const char*, const char*, vr::EVRSettingsError*) override { }
    bool GetBool(const char* section, const char* key, vr::EVRSettingsError* error) override
    {
        const auto found = bools.find(std::string(section) + "." + key);
        if (error) {
            *error = found == bools.end() ? vr::VRSettingsError_UnsetSettingHasNoDefault : vr::VRSettingsError_None;
        }
        return found != bools.end() && found->second;
    }
    int32_t GetInt32(const char* section, const char* key, vr::EVRSettingsError* error) override
    {
        const auto found = ints.find(std::string(section) + "." + key);
        if (error) {
            *error = found == ints.end() ? vr::VRSettingsError_UnsetSettingHasNoDefault : vr::VRSettingsError_None;
        }
        return found == ints.end() ? 0 : found->second;
    }
    float GetFloat(const char*, const char*, vr::EVRSettingsError*) override { return 0.f; }
    void GetString(const char*, const char*, char* value, uint32_t length, vr::EVRSettingsError*) override
    {
        if (length) {
            value[0] = 0;
        }
    }
    void RemoveSection(const char*, vr::EVRSettingsError*) override { }
    void RemoveKeyInSection(const char*, const char*, vr::EVRSettingsError*) override { }
};

struct Properties : vr::IVRProperties {
    struct Device {
        std::string serial, model, trackingSystem;
        int32_t deviceClass = 0;
    };
    std::map<uint32_t, Device> devices;

    vr::ETrackedPropertyError ReadPropertyBatch(vr::PropertyContainerHandle_t container, vr::PropertyRead_t* batch, uint32_t count) override
    {
        const auto found = devices.find(static_cast<uint32_t>(container - 1));
        for (uint32_t i = 0; i < count; i++) {
            vr::PropertyRead_t& read = batch[i];
            read.eError = vr::TrackedProp_UnknownProperty;
            read.unRequiredBufferSize = 0;
            if (found == devices.end()) {
                continue;
            }
            const std::string* text = read.prop == vr::Prop_SerialNumber_String ? &found->second.serial
                : read.prop == vr::Prop_ModelNumber_String ? &found->second.model
                : read.prop == vr::Prop_TrackingSystemName_String ? &found->second.trackingSystem
                : nullptr;
            if (text) {
                read.unTag = vr::k_unStringPropertyTag;
                read.unRequiredBufferSize = static_cast<uint32_t>(text->size() + 1);
                if (read.unBufferSize < read.unRequiredBufferSize) {
                    read.eError = vr::TrackedProp_BufferTooSmall;
                } else {
                    std::memcpy(read.pvBuffer, text->c_str(), text->size() + 1);
                    read.eError = vr::TrackedProp_Success;
                }
            } else if (read.prop == vr::Prop_DeviceClass_Int32 && read.unBufferSize >= sizeof(int32_t)) {
                read.unTag = vr::k_unInt32PropertyTag;
                read.unRequiredBufferSize = sizeof(int32_t);
                std::memcpy(read.pvBuffer, &found->second.deviceClass, sizeof(int32_t));
                read.eError = vr::TrackedProp_Success;
            }
        }
        return vr::TrackedProp_Success;
    }
    vr::ETrackedPropertyError WritePropertyBatch(vr::PropertyContainerHandle_t, vr::PropertyWrite_t*, uint32_t) override { return vr::TrackedProp_Success; }
    const char* GetPropErrorNameFromEnum(vr::ETrackedPropertyError) override { return ""; }
    vr::PropertyContainerHandle_t TrackedDeviceToPropertyContainer(vr::TrackedDeviceIndex_t device) override
    {
        return devices.count(device) ? device + 1 : vr::k_ulInvalidPropertyContainer;
    }
};

struct Context : vr::IVRDriverContext {
    Settings settings;
    Properties properties;

    void* GetGenericInterface(const char* version, vr::EVRInitError* error) override
    {
        if (error) {
            *error = vr::VRInitError_None;
        }
        if (!std::strcmp(version, vr::IVRSettings_Version)) {
            return static_cast<vr::IVRSettings*>(&settings);
        }
        if (!std::strcmp(version, vr::IVRProperties_Version)) {
            return static_cast<vr::IVRProperties*>(&properties);
        }
        if (error) {
            *error = vr::VRInitError_Init_InterfaceNotFound;
        }
        return nullptr;
    }
    vr::DriverHandle_t GetDriverHandle() override { return 1; }
};

// --- the session ------------------------------------------------------------------------------------------------------------------

static vr::DriverPose_t poseAt(double x, double worldY = 0.0)
{
    vr::DriverPose_t pose = {};
    pose.poseIsValid = true;
    pose.deviceIsConnected = true;
    pose.result = vr::TrackingResult_Running_OK;
    pose.poseTimeOffset = -0.004;
    pose.qWorldFromDriverRotation = { 1.0, 0.0, 0.0, 0.0 };
    pose.qDriverFromHeadRotation = { 1.0, 0.0, 0.0, 0.0 };
    pose.qRotation = { 0.6, 0.0, 0.8, 0.0 };
    pose.vecWorldFromDriverTranslation[1] = worldY;
    pose.vecPosition[0] = x;
    pose.vecVelocity[2] = 0.25;
    pose.vecAngularVelocity[1] = -1.5;
    return pose;
}

static std::vector<Record> readAll(const fs::path& file)
{
    std::vector<Record> records;
    Reader reader;
    CHECK(reader.open(file));
    Record record;
    while (reader.next(record)) {
        records.push_back(record);
    }
    return records;
}

static std::vector<fs::path> recordingsIn(const fs::path& directory)
{
    std::vector<fs::path> files;
    std::error_code ec;
    for (fs::directory_iterator it(directory, ec), end; !ec && it != end; it.increment(ec)) {
        files.push_back(it->path());
    }
    std::sort(files.begin(), files.end());
    return files;
}

static double secondsSince(std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

int main()
{
    const fs::path directory = fs::temp_directory_path() / ("spacecal_recorder_test_" + std::to_string(std::rand()));
    fs::remove_all(directory);

    Context context;
    vr::InitServerDriverContext(&context); // most of SteamVR is not there, and the recorder asks for none of it
    context.properties.devices[0] = { "HEADSET-1", "Some Headset", "oculus", vr::TrackedDeviceClass_HMD };
    context.properties.devices[3] = { "LHR-TEST0003", "Some Tracker", "lighthouse", vr::TrackedDeviceClass_GenericTracker };
    context.properties.devices[4] = { "LHR-TEST0004", "Some Tracker", "lighthouse", vr::TrackedDeviceClass_GenericTracker };
    context.properties.devices[5] = { "CTRL-5", "Some Controller", "oculus", vr::TrackedDeviceClass_Controller };

    ipc::protocol::Command_SetDeviceTransform_t transforms[vr::k_unMaxTrackedDeviceCount] = {};

    // switched off in the settings: no folder, no file, and every call is nothing
    {
        context.settings.bools["driver_01spacecalibrator.recordPoses"] = false;
        SessionRecorder recorder;
        recorder.start(directory / "off");
        recorder.onPose(0, poseAt(1.0));
        recorder.onApplied(3, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
        recorder.onFrame(transforms, 90.0);
        recorder.stop();
        CHECK(!fs::exists(directory / "off"));
        CHECK(logged("Pose recording is switched off"));
        context.settings.bools.clear();
    }

    // a session: the headset (0), the calibration's tracker (3) and another of its system (4), a controller nobody calibrates (5)
    const fs::path recordings = directory / "recordings";
    SessionRecorder recorder;
    recorder.start(recordings);
    CHECK(logged("Recording this session's poses to"));
    CHECK(recordingsIn(recordings).size() == 1);

    // before the overlay has said anything, each device shows up once: SteamVR knows who they are from here
    for (uint32_t device : { 0u, 3u, 4u, 5u }) {
        recorder.onPose(device, poseAt(0.0));
    }
    recorder.onPose(64, poseAt(0.0)); // no such device: nothing
    recorder.onFrame(transforms, 90.0);

    // the overlay sets a relative calibration for the lighthouse devices, between the headset and tracker 3
    for (uint32_t device : { 3u, 4u }) {
        transforms[device].enabled(true);
        transforms[device].relativeCoordSystem(true);
        transforms[device].unTargetOpenVrDeviceId = device;
        transforms[device].unRelativeReferenceOpenvrDeviceId = 0;
        transforms[device].unRelativeTargetOpenvrDeviceId = 3;
        transforms[device].translation = { { 0.001, 0.084, -0.055 } };
        transforms[device].rotation = { 0.5, 0.5, -0.5, 0.5 };
    }
    // and switches one off that it never switched on, as it does for the devices of other tracking systems
    transforms[5].enabled(false);
    transforms[5].unTargetOpenVrDeviceId = 5;
    recorder.onFrame(transforms, 90.0);

    constexpr double k_DURATION = 1.1;
    const auto start = std::chrono::steady_clock::now();
    int headsetPoses = 0, trackerPoses = 0, controllerPoses = 0;
    std::thread headset([&] {
        while (secondsSince(start) < k_DURATION) {
            // the headset's driver recentres its space two thirds of the way in
            recorder.onPose(0, poseAt(0.001 * headsetPoses, secondsSince(start) < 0.7 ? 0.0 : 0.25));
            headsetPoses++;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });
    std::thread lighthouse([&] {
        while (secondsSince(start) < k_DURATION) {
            recorder.onPose(3, poseAt(0.001 * trackerPoses));
            recorder.onApplied(3, Eigen::Quaterniond(0.8, 0.0, 0.6, 0.0), Eigen::Vector3d(1.0, 2.0, 3.0));
            recorder.onPose(4, poseAt(0.001 * trackerPoses));
            recorder.onApplied(4, Eigen::Quaterniond(0.8, 0.0, 0.6, 0.0), Eigen::Vector3d(1.0, 2.0, 3.0));
            trackerPoses++;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    std::thread controller([&] {
        while (secondsSince(start) < k_DURATION) {
            recorder.onPose(5, poseAt(0.001 * controllerPoses));
            controllerPoses++;
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
    });
    bool recalibrated = false;
    while (secondsSince(start) < k_DURATION) {
        if (!recalibrated && secondsSince(start) > 0.5) {
            // a new calibration from the overlay, and SteamVR's own name for the tracker arriving late
            recalibrated = true;
            transforms[3].translation = { { 0.002, 0.085, -0.056 } };
            transforms[4].translation = { { 0.002, 0.085, -0.056 } };
        }
        recorder.onFrame(transforms, 90.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(11));
    }
    headset.join();
    lighthouse.join();
    controller.join();
    recorder.stop();

    const std::vector<fs::path> files = recordingsIn(recordings);
    CHECK(files.size() == 1);
    const std::vector<Record> records = files.empty() ? std::vector<Record> {} : readAll(files[0]);
    CHECK(records.size() > 100);

    std::map<int, int> poses, frames, transformsOf, applied, devices;
    std::map<int, double> lastPoseTime;
    int latencies = 0, displayNotes = 0;
    bool ordered = true;
    for (const Record& record : records) {
        switch (record.type) {
        case RecordType::Pose:
            poses[record.device]++;
            ordered = ordered && record.time >= lastPoseTime[record.device];
            lastPoseTime[record.device] = record.time;
            CHECK((record.pose.flags & PoseFlag_Valid) && record.pose.result == vr::TrackingResult_Running_OK);
            CHECK(record.pose.rotation[0] == 0.6f && record.pose.rotation[2] == 0.8f && record.pose.angularVelocity[1] == -1.5);
            CHECK(std::abs(record.pose.poseTimeOffset + 0.004) < 1e-9);
            break;
        case RecordType::Frame:
            frames[record.device]++;
            break;
        case RecordType::Transform:
            transformsOf[record.device]++;
            CHECK((record.transform.flags & TransformFlag_Enabled) && (record.transform.flags & TransformFlag_RelativeCoordSystem));
            CHECK(record.transform.referenceDevice == 0 && record.transform.targetDevice == 3 && record.transform.rotation[2] == -0.5);
            break;
        case RecordType::Applied:
            applied[record.device]++;
            CHECK(record.applied.rotation[0] == 0.8 && record.applied.rotation[2] == 0.6 && record.applied.translation[2] == 3.0);
            break;
        case RecordType::Latency:
            latencies++;
            CHECK(record.device == 3 && record.latency.referenceDevice == 0);
            CHECK(std::abs(record.latency.latency - 0.0125) < 1e-8 && record.latency.confidence == 0.75);
            break;
        case RecordType::Device:
            devices[record.device]++;
            break;
        case RecordType::Note:
            displayNotes += record.text.rfind("display_frequency=90", 0) == 0;
            break;
        default:
            break;
        }
    }

    CHECK(!records.empty() && records.front().type == RecordType::Session);
    CHECK(!records.empty() && records.front().text.find("format=1\n") == 0 && records.front().text.find("program=Space Calibrator driver") != std::string::npos);
    CHECK(!records.empty() && records.back().type == RecordType::Note && records.back().text == "end of session");

    // the headset and the calibration's two devices: every pose. (one more each from before the overlay spoke)
    CHECK(poses[0] == headsetPoses + 1);
    CHECK(poses[3] == trackerPoses + 1);
    CHECK(ordered);
    // the others five times a second: the tracker that only shares the calibration, and the controller
    CHECK(poses[4] >= 5 && poses[4] <= 8);
    CHECK(poses[5] >= 5 && poses[5] <= 8);
    CHECK(trackerPoses > 400 && controllerPoses > 100);
    // each device's frame once, and the headset's again when its space moved
    CHECK(frames[0] == 2 && frames[3] == 1 && frames[4] == 1 && frames[5] == 1);
    // the two calibrated devices' transforms, each as set and as changed. nothing for the 62 the overlay never calibrated
    CHECK(transformsOf.size() == 2 && transformsOf[3] == 2 && transformsOf[4] == 2);
    // what the driver applied: ten times a second for the target, once a second for the rest of its system
    CHECK(applied[3] >= 9 && applied[3] <= 13);
    CHECK(applied[4] >= 1 && applied[4] <= 3);
    CHECK(latencies == 0); // the driver uses none: the record is of recordings made before
    // who the devices are, once each
    CHECK(devices.size() == 4 && devices[0] == 1 && devices[3] == 1 && devices[4] == 1 && devices[5] == 1);
    for (const Record& record : records) {
        if (record.type == RecordType::Device && record.device == 3) {
            CHECK(record.text == "LHR-TEST0003\tSome Tracker\tlighthouse\t" + std::to_string(vr::TrackedDeviceClass_GenericTracker));
        }
    }
    CHECK(displayNotes == 1);

    // SteamVR restarted at once: a second recording beside the first, which is as it was
    const uintmax_t firstSize = files.empty() ? 0 : fs::file_size(files[0]);
    recorder.start(recordings);
    recorder.onPose(0, poseAt(5.0));
    recorder.onFrame(transforms, 90.0);
    recorder.stop();
    const std::vector<fs::path> after = recordingsIn(recordings);
    CHECK(after.size() == 2);
    if (after.size() == 2) {
        const fs::path second = after[0] == files[0] ? after[1] : after[0];
        CHECK(fs::file_size(files[0]) == firstSize);
        const std::vector<Record> again = readAll(second);
        int secondPoses = 0, secondTransforms = 0, secondFrames = 0;
        for (const Record& record : again) {
            secondPoses += record.type == RecordType::Pose;
            secondTransforms += record.type == RecordType::Transform;
            secondFrames += record.type == RecordType::Frame;
        }
        // it starts from nothing known: the calibration in force and the device's frame are in it again
        CHECK(secondPoses == 1 && secondTransforms == 2 && secondFrames == 1);
    }

    // a folder that cannot be made: no recording, a line in the log, and the driver goes on
    {
        std::FILE* blocker = std::fopen((directory / "blocked").string().c_str(), "wb");
        if (blocker) {
            std::fclose(blocker);
        }
        SessionRecorder unable;
        unable.start(directory / "blocked" / "recordings");
        unable.onPose(0, poseAt(1.0));
        unable.onFrame(transforms, 90.0);
        unable.stop();
        CHECK(logged("Not recording poses"));
    }

    vr::CleanupDriverContext();
    fs::remove_all(directory);
    if (g_failures) {
        std::printf("%d checks failed\n", g_failures);
        return 1;
    }
    std::printf("the driver's recorder holds\n");
    return 0;
}
