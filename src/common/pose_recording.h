#pragma once

// A recording of what the driver sees: every pose a tracking driver hands SteamVR for the calibration's own two devices, before Space
// Calibrator touches it, with the time it came in, and what the overlay had the driver apply at that time. It is what a calibration
// is computed from and judged by, kept so that the calibration can be run again over a real session and a change to it measured
// before it is used.
//
// The file is a sequence of records after an 8 byte magic. Every record has the same 12 byte header: type (u8), device index (u8,
// 0xFF for none), payload length (u16), time (f64, seconds since the recording began, monotonic). All values are little-endian. A
// reader skips a type it does not know, so a later version can add record types without breaking an older reader.

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

namespace spacecal::recording {

constexpr char k_MAGIC[8] = { 'S', 'C', 'R', 'E', 'C', '0', '1', '\n' };
constexpr uint8_t k_NO_DEVICE = 0xFF;
constexpr size_t k_MAX_DEVICES = 64; // vr::k_unMaxTrackedDeviceCount
constexpr const char* k_FILE_PREFIX = "poses_";
constexpr const char* k_FILE_EXTENSION = ".screc";

enum class RecordType : uint8_t {
    Session = 1, // text, "key=value" lines: what made the recording and when
    Device = 2, // text, tab separated: serial, model, tracking system, device class. written when a device is first seen or changes
    Frame = 3, // FrameRecord: the device's world-from-driver and driver-from-head. written when they change
    Pose = 4, // PoseRecord: one pose update, as the tracking driver gave it
    Transform = 5, // TransformRecord: what the overlay tells the driver to apply to the device. written when it changes
    Applied = 6, // AppliedRecord: the world calibration the driver applied to the device at this time (relative calibration)
    Latency = 7, // LatencyRecord: the driver's own estimate of the target's latency against the reference
    Note = 8, // text: anything else worth a line
};

enum PoseFlags : uint8_t {
    PoseFlag_Valid = 1 << 0, // poseIsValid
    PoseFlag_Connected = 1 << 1, // deviceIsConnected
    PoseFlag_WillDriftInYaw = 1 << 2,
    PoseFlag_ShouldApplyHeadModel = 1 << 3,
    PoseFlag_HasAcceleration = 1 << 4, // the two accelerations follow the pose (they are zero for most drivers)
};

// one pose update. positions in metres, rotations as w x y z, in the driver's own space (FrameRecord takes it to the world)
struct PoseRecord {
    uint8_t flags = 0;
    int32_t result = 0; // vr::ETrackingResult
    double poseTimeOffset = 0.0;
    double position[3] = {};
    double rotation[4] = { 1.0, 0.0, 0.0, 0.0 };
    double velocity[3] = {};
    double angularVelocity[3] = {};
    double acceleration[3] = {};
    double angularAcceleration[3] = {};
};

struct FrameRecord {
    double worldFromDriverRotation[4] = { 1.0, 0.0, 0.0, 0.0 };
    double worldFromDriverTranslation[3] = {};
    double driverFromHeadRotation[4] = { 1.0, 0.0, 0.0, 0.0 };
    double driverFromHeadTranslation[3] = {};

    bool operator==(const FrameRecord&) const = default;
};

enum TransformFlags : uint8_t {
    TransformFlag_Enabled = 1 << 0,
    TransformFlag_LerpCalibrations = 1 << 1,
    TransformFlag_HideContinuousTracker = 1 << 2,
    TransformFlag_RelativeCoordSystem = 1 << 3,
    TransformFlag_CalibrateMotionVecs = 1 << 4,
};

struct TransformRecord {
    uint8_t flags = 0;
    uint16_t quirks = 0;
    uint32_t referenceDevice = 0xFFFFFFFF;
    uint32_t targetDevice = 0xFFFFFFFF;
    double translation[3] = {};
    double rotation[4] = { 1.0, 0.0, 0.0, 0.0 };
    double scale = 1.0;

    bool operator==(const TransformRecord&) const = default;
};

struct AppliedRecord {
    double rotation[4] = { 1.0, 0.0, 0.0, 0.0 };
    double translation[3] = {};
};

struct LatencyRecord {
    uint8_t referenceDevice = k_NO_DEVICE;
    double latency = 0.0;
    double confidence = 0.0;
};

// Writes a recording. pose() and the other record calls only append to a buffer in memory, under a lock held for the copy alone, so
// they are safe from any thread and cost a pose update next to nothing; flush() is what touches the disk, and is meant for one
// thread that can afford it. Nothing here throws: a recording that cannot be written is dropped, and the driver goes on without it.
class Writer {
public:
    Writer() = default;
    ~Writer();
    Writer(const Writer&) = delete;
    Writer& operator=(const Writer&) = delete;

    // makes room in `directory` (the oldest recordings go until what is left is under maxTotalBytes, the new one counted at nothing)
    // and opens a new recording there. false, with the reason in error(), if it cannot
    bool open(const std::filesystem::path& directory, uint64_t maxTotalBytes);
    // opens a recording at this exact path
    bool openFile(const std::filesystem::path& file);
    void close();

    [[nodiscard]] bool isOpen() const { return m_isOpen; }
    [[nodiscard]] const std::string& error() const { return m_error; }
    [[nodiscard]] const std::filesystem::path& path() const { return m_path; }
    [[nodiscard]] uint64_t droppedRecords() const { return m_dropped; }

    void session(double time, const std::string& text);
    void device(double time, uint8_t deviceIndex, const std::string& serial, const std::string& model, const std::string& trackingSystem, int deviceClass);
    void frame(double time, uint8_t deviceIndex, const FrameRecord& record);
    void pose(double time, uint8_t deviceIndex, const PoseRecord& record);
    void transform(double time, uint8_t deviceIndex, const TransformRecord& record);
    void applied(double time, uint8_t deviceIndex, const AppliedRecord& record);
    void latency(double time, uint8_t deviceIndex, const LatencyRecord& record);
    void note(double time, const std::string& text);

    // writes what was appended since the last call to the file. `sync` also has the OS take it, so that it survives the process
    void flush(bool sync);

private:
    void append(RecordType type, uint8_t deviceIndex, double time, const uint8_t* payload, size_t length);

    std::mutex m_mutex;
    std::vector<uint8_t> m_pending;
    std::vector<uint8_t> m_writing;
    FILE* m_file = nullptr;
    std::atomic<bool> m_isOpen = false;
    std::atomic<uint64_t> m_dropped = 0;
    std::string m_error;
    std::filesystem::path m_path;
};

// Removes the oldest recordings in `directory` until the rest take no more than maxTotalBytes. Returns how many were removed
size_t pruneRecordings(const std::filesystem::path& directory, uint64_t maxTotalBytes);

// One record as read back
struct Record {
    RecordType type = RecordType::Note;
    uint8_t device = k_NO_DEVICE;
    double time = 0.0;
    std::string text; // Session, Device, Note
    PoseRecord pose;
    FrameRecord frame;
    TransformRecord transform;
    AppliedRecord applied;
    LatencyRecord latency;
};

// Reads a recording, in order. A recording cut off mid-record (a crash, a full disk) reads up to its last whole record
class Reader {
public:
    ~Reader();
    bool open(const std::filesystem::path& file);
    // false at the end of the recording. a record of an unknown type is skipped
    bool next(Record& record);
    [[nodiscard]] const std::string& error() const { return m_error; }
    [[nodiscard]] uint64_t skippedRecords() const { return m_skipped; }

private:
    FILE* m_file = nullptr;
    std::string m_error;
    uint64_t m_skipped = 0;
    std::vector<uint8_t> m_payload;
};

} // namespace spacecal::recording
