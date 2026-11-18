#include "pose_recording.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <ctime>
#include <system_error>

namespace spacecal::recording {

namespace fs = std::filesystem;

constexpr size_t k_RECORD_HEADER_SIZE = 12;
constexpr size_t k_MAX_PAYLOAD_SIZE = 0xFFFF;
// what may wait in memory for the next flush. past it records are dropped and counted: the disk has stalled, or nothing is flushing
constexpr size_t k_MAX_PENDING_BYTES = 16 * 1024 * 1024;

// the values are written as the bytes x86 keeps them in, which is the little-endian the format names
template <typename T>
static void put(std::vector<uint8_t>& out, T value)
{
    uint8_t bytes[sizeof(T)];
    std::memcpy(bytes, &value, sizeof(T));
    out.insert(out.end(), bytes, bytes + sizeof(T));
}

template <typename T>
static bool get(const std::vector<uint8_t>& in, size_t& offset, T& value)
{
    if (offset + sizeof(T) > in.size()) {
        return false;
    }
    std::memcpy(&value, in.data() + offset, sizeof(T));
    offset += sizeof(T);
    return true;
}

template <size_t N>
static void putFloats(std::vector<uint8_t>& out, const double (&values)[N])
{
    for (size_t i = 0; i < N; i++) {
        put(out, static_cast<float>(values[i]));
    }
}

template <size_t N>
static bool getFloats(const std::vector<uint8_t>& in, size_t& offset, double (&values)[N])
{
    for (size_t i = 0; i < N; i++) {
        float value = 0.0f;
        if (!get(in, offset, value)) {
            return false;
        }
        values[i] = static_cast<double>(value);
    }
    return true;
}

template <size_t N>
static void putDoubles(std::vector<uint8_t>& out, const double (&values)[N])
{
    for (size_t i = 0; i < N; i++) {
        put(out, values[i]);
    }
}

template <size_t N>
static bool getDoubles(const std::vector<uint8_t>& in, size_t& offset, double (&values)[N])
{
    for (size_t i = 0; i < N; i++) {
        if (!get(in, offset, values[i])) {
            return false;
        }
    }
    return true;
}

static FILE* openBinary(const fs::path& file, bool forWriting)
{
#if defined(_WIN32)
    FILE* handle = nullptr;
    if (_wfopen_s(&handle, file.c_str(), forWriting ? L"wb" : L"rb") != 0) {
        return nullptr;
    }
    return handle;
#else
    return std::fopen(file.c_str(), forWriting ? "wb" : "rb");
#endif
}

static bool isRecording(const fs::directory_entry& entry)
{
    std::error_code ec;
    if (!entry.is_regular_file(ec) || ec) {
        return false;
    }
    const std::string name = entry.path().filename().string();
    return name.rfind(k_FILE_PREFIX, 0) == 0 && entry.path().extension() == k_FILE_EXTENSION;
}

size_t pruneRecordings(const fs::path& directory, uint64_t maxTotalBytes)
{
    struct Entry {
        fs::path path;
        uint64_t size = 0;
    };
    std::vector<Entry> entries;
    uint64_t totalBytes = 0;

    std::error_code ec;
    for (fs::directory_iterator it(directory, ec), end; !ec && it != end; it.increment(ec)) {
        if (!isRecording(*it)) {
            continue;
        }
        std::error_code sizeError;
        const uint64_t size = it->file_size(sizeError);
        if (sizeError) {
            continue;
        }
        entries.push_back({ it->path(), size });
        totalBytes += size;
    }

    // the name carries the time the recording began, so the names in order are the recordings in order
    std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) { return a.path.filename() < b.path.filename(); });

    size_t removed = 0;
    for (const Entry& entry : entries) {
        if (totalBytes <= maxTotalBytes) {
            break;
        }
        std::error_code removeError;
        if (fs::remove(entry.path, removeError) && !removeError) {
            totalBytes -= entry.size;
            removed++;
        }
    }
    return removed;
}

Writer::~Writer()
{
    close();
}

bool Writer::open(const fs::path& directory, uint64_t maxTotalBytes)
{
    std::error_code ec;
    fs::create_directories(directory, ec);
    if (ec) {
        m_error = "cannot create " + directory.string() + ": " + ec.message();
        return false;
    }

    pruneRecordings(directory, maxTotalBytes);

    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm local = {};
#if defined(_WIN32)
    localtime_s(&local, &now);
#else
    localtime_r(&now, &local);
#endif
    char stamp[32] = {};
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local);

    return openFile(directory / (std::string(k_FILE_PREFIX) + stamp + k_FILE_EXTENSION));
}

bool Writer::openFile(const fs::path& file)
{
    close();

    FILE* handle = openBinary(file, true);
    if (!handle) {
        m_error = "cannot open " + file.string() + " for writing";
        return false;
    }
    if (std::fwrite(k_MAGIC, 1, sizeof(k_MAGIC), handle) != sizeof(k_MAGIC)) {
        std::fclose(handle);
        m_error = "cannot write to " + file.string();
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    m_file = handle;
    m_path = file;
    m_pending.clear();
    m_pending.reserve(1024 * 1024);
    m_writing.reserve(1024 * 1024);
    m_dropped = 0;
    m_error.clear();
    m_isOpen = true;
    return true;
}

void Writer::close()
{
    if (!m_isOpen) {
        return;
    }
    flush(true);

    std::lock_guard<std::mutex> lock(m_mutex);
    m_isOpen = false;
    if (m_file) {
        std::fclose(m_file);
        m_file = nullptr;
    }
}

void Writer::append(RecordType type, uint8_t deviceIndex, double time, const uint8_t* payload, size_t length)
{
    if (length > k_MAX_PAYLOAD_SIZE) {
        length = k_MAX_PAYLOAD_SIZE;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_isOpen) {
        return;
    }
    if (m_pending.size() + k_RECORD_HEADER_SIZE + length > k_MAX_PENDING_BYTES) {
        m_dropped++;
        return;
    }
    put(m_pending, static_cast<uint8_t>(type));
    put(m_pending, deviceIndex);
    put(m_pending, static_cast<uint16_t>(length));
    put(m_pending, time);
    m_pending.insert(m_pending.end(), payload, payload + length);
}

void Writer::session(double time, const std::string& text)
{
    append(RecordType::Session, k_NO_DEVICE, time, reinterpret_cast<const uint8_t*>(text.data()), text.size());
}

void Writer::device(double time, uint8_t deviceIndex, const std::string& serial, const std::string& model, const std::string& trackingSystem, int deviceClass)
{
    const std::string text = serial + "\t" + model + "\t" + trackingSystem + "\t" + std::to_string(deviceClass);
    append(RecordType::Device, deviceIndex, time, reinterpret_cast<const uint8_t*>(text.data()), text.size());
}

void Writer::note(double time, const std::string& text)
{
    append(RecordType::Note, k_NO_DEVICE, time, reinterpret_cast<const uint8_t*>(text.data()), text.size());
}

void Writer::frame(double time, uint8_t deviceIndex, const FrameRecord& record)
{
    std::vector<uint8_t> payload;
    payload.reserve(14 * sizeof(double));
    putDoubles(payload, record.worldFromDriverRotation);
    putDoubles(payload, record.worldFromDriverTranslation);
    putDoubles(payload, record.driverFromHeadRotation);
    putDoubles(payload, record.driverFromHeadTranslation);
    append(RecordType::Frame, deviceIndex, time, payload.data(), payload.size());
}

void Writer::pose(double time, uint8_t deviceIndex, const PoseRecord& record)
{
    // the pose is kept as floats: a tenth of a micrometre at the distances a room has, far under what any tracking resolves. the
    // rarely changing frame around it (FrameRecord) keeps its doubles
    uint8_t payload[4 + 4 + 12 + 16 + 12 + 12 + 24];
    size_t length = 0;
    const auto putByte = [&](uint8_t value) { payload[length++] = value; };
    const auto putValue = [&](auto value) {
        std::memcpy(payload + length, &value, sizeof(value));
        length += sizeof(value);
    };
    const auto putVector = [&](const double* values, size_t count) {
        for (size_t i = 0; i < count; i++) {
            putValue(static_cast<float>(values[i]));
        }
    };

    putByte(record.flags);
    putByte(0);
    putValue(static_cast<int16_t>(record.result));
    putValue(static_cast<float>(record.poseTimeOffset));
    putVector(record.position, 3);
    putVector(record.rotation, 4);
    putVector(record.velocity, 3);
    putVector(record.angularVelocity, 3);
    if (record.flags & PoseFlag_HasAcceleration) {
        putVector(record.acceleration, 3);
        putVector(record.angularAcceleration, 3);
    }
    append(RecordType::Pose, deviceIndex, time, payload, length);
}

void Writer::transform(double time, uint8_t deviceIndex, const TransformRecord& record)
{
    std::vector<uint8_t> payload;
    payload.reserve(80);
    put(payload, record.flags);
    put(payload, static_cast<uint8_t>(0));
    put(payload, record.quirks);
    put(payload, record.referenceDevice);
    put(payload, record.targetDevice);
    putDoubles(payload, record.translation);
    putDoubles(payload, record.rotation);
    put(payload, record.scale);
    append(RecordType::Transform, deviceIndex, time, payload.data(), payload.size());
}

void Writer::applied(double time, uint8_t deviceIndex, const AppliedRecord& record)
{
    std::vector<uint8_t> payload;
    payload.reserve(7 * sizeof(double));
    putDoubles(payload, record.rotation);
    putDoubles(payload, record.translation);
    append(RecordType::Applied, deviceIndex, time, payload.data(), payload.size());
}

void Writer::latency(double time, uint8_t deviceIndex, const LatencyRecord& record)
{
    std::vector<uint8_t> payload;
    payload.reserve(12);
    put(payload, record.referenceDevice);
    put(payload, static_cast<uint8_t>(0));
    put(payload, static_cast<uint16_t>(0));
    put(payload, static_cast<float>(record.latency));
    put(payload, static_cast<float>(record.confidence));
    append(RecordType::Latency, deviceIndex, time, payload.data(), payload.size());
}

void Writer::flush(bool sync)
{
    FILE* handle = nullptr;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_isOpen || !m_file) {
            return;
        }
        m_writing.clear();
        m_writing.swap(m_pending);
        handle = m_file;
    }

    bool failed = false;
    if (!m_writing.empty()) {
        failed = std::fwrite(m_writing.data(), 1, m_writing.size(), handle) != m_writing.size();
    }
    if (!failed && sync) {
        failed = std::fflush(handle) != 0;
    }

    if (failed) {
        // a disk that is full or gone: stop here and leave what was written. the recording reads up to its last whole record
        std::lock_guard<std::mutex> lock(m_mutex);
        m_error = "cannot write to " + m_path.string();
        m_isOpen = false;
        std::fclose(m_file);
        m_file = nullptr;
    }
}

Reader::~Reader()
{
    if (m_file) {
        std::fclose(m_file);
    }
}

bool Reader::open(const fs::path& file)
{
    if (m_file) {
        std::fclose(m_file);
        m_file = nullptr;
    }
    m_skipped = 0;

    m_file = openBinary(file, false);
    if (!m_file) {
        m_error = "cannot open " + file.string();
        return false;
    }
    char magic[sizeof(k_MAGIC)] = {};
    if (std::fread(magic, 1, sizeof(magic), m_file) != sizeof(magic) || std::memcmp(magic, k_MAGIC, sizeof(magic)) != 0) {
        m_error = file.string() + " is not a pose recording";
        std::fclose(m_file);
        m_file = nullptr;
        return false;
    }
    return true;
}

bool Reader::next(Record& record)
{
    while (m_file) {
        uint8_t header[k_RECORD_HEADER_SIZE];
        if (std::fread(header, 1, sizeof(header), m_file) != sizeof(header)) {
            return false;
        }
        uint16_t length = 0;
        std::memcpy(&length, header + 2, sizeof(length));
        std::memcpy(&record.time, header + 4, sizeof(record.time));
        record.type = static_cast<RecordType>(header[0]);
        record.device = header[1];

        m_payload.resize(length);
        if (length > 0 && std::fread(m_payload.data(), 1, length, m_file) != length) {
            return false;
        }

        size_t offset = 0;
        bool ok = true;
        switch (record.type) {
        case RecordType::Session:
        case RecordType::Device:
        case RecordType::Note:
            record.text.assign(m_payload.begin(), m_payload.end());
            break;
        case RecordType::Frame:
            ok = getDoubles(m_payload, offset, record.frame.worldFromDriverRotation)
                && getDoubles(m_payload, offset, record.frame.worldFromDriverTranslation)
                && getDoubles(m_payload, offset, record.frame.driverFromHeadRotation)
                && getDoubles(m_payload, offset, record.frame.driverFromHeadTranslation);
            break;
        case RecordType::Pose: {
            uint8_t reserved = 0;
            int16_t result = 0;
            float poseTimeOffset = 0.0f;
            record.pose = PoseRecord {};
            ok = get(m_payload, offset, record.pose.flags) && get(m_payload, offset, reserved) && get(m_payload, offset, result)
                && get(m_payload, offset, poseTimeOffset)
                && getFloats(m_payload, offset, record.pose.position)
                && getFloats(m_payload, offset, record.pose.rotation)
                && getFloats(m_payload, offset, record.pose.velocity)
                && getFloats(m_payload, offset, record.pose.angularVelocity);
            if (ok && (record.pose.flags & PoseFlag_HasAcceleration)) {
                ok = getFloats(m_payload, offset, record.pose.acceleration) && getFloats(m_payload, offset, record.pose.angularAcceleration);
            }
            record.pose.result = result;
            record.pose.poseTimeOffset = poseTimeOffset;
            break;
        }
        case RecordType::Transform: {
            uint8_t reserved = 0;
            ok = get(m_payload, offset, record.transform.flags) && get(m_payload, offset, reserved)
                && get(m_payload, offset, record.transform.quirks)
                && get(m_payload, offset, record.transform.referenceDevice)
                && get(m_payload, offset, record.transform.targetDevice)
                && getDoubles(m_payload, offset, record.transform.translation)
                && getDoubles(m_payload, offset, record.transform.rotation)
                && get(m_payload, offset, record.transform.scale);
            break;
        }
        case RecordType::Applied:
            ok = getDoubles(m_payload, offset, record.applied.rotation) && getDoubles(m_payload, offset, record.applied.translation);
            break;
        case RecordType::Latency: {
            uint8_t reserved8 = 0;
            uint16_t reserved16 = 0;
            float latency = 0.0f, confidence = 0.0f;
            ok = get(m_payload, offset, record.latency.referenceDevice) && get(m_payload, offset, reserved8)
                && get(m_payload, offset, reserved16) && get(m_payload, offset, latency) && get(m_payload, offset, confidence);
            record.latency.latency = latency;
            record.latency.confidence = confidence;
            break;
        }
        default:
            ok = false;
            break;
        }

        if (ok) {
            return true;
        }
        m_skipped++;
    }
    return false;
}

} // namespace spacecal::recording
