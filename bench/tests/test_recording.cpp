// The recording's writer and reader (src/common/pose_recording.h), each held to what the other makes of it.

#include "pose_recording.h"
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <thread>

using namespace spacecal::recording;
namespace fs = std::filesystem;

static int g_failures = 0;

#define CHECK(condition)                                                    \
    do {                                                                    \
        if (!(condition)) {                                                 \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            g_failures++;                                                   \
        }                                                                   \
    } while (0)

static PoseRecord somePose(int seed)
{
    PoseRecord pose;
    pose.flags = PoseFlag_Valid | PoseFlag_Connected;
    pose.result = 200;
    pose.poseTimeOffset = -0.009;
    for (int i = 0; i < 3; i++) {
        pose.position[i] = 1.25 * (i + 1) + 0.001 * seed;
        pose.velocity[i] = -0.5 * (i + 1);
        pose.angularVelocity[i] = 0.25 * (i + 1);
    }
    pose.rotation[0] = 0.5;
    pose.rotation[1] = -0.5;
    pose.rotation[2] = 0.5;
    pose.rotation[3] = 0.5;
    return pose;
}

static std::vector<Record> readAll(const fs::path& file, uint64_t* skipped = nullptr)
{
    std::vector<Record> records;
    Reader reader;
    CHECK(reader.open(file));
    Record record;
    while (reader.next(record)) {
        records.push_back(record);
    }
    if (skipped) {
        *skipped = reader.skippedRecords();
    }
    return records;
}

static void everyRecordComesBackAsWritten(const fs::path& directory)
{
    const fs::path file = directory / "roundtrip.screc";
    Writer writer;
    CHECK(writer.openFile(file));

    FrameRecord frame;
    frame.worldFromDriverRotation[0] = 0.7071067811865476;
    frame.worldFromDriverRotation[2] = 0.7071067811865476;
    frame.worldFromDriverTranslation[1] = 1.0000000001; // a frame keeps its doubles
    frame.driverFromHeadTranslation[2] = 0.159;

    TransformRecord transform;
    transform.flags = TransformFlag_Enabled | TransformFlag_RelativeCoordSystem;
    transform.quirks = 8;
    transform.referenceDevice = 0;
    transform.targetDevice = 5;
    transform.translation[1] = 0.084123456789;
    transform.rotation[0] = 0.6;
    transform.rotation[3] = 0.8;
    transform.scale = 1.0;

    AppliedRecord applied;
    applied.rotation[0] = 0.8;
    applied.rotation[2] = -0.6;
    applied.translation[0] = -0.93;

    LatencyRecord latency;
    latency.referenceDevice = 0;
    latency.latency = 0.0125;
    latency.confidence = 0.75;

    PoseRecord accelerated = somePose(7);
    accelerated.flags |= PoseFlag_HasAcceleration;
    accelerated.acceleration[1] = 9.5;
    accelerated.angularAcceleration[2] = -3.0;

    writer.session(0.0, "format=1\nprogram=test\n");
    writer.device(0.1, 5, "LHR-TEST", "Some Tracker", "lighthouse", 3);
    writer.frame(0.2, 5, frame);
    writer.pose(0.3, 5, somePose(1));
    writer.pose(0.304, 5, accelerated);
    writer.transform(0.4, 5, transform);
    writer.applied(0.5, 5, applied);
    writer.latency(0.6, 5, latency);
    writer.note(0.7, "end of session");
    writer.close();
    CHECK(writer.error().empty());

    const std::vector<Record> records = readAll(file);
    CHECK(records.size() == 9);
    if (records.size() != 9) {
        return;
    }
    CHECK(records[0].type == RecordType::Session && records[0].text == "format=1\nprogram=test\n" && records[0].device == k_NO_DEVICE);
    CHECK(records[1].type == RecordType::Device && records[1].text == "LHR-TEST\tSome Tracker\tlighthouse\t3" && records[1].device == 5);
    CHECK(records[2].type == RecordType::Frame && records[2].frame == frame && records[2].time == 0.2);

    const PoseRecord& pose = records[3].pose;
    const PoseRecord written = somePose(1);
    CHECK(records[3].type == RecordType::Pose && pose.flags == written.flags && pose.result == 200);
    CHECK(std::abs(pose.poseTimeOffset - written.poseTimeOffset) < 1e-9);
    for (int i = 0; i < 3; i++) {
        // floats: within a micrometre at these distances
        CHECK(std::abs(pose.position[i] - written.position[i]) < 1e-6);
        CHECK(pose.velocity[i] == written.velocity[i] && pose.angularVelocity[i] == written.angularVelocity[i]);
        CHECK(pose.acceleration[i] == 0.0);
    }
    CHECK(pose.rotation[0] == 0.5 && pose.rotation[1] == -0.5);

    CHECK(records[4].pose.flags & PoseFlag_HasAcceleration);
    CHECK(records[4].pose.acceleration[1] == 9.5 && records[4].pose.angularAcceleration[2] == -3.0 && records[4].time == 0.304);
    CHECK(records[5].type == RecordType::Transform && records[5].transform == transform);
    CHECK(records[6].type == RecordType::Applied && records[6].applied.rotation[2] == -0.6 && records[6].applied.translation[0] == -0.93);
    CHECK(records[7].type == RecordType::Latency && records[7].latency.referenceDevice == 0);
    CHECK(std::abs(records[7].latency.latency - 0.0125) < 1e-8 && records[7].latency.confidence == 0.75);
    CHECK(records[8].type == RecordType::Note && records[8].text == "end of session");
}

static void aRecordingCutShortReadsToItsLastWholeRecord(const fs::path& directory)
{
    const fs::path file = directory / "cut.screc";
    {
        Writer writer;
        CHECK(writer.openFile(file));
        for (int i = 0; i < 100; i++) {
            writer.pose(0.004 * i, 1, somePose(i));
        }
        writer.close();
    }
    const uintmax_t whole = fs::file_size(file);
    // 100 poses of 12 + 60 bytes after the magic
    CHECK(whole == 8 + 100 * 72);
    fs::resize_file(file, whole - 30);
    CHECK(readAll(file).size() == 99);
    fs::resize_file(file, 8 + 5); // mid-header of the first record
    CHECK(readAll(file).empty());

    // and something that is no recording is turned down
    {
        std::ofstream other(directory / "other.screc", std::ios::binary);
        other << "not a recording at all";
    }
    Reader reader;
    CHECK(!reader.open(directory / "other.screc"));
    CHECK(!reader.open(directory / "missing.screc"));
}

static void aRecordOfAnUnknownTypeIsSkipped(const fs::path& directory)
{
    const fs::path file = directory / "unknown.screc";
    {
        Writer writer;
        CHECK(writer.openFile(file));
        writer.note(1.0, "before");
        writer.close();
    }
    {
        // a record type from some later version: type 99, device 3, 4 bytes of payload
        std::ofstream out(file, std::ios::binary | std::ios::app);
        const unsigned char header[4] = { 99, 3, 4, 0 };
        const double time = 2.0;
        out.write(reinterpret_cast<const char*>(header), 4);
        out.write(reinterpret_cast<const char*>(&time), 8);
        out.write("abcd", 4);
        // and a note after it, written by hand the same way
        const unsigned char noteHeader[4] = { 8, 0xFF, 5, 0 };
        const double noteTime = 3.0;
        out.write(reinterpret_cast<const char*>(noteHeader), 4);
        out.write(reinterpret_cast<const char*>(&noteTime), 8);
        out.write("after", 5);
    }
    uint64_t skipped = 0;
    const std::vector<Record> records = readAll(file, &skipped);
    CHECK(records.size() == 2 && skipped == 1);
    CHECK(records.size() == 2 && records[1].text == "after" && records[1].time == 3.0);
}

static void theOldestRecordingsGoFirst(const fs::path& directory)
{
    const fs::path recordings = directory / "recordings";
    fs::create_directories(recordings);
    const auto make = [&](const char* name, size_t size) {
        std::ofstream out(recordings / name, std::ios::binary);
        out << std::string(size, 'x');
    };
    make("poses_20261101_100000.screc", 4000);
    make("poses_20261102_100000.screc", 4000);
    make("poses_20261103_100000.screc", 4000);
    make("notes.txt", 100000); // not a recording: never counted, never touched
    make("poses_20261101_090000.txt", 100000);

    CHECK(pruneRecordings(recordings, 100000) == 0);
    CHECK(pruneRecordings(recordings, 9000) == 1);
    CHECK(!fs::exists(recordings / "poses_20261101_100000.screc") && fs::exists(recordings / "poses_20261102_100000.screc"));
    CHECK(fs::exists(recordings / "notes.txt") && fs::exists(recordings / "poses_20261101_090000.txt"));

    // a new recording in the folder makes its room first (the limit less a quarter of it, for what it will write before the
    // folder is next looked at), and is named for when it began
    Writer writer;
    CHECK(writer.open(recordings, 8000));
    CHECK(writer.isOpen());
    const std::string name = writer.path().filename().string();
    CHECK(name.rfind("poses_", 0) == 0 && name.size() == std::string("poses_20261118_142500.screc").size() && writer.path().extension() == ".screc");
    CHECK(!fs::exists(recordings / "poses_20261102_100000.screc") && fs::exists(recordings / "poses_20261103_100000.screc"));
    writer.close();

    CHECK(pruneRecordings(directory / "no such folder", 0) == 0);
    Writer nowhere;
    CHECK(!nowhere.openFile(directory / "no such folder" / "x.screc") && !nowhere.isOpen() && !nowhere.error().empty());
    nowhere.pose(0.0, 0, somePose(0)); // nothing open: nothing happens
    nowhere.flush(true);
}

static void posesFromManyThreadsAllArrive(const fs::path& directory)
{
    const fs::path file = directory / "threads.screc";
    constexpr int k_THREADS = 6, k_POSES = 40000;
    Writer writer;
    CHECK(writer.openFile(file));

    std::atomic<bool> done = false;
    std::thread flusher([&] {
        while (!done) {
            writer.flush(false);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    std::vector<std::thread> threads;
    for (int t = 0; t < k_THREADS; t++) {
        threads.emplace_back([&, t] {
            for (int i = 0; i < k_POSES; i++) {
                writer.pose(1e-6 * i, static_cast<uint8_t>(t), somePose(i));
            }
        });
    }
    for (std::thread& thread : threads) {
        thread.join();
    }
    done = true;
    flusher.join();
    writer.close();
    CHECK(writer.droppedRecords() == 0);

    // every device's poses are all there, whole and in the order its thread wrote them
    int counts[k_THREADS] = {};
    bool ordered = true;
    for (const Record& record : readAll(file)) {
        CHECK(record.type == RecordType::Pose && record.device < k_THREADS);
        if (record.device < k_THREADS) {
            ordered = ordered && std::abs(record.time - 1e-6 * counts[record.device]) < 1e-12;
            counts[record.device]++;
        }
    }
    for (int t = 0; t < k_THREADS; t++) {
        CHECK(counts[t] == k_POSES);
    }
    CHECK(ordered);
}

static void whatNothingWritesOutIsDroppedNotKept(const fs::path& directory)
{
    const fs::path file = directory / "stalled.screc";
    Writer writer;
    CHECK(writer.openFile(file));
    // 400 000 poses of 72 bytes with no flush between them: more than the writer keeps in memory
    for (int i = 0; i < 400000; i++) {
        writer.pose(0.004 * i, 1, somePose(i));
    }
    CHECK(writer.droppedRecords() > 0);
    const uint64_t dropped = writer.droppedRecords();
    writer.close();
    const size_t kept = readAll(file).size();
    CHECK(kept + dropped == 400000);
    CHECK(kept * 72 <= 16u * 1024 * 1024);
}

static void aSessionGrowsOnlyAsFarAsTheFolderMay(const fs::path& directory)
{
    const fs::path recordings = directory / "limit";
    fs::create_directories(recordings);
    const auto make = [&](const char* name, size_t size) {
        std::ofstream out(recordings / name, std::ios::binary);
        out << std::string(size, 'x');
    };
    make("poses_20261101_100000.screc", 60000);
    make("poses_20261102_100000.screc", 60000);
    make("poses_20261103_100000.screc", 60000);
    constexpr uint64_t k_LIMIT = 200000;
    const auto folderSize = [&] {
        uint64_t total = 0;
        for (const auto& entry : fs::directory_iterator(recordings)) {
            total += entry.file_size();
        }
        return total;
    };

    Writer writer;
    CHECK(writer.open(recordings, k_LIMIT));
    const fs::path file = writer.path();
    // room is made for what the new one writes first: the oldest goes at once, the other two stay for now
    CHECK(!fs::exists(recordings / "poses_20261101_100000.screc") && fs::exists(recordings / "poses_20261102_100000.screc"));

    uint64_t largest = 0;
    int written = 0;
    for (; written < 5000 && writer.isOpen(); written++) {
        writer.pose(0.004 * written, 1, somePose(written));
        if (written % 50 == 49) {
            writer.flush(true); // 3600 bytes at a time
            largest = std::max(largest, folderSize());
        }
    }
    // the older ones went one by one as it grew, the folder never held more than its limit, and the recording ended at it
    CHECK(largest <= k_LIMIT);
    CHECK(!writer.isOpen() && !writer.error().empty());
    CHECK(written < 5000);
    CHECK(!fs::exists(recordings / "poses_20261102_100000.screc") && !fs::exists(recordings / "poses_20261103_100000.screc"));
    CHECK(fs::file_size(file) <= k_LIMIT && fs::file_size(file) > k_LIMIT - 4000);
    writer.pose(100.0, 1, somePose(0)); // ended: nothing more is taken
    writer.close();
    const size_t kept = readAll(file).size();
    CHECK(kept * 72 + 8 == fs::file_size(file));

    // two recordings begun within the same second have a file each
    Writer first, second;
    CHECK(first.open(recordings, 10 * k_LIMIT) && second.open(recordings, 10 * k_LIMIT));
    CHECK(first.path() != second.path());
    first.close();
    second.close();
}

int main()
{
    const fs::path directory = fs::temp_directory_path() / ("spacecal_recording_test_" + std::to_string(std::rand()));
    fs::remove_all(directory);
    fs::create_directories(directory);

    everyRecordComesBackAsWritten(directory);
    aRecordingCutShortReadsToItsLastWholeRecord(directory);
    aRecordOfAnUnknownTypeIsSkipped(directory);
    theOldestRecordingsGoFirst(directory);
    posesFromManyThreadsAllArrive(directory);
    whatNothingWritesOutIsDroppedNotKept(directory);
    aSessionGrowsOnlyAsFarAsTheFolderMay(directory);

    fs::remove_all(directory);
    if (g_failures) {
        std::printf("%d checks failed\n", g_failures);
        return 1;
    }
    std::printf("the recording's checks hold\n");
    return 0;
}
