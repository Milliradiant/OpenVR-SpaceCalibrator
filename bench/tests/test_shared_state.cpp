// The slots the driver and the overlay share their poses, transforms and the headset's metadata through (src/common/shared_state.h),
// hammered from several threads and from two processes at once: every copy a reader gets has to be one that a writer wrote whole,
// and none may be older than one it already had. A writer that stops halfway must leave the slot readable.

#include "shared_state.h"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>
#if defined(__linux__)
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

using namespace ipc::protocol;

static int g_failures = 0;

#define CHECK(condition)                                                    \
    do {                                                                    \
        if (!(condition)) {                                                 \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            g_failures++;                                                   \
        }                                                                   \
    } while (0)

// a pose every field of which says which write it came from: `n`, and the writer
static vr::DriverPose_t poseOf(uint64_t n, int writer)
{
    vr::DriverPose_t pose = {};
    const double value = static_cast<double>(n);
    pose.poseTimeOffset = value;
    for (int i = 0; i < 3; i++) {
        pose.vecPosition[i] = value + i;
        pose.vecVelocity[i] = -value - i;
        pose.vecAcceleration[i] = value * 2.0 + i;
        pose.vecAngularVelocity[i] = value * 3.0 + i;
        pose.vecAngularAcceleration[i] = value * 4.0 + i;
        pose.vecWorldFromDriverTranslation[i] = value * 5.0 + i;
        pose.vecDriverFromHeadTranslation[i] = value * 6.0 + i;
    }
    pose.qRotation = { value, value + 0.25, value + 0.5, value + 0.75 };
    pose.qWorldFromDriverRotation = { -value, value, -value, value };
    pose.qDriverFromHeadRotation = { value, -value, value, -value };
    pose.result = static_cast<vr::ETrackingResult>(writer);
    pose.poseIsValid = (n & 1) != 0;
    pose.deviceIsConnected = (n & 2) != 0;
    pose.willDriftInYaw = (n & 4) != 0;
    pose.shouldApplyHeadModel = (n & 8) != 0;
    return pose;
}

static bool isWhole(const vr::DriverPose_t& pose)
{
    const uint64_t n = static_cast<uint64_t>(pose.poseTimeOffset);
    const vr::DriverPose_t expected = poseOf(n, static_cast<int>(pose.result));
    return std::memcmp(&pose, &expected, sizeof(pose)) == 0;
}

static Command_SetDeviceTransform_t transformOf(uint64_t n)
{
    Command_SetDeviceTransform_t transform = {};
    const double value = static_cast<double>(n);
    transform.enabled((n & 1) != 0);
    transform.relativeCoordSystem((n & 2) != 0);
    transform.unTargetOpenVrDeviceId = static_cast<vr::TrackedDeviceIndex_t>(n);
    transform.unRelativeReferenceOpenvrDeviceId = static_cast<vr::TrackedDeviceIndex_t>(n * 3);
    transform.unRelativeTargetOpenvrDeviceId = static_cast<vr::TrackedDeviceIndex_t>(n * 7);
    transform.translation = { { value, value * 2.0, value * 3.0 } };
    transform.rotation = { value, -value, value * 0.5, -value * 0.5 };
    transform.scale = value + 1.0;
    return transform;
}

// field by field: the struct has a byte of padding, which nothing initializes
static bool isWhole(const Command_SetDeviceTransform_t& transform)
{
    const Command_SetDeviceTransform_t expected = transformOf(static_cast<uint64_t>(transform.scale - 1.0));
    return transform.enabled() == expected.enabled() && transform.relativeCoordSystem() == expected.relativeCoordSystem()
        && transform.unTargetOpenVrDeviceId == expected.unTargetOpenVrDeviceId
        && transform.unRelativeReferenceOpenvrDeviceId == expected.unRelativeReferenceOpenvrDeviceId
        && transform.unRelativeTargetOpenvrDeviceId == expected.unRelativeTargetOpenvrDeviceId
        && std::memcmp(&transform.translation, &expected.translation, sizeof(transform.translation)) == 0
        && std::memcmp(&transform.rotation, &expected.rotation, sizeof(transform.rotation)) == 0 && transform.scale == expected.scale;
}

// one writer, several readers, for a while
static void singleWriter()
{
    static SharedSlot_t<Command_SetDeviceTransform_t> slot = {};
    storeSlot(slot, transformOf(0));
    std::atomic<bool> stop = false;
    std::atomic<uint64_t> reads = 0, torn = 0, backwards = 0;

    std::vector<std::thread> readers;
    for (int r = 0; r < 3; r++) {
        readers.emplace_back([&] {
            uint64_t last = 0;
            while (!stop.load(std::memory_order_relaxed)) {
                Command_SetDeviceTransform_t transform;
                if (!loadSlot(slot, transform)) {
                    continue;
                }
                reads.fetch_add(1, std::memory_order_relaxed);
                if (!isWhole(transform)) {
                    torn.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                const uint64_t n = static_cast<uint64_t>(transform.scale - 1.0);
                if (n < last) {
                    backwards.fetch_add(1, std::memory_order_relaxed);
                }
                last = n;
            }
        });
    }
    uint64_t n = 1;
    for (const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(700); std::chrono::steady_clock::now() < end; n++) {
        storeSlot(slot, transformOf(n));
    }
    stop = true;
    for (std::thread& reader : readers) {
        reader.join();
    }
    Command_SetDeviceTransform_t last;
    CHECK(loadSlot(slot, last) && isWhole(last) && static_cast<uint64_t>(last.scale - 1.0) == n - 1);
    std::printf("one writer: %llu writes, %llu reads, %llu torn, %llu older than one read before\n", (unsigned long long)n, (unsigned long long)reads.load(), (unsigned long long)torn.load(), (unsigned long long)backwards.load());
    CHECK(torn == 0 && backwards == 0 && reads > 1000);
}

// several writers taking turns at one slot (the driver: a device's own thread and the frame thread), and readers
static void sharedWriters()
{
    static SharedSlot_t<vr::DriverPose_t> slot = {};
    storeSlotShared(slot, poseOf(1, 1)); // the readers start before the writers: a pose is there for them from the first
    std::atomic<bool> stop = false;
    std::atomic<uint64_t> reads = 0, torn = 0, writes = 0;

    std::vector<std::thread> threads;
    for (int r = 0; r < 2; r++) {
        threads.emplace_back([&] {
            while (!stop.load(std::memory_order_relaxed)) {
                vr::DriverPose_t pose;
                if (loadSlot(slot, pose)) {
                    reads.fetch_add(1, std::memory_order_relaxed);
                    if (!isWhole(pose)) {
                        torn.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }
        });
    }
    for (int w = 0; w < 3; w++) {
        threads.emplace_back([&, w] {
            for (uint64_t n = 1; !stop.load(std::memory_order_relaxed); n++) {
                storeSlotShared(slot, poseOf(n * 4 + static_cast<uint64_t>(w), w + 1));
                writes.fetch_add(1, std::memory_order_relaxed);
            }
        });
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    stop = true;
    for (std::thread& thread : threads) {
        thread.join();
    }
    vr::DriverPose_t last;
    CHECK(loadSlot(slot, last) && isWhole(last));
    std::printf("three writers: %llu writes, %llu reads, %llu torn\n", (unsigned long long)writes.load(), (unsigned long long)reads.load(), (unsigned long long)torn.load());
    CHECK(torn == 0 && reads > 1000 && writes > 1000);
}

// a writer stops halfway: the copy in use is still whole, and the next writer takes the slot up
static void stoppedWriter()
{
    static SharedSlot_t<vr::DriverPose_t> slot = {};
    storeSlot(slot, poseOf(5, 1));
    // what a write does up to its last step: the sequence made odd and the other copy half written
    std::atomic_ref<uint32_t>(slot.sequence).fetch_add(1);
    const vr::DriverPose_t half = poseOf(6, 1);
    std::memcpy(slot.copies[1 - shared::copyInUse(slot.sequence)], &half, sizeof(half) / 2);

    vr::DriverPose_t pose;
    CHECK(loadSlot(slot, pose) && isWhole(pose) && pose.poseTimeOffset == 5.0);
    storeSlot(slot, poseOf(7, 1));
    CHECK(loadSlot(slot, pose) && isWhole(pose) && pose.poseTimeOffset == 7.0);
    CHECK((slot.sequence & 1u) == 0);
    storeSlot(slot, poseOf(8, 1));
    CHECK(loadSlot(slot, pose) && isWhole(pose) && pose.poseTimeOffset == 8.0);

    // a slot never written reads as zeros: no pose, a disabled transform
    static SharedSlot_t<Command_SetDeviceTransform_t> empty = {};
    Command_SetDeviceTransform_t transform = transformOf(9);
    CHECK(loadSlot(empty, transform) && !transform.enabled() && transform.scale == 0.0);
    // the sequence wrapping round
    static SharedSlot_t<vr::DriverPose_t> wrapping = {};
    wrapping.sequence = 0xFFFFFFFEu;
    storeSlot(wrapping, poseOf(10, 1));
    storeSlotShared(wrapping, poseOf(11, 2));
    CHECK(loadSlot(wrapping, pose) && isWhole(pose) && pose.poseTimeOffset == 11.0);
}

// the shared state between two processes, as the driver and the overlay have it
static void twoProcesses()
{
#if defined(__linux__)
    void* memory = mmap(nullptr, sizeof(SharedState_t), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    CHECK(memory != MAP_FAILED);
    if (memory == MAP_FAILED) {
        return;
    }
    SharedState_t* state = static_cast<SharedState_t*>(memory);
    std::memset(state, 0, sizeof(*state));
    CHECK(!hasSharedState(state, sizeof(*state)));
    std::atomic_ref<uint64_t>(state->magic).store(k_SHARED_STATE_MAGIC, std::memory_order_release);
    CHECK(hasSharedState(state, sizeof(*state)));
    CHECK(!hasSharedState(state, sizeof(*state) - 1));

    std::fflush(stdout); // or the child has a copy of what is still buffered
    const pid_t child = fork();
    if (child == 0) {
        // the driver: poses of three devices from two threads, the transforms read
        std::atomic<bool> stop = false;
        uint64_t bad = 0;
        std::vector<std::thread> writers;
        for (int w = 0; w < 2; w++) {
            writers.emplace_back([&, w] {
                for (uint64_t n = 1; !stop.load(std::memory_order_relaxed); n++) {
                    storeSlotShared(state->poses[n % 3], poseOf(n * 2 + static_cast<uint64_t>(w), w + 1));
                }
            });
        }
        for (const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(600); std::chrono::steady_clock::now() < end;) {
            for (int device = 0; device < 3; device++) {
                Command_SetDeviceTransform_t transform;
                if (loadSlot(state->transforms[device], transform) && transform.scale != 0.0 && !isWhole(transform)) {
                    bad++;
                }
            }
        }
        stop = true;
        for (std::thread& writer : writers) {
            writer.join();
        }
        _exit(bad == 0 ? 0 : 1);
    }

    // the overlay: the transforms written, the poses read
    uint64_t reads = 0, torn = 0;
    uint64_t n = 1;
    for (const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(600); std::chrono::steady_clock::now() < end; n++) {
        storeSlot(state->transforms[n % 3], transformOf(n));
        for (int device = 0; device < 3; device++) {
            vr::DriverPose_t pose;
            if (loadSlot(state->poses[device], pose) && pose.poseTimeOffset != 0.0) {
                reads++;
                if (!isWhole(pose)) {
                    torn++;
                }
            }
        }
    }
    int status = 0;
    waitpid(child, &status, 0);
    std::printf("two processes: %llu transforms written, %llu poses read, %llu torn; the other process found %s\n", (unsigned long long)n, (unsigned long long)reads, (unsigned long long)torn, WIFEXITED(status) && WEXITSTATUS(status) == 0 ? "no torn transform" : "torn transforms");
    CHECK(WIFEXITED(status) && WEXITSTATUS(status) == 0);
    CHECK(torn == 0 && reads > 1000);
    munmap(memory, sizeof(SharedState_t));
#endif
}

int main()
{
    std::printf("the shared state: %zu bytes (a pose's slot %zu, a transform's %zu)\n", sizeof(SharedState_t), sizeof(SharedSlot_t<vr::DriverPose_t>), sizeof(SharedSlot_t<Command_SetDeviceTransform_t>));
    singleWriter();
    sharedWriters();
    stoppedWriter();
    twoProcesses();
    if (g_failures) {
        std::printf("%d checks failed\n", g_failures);
        return 1;
    }
    std::printf("the shared state holds\n");
    return 0;
}
