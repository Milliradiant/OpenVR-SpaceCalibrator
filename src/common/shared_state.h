#pragma once

// What the driver and the overlay share through the IPC's shared memory (protocol 6): each device's latest pose, which the
// driver writes, the calibration the overlay has the driver apply to each device, and the headset's metadata.
//
// Each of them is a slot of two copies and a sequence number, and nobody takes a lock on it. A writer writes the copy not in use
// and then moves the sequence on to it; a reader copies the one in use and checks, by the sequence, that no write has begun on
// that copy in the meantime. So a reader never waits for a writer, a writer that stops halfway (its process gone) leaves the
// copy in use whole, and a pose update writes its own device's pose and nothing else: the overlay reading the poses, or another
// device's update, can never hold it up. The sequence is odd while a write is in progress, and its next even value names the
// copy that write is making the one in use.
//
// Everything in a slot is read and written as 64-bit words through std::atomic_ref, so that the two processes share no data that
// is not atomic.

#include "protocol.h"

#include <atomic>
#include <cstdint>
#include <cstring>
#include <thread>
#include <type_traits>

namespace ipc::protocol {

template <typename T>
struct alignas(64) SharedSlot_t {
    static constexpr size_t k_WORDS = (sizeof(T) + 7) / 8;
    static_assert(std::is_trivially_copyable_v<T>, "a slot holds plain data");

    uint32_t sequence;
    uint32_t _padding;
    uint64_t copies[2][k_WORDS];
};

namespace shared {

    inline uint32_t loadSequence(const uint32_t& sequence, std::memory_order order)
    {
        return std::atomic_ref<uint32_t>(const_cast<uint32_t&>(sequence)).load(order);
    }

    // the copy a sequence's value has in use: while a write is in progress (odd) still the one from before it
    inline size_t copyInUse(uint32_t sequence)
    {
        return (sequence >> 1) & 1u;
    }

    template <typename T>
    void writeCopy(SharedSlot_t<T>& slot, size_t copy, const T& value)
    {
        uint64_t words[SharedSlot_t<T>::k_WORDS] = {};
        std::memcpy(words, &value, sizeof(T));
        for (size_t i = 0; i < SharedSlot_t<T>::k_WORDS; i++) {
            std::atomic_ref<uint64_t>(slot.copies[copy][i]).store(words[i], std::memory_order_relaxed);
        }
    }

} // namespace shared

// For a slot with one writer. A sequence left odd by a writer that stopped halfway is taken up where it stood.
template <typename T>
void storeSlot(SharedSlot_t<T>& slot, const T& value)
{
    std::atomic_ref<uint32_t> sequence(slot.sequence);
    uint32_t begun = sequence.load(std::memory_order_relaxed);
    if ((begun & 1u) == 0) {
        // an RMW, so that a reader seeing the odd value still synchronizes with the write that made the copy in use
        begun = sequence.fetch_add(1, std::memory_order_relaxed) + 1;
    }
    std::atomic_thread_fence(std::memory_order_release);
    shared::writeCopy(slot, 1 - shared::copyInUse(begun), value);
    sequence.store(begun + 1, std::memory_order_release);
}

// For a slot that several threads of one process write: they take turns by the sequence, a writer waiting while another one's
// write is in progress (some tens of nanoseconds, longer only if that thread is preempted in it).
template <typename T>
void storeSlotShared(SharedSlot_t<T>& slot, const T& value)
{
    std::atomic_ref<uint32_t> sequence(slot.sequence);
    uint32_t current = sequence.load(std::memory_order_relaxed);
    for (uint32_t spins = 0;; spins++) {
        if ((current & 1u) == 0 && sequence.compare_exchange_weak(current, current + 1, std::memory_order_acquire, std::memory_order_relaxed)) {
            break;
        }
        if (spins >= 64) {
            std::this_thread::yield();
        }
        current = sequence.load(std::memory_order_relaxed);
    }
    const uint32_t begun = current + 1;
    std::atomic_thread_fence(std::memory_order_release);
    shared::writeCopy(slot, 1 - shared::copyInUse(begun), value);
    sequence.store(begun + 1, std::memory_order_release);
}

// A copy of what the slot holds. False only if writes kept starting on the copy being read, every time it was tried (each try
// takes tens of nanoseconds, and a slot is written a thousand times a second at most): `value` is then left as it was.
template <typename T>
bool loadSlot(const SharedSlot_t<T>& slot, T& value)
{
    constexpr int k_TRIES = 8;
    uint64_t words[SharedSlot_t<T>::k_WORDS];
    for (int attempt = 0; attempt < k_TRIES; attempt++) {
        const uint32_t before = shared::loadSequence(slot.sequence, std::memory_order_acquire);
        const size_t copy = shared::copyInUse(before);
        for (size_t i = 0; i < SharedSlot_t<T>::k_WORDS; i++) {
            words[i] = std::atomic_ref<uint64_t>(const_cast<uint64_t&>(slot.copies[copy][i])).load(std::memory_order_relaxed);
        }
        std::atomic_thread_fence(std::memory_order_acquire);
        const uint32_t after = shared::loadSequence(slot.sequence, std::memory_order_relaxed);
        // the copy read is written again by the second write to begin after `before`, which makes the sequence (before | 1) + 2
        if (after - before <= ((before & 1u) ? 1u : 2u)) {
            std::memcpy(&value, words, sizeof(T));
            return true;
        }
    }
    return false;
}

// "SCNOVA" and the protocol: the driver writes it once the rest is in place, and a reader that finds anything else is looking at
// an older or a newer driver's memory, or at memory not yet set up
constexpr uint64_t k_SHARED_STATE_MAGIC = 0x53434E4F56410000ull | static_cast<uint64_t>(IPC_PROTOCOL_CURRENT);

struct SharedState_t {
    uint64_t magic;
    uint64_t _reserved[7];
    SharedSlot_t<vr::DriverPose_t> poses[vr::k_unMaxTrackedDeviceCount]; // the driver's, every pose as its tracking driver gave it
    SharedSlot_t<Command_SetDeviceTransform_t> transforms[vr::k_unMaxTrackedDeviceCount]; // the overlay's
    SharedSlot_t<SharedData_HmdMetadata> hmdMetadata; // the driver's
};

// both sides build these from their own headers (the overlay's DriverPose_t is protocol.h's copy of OpenVR's): any difference
// between the two would put them at odds about where everything is
static_assert(sizeof(vr::DriverPose_t) == 280, "DriverPose_t is not OpenVR's");
static_assert(sizeof(Command_SetDeviceTransform_t) == 80, "Command_SetDeviceTransform_t changed: raise the protocol");
static_assert(sizeof(SharedData_HmdMetadata) == 8, "SharedData_HmdMetadata changed: raise the protocol");
static_assert(std::is_trivially_copyable_v<SharedState_t>, "the shared state is plain data");
static_assert(sizeof(SharedState_t) == 49280, "the shared state's layout changed: raise the protocol");
static_assert(std::atomic_ref<uint32_t>::is_always_lock_free && std::atomic_ref<uint64_t>::is_always_lock_free, "the shared state needs lock-free atomics");

inline bool hasSharedState(const void* buffer, size_t size)
{
    if (!buffer || size < sizeof(SharedState_t)) {
        return false;
    }
    const SharedState_t* state = static_cast<const SharedState_t*>(buffer);
    return std::atomic_ref<uint64_t>(const_cast<uint64_t&>(state->magic)).load(std::memory_order_acquire) == k_SHARED_STATE_MAGIC;
}

} // namespace ipc::protocol
