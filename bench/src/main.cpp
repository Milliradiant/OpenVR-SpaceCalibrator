// spacecal-bench: reads the driver's pose recordings (src/common/pose_recording.h) and measures a calibration against them.
//
//   spacecal-bench info RECORDING                 what the recording holds
//   spacecal-bench reference RECORDING            the mount, the time offset and the world calibration, with hindsight
//   spacecal-bench score RECORDING                what was applied during the session, against that reference
//   spacecal-bench replay RECORDING [options]     the overlay's own calibration code run over the recording, scored the same way
//   spacecal-bench check RECORDING                what each driver's poses say of themselves: rates, ages, velocities
//   spacecal-bench simulate RECORDING [options]   a made-up session with a known answer
//
// README.md has what each prints and how to read it.

#include "reference.h"
#include "replay.h"
#include "replay_driver.h"
#include "session.h"
#include "simulate.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace bench;
using namespace spacecal::recording;

namespace {

struct Arguments {
    std::string command;
    std::string recording;
    std::vector<std::pair<std::string, std::string>> options;

    [[nodiscard]] const char* get(const char* name) const
    {
        for (const auto& [key, value] : options) {
            if (key == name) {
                return value.c_str();
            }
        }
        return nullptr;
    }
    [[nodiscard]] double number(const char* name, double fallback) const
    {
        const char* value = get(name);
        return value ? std::atof(value) : fallback;
    }
};

std::string clock(double seconds)
{
    char text[32];
    const int whole = static_cast<int>(seconds);
    std::snprintf(text, sizeof(text), "%02d:%02d:%04.1f", whole / 3600, (whole / 60) % 60, seconds - 60.0 * (whole / 60));
    return text;
}

std::string mountText(const Pose& mount)
{
    char text[160];
    const Eigen::Vector3d euler = logEuler(mount.q);
    std::snprintf(text, sizeof(text), "yaw=%.2f pitch=%.2f roll=%.2f deg, x=%.2f y=%.2f z=%.2f cm", euler[0], euler[1], euler[2], mount.p.x() * 100.0, mount.p.y() * 100.0, mount.p.z() * 100.0);
    return text;
}

bool loadSession(const Arguments& arguments, Session& session)
{
    std::string error;
    if (!session.load(arguments.recording, error)) {
        std::fprintf(stderr, "spacecal-bench: %s\n", error.c_str());
        return false;
    }
    if (arguments.get("--reference")) {
        session.referenceIndex = std::atoi(arguments.get("--reference"));
    }
    if (arguments.get("--target")) {
        session.targetIndex = std::atoi(arguments.get("--target"));
    }
    return true;
}

ReferenceOptions referenceOptions(const Arguments& arguments)
{
    ReferenceOptions options;
    options.smoothWindow = arguments.number("--window", options.smoothWindow);
    options.feetBelowTarget = arguments.number("--feet", options.feetBelowTarget);
    options.feetSpread = arguments.number("--spread", options.feetSpread);
    options.usePoseTimeOffset = arguments.get("--pose-time") != nullptr;
    return options;
}

void printDistribution(const char* label, std::vector<double> values, double scale, const char* unit)
{
    if (values.empty()) {
        std::printf("  %-46s none\n", label);
        return;
    }
    for (double& value : values) {
        value *= scale;
    }
    std::printf("  %-46s median %.1f, 90%% %.1f, 99%% %.1f, most %.1f %s (%zu)\n", label, median(values), percentile(values, 0.9), percentile(values, 0.99), percentile(values, 1.0), unit, values.size());
}

int commandInfo(const Arguments& arguments)
{
    Session session;
    if (!loadSession(arguments, session)) {
        return 1;
    }
    std::printf("%s: %s, %llu records", session.path.filename().string().c_str(), clock(session.duration).c_str(), static_cast<unsigned long long>(session.records));
    if (session.skippedRecords) {
        std::printf(" (%llu not understood and skipped)", static_cast<unsigned long long>(session.skippedRecords));
    }
    std::printf("\n");
    for (const auto& [key, value] : session.info) {
        std::printf("  %s = %s\n", key.c_str(), value.c_str());
    }

    std::printf("devices (index, serial, model, tracking system; poses, a second, valid, the age its driver reports):\n");
    for (const auto& [index, device] : session.devices) {
        std::vector<double> steps, ages;
        size_t valid = 0;
        for (size_t i = 0; i < device.samples.size(); i++) {
            valid += device.samples[i].valid;
            ages.push_back(-device.samples[i].poseTimeOffset);
            if (i > 0) {
                steps.push_back(device.samples[i].time - device.samples[i - 1].time);
            }
        }
        const char* role = index == session.referenceIndex ? " reference" : index == session.targetIndex ? " target" : "";
        std::printf("  [%2d] %-16s %-22s %-12s %8zu poses", index, device.serial.c_str(), device.model.c_str(), device.trackingSystem.c_str(), device.samples.size());
        if (!steps.empty()) {
            std::printf(", %6.1f/s, %5.1f%% valid, age %.1f ms", 1.0 / median(steps), 100.0 * static_cast<double>(valid) / static_cast<double>(device.samples.size()), median(ages) * 1e3);
        }
        std::printf("%s\n", role);
        if (!device.frameChanges.empty()) {
            std::printf("       its frame (world-from-driver, driver-from-head) changed %zu times, first at %s\n", device.frameChanges.size(), clock(device.frameChanges.front()).c_str());
        }
    }

    if (const Device* target = session.target()) {
        size_t relative = 0;
        for (const TransformEvent& event : target->transforms) {
            relative += (event.record.flags & TransformFlag_RelativeCoordSystem) ? 1 : 0;
        }
        std::printf("the overlay set the target's calibration %zu times (%zu of them relative); the driver's applied calibration is recorded %zu times, its latency estimate %zu times\n",
            target->transforms.size(), relative, target->applied.size(), target->latency.size());

        // stretches where one of the two devices gave no valid pose
        for (const Device* device : { session.reference(), target }) {
            if (!device) {
                continue;
            }
            double lost = 0.0;
            size_t gaps = 0;
            double lastValid = -1.0;
            for (const Sample& sample : device->samples) {
                if (!sample.valid) {
                    continue;
                }
                if (lastValid >= 0.0 && sample.time - lastValid > 0.1) {
                    lost += sample.time - lastValid;
                    gaps++;
                }
                lastValid = sample.time;
            }
            std::printf("  %s: %zu gaps over 0.1 s without a valid pose, %.1f s in all\n", device->name().c_str(), gaps, lost);
        }
    } else {
        std::printf("the overlay never named a calibration's two devices in this recording\n");
    }

    for (const auto& [time, text] : session.notes) {
        std::printf("  note %s  %s\n", clock(time).c_str(), text.c_str());
    }
    return 0;
}

bool solve(const Arguments& arguments, Session& session, Reference& reference, bool print)
{
    if (!loadSession(arguments, session)) {
        return false;
    }
    reference = solveReference(session, referenceOptions(arguments));
    if (!reference.ok) {
        std::fprintf(stderr, "spacecal-bench: no reference: %s\n", reference.error.c_str());
        return false;
    }
    if (!print) {
        return true;
    }

    std::printf("reference %s, target %s, over %s\n", session.reference()->name().c_str(), session.target()->name().c_str(), clock(session.duration).c_str());
    std::printf("time offset: the target's poses are %.1f ms behind the reference's (correlation %.3f)", reference.timeOffset * 1e3, reference.timeOffsetCorrelation);
    if (reference.windowOffsets.size() >= 3) {
        std::printf("; by 20 s windows: median %.1f, from %.1f to %.1f ms (%zu windows)", median(reference.windowOffsets) * 1e3, percentile(reference.windowOffsets, 0.1) * 1e3, percentile(reference.windowOffsets, 0.9) * 1e3, reference.windowOffsets.size());
    }
    std::printf("\n");
    std::printf("mount (the target in the reference's frame): %s\n", mountText(reference.mount.mount).c_str());
    std::printf("  from %zu motion pairs; turning axes agree to %.2f deg, the translation's equations to %.1f mm (medians); conditioning %.3f\n", reference.mount.pairs, reference.mount.rotationResidualDeg, reference.mount.translationResidualMm, reference.mount.translationCondition);
    if (reference.firstHalf.ok && reference.secondHalf.ok) {
        std::printf("  each half of the recording alone: %.2f deg and %.1f mm apart\n", angleBetween(reference.firstHalf.mount.q, reference.secondHalf.mount.q) * k_DEG, (reference.firstHalf.mount.p - reference.secondHalf.mount.p).norm() * 1e3);
    }
    return true;
}

int commandReference(const Arguments& arguments)
{
    Session session;
    Reference reference;
    if (!solve(arguments, session, reference, true)) {
        return 1;
    }

    // a known answer, if the recording is a simulated one
    for (const auto& [time, text] : session.notes) {
        if (text.rfind("truth_mount=", 0) == 0) {
            double v[7] = {};
            if (std::sscanf(text.c_str() + 12, "%lf %lf %lf %lf %lf %lf %lf", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6]) == 7) {
                const Pose truth { Eigen::Quaterniond(v[0], v[1], v[2], v[3]), Eigen::Vector3d(v[4], v[5], v[6]) };
                std::printf("  the simulation's own mount: %s; found within %.3f deg and %.2f mm\n", mountText(truth).c_str(), angleBetween(truth.q, reference.mount.mount.q) * k_DEG, (truth.p - reference.mount.mount.p).norm() * 1e3);
            }
        }
        if (text.rfind("truth_time_offset=", 0) == 0) {
            std::printf("  the simulation's own time offset: %.1f ms\n", std::atof(text.c_str() + 18) * 1e3);
        }
    }

    std::vector<double> scatter;
    for (size_t i = 0; i < reference.times.size(); i++) {
        scatter.push_back(reference.feetError(reference.times[i], reference.instant[i]));
    }
    std::printf("world calibration, judged at the feet (%.1f m below the target and %.1f m to each side of that, the worst of them):\n", reference.options.feetBelowTarget, reference.options.feetSpread);
    printDistribution("one pose pair against the reference", scatter, 1e3, "mm");

    // how far the reference itself moved: the two tracking systems drifting against each other
    double travelled = 0.0;
    for (size_t i = 1; i < reference.times.size(); i++) {
        if (reference.segment[i] != reference.segment[i - 1]) {
            continue;
        }
        const Eigen::Vector3d feet = reference.targetPositions[i] - Eigen::Vector3d(0.0, reference.options.feetBelowTarget, 0.0);
        travelled += (reference.smooth[i] * feet - reference.smooth[i - 1] * feet).norm();
    }
    const Eigen::Vector3d firstFeet = reference.targetPositions.front() - Eigen::Vector3d(0.0, reference.options.feetBelowTarget, 0.0);
    std::printf("  the reference moved %.0f mm at the feet from the recording's start to its end, %.0f mm along the way, with %zu steps over %.0f mm\n",
        (reference.smooth.back() * firstFeet - reference.smooth.front() * firstFeet).norm() * 1e3, travelled * 1e3, reference.jumps.size(), reference.options.jumpThreshold * 1e3);
    for (double jump : reference.jumps) {
        std::printf("    step at %s\n", clock(jump).c_str());
    }
    return 0;
}

// what a calibration the overlay set does at the feet, from when it was set until the next one
void scoreEvents(const Session& session, const Reference& reference, const std::vector<CalibrationEvent>& events, bool listEach)
{
    std::vector<double> worldErrors, relativeErrors, mountAngles, mountDistances;
    for (size_t i = 0; i < events.size(); i++) {
        const CalibrationEvent& event = events[i];
        const double until = i + 1 < events.size() ? events[i + 1].time : session.duration;
        if (!event.enabled) {
            continue;
        }
        std::vector<double> errors;
        for (double time = event.time; time < until; time += 0.1) {
            const double error = event.relative ? reference.mountFeetError(session, time, event.calibration) : reference.feetError(time, event.calibration);
            if (error >= 0.0) {
                errors.push_back(error);
            }
        }
        (event.relative ? relativeErrors : worldErrors).insert((event.relative ? relativeErrors : worldErrors).end(), errors.begin(), errors.end());
        if (event.relative) {
            mountAngles.push_back(angleBetween(event.calibration.q, reference.mount.mount.q) * k_DEG);
            mountDistances.push_back((event.calibration.p - reference.mount.mount.p).norm() * 1e3);
        }
        if (listEach && !errors.empty()) {
            if (event.relative) {
                std::printf("  %s  relative, for %6.1f s: mount %.2f deg and %.1f mm off, feet %.1f mm (median)  %s\n", clock(event.time).c_str(), until - event.time, mountAngles.back(), mountDistances.back(), median(errors) * 1e3, event.note.c_str());
            } else {
                std::printf("  %s  world,    for %6.1f s: feet %.1f mm (median), %.1f at its end  %s\n", clock(event.time).c_str(), until - event.time, median(errors) * 1e3, errors.back() * 1e3, event.note.c_str());
            }
        }
    }
    printDistribution("feet error under a world calibration", worldErrors, 1e3, "mm");
    printDistribution("feet error under a relative calibration", relativeErrors, 1e3, "mm");
    if (!mountAngles.empty()) {
        printDistribution("a relative calibration's mount, rotation", mountAngles, 1.0, "deg");
        printDistribution("a relative calibration's mount, translation", mountDistances, 1.0, "mm");
    }
}

// the calibrations the overlay handed the driver for the target during the session
std::vector<CalibrationEvent> recordedEvents(const Device& target)
{
    std::vector<CalibrationEvent> events;
    for (const TransformEvent& transform : target.transforms) {
        CalibrationEvent event;
        event.time = transform.time;
        event.enabled = transform.record.flags & TransformFlag_Enabled;
        event.relative = transform.record.flags & TransformFlag_RelativeCoordSystem;
        event.calibration = Pose {
            Eigen::Quaterniond(transform.record.rotation[0], transform.record.rotation[1], transform.record.rotation[2], transform.record.rotation[3]).normalized(),
            Eigen::Vector3d(transform.record.translation[0], transform.record.translation[1], transform.record.translation[2]),
        };
        events.push_back(event);
    }
    return events;
}

// what the driver made of a relative calibration, pose by pose: how good what it applied was, overall and stretch by stretch of
// relative calibration, and whether it followed anything at all
void scoreApplied(const Session& session, const Reference& reference, const std::vector<CalibrationEvent>& events, const std::vector<AppliedEvent>& appliedEvents)
{
    std::vector<double> errors;
    for (const AppliedEvent& applied : appliedEvents) {
        const double error = reference.feetError(applied.time, applied.calibration);
        if (error >= 0.0) {
            errors.push_back(error);
        }
    }
    printDistribution("feet error", errors, 1e3, "mm");

    for (size_t i = 0; i < events.size(); i++) {
        if (!(events[i].enabled && events[i].relative) || (i > 0 && events[i - 1].enabled && events[i - 1].relative)) {
            continue;
        }
        size_t last = i;
        while (last + 1 < events.size() && events[last + 1].enabled && events[last + 1].relative) {
            last++;
        }
        const double begin = events[i].time;
        const double end = last + 1 < events.size() ? events[last + 1].time : session.duration;

        std::vector<double> stretch, settled;
        const AppliedEvent* first = nullptr;
        double moved = 0.0, atEnd = -1.0;
        for (const AppliedEvent& applied : appliedEvents) {
            if (applied.time < begin || applied.time >= end) {
                continue;
            }
            Pose unused;
            Eigen::Vector3d feet;
            if (!reference.at(applied.time, unused, feet)) {
                continue;
            }
            if (!first) {
                first = &applied;
            }
            moved = std::max(moved, reference.feetDistance(applied.calibration, first->calibration, feet));
            atEnd = reference.feetError(applied.time, applied.calibration);
            stretch.push_back(atEnd);
            if (applied.time >= begin + 30.0) {
                settled.push_back(atEnd);
            }
        }
        if (stretch.empty()) {
            continue;
        }
        std::printf("  %s  for %6.1f s, %zu calibrations from the overlay: feet %.1f mm (median), 90%% %.1f, most %.1f; after its first 30 s %.1f; it moved %.1f mm from where it began\n",
            clock(begin).c_str(), end - begin, last - i + 1, 1e3 * median(stretch), 1e3 * percentile(stretch, 0.9), 1e3 * percentile(stretch, 1.0),
            settled.empty() ? -1.0 : 1e3 * median(settled), 1e3 * moved);
    }
}

void printLatency(const std::vector<LatencyEvent>& events)
{
    std::vector<double> latencies, confidences;
    for (const LatencyEvent& event : events) {
        latencies.push_back(event.latency);
        confidences.push_back(event.confidence);
    }
    if (latencies.empty()) {
        std::printf("  the driver's latency estimate: none\n");
        return;
    }
    std::printf("  the driver's latency estimate (%zu readings): median %.1f ms, from %.1f to %.1f; its confidence: median %.2f, from %.2f to %.2f\n", latencies.size(),
        1e3 * median(latencies), 1e3 * percentile(latencies, 0.1), 1e3 * percentile(latencies, 0.9), median(confidences), percentile(confidences, 0.1), percentile(confidences, 0.9));
}

int commandScore(const Arguments& arguments)
{
    Session session;
    Reference reference;
    if (!solve(arguments, session, reference, true)) {
        return 1;
    }
    const Device& target = *session.target();

    const std::vector<CalibrationEvent> events = recordedEvents(target);
    std::printf("what the overlay set during the session (%zu calibrations):\n", events.size());
    scoreEvents(session, reference, events, arguments.get("--each") != nullptr || events.size() <= 40);

    if (!target.applied.empty()) {
        std::printf("what the driver applied under relative calibration (its blend, recorded %zu times):\n", target.applied.size());
        scoreApplied(session, reference, events, target.applied);
    }
    return 0;
}

// The driver's own relative calibration run over the recording with the calibrations the overlay set live. A recording made by
// the same driver code has to come out as it was recorded: that is what holds the replay to the driver.
int commandDriver(const Arguments& arguments)
{
    Session session;
    Reference reference;
    if (!solve(arguments, session, reference, true)) {
        return 1;
    }
    const Device& target = *session.target();
    std::vector<CalibrationEvent> events = recordedEvents(target);
    if (arguments.get("--reference-mount") != nullptr) {
        // the driver alone: handed the hindsight mount from the first pose on, whatever the overlay set
        CalibrationEvent event;
        event.enabled = true;
        event.relative = true;
        event.calibration = reference.mount.mount;
        events = { event };
    }

    // where each recorded stretch of relative calibration began: the driver's first applied calibration of it
    std::vector<DriverProbe> probes;
    std::vector<const AppliedEvent*> recordedStarts;
    for (size_t i = 0; i < events.size(); i++) {
        if (!(events[i].enabled && events[i].relative) || (i > 0 && events[i - 1].enabled && events[i - 1].relative)) {
            continue;
        }
        for (const AppliedEvent& applied : target.applied) {
            // the driver has the overlay's transform up to a frame before the recording notes it
            if (applied.time >= events[i].time - 0.5) {
                if (applied.time <= events[i].time + 0.5) {
                    probes.push_back({ applied.time + 1e-4, events[i].calibration });
                    recordedStarts.push_back(&applied);
                }
                break;
            }
        }
    }

    const DriverReplayResult result = replayDriver(session, events, probes);

    std::printf("the driver's relative calibration over the recording, with %s:\n", arguments.get("--reference-mount") ? "the hindsight mount throughout" : "the calibrations the overlay set live");
    for (size_t i = 0; i < probes.size(); i++) {
        Pose unused;
        Eigen::Vector3d feet;
        if (!reference.at(recordedStarts[i]->time, unused, feet)) {
            continue;
        }
        std::printf("  %s  the calibration the recorded driver began this stretch with, against this code's at the same pose: %.2f mm apart at the feet\n",
            clock(recordedStarts[i]->time).c_str(), 1e3 * reference.feetDistance(result.probes[i], recordedStarts[i]->calibration, feet));
    }
    std::printf("what this code applies (%zu readings):\n", result.applied.size());
    scoreApplied(session, reference, events, result.applied);
    if (!target.applied.empty()) {
        std::printf("what the recorded driver applied (%zu readings):\n", target.applied.size());
        scoreApplied(session, reference, events, target.applied);
        printLatency(target.latency);
    }
    return 0;
}

int commandCheck(const Arguments& arguments)
{
    Session session;
    if (!loadSession(arguments, session)) {
        return 1;
    }
    std::printf("each device's reported motion against its own poses a moment apart (medians; the smaller of the two turning-rate columns is the frame the driver gives the rate in):\n");
    std::printf("  %-22s %10s %12s %14s %14s %14s\n", "device", "poses/s", "velocity", "rate, own axes", "rate, driver's", "turning");
    for (const auto& [index, device] : session.devices) {
        if (index != session.referenceIndex && index != session.targetIndex && index != 0) {
            continue;
        }
        std::vector<double> steps, velocityErrors, ownErrors, driverErrors, speeds;
        for (size_t i = 0, j = 0; i < device.samples.size(); i++) {
            const Sample& a = device.samples[i];
            if (i > 0) {
                steps.push_back(a.time - device.samples[i - 1].time);
            }
            j = std::max(j, i + 1);
            while (j < device.samples.size() && device.samples[j].time - a.time < 0.02) {
                j++;
            }
            if (j >= device.samples.size()) {
                break;
            }
            const Sample& b = device.samples[j];
            const double span = b.time - a.time;
            if (!a.valid || !b.valid || span > 0.06) {
                continue;
            }
            const Eigen::Vector3d ownRate = rotationVector(a.raw.q.conjugate() * b.raw.q) / span;
            if (ownRate.norm() < 0.5) {
                continue; // too slow a turn to tell the frames apart
            }
            const Eigen::Vector3d driverRate = rotationVector(b.raw.q * a.raw.q.conjugate()) / span;
            // the reported values at the middle of the span
            const Eigen::Vector3d reportedRate = 0.5 * (a.angularVelocity + b.angularVelocity);
            const Eigen::Vector3d reportedVelocity = 0.5 * (a.velocity + b.velocity);
            velocityErrors.push_back((reportedVelocity - (b.raw.p - a.raw.p) / span).norm());
            ownErrors.push_back((reportedRate - ownRate).norm());
            driverErrors.push_back((reportedRate - driverRate).norm());
            speeds.push_back(ownRate.norm());
        }
        if (speeds.size() < 50) {
            std::printf("  %-22s %10.1f   turned too little to tell\n", device.name().c_str(), steps.empty() ? 0.0 : 1.0 / median(steps));
            continue;
        }
        std::printf("  %-22s %10.1f %8.3f m/s %10.3f rad/s %10.3f rad/s %10.2f rad/s\n", device.name().c_str(), 1.0 / median(steps), median(velocityErrors), median(ownErrors), median(driverErrors), median(speeds));
    }

    // the driver's own latency estimate, next to the offset the whole recording gives
    Reference reference = solveReference(session, referenceOptions(arguments));
    if (reference.ok) {
        std::printf("time offset over the recording, poses taken as of when they came in: target %.1f ms behind the reference\n", reference.timeOffset * 1e3);
        ReferenceOptions byPoseTime = referenceOptions(arguments);
        byPoseTime.usePoseTimeOffset = true;
        const Reference other = solveReference(session, byPoseTime);
        if (other.ok) {
            std::printf("  and taken as of the time each driver says (poseTimeOffset): %.1f ms\n", other.timeOffset * 1e3);
        }
        if (const Device* target = session.target(); target && !target->latency.empty()) {
            std::vector<double> latencies, confidences;
            for (const LatencyEvent& event : target->latency) {
                confidences.push_back(event.confidence);
                if (event.confidence > 0.5) {
                    latencies.push_back(event.latency);
                }
            }
            std::printf("  the driver's own estimate while it was confident (%zu of %zu readings): ", latencies.size(), target->latency.size());
            if (latencies.empty()) {
                std::printf("never\n");
            } else {
                std::printf("median %.1f, from %.1f to %.1f ms\n", median(latencies) * 1e3, percentile(latencies, 0.1) * 1e3, percentile(latencies, 0.9) * 1e3);
            }
            // the driver blends toward a relative calibration at a rate times this confidence: at zero it does not move
            std::printf("  its confidence, which the driver's blend is multiplied by: median %.2f, 90%% %.2f, most %.2f\n", median(confidences), percentile(confidences, 0.9), percentile(confidences, 1.0));
        }
    }
    return 0;
}

int commandReplay(const Arguments& arguments)
{
    Session session;
    Reference reference;
    if (!solve(arguments, session, reference, true)) {
        return 1;
    }
    ReplayOptions options;
    options.relative = arguments.number("--relative", 1.0) != 0.0;
    options.samples = static_cast<int>(arguments.number("--samples", 100.0));
    options.tickRate = arguments.number("--tick", options.tickRate);
    options.verbose = arguments.get("--log") != nullptr;
    const ReplayResult result = replayOverlay(session, options);
    if (!result.ok) {
        std::fprintf(stderr, "spacecal-bench: replay: %s\n", result.error.c_str());
        return 1;
    }
    std::printf("the overlay's calibration over the recording (%s, %d samples a calibration): %zu computed, %zu applied\n", options.relative ? "relative" : "world", options.samples, result.computed, result.events.size());
    for (const auto& [reason, count] : result.rejections) {
        std::printf("  rejected for %s: %zu\n", reason.c_str(), count);
    }
    scoreEvents(session, reference, result.events, arguments.get("--each") != nullptr || result.events.size() <= 40);

    if (arguments.get("--driver") != nullptr && options.relative) {
        // and the driver's own relative calibration over the same poses, with what the overlay's code just set
        const DriverReplayResult driver = replayDriver(session, result.events);
        std::printf("what the driver applies of it (%zu readings):\n", driver.applied.size());
        scoreApplied(session, reference, result.events, driver.applied);
    }
    return 0;
}

int commandSimulate(const Arguments& arguments)
{
    SimulateOptions options;
    options.duration = arguments.number("--duration", options.duration);
    options.seed = static_cast<unsigned>(arguments.number("--seed", options.seed));
    options.targetLatency = arguments.number("--target-latency", options.targetLatency * 1e3) * 1e-3;
    options.referenceLatency = arguments.number("--reference-latency", options.referenceLatency * 1e3) * 1e-3;
    options.jumpTime = arguments.number("--jump", options.jumpTime);
    options.targetFrameStepTime = arguments.number("--frame-step", options.targetFrameStepTime);
    options.referenceFrameStepTime = arguments.number("--reference-frame-step", options.referenceFrameStepTime);
    options.driftMetres = arguments.number("--drift", options.driftMetres * 1e3) * 1e-3;
    std::string error;
    if (!simulate(arguments.recording, options, error)) {
        std::fprintf(stderr, "spacecal-bench: %s\n", error.c_str());
        return 1;
    }
    std::printf("wrote %s: %.0f s\n", arguments.recording.c_str(), options.duration);
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    Arguments arguments;
    for (int i = 1; i < argc; i++) {
        const std::string argument = argv[i];
        if (argument.rfind("--", 0) == 0) {
            const bool hasValue = i + 1 < argc && std::strncmp(argv[i + 1], "--", 2) != 0;
            arguments.options.emplace_back(argument, hasValue ? argv[++i] : "");
        } else if (arguments.command.empty()) {
            arguments.command = argument;
        } else if (arguments.recording.empty()) {
            arguments.recording = argument;
        }
    }

    if (!arguments.recording.empty()) {
        if (arguments.command == "info") {
            return commandInfo(arguments);
        }
        if (arguments.command == "reference") {
            return commandReference(arguments);
        }
        if (arguments.command == "score") {
            return commandScore(arguments);
        }
        if (arguments.command == "replay") {
            return commandReplay(arguments);
        }
        if (arguments.command == "driver") {
            return commandDriver(arguments);
        }
        if (arguments.command == "check") {
            return commandCheck(arguments);
        }
        if (arguments.command == "simulate") {
            return commandSimulate(arguments);
        }
    }

    std::fprintf(stderr,
        "usage: spacecal-bench info|reference|score|check|driver RECORDING [--reference N --target N] [--window S] [--feet M] [--pose-time]\n"
        "       spacecal-bench replay RECORDING [--relative 0|1] [--samples 100|250|500] [--each] [--log]\n"
        "       spacecal-bench simulate RECORDING [--duration S] [--seed N] [--target-latency MS] [--reference-latency MS] [--jump S] [--frame-step S] [--reference-frame-step S] [--drift MM]\n");
    return 2;
}
