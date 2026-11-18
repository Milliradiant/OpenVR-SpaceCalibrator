#include "simulate.h"

#include "geometry.h"
#include "pose_recording.h"
#include <random>
#include <sstream>

namespace bench {

using namespace spacecal::recording;

namespace {

    constexpr int k_RUNNING_OK = 200;

    struct World {
        SimulateOptions options;
        Pose mount; // the target in the reference's frame
        Pose calibration; // the world calibration at the start
        Pose targetWorldFromDriver;

        // how lively the head is: stretches of looking about, and stretches of holding nearly still
        double activity(double time) const
        {
            const double phase = std::fmod(time, 40.0);
            const double edge = 1.5;
            if (phase < 24.0) {
                return std::min({ 1.0, phase / edge, (24.0 - phase) / edge });
            }
            return 0.0;
        }

        // the reference device (a headset) in its own tracking system's world
        Pose head(double time) const
        {
            // a little sway is left when the head holds still
            const double a = 0.04 + 0.96 * activity(time);
            const Eigen::AngleAxisd yaw(0.9 * a * std::sin(0.83 * time) + 0.3 * std::sin(0.05 * time), Eigen::Vector3d::UnitY());
            const Eigen::AngleAxisd pitch(0.45 * a * std::sin(1.31 * time + 0.7), Eigen::Vector3d::UnitX());
            const Eigen::AngleAxisd roll(0.25 * a * std::sin(1.07 * time + 2.1), Eigen::Vector3d::UnitZ());
            const Eigen::Vector3d position(
                0.2 + 0.25 * std::sin(0.11 * time) + 0.08 * a * std::sin(1.9 * time),
                1.62 + 0.03 * a * std::sin(2.3 * time + 0.4),
                -0.3 + 0.2 * std::sin(0.07 * time + 1.0) + 0.07 * a * std::sin(1.6 * time + 1.3));
            return Pose { Eigen::Quaterniond(yaw * pitch * roll), position };
        }

        // the world calibration at a time: a slow drift of the reference's space, and one step if asked for
        Pose calibrationAt(double time) const
        {
            Pose drift;
            drift.p = Eigen::Vector3d(std::sin(0.004 * time), 0.0, 1.0 - std::cos(0.003 * time)) * options.driftMetres;
            drift.q = Eigen::Quaterniond(Eigen::AngleAxisd(options.driftDegrees / k_DEG * std::sin(0.0035 * time), Eigen::Vector3d::UnitY()));
            Pose step;
            if (options.jumpTime > 0.0 && time >= options.jumpTime) {
                step.p = Eigen::Vector3d(0.04, 0.0, -0.03);
                step.q = Eigen::Quaterniond(Eigen::AngleAxisd(2.0 / k_DEG, Eigen::Vector3d::UnitY()));
            }
            return step * drift * calibration;
        }

        // the target device in its own tracking system's world: C^-1 * H * L
        Pose tracker(double time) const
        {
            return calibrationAt(time).inverse() * head(time) * mount;
        }
    };

    Pose noisy(const Pose& pose, double positionNoise, double rotationNoise, std::mt19937& rng)
    {
        std::normal_distribution<double> normal(0.0, 1.0);
        Pose out = pose;
        out.p += positionNoise * Eigen::Vector3d(normal(rng), normal(rng), normal(rng));
        out.q = (out.q * fromRotationVector(rotationNoise * Eigen::Vector3d(normal(rng), normal(rng), normal(rng)))).normalized();
        return out;
    }

    PoseRecord recordOf(const Pose& raw, const Pose& before, const Pose& after, double span, double poseTimeOffset)
    {
        PoseRecord record;
        record.flags = PoseFlag_Valid | PoseFlag_Connected;
        record.result = k_RUNNING_OK;
        record.poseTimeOffset = poseTimeOffset;
        const Eigen::Vector3d velocity = (after.p - before.p) / span;
        // the body's turning rate in its own axes, as SteamVR takes it
        const Eigen::Vector3d rate = rotationVector(before.q.conjugate() * after.q) / span;
        for (int i = 0; i < 3; i++) {
            record.position[i] = raw.p[i];
            record.velocity[i] = velocity[i];
            record.angularVelocity[i] = rate[i];
        }
        record.rotation[0] = raw.q.w();
        record.rotation[1] = raw.q.x();
        record.rotation[2] = raw.q.y();
        record.rotation[3] = raw.q.z();
        return record;
    }

    FrameRecord frameOf(const Pose& worldFromDriver)
    {
        FrameRecord frame;
        frame.worldFromDriverRotation[0] = worldFromDriver.q.w();
        frame.worldFromDriverRotation[1] = worldFromDriver.q.x();
        frame.worldFromDriverRotation[2] = worldFromDriver.q.y();
        frame.worldFromDriverRotation[3] = worldFromDriver.q.z();
        for (int i = 0; i < 3; i++) {
            frame.worldFromDriverTranslation[i] = worldFromDriver.p[i];
        }
        return frame;
    }

    std::string poseText(const Pose& pose)
    {
        std::ostringstream out;
        out.precision(12);
        out << pose.q.w() << " " << pose.q.x() << " " << pose.q.y() << " " << pose.q.z() << " " << pose.p.x() << " " << pose.p.y() << " " << pose.p.z();
        return out.str();
    }

} // namespace

bool simulate(const std::filesystem::path& file, const SimulateOptions& options, std::string& error)
{
    World world;
    world.options = options;
    // a tracker strapped to the top of a headset, turned every which way, and a second tracking system's world a few metres and
    // a third of a turn off the first's
    world.mount = Pose {
        Eigen::Quaterniond(Eigen::AngleAxisd(-95.4 / k_DEG, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-50.7 / k_DEG, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(130.7 / k_DEG, Eigen::Vector3d::UnitX())),
        Eigen::Vector3d(0.001, 0.084, -0.055),
    };
    world.calibration = Pose {
        Eigen::Quaterniond(Eigen::AngleAxisd(-36.3 / k_DEG, Eigen::Vector3d::UnitY())),
        Eigen::Vector3d(-0.93, 2.30, -0.67),
    };
    world.targetWorldFromDriver = Pose {
        Eigen::Quaterniond(Eigen::AngleAxisd(17.0 / k_DEG, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(1.2 / k_DEG, Eigen::Vector3d::UnitX())),
        Eigen::Vector3d(0.4, -0.02, 1.1),
    };

    Writer writer;
    if (!writer.openFile(file)) {
        error = writer.error();
        return false;
    }

    writer.session(0.0, "format=1\nprogram=spacecal-bench simulate\nseed=" + std::to_string(options.seed) + "\n");
    writer.device(0.0, 0, "SIM-HEADSET", "Simulated headset", "simulated_reference", 1);
    writer.device(0.0, 1, "SIM-TRACKER", "Simulated tracker", "simulated_target", 3);
    writer.note(0.0, "truth_mount=" + poseText(world.mount));
    writer.note(0.0, "truth_time_offset=" + std::to_string(options.targetLatency - options.referenceLatency));
    writer.frame(0.0, 0, frameOf(Pose {}));
    writer.frame(0.0, 1, frameOf(world.targetWorldFromDriver));

    // the overlay naming the pair, with the world calibration the session begins at applied and never renewed
    TransformRecord transform;
    transform.flags = TransformFlag_Enabled | TransformFlag_CalibrateMotionVecs;
    transform.referenceDevice = 0;
    transform.targetDevice = 1;
    transform.rotation[0] = world.calibration.q.w();
    transform.rotation[1] = world.calibration.q.x();
    transform.rotation[2] = world.calibration.q.y();
    transform.rotation[3] = world.calibration.q.z();
    for (int i = 0; i < 3; i++) {
        transform.translation[i] = world.calibration.p[i];
    }
    writer.transform(0.5, 1, transform);

    std::mt19937 rng(options.seed);
    std::uniform_real_distribution<double> jitter(-0.0004, 0.0004);
    const Pose targetDriverFromWorld = world.targetWorldFromDriver.inverse();
    const double span = 0.002;

    // each device's poses come in at its own rate, each of where the device was its own latency ago
    double nextReference = 0.0, nextTarget = 0.0013;
    const Pose frameStep { Eigen::Quaterniond(Eigen::AngleAxisd(1.5 / k_DEG, Eigen::Vector3d::UnitY())), Eigen::Vector3d(0.15, 0.02, -0.12) };
    bool targetStepped = false, referenceStepped = false;
    while (nextReference < options.duration || nextTarget < options.duration) {
        if (nextReference <= nextTarget) {
            const double time = nextReference, of = time - options.referenceLatency;
            const Pose raw = noisy(world.head(of), options.referencePositionNoise, options.referenceRotationNoise, rng);
            if (options.referenceFrameStepTime > 0.0 && !referenceStepped && time >= options.referenceFrameStepTime) {
                referenceStepped = true;
                writer.frame(time, 0, frameOf(frameStep));
            }
            writer.pose(time, 0, recordOf(raw, world.head(of - span), world.head(of + span), 2.0 * span, 0.0));
            nextReference += 1.0 / options.referenceRate + jitter(rng);
        } else {
            const double time = nextTarget, of = time - options.targetLatency;
            if (options.targetFrameStepTime > 0.0 && !targetStepped && time >= options.targetFrameStepTime) {
                targetStepped = true;
                writer.frame(time, 1, frameOf(frameStep * world.targetWorldFromDriver));
            }
            const Pose raw = targetDriverFromWorld * noisy(world.tracker(of), options.targetPositionNoise, options.targetRotationNoise, rng);
            writer.pose(time, 1, recordOf(raw, targetDriverFromWorld * world.tracker(of - span), targetDriverFromWorld * world.tracker(of + span), 2.0 * span, -options.reportedTargetAge));
            nextTarget += 1.0 / options.targetRate + jitter(rng);
        }
        if (std::fmod(std::min(nextReference, nextTarget), 5.0) < 0.004) {
            writer.flush(false);
        }
    }

    writer.note(options.duration, "end of session");
    writer.close();
    if (!writer.error().empty()) {
        error = writer.error();
        return false;
    }
    return true;
}

} // namespace bench
