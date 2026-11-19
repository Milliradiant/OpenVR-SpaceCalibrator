// What the driver applies of a calibration (src/driver/applied_calibration.cpp), on made-up poses: a tracker fixed to a headset, the
// two tracking systems' worlds a known calibration apart. What it applies has to come to that calibration, follow it when the two
// spaces move against each other, and never move the devices with a step.

#include "applied_calibration.h"
#include <cmath>
#include <cstdio>
#include <memory>

using namespace spacecal;

static int g_failures = 0;

#define CHECK(condition)                                                    \
    do {                                                                    \
        if (!(condition)) {                                                 \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            g_failures++;                                                   \
        }                                                                   \
    } while (0)

static vr::DriverPose_t driverPose(const Eigen::Affine3d& world, const Eigen::Affine3d& worldFromDriver = Eigen::Affine3d::Identity())
{
    vr::DriverPose_t pose = {};
    const Eigen::Affine3d driver = worldFromDriver.inverse() * world;
    const Eigen::Quaterniond rotation(driver.linear());
    const Eigen::Quaterniond frameRotation(worldFromDriver.linear());
    pose.qRotation = { rotation.w(), rotation.x(), rotation.y(), rotation.z() };
    pose.qWorldFromDriverRotation = { frameRotation.w(), frameRotation.x(), frameRotation.y(), frameRotation.z() };
    pose.qDriverFromHeadRotation = { 1.0, 0.0, 0.0, 0.0 };
    for (int i = 0; i < 3; i++) {
        pose.vecPosition[i] = driver.translation()[i];
        pose.vecWorldFromDriverTranslation[i] = worldFromDriver.translation()[i];
        // velocities a driver might report: the calibration takes the pose as it is and must not be moved by them
        pose.vecVelocity[i] = 3.0;
        pose.vecAngularVelocity[i] = 5.0;
    }
    pose.poseIsValid = true;
    pose.deviceIsConnected = true;
    pose.result = vr::TrackingResult_Running_OK;
    return pose;
}

static Eigen::Affine3d headAt(double time)
{
    return Eigen::Translation3d(0.3 * std::sin(time), 1.6, 0.2 * std::cos(0.7 * time)) * Eigen::AngleAxisd(0.6 * std::sin(1.3 * time), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0.2 * std::sin(0.9 * time), Eigen::Vector3d::UnitX());
}

// how far two calibrations put a point a metre and a half below the tracker apart
static double apart(const Pose_t& one, const Eigen::Affine3d& other, const Eigen::Vector3d& tracker)
{
    const Eigen::Vector3d feet = tracker - Eigen::Vector3d(0.0, 1.5, 0.0);
    return ((one.pos + one.rot * feet) - other * feet).norm();
}
static double apart(const Pose_t& one, const Pose_t& other, const Eigen::Vector3d& tracker)
{
    return apart(one, Eigen::Affine3d(Eigen::Translation3d(other.pos) * other.rot), tracker);
}

int main()
{
    const Eigen::Affine3d mount = Eigen::Translation3d(0.001, 0.084, -0.055) * Eigen::AngleAxisd(2.1, Eigen::Vector3d(0.3, -0.8, 0.5).normalized());
    const Pose_t mountPose = { .rot = Eigen::Quaterniond(mount.linear()), .pos = mount.translation() };
    Eigen::Affine3d calibration = Eigen::Translation3d(-1.0, 2.2, -0.7) * Eigen::AngleAxisd(-2.5, Eigen::Vector3d::UnitY());
    Eigen::Affine3d targetFrame = Eigen::Affine3d::Identity(); // the tracker's system's world-from-driver

    auto applied = std::make_unique<AppliedCalibration>();
    vr::DriverPose_t headset = {}, tracker = {};
    double time = 100.0;
    Eigen::Vector3d trackerPosition = Eigen::Vector3d::Zero();
    Pose_t now;
    bool headsetStopped = false;
    Eigen::Affine3d stoppedHead = Eigen::Affine3d::Identity();
    double largestStep = 0.0; // the most the applied calibration moved the feet from one pose to the next

    // the headset moving about as in play (turning up to some 50 degrees a second), the tracker where the mount and the calibration
    // of the moment put it, 100 headset poses a second and 500 of the tracker, as a Quest and a lighthouse tracker give them
    const auto run = [&](double seconds) {
        for (const double end = time + seconds; time < end; time += 0.002) {
            const Eigen::Affine3d head = headAt(time);
            const Eigen::Affine3d trackerWorld = calibration.inverse() * head * mount;
            if (std::fmod(time, 0.01) < 0.002) {
                // a stopped headset's poses still differ in their last digits, as a predicting driver's do
                headset = driverPose(headsetStopped ? Eigen::Affine3d(Eigen::Translation3d(1e-7 * std::sin(1e3 * time), 0.0, 0.0) * stoppedHead) : head);
            }
            tracker = driverPose(trackerWorld, targetFrame);
            trackerPosition = trackerWorld.translation();
            const Pose_t before = now;
            const bool had = applied->current(time, now);
            applied->addPair(time, headset, tracker, mountPose);
            applied->current(time, now);
            if (had) {
                largestStep = std::max(largestStep, apart(now, before, trackerPosition));
            }
        }
    };

    // nothing before the first pose pair; from it on, the calibration itself
    CHECK(!applied->current(time, now));
    run(0.01);
    CHECK(applied->current(time, now));
    CHECK(apart(now, calibration, trackerPosition) < 1e-6);
    run(3.0);
    CHECK(apart(now, calibration, trackerPosition) < 1e-6);

    // the two spaces drift 30 mm apart: not at once, so that one pose pair's noise does not pass straight through, and gently
    calibration = Eigen::Translation3d(0.03, 0.0, 0.0) * calibration;
    largestStep = 0.0;
    run(0.1);
    CHECK(apart(now, calibration, trackerPosition) > 0.029);
    run(12.0);
    CHECK(apart(now, calibration, trackerPosition) < 0.005);
    CHECK(largestStep < 0.0001); // a tenth of a millimetre a headset pose at most: 10 mm a second
    run(8.0);
    CHECK(apart(now, calibration, trackerPosition) < 0.001);

    // a space steps 20 cm and 3 degrees (a recentre): caught up with within a couple of seconds, still without a step
    calibration = Eigen::Translation3d(0.2, 0.0, -0.05) * Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY()) * calibration;
    largestStep = 0.0;
    run(0.3);
    CHECK(!applied->status().catchingUp);
    CHECK(apart(now, calibration, trackerPosition) > 0.15);
    run(4.0);
    CHECK(apart(now, calibration, trackerPosition) < 0.01);
    CHECK(largestStep < 0.005); // a glide of a second or so, at most half a metre a second at the feet: no step
    run(3.0);
    CHECK(apart(now, calibration, trackerPosition) < 0.001);
    CHECK(!applied->status().catchingUp);

    // the headset loses tracking: the calibration stays as it was
    Pose_t before = now;
    headset.poseIsValid = false;
    applied->addPair(time, headset, tracker, mountPose);
    CHECK(applied->current(time, now));
    CHECK(apart(now, before, trackerPosition) < 1e-9);
    run(1.0);

    // the headset's tracking stops while it goes on saying its pose is valid, and the tracker on it moves on: the two devices'
    // motions disagree, and the calibration does not follow what the pairs would make of them
    before = now;
    stoppedHead = headAt(time);
    headsetStopped = true;
    run(8.0);
    CHECK(applied->status().holding);
    CHECK(apart(now, before, trackerPosition) < 0.003);
    // and once its tracking is back and the two have moved together for a while, the pairs count again
    headsetStopped = false;
    run(3.0);
    CHECK(!applied->status().holding);
    CHECK(apart(now, calibration, trackerPosition) < 0.003);

    // the tracker's system steps its world (its world-from-driver moves 10 cm and 2 degrees while its poses in the driver run on):
    // the devices did not move, so the calibration takes the step back at once and nothing moves at the feet
    run(2.0);
    const Eigen::Affine3d step = Eigen::Translation3d(0.1, 0.0, 0.03) * Eigen::AngleAxisd(0.035, Eigen::Vector3d::UnitY());
    const Eigen::Vector3d placeBefore = now.pos + now.rot * (trackerPosition - Eigen::Vector3d(0.0, 1.5, 0.0));
    targetFrame = step * targetFrame;
    calibration = calibration * step.inverse(); // a tracker's world pose now comes out `step` on, for the same place in the room
    run(0.002);
    const Eigen::Vector3d placeAfter = now.pos + now.rot * (trackerPosition - Eigen::Vector3d(0.0, 1.5, 0.0));
    CHECK((placeAfter - placeBefore).norm() < 0.002);
    CHECK(apart(now, calibration, trackerPosition) < 0.001);
    run(2.0);
    CHECK(apart(now, calibration, trackerPosition) < 0.001);

    // the relative session ends and another begins after the spaces moved: it starts from its own first pose pair
    applied->reset();
    CHECK(!applied->current(time, now));
    calibration = Eigen::Translation3d(0.0, 0.0, 0.4) * calibration;
    run(0.004);
    CHECK(applied->current(time, now) && apart(now, calibration, trackerPosition) < 0.02); // its first pair's, which may be of a headset pose 10 ms old
    run(1.5);
    CHECK(apart(now, calibration, trackerPosition) < 0.001);

    // a world calibration from the overlay: a continuous calibration's new one is gone to without a step, within seconds
    const Pose_t world = { .rot = Eigen::Quaterniond(Eigen::AngleAxisd(-2.45, Eigen::Vector3d::UnitY())), .pos = Eigen::Vector3d(-0.95, 2.2, -0.68) };
    auto worldOnly = std::make_unique<AppliedCalibration>();
    worldOnly->setWorld(time, world, true);
    CHECK(worldOnly->current(time, now) && apart(now, world, trackerPosition) < 1e-9); // the first at once
    const Pose_t moved = { .rot = Eigen::Quaterniond(Eigen::AngleAxisd(-2.44, Eigen::Vector3d::UnitY())), .pos = Eigen::Vector3d(-0.94, 2.2, -0.68) };
    largestStep = 0.0;
    Pose_t last = now;
    for (const double end = time + 14.0; time < end; time += 0.002) {
        worldOnly->setWorld(time, moved, true);
        worldOnly->watchFrames(driverPose(Eigen::Translation3d(trackerPosition) * Eigen::Quaterniond::Identity()), nullptr);
        worldOnly->current(time, now);
        largestStep = std::max(largestStep, apart(now, last, trackerPosition));
        last = now;
    }
    CHECK(apart(now, moved, trackerPosition) < 0.001);
    CHECK(largestStep < 0.0001);
    // one that is not a continuous calibration's is applied at once
    worldOnly->setWorld(time, world, false);
    CHECK(worldOnly->current(time, now) && apart(now, world, trackerPosition) < 1e-9);

    if (g_failures) {
        std::printf("%d checks failed\n", g_failures);
        return 1;
    }
    std::printf("what the driver applies of a calibration holds\n");
    return 0;
}
