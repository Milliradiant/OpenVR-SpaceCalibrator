// The driver's relative calibration (src/driver/relative_calibration.cpp) on made-up poses: a tracker fixed to a headset, the two
// tracking systems' worlds a known calibration apart. What it applies has to be that calibration, and to follow it when it moves.

#include "relative_calibration.h"
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

static vr::DriverPose_t driverPose(const Eigen::Affine3d& world)
{
    vr::DriverPose_t pose = {};
    const Eigen::Quaterniond rotation(world.rotation());
    pose.qRotation = { rotation.w(), rotation.x(), rotation.y(), rotation.z() };
    pose.qWorldFromDriverRotation = { 1.0, 0.0, 0.0, 0.0 };
    pose.qDriverFromHeadRotation = { 1.0, 0.0, 0.0, 0.0 };
    for (int i = 0; i < 3; i++) {
        pose.vecPosition[i] = world.translation()[i];
        // velocities a driver might report: the calibration takes the pose as it is and must not be moved by them
        pose.vecVelocity[i] = 3.0;
        pose.vecAngularVelocity[i] = 5.0;
    }
    pose.poseIsValid = true;
    pose.deviceIsConnected = true;
    pose.result = vr::TrackingResult_Running_OK;
    return pose;
}

// how far two calibrations put a point a metre and a half below the tracker apart
static double apart(const Pose_t& one, const Eigen::Affine3d& other, const Eigen::Vector3d& tracker)
{
    const Eigen::Vector3d feet = tracker - Eigen::Vector3d(0.0, 1.5, 0.0);
    return ((one.pos + one.rot * feet) - other * feet).norm();
}

int main()
{
    constexpr vr::TrackedDeviceIndex_t k_HEADSET = 0, k_TRACKER = 3, k_FOOT = 5;
    const Eigen::Affine3d mount = Eigen::Translation3d(0.001, 0.084, -0.055) * Eigen::AngleAxisd(2.1, Eigen::Vector3d(0.3, -0.8, 0.5).normalized());
    Eigen::Affine3d calibration = Eigen::Translation3d(-1.0, 2.2, -0.7) * Eigen::AngleAxisd(-2.5, Eigen::Vector3d::UnitY());

    ipc::protocol::Command_SetDeviceTransform_t transform = {};
    transform.enabled(true);
    transform.relativeCoordSystem(true);
    transform.unRelativeReferenceOpenvrDeviceId = k_HEADSET;
    transform.unRelativeTargetOpenvrDeviceId = k_TRACKER;
    transform.translation = { { mount.translation().x(), mount.translation().y(), mount.translation().z() } };
    const Eigen::Quaterniond mountRotation(mount.rotation());
    transform.rotation = { mountRotation.w(), mountRotation.x(), mountRotation.y(), mountRotation.z() };

    auto relative = std::make_unique<RelativeCalibration>();
    static vr::DriverPose_t poses[vr::k_unMaxTrackedDeviceCount] = {};
    double time = 100.0;
    Eigen::Vector3d trackerPosition = Eigen::Vector3d::Zero();
    Pose_t applied;

    // the headset moving about, the tracker where the mount and the calibration of the moment put it, 500 poses a second
    const auto run = [&](double seconds, vr::TrackedDeviceIndex_t device) {
        RelativeCalibration::Update update;
        for (const double end = time + seconds; time < end; time += 0.002) {
            const Eigen::Affine3d head = Eigen::Translation3d(0.3 * std::sin(time), 1.6, 0.2 * std::cos(0.7 * time)) * Eigen::AngleAxisd(1.2 * std::sin(1.3 * time), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0.4 * std::sin(0.9 * time), Eigen::Vector3d::UnitX());
            const Eigen::Affine3d tracker = calibration.inverse() * head * mount;
            poses[k_HEADSET] = driverPose(head);
            poses[k_TRACKER] = driverPose(tracker);
            trackerPosition = tracker.translation();
            update = relative->update(device, transform, poses, time, applied);
        }
        return update;
    };

    // nothing before the first pose pair; from it on, the calibration itself
    CHECK(!relative->update(k_TRACKER, transform, poses, time, applied).hasCalibration);
    RelativeCalibration::Update update = run(0.01, k_TRACKER);
    CHECK(update.hasCalibration && update.computed);
    CHECK(apart(applied, calibration, trackerPosition) < 1e-9);
    run(3.0, k_TRACKER);
    CHECK(apart(applied, calibration, trackerPosition) < 1e-6);

    // the two spaces drift 30 mm apart: followed within a few seconds. (it did not move at all when its speed was multiplied by
    // a latency confidence that was zero)
    calibration = Eigen::Translation3d(0.03, 0.0, 0.0) * calibration;
    run(0.1, k_TRACKER);
    CHECK(apart(applied, calibration, trackerPosition) > 0.02); // not at once: one pose pair's noise must not pass straight through
    run(3.0, k_TRACKER);
    CHECK(apart(applied, calibration, trackerPosition) < 0.002);

    // a space steps 20 cm and 3 degrees (a recentre): most of the way within half a second, at its fastest, and the rest gently
    calibration = Eigen::Translation3d(0.2, 0.0, -0.05) * Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY()) * calibration;
    run(0.5, k_TRACKER);
    CHECK(apart(applied, calibration, trackerPosition) < 0.03);
    run(2.0, k_TRACKER);
    CHECK(apart(applied, calibration, trackerPosition) < 0.005);
    run(3.0, k_TRACKER);
    CHECK(apart(applied, calibration, trackerPosition) < 0.001);

    // the headset loses tracking: the calibration stays as it was, and goes on from there when it is back
    const Pose_t before = applied;
    poses[k_HEADSET].poseIsValid = false;
    update = relative->update(k_TRACKER, transform, poses, time, applied);
    CHECK(update.hasCalibration && !update.computed);
    CHECK(applied.pos == before.pos && applied.rot.coeffs() == before.rot.coeffs());

    // another device of the target's tracking system has a calibration of its own, begun at its own first pose
    Pose_t other;
    CHECK(!relative->update(k_FOOT, transform, poses, time, other).hasCalibration);
    run(0.01, k_FOOT);
    CHECK(apart(applied, calibration, trackerPosition) < 1e-9);

    // the relative session ends and another begins after the spaces moved: it starts from its own first pose pair
    relative->drop(k_TRACKER);
    calibration = Eigen::Translation3d(0.0, 0.0, 0.4) * calibration;
    update = run(0.004, k_TRACKER);
    CHECK(update.hasCalibration && apart(applied, calibration, trackerPosition) < 1e-9);

    // an index that is no device
    CHECK(!relative->update(vr::k_unTrackedDeviceIndexInvalid, transform, poses, time, applied).hasCalibration);
    relative->drop(vr::k_unTrackedDeviceIndexInvalid);

    if (g_failures) {
        std::printf("%d checks failed\n", g_failures);
        return 1;
    }
    std::printf("the driver's relative calibration holds\n");
    return 0;
}
