// The mount estimator (src/overlay/mount_estimator.cpp) on made-up motions of two devices fixed to each other with a known mount.

#include "mount_estimator.h"
#include <cstdio>
#include <random>

using namespace spacecal;

static int g_failures = 0;

#define CHECK(condition)                                                    \
    do {                                                                    \
        if (!(condition)) {                                                 \
            std::printf("FAILED %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            g_failures++;                                                   \
        }                                                                   \
    } while (0)

struct Mount {
    Eigen::Quaterniond rotation;
    Eigen::Vector3d translation;
};

static std::mt19937 g_rng(7);

static Eigen::Vector3d randomVector(double scale)
{
    std::normal_distribution<double> normal(0.0, 1.0);
    return scale * Eigen::Vector3d(normal(g_rng), normal(g_rng), normal(g_rng));
}

static Eigen::Quaterniond rotationOf(const Eigen::Vector3d& turn)
{
    const double angle = turn.norm();
    return angle < 1e-12 ? Eigen::Quaterniond::Identity() : Eigen::Quaterniond(Eigen::AngleAxisd(angle, turn / angle));
}

// a motion of the reference, and the target's with it: B = L^-1 * A * L, each pose a little off as a tracked pose is
static MountMotion motionWith(const Mount& mount, const Eigen::Vector3d& axisWeights, double turnNoise, double moveNoise)
{
    std::uniform_real_distribution<double> angle(0.1, 0.5);
    Eigen::Vector3d axis = randomVector(1.0).cwiseProduct(axisWeights);
    axis.normalize();
    const Eigen::Quaterniond referenceRotation = rotationOf(axis * angle(g_rng));
    const Eigen::Vector3d referenceMove = randomVector(0.05);

    const Eigen::Quaterniond targetRotation = mount.rotation.conjugate() * referenceRotation * mount.rotation;
    const Eigen::Vector3d targetMove = mount.rotation.conjugate() * (referenceRotation * mount.translation + referenceMove - mount.translation);

    MountMotion motion;
    Eigen::Vector3d unused;
    motionBetween(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), referenceRotation * rotationOf(randomVector(turnNoise)), referenceMove + randomVector(moveNoise), motion.referenceTurn, motion.referenceMove);
    motionBetween(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), targetRotation * rotationOf(randomVector(turnNoise)), targetMove + randomVector(moveNoise), motion.targetTurn, motion.targetMove);
    return motion;
}

static double degreesOff(const MountEstimator& estimator, const Mount& mount)
{
    return estimator.rotation().angularDistance(mount.rotation) * 180.0 / EIGEN_PI;
}

static double millimetresOff(const MountEstimator& estimator, const Mount& mount)
{
    return (estimator.translation() - mount.translation).norm() * 1e3;
}

int main()
{
    const Mount mount { Eigen::Quaterniond(Eigen::AngleAxisd(2.1, Eigen::Vector3d(0.3, -0.8, 0.5).normalized())), Eigen::Vector3d(0.002, 0.084, -0.056) };
    const Eigen::Vector3d everyAxis(1.0, 1.0, 1.0);
    // a tracked pose's noise: a twentieth of a degree and half a millimetre
    constexpr double k_TURN_NOISE = 0.001, k_MOVE_NOISE = 0.0005;

    // nothing to go by, then too little, then an estimate that gets better with every motion
    MountEstimator estimator;
    CHECK(!estimator.ready() && estimator.weight() == 0.0);
    for (int i = 0; i < 100; i++) {
        CHECK(estimator.add(motionWith(mount, everyAxis, k_TURN_NOISE, k_MOVE_NOISE)) == MountEstimator::Added::Taken);
    }
    CHECK(!estimator.ready());
    for (int i = 0; i < 400; i++) {
        estimator.add(motionWith(mount, everyAxis, k_TURN_NOISE, k_MOVE_NOISE));
    }
    CHECK(estimator.ready());
    CHECK(degreesOff(estimator, mount) < 0.1 && millimetresOff(estimator, mount) < 1.0);
    for (int i = 0; i < 5000; i++) {
        estimator.add(motionWith(mount, everyAxis, k_TURN_NOISE, k_MOVE_NOISE));
    }
    CHECK(degreesOff(estimator, mount) < 0.02 && millimetresOff(estimator, mount) < 0.2);
    CHECK(estimator.turnResidual() > 0.0005 && estimator.turnResidual() < 0.01 && estimator.moveResidual() < 0.005);

    // a motion that turned too little says nothing; one in which the two devices did not move as one is left out
    MountMotion still;
    still.referenceTurn = Eigen::Vector3d(0.01, 0.0, 0.0);
    still.targetTurn = mount.rotation.conjugate() * still.referenceTurn;
    CHECK(estimator.add(still) == MountEstimator::Added::TooLittleTurn);
    const double weight = estimator.weight();
    MountMotion glitch = motionWith(mount, everyAxis, 0.0, 0.0);
    glitch.targetTurn *= 1.6; // the target turned much further than the reference
    CHECK(estimator.add(glitch) == MountEstimator::Added::Rejected);
    glitch = motionWith(mount, everyAxis, 0.0, 0.0);
    glitch.targetMove += Eigen::Vector3d(0.05, 0.0, 0.0); // or jumped 5 cm
    CHECK(estimator.add(glitch) == MountEstimator::Added::Rejected);
    CHECK(estimator.weight() == weight);
    // one bad motion in ten among good ones does not move the estimate
    for (int i = 0; i < 2000; i++) {
        MountMotion motion = motionWith(mount, everyAxis, k_TURN_NOISE, k_MOVE_NOISE);
        if (i % 10 == 0) {
            motion.targetMove += randomVector(0.05);
            motion.targetTurn += randomVector(0.05);
        }
        estimator.add(motion);
    }
    CHECK(degreesOff(estimator, mount) < 0.02 && millimetresOff(estimator, mount) < 0.2);
    CHECK(estimator.startedOver() == 0);

    // what a profile keeps of it gives the same estimate back, and goes on from there
    const std::vector<double> kept = estimator.state();
    CHECK(kept.size() == MountEstimator::k_STATE_SIZE);
    MountEstimator restored;
    CHECK(restored.restore(kept) && restored.ready());
    CHECK(restored.rotation().angularDistance(estimator.rotation()) < 1e-12 && (restored.translation() - estimator.translation()).norm() < 1e-12);
    CHECK(restored.weight() == estimator.weight());
    CHECK(restored.add(motionWith(mount, everyAxis, k_TURN_NOISE, k_MOVE_NOISE)) == MountEstimator::Added::Taken);
    // anything else is not taken, and leaves nothing behind
    std::vector<double> damaged = kept;
    damaged.pop_back();
    CHECK(!restored.restore(damaged) && !restored.ready() && restored.weight() == 0.0);
    damaged = kept;
    damaged[7] = std::nan("");
    CHECK(!restored.restore(damaged) && !restored.ready());
    damaged = kept;
    damaged[0] = 2.0; // a layout of some later version
    CHECK(!restored.restore(damaged) && !restored.ready());
    CHECK(!restored.restore({}) && !restored.ready());

    // the tracker is moved on the headset: before long hardly any motion fits, the estimate starts over, and finds the new mount
    const Mount moved { mount.rotation * Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX())), mount.translation + Eigen::Vector3d(0.03, 0.0, 0.01) };
    int untilStartedOver = 0;
    while (untilStartedOver < 2000 && estimator.add(motionWith(moved, everyAxis, k_TURN_NOISE, k_MOVE_NOISE)) != MountEstimator::Added::StartedOver) {
        untilStartedOver++;
    }
    CHECK(untilStartedOver < 500 && estimator.startedOver() == 1 && !estimator.ready());
    for (int i = 0; i < 1500; i++) {
        estimator.add(motionWith(moved, everyAxis, k_TURN_NOISE, k_MOVE_NOISE));
    }
    CHECK(estimator.ready() && degreesOff(estimator, moved) < 0.05 && millimetresOff(estimator, moved) < 0.5);

    // turns about one axis alone do not say how the target sits about that axis, nor where along it: no estimate from them
    MountEstimator oneAxis;
    for (int i = 0; i < 2000; i++) {
        oneAxis.add(motionWith(mount, Eigen::Vector3d(0.0, 1.0, 0.0), k_TURN_NOISE, k_MOVE_NOISE));
    }
    CHECK(!oneAxis.ready());
    // nodding and shaking the head, as a person does, is two axes and enough
    MountEstimator twoAxes;
    for (int i = 0; i < 3000; i++) {
        twoAxes.add(motionWith(mount, i % 2 ? Eigen::Vector3d(0.0, 1.0, 0.0) : Eigen::Vector3d(1.0, 0.0, 0.0), k_TURN_NOISE, k_MOVE_NOISE));
    }
    CHECK(twoAxes.ready() && degreesOff(twoAxes, mount) < 0.05 && millimetresOff(twoAxes, mount) < 0.5);

    // the oldest motions fade once the sums are full: the weight stops growing
    MountEstimator::Options small;
    small.maxWeight = 1000.0;
    MountEstimator fading(small);
    for (int i = 0; i < 3000; i++) {
        fading.add(motionWith(mount, everyAxis, k_TURN_NOISE, k_MOVE_NOISE));
    }
    CHECK(fading.weight() <= 1000.0 + 1e-9 && fading.weight() > 990.0 && degreesOff(fading, mount) < 0.05);

    if (g_failures) {
        std::printf("%d checks failed\n", g_failures);
        return 1;
    }
    std::printf("the mount estimator holds\n");
    return 0;
}
