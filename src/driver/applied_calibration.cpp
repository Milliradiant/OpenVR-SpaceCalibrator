#include "applied_calibration.h"

#include <algorithm>
#include <cmath>

namespace spacecal {

namespace {

    constexpr double k_DEGREES = EIGEN_PI / 180.0;

    Eigen::Quaterniond quaternionOf(const vr::HmdQuaternion_t& quat)
    {
        return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
    }
    Eigen::Vector3d vectorOf(const double vector[3])
    {
        return Eigen::Vector3d(vector[0], vector[1], vector[2]);
    }
    bool isPoseValid(const vr::DriverPose_t& pose)
    {
        return pose.deviceIsConnected && pose.poseIsValid && pose.result == vr::ETrackingResult::TrackingResult_Running_OK;
    }
    Eigen::Affine3d frameOf(const vr::DriverPose_t& pose)
    {
        return Eigen::Translation3d(vectorOf(pose.vecWorldFromDriverTranslation)) * quaternionOf(pose.qWorldFromDriverRotation).normalized();
    }
    Eigen::Affine3d affineOf(const Pose_t& pose)
    {
        return Eigen::Translation3d(pose.pos) * pose.rot;
    }
    Pose_t poseOf(const Eigen::Affine3d& transform)
    {
        // rigid, so its linear part is its rotation
        return Pose_t { .rot = Eigen::Quaterniond(transform.linear()).normalized(), .pos = transform.translation() };
    }
    double angleOf(const Eigen::Quaterniond& rotation)
    {
        return 2.0 * std::atan2(rotation.vec().norm(), std::abs(rotation.w()));
    }
    bool sameFrame(const Eigen::Affine3d& one, const Eigen::Affine3d& other)
    {
        return (one.translation() - other.translation()).norm() < 1e-7 && angleOf(Eigen::Quaterniond(one.linear() * other.linear().transpose())) < 1e-7;
    }

    // a calibration moved a fraction of the way to another, turning about a point that goes in a straight line
    Pose_t towards(const Pose_t& from, const Pose_t& to, double fraction, const Eigen::Vector3d& pivot)
    {
        const Eigen::Vector3d fromPoint = from.pos + from.rot * pivot;
        const Eigen::Vector3d toPoint = to.pos + to.rot * pivot;
        const Eigen::Quaterniond rot = from.rot.slerp(fraction, to.rot).normalized();
        return Pose_t { .rot = rot, .pos = fromPoint + (toPoint - fromPoint) * fraction - rot * pivot };
    }

    double fractionOf(double rate, double seconds)
    {
        return 1.0 - std::exp(-rate * seconds);
    }

} // namespace

Eigen::Affine3d getWorldFromDriverPose(const vr::DriverPose_t& pose)
{
    const Eigen::Quaterniond worldFromDriverRot = quaternionOf(pose.qWorldFromDriverRotation);
    const Eigen::Vector3d worldFromDriverPos = vectorOf(pose.vecWorldFromDriverTranslation);

    const Eigen::Quaterniond rot = worldFromDriverRot * quaternionOf(pose.qRotation);
    const Eigen::Vector3d pos = worldFromDriverPos + worldFromDriverRot * vectorOf(pose.vecPosition);

    return Eigen::Translation3d(pos) * rot;
}

// how far apart two calibrations put the target device, and the feet below it and half a metre to each side of them (a turn about
// the vertical moves nothing right under the device): the most of the six
double AppliedCalibration::difference(const Pose_t& one, const Pose_t& other, const Eigen::Vector3d& at) const
{
    const Eigen::Vector3d feet = at - Eigen::Vector3d(0.0, m_settings.feetBelowMeters, 0.0);
    const Eigen::Vector3d points[] = { at, feet, feet + Eigen::Vector3d(0.5, 0.0, 0.0), feet - Eigen::Vector3d(0.5, 0.0, 0.0), feet + Eigen::Vector3d(0.0, 0.0, 0.5), feet - Eigen::Vector3d(0.0, 0.0, 0.5) };
    double most = 0.0;
    for (const Eigen::Vector3d& point : points) {
        most = std::max(most, ((one.pos + one.rot * point) - (other.pos + other.rot * point)).norm());
    }
    return most;
}

// whether the two devices' motions from `before` to `now` agree: fixed to each other they turn and move together, whatever the
// calibration is, H_a^-1 H_b = L (T_a^-1 T_b) L^-1. Their poses are of moments some 10 ms apart and the headset's is predicted, so
// a fast turn's or a jolt's own disagreement grows with it. `turnRate` is the faster of the two's, degrees a second
AppliedCalibration::Agreement AppliedCalibration::agreement(const Pair& before, const Pair& now, const Pose_t& mount, double& turnRate, bool& moving) const
{
    const double span = now.time - before.time;
    const Eigen::Quaterniond referenceTurn(Eigen::Quaterniond(before.reference.linear()).conjugate() * Eigen::Quaterniond(now.reference.linear()));
    const Eigen::Quaterniond targetTurn(Eigen::Quaterniond(before.target.linear()).conjugate() * Eigen::Quaterniond(now.target.linear()));
    const Eigen::Quaterniond targetTurnAsReference = mount.rot * targetTurn * mount.rot.conjugate();

    const double referenceRate = angleOf(referenceTurn) / span / k_DEGREES;
    const double targetRate = angleOf(targetTurn) / span / k_DEGREES;
    const double turnDisagreement = angleOf(referenceTurn.conjugate() * targetTurnAsReference) / span / k_DEGREES;
    turnRate = std::max(referenceRate, targetRate);

    // the target's place on the reference, in the reference's world, against the target in its own, through the calibration
    const Eigen::Vector3d referenceMove = (now.reference * mount.pos - before.reference * mount.pos) / span;
    const Eigen::Vector3d targetMove = m_applied.rot * (now.target.translation() - before.target.translation()) / span;
    const double moveDisagreement = (referenceMove - targetMove).norm();

    moving = turnRate > m_settings.movingTurnDeg || std::max(referenceMove.norm(), targetMove.norm()) > m_settings.movingMeters;

    const bool referenceStopped = referenceRate < m_settings.stoppedTurnDeg && referenceMove.norm() < m_settings.stoppedMoveMeters;
    const bool targetStopped = targetRate < m_settings.stoppedTurnDeg && targetMove.norm() < m_settings.stoppedMoveMeters;
    const double turnShare = turnDisagreement / (m_settings.disagreeTurnDeg + m_settings.disagreeTurnFraction * std::min(referenceRate, targetRate));
    const double moveShare = moveDisagreement / (m_settings.disagreeMoveMeters + m_settings.disagreeMoveFraction * std::min(referenceMove.norm(), targetMove.norm()));
    const double share = std::max(turnShare, moveShare);
    if ((referenceStopped && targetRate > m_settings.otherTurningDeg) || (targetStopped && referenceRate > m_settings.otherTurningDeg) || share > m_settings.brokenFactor) {
        return Agreement::Broken;
    }
    return share > 1.0 ? Agreement::Disagree : Agreement::Agree;
}

void AppliedCalibration::begin(double now, const Pose_t& calibration)
{
    m_hasCalibration = true;
    m_applied = calibration;
    m_following = calibration;
    m_time = now;
    m_beganTime = now;
    m_catchingUp = true;
    m_holding = false;
    m_agreeingSeconds = 0.0;
    m_differingSeconds = 0.0;
    m_meanDifference = 0.0;
}

// A step of either tracking system's world: the target's world-from-driver (and so every device of its system) or the
// reference's. The devices did not move, so the calibration takes the step back at once: C' = G' G^-1 C F F'^-1
void AppliedCalibration::followFrames(const vr::DriverPose_t& target, const vr::DriverPose_t* reference)
{
    const Eigen::Affine3d targetFrame = frameOf(target);
    const Eigen::Affine3d referenceFrame = reference ? frameOf(*reference) : m_frames.reference;
    if (m_frames.known && m_hasCalibration) {
        const bool targetStepped = !sameFrame(targetFrame, m_frames.target);
        const bool referenceStepped = reference && !sameFrame(referenceFrame, m_frames.reference);
        if (targetStepped || referenceStepped) {
            const Eigen::Affine3d before = referenceStepped ? Eigen::Affine3d(referenceFrame * m_frames.reference.inverse()) : Eigen::Affine3d::Identity();
            const Eigen::Affine3d after = targetStepped ? Eigen::Affine3d(m_frames.target * targetFrame.inverse()) : Eigen::Affine3d::Identity();
            for (Pose_t* calibration : { &m_applied, &m_following, &m_world }) {
                *calibration = poseOf(before * affineOf(*calibration) * after);
            }
            if (targetStepped) {
                m_pivot = targetFrame * m_frames.target.inverse() * m_pivot;
            }
            // a motion across the step is no motion of the devices
            m_pairCount = 0;
        }
    }
    m_frames.known = true;
    m_frames.target = targetFrame;
    m_frames.reference = referenceFrame;
}

void AppliedCalibration::addPair(double now, const vr::DriverPose_t& reference, const vr::DriverPose_t& target, const Pose_t& mount)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!isPoseValid(reference) || !isPoseValid(target)) {
        return;
    }
    if (m_relative) {
        followFrames(target, &reference);
    }
    // the reference's pose is the slower of the two: a pair is taken once for each new one
    const Eigen::Vector3d referencePosition = vectorOf(reference.vecPosition);
    const Eigen::Quaterniond referenceRotation = quaternionOf(reference.qRotation);
    if (m_relative && referencePosition == m_lastReferencePosition && referenceRotation.coeffs() == m_lastReferenceRotation.coeffs()) {
        return;
    }
    m_lastReferencePosition = referencePosition;
    m_lastReferenceRotation = referenceRotation;

    if (!m_relative) {
        // from a world calibration, or from none: what is applied goes on from where it is
        m_relative = true;
        m_pairCount = 0;
        m_holding = false;
        m_agreeingSeconds = 0.0;
        m_differingSeconds = 0.0;
        m_meanDifference = -1.0;
        m_catchingUp = false;
        followFrames(target, &reference);
    }

    const Pair pair { .time = now, .reference = getWorldFromDriverPose(reference), .target = getWorldFromDriverPose(target) };
    const Pose_t measured = poseOf(pair.reference * affineOf(mount) * pair.target.inverse());
    m_pivot = pair.target.translation();

    if (m_meanDifference < 0.0) {
        m_meanDifference = difference(m_applied, measured, m_pivot); // the first pair since a world calibration: from it
    }
    if (!m_hasCalibration) {
        begin(now, measured);
        m_pairs[m_pairNext] = pair;
        m_pairNext = (m_pairNext + 1) % k_PAIRS;
        m_pairCount = 1;
        return;
    }
    const double seconds = std::clamp(now - m_time, 0.0, 0.1);
    m_time = now;

    // the two devices' motions since the newest pair at least the baseline before this one
    const Pair* before = nullptr;
    for (size_t i = 1; i <= m_pairCount; i++) {
        const Pair& candidate = m_pairs[(m_pairNext + k_PAIRS - i) % k_PAIRS];
        if (now - candidate.time >= m_settings.motionBaselineSeconds) {
            before = &candidate;
            break;
        }
    }
    double weight = 1.0;
    if (before) {
        double turnRate = 0.0;
        bool moving = false;
        const Agreement agreed = agreement(*before, pair, mount, turnRate, moving);
        if (agreed == Agreement::Broken) {
            m_holding = true;
            m_agreeingSeconds = 0.0;
        } else if (m_holding && agreed == Agreement::Agree && moving) {
            m_agreeingSeconds += seconds;
            m_holding = m_agreeingSeconds < m_settings.agreeingMotionSeconds;
        }
        weight = agreed == Agreement::Agree ? 1.0 / (1.0 + std::pow(turnRate / m_settings.turnRateScaleDeg, 2.0)) : 0.0;
    }
    if (m_holding) {
        weight = 0.0;
    }
    m_pairs[m_pairNext] = pair;
    m_pairNext = (m_pairNext + 1) % k_PAIRS;
    m_pairCount = std::min(m_pairCount + 1, k_PAIRS);

    // how far the pairs taken with the head nearly still are from what is applied, half a second's worth of them: a fast turn's
    // pairs are off by centimetres for as long as it lasts, which is no reason to catch up with anything
    if (weight >= m_settings.catchUpEvidenceWeight) {
        const double difference = this->difference(m_applied, measured, m_pivot);
        m_meanDifference += (difference - m_meanDifference) * fractionOf(1.0 / m_settings.catchUpEvidenceSeconds, seconds);
        m_differingSeconds = m_meanDifference > m_settings.catchUpMeters ? m_differingSeconds + seconds : 0.0;
    }
    if (m_differingSeconds >= m_settings.catchUpAfterSeconds) {
        m_catchingUp = true;
    }
    if (m_catchingUp && m_meanDifference < m_settings.caughtUpMeters && now - m_beganTime >= m_settings.startSeconds) {
        m_catchingUp = false;
        m_differingSeconds = 0.0;
    }

    // catching up, the difference is well over what a fast turn makes a pair off by, and even those pairs take it there
    const double rate = m_catchingUp ? m_settings.catchUpRate : 1.0 / m_settings.followSeconds;
    const double counted = m_catchingUp && !m_holding ? std::max(weight, m_settings.catchUpLeastWeight) : weight;
    m_following = towards(m_following, measured, fractionOf(rate * counted, seconds), m_pivot);
    m_applied = towards(m_applied, m_following, fractionOf(rate, seconds), m_pivot);
}

void AppliedCalibration::setWorld(double now, const Pose_t& calibration, bool glide)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_relative) {
        // from a relative calibration: what is applied goes on from where it is
        m_relative = false;
    }
    m_world = calibration;
    m_worldGlide = glide;
    if (!m_hasCalibration || !glide) {
        m_hasCalibration = true;
        m_applied = calibration;
        m_following = calibration;
        m_time = now;
        m_beganTime = now;
        m_catchingUp = false;
        return;
    }
    // one well away from what is applied (a batch's first calibration, the overlay's recovery from a lost world) is gone to in a
    // second or two
    if (!m_catchingUp && difference(m_applied, calibration, m_pivot) > m_settings.catchUpMeters) {
        m_catchingUp = true;
    }
}

void AppliedCalibration::watchFrames(const vr::DriverPose_t& target, const vr::DriverPose_t* reference)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_relative) {
        return; // addPair has the frames
    }
    if (isPoseValid(target)) {
        followFrames(target, reference && isPoseValid(*reference) ? reference : nullptr);
        m_pivot = getWorldFromDriverPose(target).translation();
    }
}

void AppliedCalibration::advanceWorld(double now)
{
    const double seconds = std::clamp(now - m_time, 0.0, 0.1);
    m_time = std::max(m_time, now);
    if (!m_worldGlide || seconds <= 0.0) {
        return;
    }
    const double fraction = fractionOf(m_catchingUp ? m_settings.catchUpRate : 1.0 / m_settings.followSeconds, seconds);
    m_following = towards(m_following, m_world, fraction, m_pivot);
    m_applied = towards(m_applied, m_following, fraction, m_pivot);
    if (m_catchingUp && difference(m_applied, m_world, m_pivot) < m_settings.caughtUpMeters) {
        m_catchingUp = false;
    }
}

bool AppliedCalibration::current(double now, Pose_t& calibration)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_hasCalibration) {
        return false;
    }
    if (!m_relative) {
        advanceWorld(now);
    }
    calibration = m_applied;
    return true;
}

void AppliedCalibration::reset()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_hasCalibration = false;
    m_relative = false;
    m_worldGlide = false;
    m_frames = {};
    m_pairCount = 0;
    m_holding = false;
    m_catchingUp = false;
}

} // namespace spacecal
