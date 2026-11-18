#include "reference.h"

#include <numeric>

namespace bench {

double sampleTime(const Sample& sample, const ReferenceOptions& options)
{
    return options.usePoseTimeOffset ? sample.time + sample.poseTimeOffset : sample.time;
}

namespace {

    // a device's poses by the time each is of, valid ones only, in order
    struct Track {
        std::vector<double> times;
        std::vector<Pose> poses;

        bool at(double time, Pose& pose, double maxGap = 0.1) const
        {
            if (times.size() < 2 || time < times.front() || time > times.back()) {
                return false;
            }
            const size_t upper = static_cast<size_t>(std::lower_bound(times.begin(), times.end(), time) - times.begin());
            if (upper == 0) {
                pose = poses[0];
                return true;
            }
            const double span = times[upper] - times[upper - 1];
            if (span > maxGap) {
                return false;
            }
            pose = interpolate(poses[upper - 1], poses[upper], span > 1e-9 ? (time - times[upper - 1]) / span : 0.0);
            return true;
        }
    };

    Track trackOf(const Device& device, const ReferenceOptions& options)
    {
        Track track;
        track.times.reserve(device.samples.size());
        track.poses.reserve(device.samples.size());
        for (const Sample& sample : device.samples) {
            if (!sample.valid) {
                continue;
            }
            const double time = sampleTime(sample, options);
            // a pose that is not after the one before it (a driver's time offset moving) cannot be placed
            if (!track.times.empty() && time <= track.times.back()) {
                continue;
            }
            track.times.push_back(time);
            track.poses.push_back(sample.world);
        }
        return track;
    }

    constexpr double k_GRID_STEP = 0.005;
    constexpr double k_SPEED_SPAN = 0.02;

    // how fast a device turns, on a uniform grid from the recording's start. NAN where it is not known
    std::vector<double> turningSpeed(const Track& track, double duration)
    {
        std::vector<double> midTimes, speeds;
        size_t j = 0;
        for (size_t i = 0; i < track.times.size(); i++) {
            j = std::max(j, i + 1);
            while (j < track.times.size() && track.times[j] - track.times[i] < k_SPEED_SPAN) {
                j++;
            }
            if (j >= track.times.size()) {
                break;
            }
            const double span = track.times[j] - track.times[i];
            if (span > 0.1) {
                continue;
            }
            midTimes.push_back(0.5 * (track.times[i] + track.times[j]));
            speeds.push_back(angleBetween(track.poses[i].q, track.poses[j].q) / span);
        }

        const size_t count = static_cast<size_t>(duration / k_GRID_STEP) + 1;
        std::vector<double> grid(count, NAN);
        size_t k = 0;
        for (size_t g = 0; g < count; g++) {
            const double time = static_cast<double>(g) * k_GRID_STEP;
            while (k + 1 < midTimes.size() && midTimes[k + 1] < time) {
                k++;
            }
            if (k + 1 >= midTimes.size() || midTimes[k] > time || midTimes[k + 1] - midTimes[k] > 0.1) {
                continue;
            }
            const double fraction = (time - midTimes[k]) / (midTimes[k + 1] - midTimes[k]);
            grid[g] = speeds[k] + fraction * (speeds[k + 1] - speeds[k]);
        }
        return grid;
    }

    // the correlation of the reference's turning speed with the target's `lagSteps` grid steps later, over [from, to)
    double correlationAt(const std::vector<double>& reference, const std::vector<double>& target, long lagSteps, size_t from, size_t to, size_t& used)
    {
        double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
        size_t n = 0;
        for (size_t g = from; g < to; g++) {
            const long h = static_cast<long>(g) + lagSteps;
            if (h < 0 || h >= static_cast<long>(target.size())) {
                continue;
            }
            const double a = reference[g], b = target[static_cast<size_t>(h)];
            if (std::isnan(a) || std::isnan(b)) {
                continue;
            }
            sa += a;
            sb += b;
            saa += a * a;
            sbb += b * b;
            sab += a * b;
            n++;
        }
        used = n;
        if (n < 200) {
            return NAN;
        }
        const double count = static_cast<double>(n);
        const double va = saa - sa * sa / count, vb = sbb - sb * sb / count;
        if (va < 1e-9 || vb < 1e-9) {
            return NAN;
        }
        return (sab - sa * sb / count) / std::sqrt(va * vb);
    }

    // the lag, in seconds, at which the two turning speeds agree best over [from, to), and how well. false if they never moved enough
    bool bestLag(const std::vector<double>& reference, const std::vector<double>& target, double maxLag, size_t from, size_t to, double& lag, double& correlation)
    {
        const long maxSteps = static_cast<long>(maxLag / k_GRID_STEP);
        std::vector<double> scores(static_cast<size_t>(2 * maxSteps + 1), NAN);
        long best = 0;
        double bestScore = -2.0;
        for (long steps = -maxSteps; steps <= maxSteps; steps++) {
            size_t used = 0;
            const double score = correlationAt(reference, target, steps, from, to, used);
            scores[static_cast<size_t>(steps + maxSteps)] = score;
            if (!std::isnan(score) && score > bestScore) {
                bestScore = score;
                best = steps;
            }
        }
        if (bestScore < -1.5) {
            return false;
        }
        // the peak between the grid's steps, by the parabola through it and its neighbours
        double refined = static_cast<double>(best);
        if (best > -maxSteps && best < maxSteps) {
            const double left = scores[static_cast<size_t>(best - 1 + maxSteps)], right = scores[static_cast<size_t>(best + 1 + maxSteps)];
            const double curve = left - 2.0 * bestScore + right;
            if (!std::isnan(left) && !std::isnan(right) && curve < -1e-12) {
                refined += 0.5 * (left - right) / curve;
            }
        }
        lag = refined * k_GRID_STEP;
        correlation = bestScore;
        return true;
    }

    struct MotionPair {
        Eigen::Vector3d referenceTurn, targetTurn; // rotation vectors, each in its own device's frame
        Eigen::Matrix3d referenceRotation;
        Eigen::Vector3d referenceMove, targetMove; // the translation of each device's motion, in its own frame at the first moment
        double weight = 1.0;
    };

    std::vector<MotionPair> motionPairs(const Track& reference, const Track& target, double offset, const ReferenceOptions& options, double from, double to)
    {
        std::vector<MotionPair> pairs;
        for (double a = from; a + options.pairSpacing <= to; a += options.pairStep) {
            const double b = a + options.pairSpacing;
            Pose ha, hb, ta, tb;
            if (!reference.at(a, ha) || !reference.at(b, hb) || !target.at(a + offset, ta) || !target.at(b + offset, tb)) {
                continue;
            }
            const Pose referenceMotion = ha.inverse() * hb;
            const Pose targetMotion = ta.inverse() * tb;
            MotionPair pair;
            pair.referenceTurn = rotationVector(referenceMotion.q);
            pair.targetTurn = rotationVector(targetMotion.q);
            if (pair.referenceTurn.norm() < options.minPairAngle || pair.targetTurn.norm() < options.minPairAngle) {
                continue;
            }
            pair.referenceRotation = referenceMotion.q.toRotationMatrix();
            pair.referenceMove = referenceMotion.p;
            pair.targetMove = targetMotion.p;
            pairs.push_back(pair);
        }
        return pairs;
    }

    MountFit fitMount(std::vector<MotionPair> pairs)
    {
        MountFit fit;
        fit.pairs = pairs.size();
        if (pairs.size() < 50) {
            return fit;
        }

        // rotation: the two devices turn about the same axis, each seeing it in its own frame: axis_ref = R * axis_target. the R that
        // lines the axes up best, with the pairs that disagree most taken out over a few rounds
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        std::vector<double> residuals(pairs.size());
        for (int round = 0; round < 4; round++) {
            Eigen::Matrix3d cross = Eigen::Matrix3d::Zero();
            for (const MotionPair& pair : pairs) {
                cross += pair.weight * std::min(pair.referenceTurn.norm(), pair.targetTurn.norm()) * pair.referenceTurn.normalized() * pair.targetTurn.normalized().transpose();
            }
            const Eigen::JacobiSVD<Eigen::Matrix3d> svd(cross, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3d sign = Eigen::Matrix3d::Identity();
            sign(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant() < 0.0 ? -1.0 : 1.0;
            rotation = svd.matrixU() * sign * svd.matrixV().transpose();

            for (size_t i = 0; i < pairs.size(); i++) {
                const double cosine = pairs[i].referenceTurn.normalized().dot(rotation * pairs[i].targetTurn.normalized());
                residuals[i] = std::acos(std::clamp(cosine, -1.0, 1.0));
            }
            const double limit = std::max(3.0 * median(residuals), 0.5 / k_DEG);
            for (size_t i = 0; i < pairs.size(); i++) {
                pairs[i].weight = residuals[i] <= limit ? 1.0 : 0.0;
            }
        }
        fit.rotationResidualDeg = median(residuals) * k_DEG;

        // translation: (R_ref - I) t = R t_target - t_ref for every pair, the same way
        Eigen::Vector3d translation = Eigen::Vector3d::Zero();
        std::vector<double> translationResiduals(pairs.size());
        std::vector<double> translationWeights(pairs.size(), 1.0);
        Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
        for (int round = 0; round < 4; round++) {
            normal.setZero();
            Eigen::Vector3d right = Eigen::Vector3d::Zero();
            for (size_t i = 0; i < pairs.size(); i++) {
                const double weight = pairs[i].weight * translationWeights[i];
                const Eigen::Matrix3d a = pairs[i].referenceRotation - Eigen::Matrix3d::Identity();
                const Eigen::Vector3d b = rotation * pairs[i].targetMove - pairs[i].referenceMove;
                normal += weight * a.transpose() * a;
                right += weight * a.transpose() * b;
            }
            translation = normal.ldlt().solve(right);
            for (size_t i = 0; i < pairs.size(); i++) {
                const Eigen::Matrix3d a = pairs[i].referenceRotation - Eigen::Matrix3d::Identity();
                const Eigen::Vector3d b = rotation * pairs[i].targetMove - pairs[i].referenceMove;
                translationResiduals[i] = (a * translation - b).norm();
            }
            const double limit = std::max(3.0 * median(translationResiduals), 0.002);
            for (size_t i = 0; i < pairs.size(); i++) {
                translationWeights[i] = translationResiduals[i] <= limit ? 1.0 : 0.0;
            }
        }
        fit.translationResidualMm = median(translationResiduals) * 1e3;
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(normal);
        fit.translationCondition = eigen.eigenvalues()(2) > 0.0 ? eigen.eigenvalues()(0) / eigen.eigenvalues()(2) : 0.0;

        fit.mount = Pose { Eigen::Quaterniond(rotation).normalized(), translation };
        fit.ok = fit.translationCondition > 1e-4;
        return fit;
    }

} // namespace

bool Reference::at(double time, Pose& calibration, Eigen::Vector3d& feet) const
{
    if (times.empty() || time < times.front() || time > times.back()) {
        return false;
    }
    size_t upper = static_cast<size_t>(std::lower_bound(times.begin(), times.end(), time) - times.begin());
    upper = std::min(upper, times.size() - 1);
    const size_t lower = upper == 0 ? 0 : upper - 1;
    const size_t nearest = (time - times[lower]) < (times[upper] - time) ? lower : upper;
    if (std::abs(times[nearest] - time) > 0.5) {
        return false;
    }
    calibration = smooth[nearest];
    feet = targetPositions[nearest] - Eigen::Vector3d(0.0, options.feetBelowTarget, 0.0);
    return true;
}

double Reference::feetDistance(const Pose& one, const Pose& other, const Eigen::Vector3d& feet) const
{
    const double spread = options.feetSpread;
    const Eigen::Vector3d points[5] = {
        feet,
        feet + Eigen::Vector3d(spread, 0.0, 0.0),
        feet - Eigen::Vector3d(spread, 0.0, 0.0),
        feet + Eigen::Vector3d(0.0, 0.0, spread),
        feet - Eigen::Vector3d(0.0, 0.0, spread),
    };
    double worst = 0.0;
    for (const Eigen::Vector3d& point : points) {
        worst = std::max(worst, (one * point - other * point).norm());
    }
    return worst;
}

double Reference::feetError(double time, const Pose& calibration) const
{
    Pose reference;
    Eigen::Vector3d feet;
    if (!at(time, reference, feet)) {
        return -1.0;
    }
    return feetDistance(calibration, reference, feet);
}

double Reference::mountFeetError(const Session& session, double time, const Pose& candidate) const
{
    Pose reference;
    Eigen::Vector3d feet;
    Pose head, tracker;
    if (!at(time, reference, feet) || !session.reference() || !session.target()) {
        return -1.0;
    }
    if (!session.reference()->worldAt(time, head) || !session.target()->worldAt(time + timeOffset, tracker)) {
        return -1.0;
    }
    // the world calibration each mount gives with the two devices where they were: C = H * L * T^-1
    const Pose withCandidate = head * candidate * tracker.inverse();
    const Pose withReference = head * mount.mount * tracker.inverse();
    return feetDistance(withCandidate, withReference, feet);
}

Reference solveReference(const Session& session, const ReferenceOptions& options)
{
    Reference result;
    result.options = options;

    const Device* referenceDevice = session.reference();
    const Device* targetDevice = session.target();
    if (!referenceDevice || !targetDevice) {
        result.error = "the recording does not say which two devices the calibration is between (--reference N --target N names them)";
        return result;
    }

    const Track reference = trackOf(*referenceDevice, options);
    const Track target = trackOf(*targetDevice, options);
    if (reference.times.size() < 200 || target.times.size() < 200) {
        result.error = "too few valid poses of the reference or the target device";
        return result;
    }
    const double duration = std::min(reference.times.back(), target.times.back());

    // the time offset, from how fast each device turns
    const std::vector<double> referenceSpeed = turningSpeed(reference, duration);
    const std::vector<double> targetSpeed = turningSpeed(target, duration);
    if (!bestLag(referenceSpeed, targetSpeed, options.maxLag, 0, referenceSpeed.size(), result.timeOffset, result.timeOffsetCorrelation)) {
        result.error = "the two devices never turned enough together to line their streams up in time";
        return result;
    }
    const size_t windowSteps = static_cast<size_t>(20.0 / k_GRID_STEP);
    for (size_t from = 0; from + windowSteps <= referenceSpeed.size(); from += windowSteps) {
        double lag = 0.0, correlation = 0.0;
        if (bestLag(referenceSpeed, targetSpeed, options.maxLag, from, from + windowSteps, lag, correlation) && correlation > 0.9) {
            result.windowOffsets.push_back(lag);
        }
    }

    // the mount, from the whole recording and from each half
    const double begin = std::max(reference.times.front(), target.times.front());
    result.mount = fitMount(motionPairs(reference, target, result.timeOffset, options, begin, duration));
    if (!result.mount.ok) {
        result.error = "the two devices did not turn about enough different axes to find how one sits on the other";
        return result;
    }
    const double middle = 0.5 * (begin + duration);
    result.firstHalf = fitMount(motionPairs(reference, target, result.timeOffset, options, begin, middle));
    result.secondHalf = fitMount(motionPairs(reference, target, result.timeOffset, options, middle, duration));

    // the world calibration every reference pose gives with that mount, thinned to 50 a second
    double lastTime = -1.0;
    for (size_t i = 0; i < reference.times.size(); i++) {
        if (reference.times[i] - lastTime < 0.02) {
            continue;
        }
        Pose tracker;
        if (!target.at(reference.times[i] + result.timeOffset, tracker)) {
            continue;
        }
        lastTime = reference.times[i];
        result.times.push_back(reference.times[i]);
        result.instant.push_back(reference.poses[i] * result.mount.mount * tracker.inverse());
        result.targetPositions.push_back(tracker.p);
    }
    const size_t count = result.times.size();
    if (count < 100) {
        result.error = "too few moments with both devices tracking";
        return result;
    }

    // running sums, for the means over any stretch
    std::vector<Eigen::Matrix3d> rotationSum(count + 1, Eigen::Matrix3d::Zero());
    std::vector<Eigen::Vector4d> quaternionSum(count + 1, Eigen::Vector4d::Zero());
    std::vector<Eigen::Vector3d> translationSum(count + 1, Eigen::Vector3d::Zero()), positionSum(count + 1, Eigen::Vector3d::Zero());
    const Eigen::Vector4d side = result.instant.front().q.coeffs();
    for (size_t i = 0; i < count; i++) {
        const Eigen::Vector4d q = result.instant[i].q.coeffs();
        rotationSum[i + 1] = rotationSum[i] + result.instant[i].q.toRotationMatrix();
        quaternionSum[i + 1] = quaternionSum[i] + (q.dot(side) < 0.0 ? -q : q);
        translationSum[i + 1] = translationSum[i] + result.instant[i].p;
        positionSum[i + 1] = positionSum[i] + result.targetPositions[i];
    }
    // the mean world calibration over entries [from, to): the mean rotation, and the translation that puts the stretch's mean
    // target position where the entries put it on average (the rotation's noise would otherwise swing the far-off origin about)
    const auto meanOver = [&](size_t from, size_t to) {
        const double n = static_cast<double>(to - from);
        Pose mean;
        mean.q.coeffs() = (quaternionSum[to] - quaternionSum[from]).normalized();
        const Eigen::Vector3d pivot = (positionSum[to] - positionSum[from]) / n;
        const Eigen::Vector3d mapped = ((rotationSum[to] - rotationSum[from]) / n) * pivot + (translationSum[to] - translationSum[from]) / n;
        mean.p = mapped - mean.q * pivot;
        return mean;
    };
    const auto indexAt = [&](double time) {
        return static_cast<size_t>(std::lower_bound(result.times.begin(), result.times.end(), time) - result.times.begin());
    };

    // where the world calibration steps: a gap in the tracking, a device's frame changing, or the 2 s before and after disagreeing
    std::vector<double> boundaries;
    for (size_t i = 1; i < count; i++) {
        if (result.times[i] - result.times[i - 1] > 0.5) {
            boundaries.push_back(result.times[i]);
        }
    }
    for (const Device* device : { referenceDevice, targetDevice }) {
        boundaries.insert(boundaries.end(), device->frameChanges.begin(), device->frameChanges.end());
    }
    {
        double lastJump = -1e9, worst = 0.0, worstTime = 0.0;
        bool inJump = false;
        for (double time = result.times.front() + 2.0; time + 2.0 <= result.times.back(); time += 0.25) {
            const size_t before = indexAt(time - 2.0), here = indexAt(time), after = indexAt(time + 2.0);
            if (here - before < 20 || after - here < 20) {
                continue;
            }
            const Eigen::Vector3d feet = result.targetPositions[here] - Eigen::Vector3d(0.0, options.feetBelowTarget, 0.0);
            const double step = (meanOver(before, here) * feet - meanOver(here, after) * feet).norm();
            if (step > options.jumpThreshold) {
                if (!inJump || step > worst) {
                    worst = step;
                    worstTime = time;
                }
                inJump = true;
            } else if (inJump) {
                inJump = false;
                if (worstTime - lastJump > 2.0) {
                    result.jumps.push_back(worstTime);
                    boundaries.push_back(worstTime);
                    lastJump = worstTime;
                }
                worst = 0.0;
            }
        }
    }
    std::sort(boundaries.begin(), boundaries.end());

    result.segment.resize(count);
    result.smooth.resize(count);
    size_t boundary = 0;
    int segment = 0;
    std::vector<size_t> segmentStart = { 0 };
    for (size_t i = 0; i < count; i++) {
        while (boundary < boundaries.size() && boundaries[boundary] <= result.times[i]) {
            boundary++;
            if (segmentStart.back() != i) {
                segment++;
                segmentStart.push_back(i);
            }
        }
        result.segment[i] = segment;
    }
    segmentStart.push_back(count);

    for (size_t s = 0; s + 1 < segmentStart.size(); s++) {
        const size_t first = segmentStart[s], last = segmentStart[s + 1];
        size_t from = first, to = first;
        for (size_t i = first; i < last; i++) {
            while (result.times[i] - result.times[from] > options.smoothWindow) {
                from++;
            }
            while (to < last && result.times[to] - result.times[i] <= options.smoothWindow) {
                to++;
            }
            result.smooth[i] = meanOver(from, to);
        }
    }

    result.ok = true;
    return result;
}

} // namespace bench
