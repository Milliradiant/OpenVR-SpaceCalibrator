#include "mount_estimator.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace spacecal {

namespace {

    Eigen::Vector3d rotationVector(const Eigen::Quaterniond& rotation)
    {
        Eigen::Quaterniond q = rotation.w() < 0.0 ? Eigen::Quaterniond(-rotation.coeffs()) : rotation;
        const double sine = q.vec().norm();
        if (sine < 1e-12) {
            return 2.0 * q.vec();
        }
        return q.vec() * (2.0 * std::atan2(sine, q.w()) / sine);
    }

    Eigen::Matrix3d rotationOf(const Eigen::Vector3d& turn)
    {
        const double angle = turn.norm();
        if (angle < 1e-12) {
            return Eigen::Matrix3d::Identity();
        }
        return Eigen::AngleAxisd(angle, turn / angle).toRotationMatrix();
    }

} // namespace

void motionBetween(const Eigen::Quaterniond& rotationA, const Eigen::Vector3d& positionA, const Eigen::Quaterniond& rotationB, const Eigen::Vector3d& positionB, Eigen::Vector3d& turn, Eigen::Vector3d& move)
{
    const Eigen::Quaterniond inverseA = rotationA.conjugate();
    turn = rotationVector((inverseA * rotationB).normalized());
    move = inverseA * (positionB - positionA);
}

void MountEstimator::reset()
{
    m_sums.clear();
    m_block.clear();
    m_lastBlock.clear();
    m_lastBlockDisagreed = false;
    m_fromBlocks = false;
    m_rejectedShare = 0.0;
}

bool MountEstimator::Sums::fits(const MountMotion& motion, const Eigen::Matrix3d& m, const Options& options, double& turnResidual2, double& moveResidual2) const
{
    turnResidual2 = 0.0;
    moveResidual2 = 0.0;
    if (!ready) {
        return true;
    }
    // once there is an estimate, the target's turn seen through it is the reference's, and the moves' equation holds:
    // (R_A - I) t = R t_B - t_A
    turnResidual2 = (motion.referenceTurn - rotation * motion.targetTurn).squaredNorm();
    moveResidual2 = (m * translation - (rotation * motion.targetMove - motion.referenceMove)).squaredNorm();
    const double turnGate = std::max(options.minTurnGate, options.gate * std::sqrt(this->turnResidual2));
    const double moveGate = std::max(options.minMoveGate, options.gate * std::sqrt(this->moveResidual2));
    return turnResidual2 <= turnGate * turnGate && moveResidual2 <= moveGate * moveGate;
}

void MountEstimator::Sums::add(const MountMotion& motion, const Eigen::Matrix3d& m, double motionTurnResidual2, double motionMoveResidual2, double maxWeight)
{
    weight += 1.0;
    turns += motion.referenceTurn * motion.targetTurn.transpose();
    const Eigen::Matrix3d mt = m.transpose();
    moveNormal += mt * m;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                moveByRotation[i][j][k] += mt(i, j) * motion.targetMove(k);
            }
        }
    }
    moveConstant += mt * motion.referenceMove;

    if (ready) {
        const double share = 1.0 / std::min(weight, 2000.0);
        turnResidual2 += share * (motionTurnResidual2 - turnResidual2);
        moveResidual2 += share * (motionMoveResidual2 - moveResidual2);
    }

    // past what the sums may hold, everything fades a little with each new motion: the oldest count for less and less
    if (weight > maxWeight) {
        const double keep = maxWeight / weight;
        weight *= keep;
        turns *= keep;
        moveNormal *= keep;
        moveConstant *= keep;
        for (auto& plane : moveByRotation) {
            for (auto& row : plane) {
                for (double& value : row) {
                    value *= keep;
                }
            }
        }
    }
}

void MountEstimator::Sums::addSums(const Sums& other)
{
    const double before = weight;
    weight += other.weight;
    turns += other.turns;
    moveNormal += other.moveNormal;
    moveConstant += other.moveConstant;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                moveByRotation[i][j][k] += other.moveByRotation[i][j][k];
            }
        }
    }
    if (weight > 0.0) {
        turnResidual2 = (before * turnResidual2 + other.weight * other.turnResidual2) / weight;
        moveResidual2 = (before * moveResidual2 + other.weight * other.moveResidual2) / weight;
    }
}

void MountEstimator::Sums::solve(const Options& options)
{
    const bool wasReady = ready;
    ready = false;
    if (weight < options.minWeight) {
        return;
    }

    // the rotation that best takes the target's turns onto the reference's (Wahba's problem)
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(turns, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector3d singular = svd.singularValues();
    if (singular(0) <= 0.0 || singular(1) / singular(0) < options.minTurnCondition) {
        return;
    }
    Eigen::Matrix3d flip = Eigen::Matrix3d::Identity();
    flip(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant() < 0.0 ? -1.0 : 1.0;
    const Eigen::Matrix3d solved = svd.matrixU() * flip * svd.matrixV().transpose();

    // and with it the translation: the sum of M^T M times t is the sum of M^T (R t_B - t_A)
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(moveNormal);
    if (eigen.eigenvalues()(2) <= 0.0 || eigen.eigenvalues()(0) / eigen.eigenvalues()(2) < options.minMoveCondition) {
        return;
    }
    Eigen::Vector3d right = -moveConstant;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                right(i) += moveByRotation[i][j][k] * solved(j, k);
            }
        }
    }

    rotation = Eigen::Quaterniond(solved).normalized();
    translation = moveNormal.ldlt().solve(right);
    ready = true;
    if (!wasReady && turnResidual2 == 0.0) {
        // nothing measured yet of how far off motions typically are: start from what the gates never refuse
        turnResidual2 = options.minTurnGate * options.minTurnGate;
        moveResidual2 = options.minMoveGate * options.minMoveGate;
    }
}

MountEstimator::Added MountEstimator::add(const MountMotion& motion)
{
    const double referenceAngle = motion.referenceTurn.norm();
    const double targetAngle = motion.targetTurn.norm();
    if (referenceAngle < m_options.minTurn || targetAngle < m_options.minTurn) {
        return Added::TooLittleTurn;
    }

    const Eigen::Matrix3d m = rotationOf(motion.referenceTurn) - Eigen::Matrix3d::Identity();

    // two devices fixed to each other turn by the same angle, whatever the mount
    const bool turnsAlike = std::abs(referenceAngle - targetAngle) <= std::max(0.0175, m_options.maxTurnMismatch * std::max(referenceAngle, targetAngle));
    double turnResidual2 = 0.0, moveResidual2 = 0.0;
    bool fits = turnsAlike && m_sums.fits(motion, m, m_options, turnResidual2, moveResidual2);

    if (m_sums.ready) {
        // the mount moved (the tracker was taken off and put back, or bumped): hardly any motion fits the old one any more
        m_rejectedShare += ((fits ? 0.0 : 1.0) - m_rejectedShare) / m_options.remountWindow;
        if (m_rejectedShare >= m_options.remountRejected) {
            const size_t startedOver = m_startedOver + 1;
            reset();
            m_startedOver = startedOver;
            return Added::StartedOver;
        }
    }

    // the latest motions on their own, judged by their own estimate rather than by the one they may be about to overturn
    double blockTurnResidual2 = 0.0, blockMoveResidual2 = 0.0;
    if (turnsAlike && m_block.fits(motion, m, m_options, blockTurnResidual2, blockMoveResidual2)) {
        m_block.add(motion, m, blockTurnResidual2, blockMoveResidual2, m_options.blockWeight);
        m_block.solve(m_options);
        if (m_block.weight >= m_options.blockWeight) {
            const auto apart = [&](const Sums& a, const Sums& b) {
                return a.rotation.angularDistance(b.rotation) > m_options.remountAngle || (a.translation - b.translation).norm() > m_options.remountDistance;
            };
            const bool disagrees = m_block.ready && m_sums.ready && apart(m_block, m_sums);
            if (disagrees && m_lastBlockDisagreed && !apart(m_block, m_lastBlock)) {
                // two blocks in a row say the same mount, and not the estimate's: the target sits differently now
                Sums latest = m_lastBlock;
                latest.addSums(m_block);
                latest.solve(m_options);
                if (latest.ready) {
                    const size_t startedOver = m_startedOver, remounted = m_remounted + 1;
                    reset();
                    m_startedOver = startedOver;
                    m_remounted = remounted;
                    m_sums = latest;
                    m_fromBlocks = true;
                    return Added::Remounted;
                }
            }
            m_lastBlock = m_block;
            m_lastBlockDisagreed = disagrees;
            m_block.clear();
        }
    }

    if (!fits) {
        return Added::Rejected;
    }
    m_sums.add(motion, m, turnResidual2, moveResidual2, m_options.maxWeight);
    m_sums.solve(m_options);
    return Added::Taken;
}

std::vector<double> MountEstimator::state() const
{
    std::vector<double> out;
    out.reserve(k_STATE_SIZE);
    out.push_back(1.0); // the layout's version
    out.push_back(m_sums.weight);
    for (int i = 0; i < 9; i++) {
        out.push_back(m_sums.turns(i / 3, i % 3));
    }
    for (int i = 0; i < 9; i++) {
        out.push_back(m_sums.moveNormal(i / 3, i % 3));
    }
    for (const auto& plane : m_sums.moveByRotation) {
        for (const auto& row : plane) {
            for (const double value : row) {
                out.push_back(value);
            }
        }
    }
    for (int i = 0; i < 3; i++) {
        out.push_back(m_sums.moveConstant(i));
    }
    out.push_back(m_sums.turnResidual2);
    out.push_back(m_sums.moveResidual2);
    return out;
}

bool MountEstimator::restore(const std::vector<double>& state)
{
    const size_t startedOver = m_startedOver, remounted = m_remounted;
    reset();
    m_startedOver = startedOver;
    m_remounted = remounted;
    if (state.size() != k_STATE_SIZE || state[0] != 1.0) {
        return false;
    }
    for (const double value : state) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    if (state[1] < 0.0 || state[50] < 0.0 || state[51] < 0.0) {
        return false;
    }
    size_t at = 1;
    m_sums.weight = state[at++];
    for (int i = 0; i < 9; i++) {
        m_sums.turns(i / 3, i % 3) = state[at++];
    }
    for (int i = 0; i < 9; i++) {
        m_sums.moveNormal(i / 3, i % 3) = state[at++];
    }
    for (auto& plane : m_sums.moveByRotation) {
        for (auto& row : plane) {
            for (double& value : row) {
                value = state[at++];
            }
        }
    }
    for (int i = 0; i < 3; i++) {
        m_sums.moveConstant(i) = state[at++];
    }
    m_sums.turnResidual2 = state[at++];
    m_sums.moveResidual2 = state[at++];
    m_sums.solve(m_options);
    return true;
}

} // namespace spacecal
