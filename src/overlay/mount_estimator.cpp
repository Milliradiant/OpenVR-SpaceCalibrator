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
    m_weight = 0.0;
    m_turns.setZero();
    m_moveNormal.setZero();
    for (auto& plane : m_moveByRotation) {
        for (auto& row : plane) {
            for (double& value : row) {
                value = 0.0;
            }
        }
    }
    m_moveConstant.setZero();
    m_turnResidual2 = 0.0;
    m_moveResidual2 = 0.0;
    m_ready = false;
    m_rotation = Eigen::Quaterniond::Identity();
    m_translation = Eigen::Vector3d::Zero();
    m_rejectedShare = 0.0;
}

MountEstimator::Added MountEstimator::add(const MountMotion& motion)
{
    const double referenceAngle = motion.referenceTurn.norm();
    const double targetAngle = motion.targetTurn.norm();
    if (referenceAngle < m_options.minTurn || targetAngle < m_options.minTurn) {
        return Added::TooLittleTurn;
    }

    const Eigen::Matrix3d m = rotationOf(motion.referenceTurn) - Eigen::Matrix3d::Identity();

    // two devices fixed to each other turn by the same angle, whatever the mount. and once there is an estimate, the target's turn
    // seen through it is the reference's, and the moves' equation holds: (R_A - I) t = R t_B - t_A
    bool fits = std::abs(referenceAngle - targetAngle) <= std::max(0.0175, m_options.maxTurnMismatch * std::max(referenceAngle, targetAngle));
    double turnResidual2 = 0.0, moveResidual2 = 0.0;
    if (fits && m_ready) {
        turnResidual2 = (motion.referenceTurn - m_rotation * motion.targetTurn).squaredNorm();
        moveResidual2 = (m * m_translation - (m_rotation * motion.targetMove - motion.referenceMove)).squaredNorm();
        const double turnGate = std::max(m_options.minTurnGate, m_options.gate * std::sqrt(m_turnResidual2));
        const double moveGate = std::max(m_options.minMoveGate, m_options.gate * std::sqrt(m_moveResidual2));
        fits = turnResidual2 <= turnGate * turnGate && moveResidual2 <= moveGate * moveGate;
    }

    if (m_ready) {
        // the mount moved (the tracker was taken off and put back, or bumped): hardly any motion fits the old one any more
        m_rejectedShare += ((fits ? 0.0 : 1.0) - m_rejectedShare) / m_options.remountWindow;
        if (m_rejectedShare >= m_options.remountRejected) {
            const size_t startedOver = m_startedOver + 1;
            reset();
            m_startedOver = startedOver;
            return Added::StartedOver;
        }
    }
    if (!fits) {
        return Added::Rejected;
    }

    m_weight += 1.0;
    m_turns += motion.referenceTurn * motion.targetTurn.transpose();
    const Eigen::Matrix3d mt = m.transpose();
    m_moveNormal += mt * m;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                m_moveByRotation[i][j][k] += mt(i, j) * motion.targetMove(k);
            }
        }
    }
    m_moveConstant += mt * motion.referenceMove;

    if (m_ready) {
        const double share = 1.0 / std::min(m_weight, 2000.0);
        m_turnResidual2 += share * (turnResidual2 - m_turnResidual2);
        m_moveResidual2 += share * (moveResidual2 - m_moveResidual2);
    }

    // past what the sums may hold, everything fades a little with each new motion: the oldest count for less and less
    if (m_weight > m_options.maxWeight) {
        const double keep = m_options.maxWeight / m_weight;
        m_weight *= keep;
        m_turns *= keep;
        m_moveNormal *= keep;
        m_moveConstant *= keep;
        for (auto& plane : m_moveByRotation) {
            for (auto& row : plane) {
                for (double& value : row) {
                    value *= keep;
                }
            }
        }
    }

    solve();
    return Added::Taken;
}

void MountEstimator::solve()
{
    const bool wasReady = m_ready;
    m_ready = false;
    if (m_weight < m_options.minWeight) {
        return;
    }

    // the rotation that best takes the target's turns onto the reference's (Wahba's problem)
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(m_turns, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector3d singular = svd.singularValues();
    if (singular(0) <= 0.0 || singular(1) / singular(0) < m_options.minTurnCondition) {
        return;
    }
    Eigen::Matrix3d flip = Eigen::Matrix3d::Identity();
    flip(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant() < 0.0 ? -1.0 : 1.0;
    const Eigen::Matrix3d rotation = svd.matrixU() * flip * svd.matrixV().transpose();

    // and with it the translation: the sum of M^T M times t is the sum of M^T (R t_B - t_A)
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(m_moveNormal);
    if (eigen.eigenvalues()(2) <= 0.0 || eigen.eigenvalues()(0) / eigen.eigenvalues()(2) < m_options.minMoveCondition) {
        return;
    }
    Eigen::Vector3d right = -m_moveConstant;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                right(i) += m_moveByRotation[i][j][k] * rotation(j, k);
            }
        }
    }

    m_rotation = Eigen::Quaterniond(rotation).normalized();
    m_translation = m_moveNormal.ldlt().solve(right);
    m_ready = true;
    if (!wasReady && m_turnResidual2 == 0.0) {
        // nothing measured yet of how far off motions typically are: start from what the gates never refuse
        m_turnResidual2 = m_options.minTurnGate * m_options.minTurnGate;
        m_moveResidual2 = m_options.minMoveGate * m_options.minMoveGate;
    }
}

std::vector<double> MountEstimator::state() const
{
    std::vector<double> out;
    out.reserve(k_STATE_SIZE);
    out.push_back(1.0); // the layout's version
    out.push_back(m_weight);
    for (int i = 0; i < 9; i++) {
        out.push_back(m_turns(i / 3, i % 3));
    }
    for (int i = 0; i < 9; i++) {
        out.push_back(m_moveNormal(i / 3, i % 3));
    }
    for (const auto& plane : m_moveByRotation) {
        for (const auto& row : plane) {
            for (const double value : row) {
                out.push_back(value);
            }
        }
    }
    for (int i = 0; i < 3; i++) {
        out.push_back(m_moveConstant(i));
    }
    out.push_back(m_turnResidual2);
    out.push_back(m_moveResidual2);
    return out;
}

bool MountEstimator::restore(const std::vector<double>& state)
{
    const size_t startedOver = m_startedOver;
    reset();
    m_startedOver = startedOver;
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
    m_weight = state[at++];
    for (int i = 0; i < 9; i++) {
        m_turns(i / 3, i % 3) = state[at++];
    }
    for (int i = 0; i < 9; i++) {
        m_moveNormal(i / 3, i % 3) = state[at++];
    }
    for (auto& plane : m_moveByRotation) {
        for (auto& row : plane) {
            for (double& value : row) {
                value = state[at++];
            }
        }
    }
    for (int i = 0; i < 3; i++) {
        m_moveConstant(i) = state[at++];
    }
    m_turnResidual2 = state[at++];
    m_moveResidual2 = state[at++];
    solve();
    return true;
}

} // namespace spacecal
