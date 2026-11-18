#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <vector>

namespace bench {

constexpr double k_PI = 3.14159265358979323846;
constexpr double k_DEG = 180.0 / k_PI;

// a rigid pose: x_world = q * x_local + p
struct Pose {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d p = Eigen::Vector3d::Zero();

    [[nodiscard]] Pose operator*(const Pose& o) const { return Pose { (q * o.q).normalized(), p + q * o.p }; }
    [[nodiscard]] Eigen::Vector3d operator*(const Eigen::Vector3d& x) const { return p + q * x; }
    [[nodiscard]] Pose inverse() const
    {
        const Eigen::Quaterniond qi = q.conjugate();
        return Pose { qi, -(qi * p) };
    }
};

// the rotation vector (axis * angle, angle in [0, pi]) of a rotation
inline Eigen::Vector3d rotationVector(const Eigen::Quaterniond& rotation)
{
    Eigen::Quaterniond q = rotation.w() < 0.0 ? Eigen::Quaterniond(-rotation.coeffs()) : rotation;
    const double sine = q.vec().norm();
    if (sine < 1e-12) {
        return 2.0 * q.vec();
    }
    return q.vec() * (2.0 * std::atan2(sine, q.w()) / sine);
}

inline Eigen::Quaterniond fromRotationVector(const Eigen::Vector3d& v)
{
    const double angle = v.norm();
    if (angle < 1e-12) {
        return Eigen::Quaterniond(1.0, 0.5 * v.x(), 0.5 * v.y(), 0.5 * v.z()).normalized();
    }
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, v / angle));
}

inline double angleBetween(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b)
{
    return rotationVector(a.conjugate() * b).norm();
}

// between two poses, a fraction of the way
inline Pose interpolate(const Pose& a, const Pose& b, double fraction)
{
    return Pose { a.q.slerp(fraction, b.q).normalized(), a.p + (b.p - a.p) * fraction };
}

// the mean of rotations that are close to each other
inline Eigen::Quaterniond meanRotation(const std::vector<Eigen::Quaterniond>& rotations)
{
    Eigen::Vector4d sum = Eigen::Vector4d::Zero();
    for (const Eigen::Quaterniond& q : rotations) {
        sum += (q.coeffs().dot(rotations.front().coeffs()) < 0.0 ? -1.0 : 1.0) * q.coeffs();
    }
    Eigen::Quaterniond mean;
    mean.coeffs() = sum.normalized();
    return mean;
}

inline double percentile(std::vector<double> values, double fraction)
{
    if (values.empty()) {
        return NAN;
    }
    const size_t index = std::min(values.size() - 1, static_cast<size_t>(fraction * static_cast<double>(values.size() - 1) + 0.5));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index), values.end());
    return values[index];
}

inline double median(const std::vector<double>& values)
{
    return percentile(values, 0.5);
}

inline double rms(const std::vector<double>& values)
{
    if (values.empty()) {
        return NAN;
    }
    double sum = 0.0;
    for (double v : values) {
        sum += v * v;
    }
    return std::sqrt(sum / static_cast<double>(values.size()));
}

// yaw, pitch, roll in degrees as the overlay's log prints a rotation (its canonicalEulerAngles(2, 1, 0), reordered)
inline Eigen::Vector3d logEuler(const Eigen::Quaterniond& q)
{
    const Eigen::Vector3d euler = q.toRotationMatrix().canonicalEulerAngles(2, 1, 0) * k_DEG;
    return Eigen::Vector3d(euler[1], euler[2], euler[0]);
}

} // namespace bench
