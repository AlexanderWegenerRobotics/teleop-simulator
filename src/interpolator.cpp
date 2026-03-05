#include "interpolator.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

Interpolator::Interpolator(const InterpolatorConfig& config)
    : config_(config)
    , n_waypoints_(config.control_freq / config.comm_freq)
    , current_idx_(0)
    , space_(InterpolationSpace::JOINT)
{
    joint_waypoints_.resize(n_waypoints_, Eigen::VectorXd::Zero(config.n_dof));
    cartesian_waypoints_.resize(n_waypoints_, Eigen::Isometry3d::Identity());
}

double Interpolator::trapezoidalProfile(double t, double duration) const {
    constexpr double ramp_fraction = 0.2;
    double t_ramp = duration * ramp_fraction;

    if (t <= 0.0)           return 0.0;
    if (t >= duration)      return 1.0;

    double pos = 0.0;
    double v_max = 1.0 / (duration * (1.0 - ramp_fraction));

    if (t < t_ramp) {
        pos = 0.5 * (v_max / t_ramp) * t * t;
    } else if (t < duration - t_ramp) {
        pos = 0.5 * v_max * t_ramp + v_max * (t - t_ramp);
    } else {
        double dt = t - (duration - t_ramp);
        pos = 1.0 - 0.5 * (v_max / t_ramp) * (t_ramp - dt) * (t_ramp - dt);
    }

    return std::clamp(pos, 0.0, 1.0);
}

double Interpolator::linearProfile(double t, double duration) const{
    if (t <= 0.0)           return 0.0;
    if (t >= duration)      return 1.0;
    double pos = 0.0;
    return std::clamp(pos, 0.0, 1.0);
}

double Interpolator::computeCartesianDuration(const Eigen::Isometry3d& T_start, const Eigen::Isometry3d& T_end) const {
    double linear_dist  = (T_end.translation() - T_start.translation()).norm();
    Eigen::AngleAxisd aa(T_start.rotation().transpose() * T_end.rotation());
    double angular_dist = std::abs(aa.angle());

    double t_linear  = linear_dist  / config_.max_linear_vel;
    double t_angular = angular_dist / config_.max_angular_vel;

    double min_duration = static_cast<double>(n_waypoints_) / config_.control_freq;
    return std::max({t_linear, t_angular, min_duration});
}

double Interpolator::computeJointDuration(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end) const {
    double max_displacement = (q_end - q_start).cwiseAbs().maxCoeff();
    double min_duration     = static_cast<double>(n_waypoints_) / config_.control_freq;
    return std::max(max_displacement / config_.max_angular_vel, min_duration);
}

void Interpolator::planJoint(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end) {
    if (q_start.size() != config_.n_dof || q_end.size() != config_.n_dof)
        throw std::invalid_argument("Joint vector size mismatch");

    space_ = InterpolationSpace::JOINT;

    double duration = computeJointDuration(q_start, q_end);
    double dt       = duration / n_waypoints_;

    for (int i = 0; i < n_waypoints_; ++i) {
        double t   = i * dt;
        double s   = trapezoidalProfile(t, duration);
        joint_waypoints_[i] = q_start + s * (q_end - q_start);
    }

    current_idx_ = 0;
}

void Interpolator::planCartesian(const Eigen::Isometry3d& T_start,
                                  const Eigen::Isometry3d& T_end) {
    space_ = InterpolationSpace::CARTESIAN;

    double duration = computeCartesianDuration(T_start, T_end);
    double dt       = duration / n_waypoints_;

    Eigen::Quaterniond q_start(T_start.rotation());
    Eigen::Quaterniond q_end(T_end.rotation());

    for (int i = 0; i < n_waypoints_; ++i) {
        double t = i * dt;
        double s = trapezoidalProfile(t, duration);

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.translation() = T_start.translation() + s * (T_end.translation() - T_start.translation());
        T.linear()      = q_start.slerp(s, q_end).toRotationMatrix();

        cartesian_waypoints_[i] = T;
    }

    current_idx_ = 0;
}

Eigen::VectorXd Interpolator::getCurrentJoint() const {
    return joint_waypoints_[std::min(current_idx_, n_waypoints_ - 1)];
}

Eigen::Isometry3d Interpolator::getCurrentCartesian() const {
    return cartesian_waypoints_[std::min(current_idx_, n_waypoints_ - 1)];
}

bool Interpolator::step() {
    if (current_idx_ < n_waypoints_ - 1) {
        ++current_idx_;
        return true;
    }
    return false;
}

bool Interpolator::isDone() const {
    return current_idx_ >= n_waypoints_ - 1;
}

void Interpolator::reset() {
    current_idx_ = 0;
}