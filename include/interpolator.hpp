#pragma once

#include <vector>
#include <cstddef>
#include <Eigen/Dense>
#include <Eigen/Geometry>

enum class InterpolationSpace {
    JOINT,
    CARTESIAN
};

struct InterpolatorConfig {
    int    control_freq;
    int    comm_freq;
    int    n_dof;
    double max_linear_vel;
    double max_angular_vel;
};

class Interpolator {
public:
    explicit Interpolator(const InterpolatorConfig& config);

    void planJoint(const Eigen::VectorXd& q_start,
                   const Eigen::VectorXd& q_end);

    void planCartesian(const Eigen::Isometry3d& T_start,
                       const Eigen::Isometry3d& T_end);

    Eigen::VectorXd     getCurrentJoint()     const;
    Eigen::Isometry3d   getCurrentCartesian() const;

    bool step();
    bool isDone() const;
    void reset();

private:
    double computeCartesianDuration(const Eigen::Isometry3d& T_start,
                                     const Eigen::Isometry3d& T_end) const;

    double computeJointDuration(const Eigen::VectorXd& q_start,
                                 const Eigen::VectorXd& q_end) const;

    double trapezoidalProfile(double t, double duration) const;

private:
    InterpolatorConfig config_;
    int                n_waypoints_;
    int                current_idx_;
    InterpolationSpace space_;

    std::vector<Eigen::VectorXd>   joint_waypoints_;
    std::vector<Eigen::Isometry3d> cartesian_waypoints_;
};