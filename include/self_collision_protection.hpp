#pragma once

#include <Eigen/Dense>
#include <string>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <algorithm>
#include <cmath>

struct CollisionState {
    Eigen::Vector3d ee_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d ee_velocity = Eigen::Vector3d::Zero();
    double weight = 1.0;
};

struct SelfCollisionConfig {
    double d_min = 0.04;
    double alpha = 5.0;
    double beta = 0.10;
    bool enabled = true;

    double q_radial = 4.0;
    double q_tangential = 1.0;
    double max_velocity = 0.5;

    bool use_uncertainty_padding = false;
    double sigma_gain = 2.0;

    double kf_process_pos = 1e-4;
    double kf_process_vel = 5e-3;
    double kf_meas_pos = 1e-4;
    double kf_meas_vel = 2e-3;
};

struct CorrectionResult {
    Eigen::Vector3d velocity_correction = Eigen::Vector3d::Zero();
    double barrier_value = 1e6;
    bool active = false;
};

class DeviceRegistry {
public:
    void publish(const std::string& name, const CollisionState& state) {
        std::lock_guard<std::mutex> lock(mtx_);
        states_[name] = state;
    }

    std::unordered_map<std::string, CollisionState> snapshot() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return states_;
    }

private:
    mutable std::mutex mtx_;
    std::unordered_map<std::string, CollisionState> states_;
};

class LinearMotionKF {
public:
    LinearMotionKF() {
        x_.setZero();
        P_.setIdentity();
        P_ *= 1e-3;
        initialized_ = false;
    }

    void reset(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity = Eigen::Vector3d::Zero()) {
        x_.head<3>() = position;
        x_.tail<3>() = velocity;
        P_.setIdentity();
        P_ *= 1e-3;
        initialized_ = true;
    }

    bool isInitialized() const {
        return initialized_;
    }

    void predict(double dt, const SelfCollisionConfig& cfg) {
        if (!initialized_) return;

        Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
        F.block<3,3>(0,3) = dt * Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
        Q.block<3,3>(0,0) = cfg.kf_process_pos * Eigen::Matrix3d::Identity();
        Q.block<3,3>(3,3) = cfg.kf_process_vel * Eigen::Matrix3d::Identity();

        x_ = F * x_;
        P_ = F * P_ * F.transpose() + Q;
    }

    void update(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity, const SelfCollisionConfig& cfg) {
        if (!initialized_) {
            reset(position, velocity);
            return;
        }

        Eigen::Matrix<double, 6, 1> z;
        z.head<3>() = position;
        z.tail<3>() = velocity;

        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
        R.block<3,3>(0,0) = cfg.kf_meas_pos * Eigen::Matrix3d::Identity();
        R.block<3,3>(3,3) = cfg.kf_meas_vel * Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 6, 1> y = z - H * x_;
        Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + R;
        Eigen::Matrix<double, 6, 6> K = P_ * H.transpose() * S.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

        x_ = x_ + K * y;

        Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
        P_ = (I - K * H) * P_;
    }

    Eigen::Vector3d position() const {
        return x_.head<3>();
    }

    Eigen::Vector3d velocity() const {
        return x_.tail<3>();
    }

    const Eigen::Matrix<double, 6, 6>& covariance() const {
        return P_;
    }

private:
    Eigen::Matrix<double, 6, 1> x_;
    Eigen::Matrix<double, 6, 6> P_;
    bool initialized_ = false;
};

class SelfCollisionProtection {
public:
    SelfCollisionProtection(const std::string& own_name, std::shared_ptr<DeviceRegistry> registry, const SelfCollisionConfig& config = {})
        : own_name_(own_name)
        , registry_(std::move(registry))
        , config_(config)
    {}

    void publishState(const CollisionState& state) {
        registry_->publish(own_name_, state);
    }

    CorrectionResult computeCorrection(const Eigen::Vector3d& nominal_vel, const CollisionState& own_state, double dt) {
        if (!config_.enabled) return {};

        auto all = registry_->snapshot();

        CorrectionResult worst;
        worst.barrier_value = 1e6;

        for (const auto& [name, other_meas] : all) {
            if (name == own_name_) continue;

            auto& kf = peer_filters_[name];
            if (!kf.isInitialized()) {
                kf.reset(other_meas.ee_position, other_meas.ee_velocity);
            } else {
                kf.predict(dt, config_);
                kf.update(other_meas.ee_position, other_meas.ee_velocity, config_);
            }

            CorrectionResult cr = filterPair(nominal_vel, own_state, other_meas.weight, kf, dt);
            if (cr.barrier_value < worst.barrier_value) {
                worst = cr;
            }
        }
        return worst;
    }

    const SelfCollisionConfig& config() const { return config_; }
    void setConfig(const SelfCollisionConfig& c) { config_ = c; }
    void setEnabled(bool e) { config_.enabled = e; }
    std::shared_ptr<DeviceRegistry> registry() const { return registry_; }

private:
    CorrectionResult filterPair(
        const Eigen::Vector3d& nominal_vel,
        const CollisionState& self,
        double other_weight,
        const LinearMotionKF& other_kf,
        double /*dt*/
    ) const {
        CorrectionResult result;

        const Eigen::Vector3d r = self.ee_position - other_kf.position();
        const double d = r.norm();

        if (d < 1e-6) {
            result.velocity_correction = Eigen::Vector3d::UnitZ() * 0.2;
            result.barrier_value = -1.0;
            result.active = true;
            return result;
        }

        const Eigen::Vector3d n = r / d;
        const Eigen::Vector3d v_other = other_kf.velocity();

        const double v_close_nom = std::max(0.0, -n.dot(nominal_vel - v_other));

        double delta_unc = 0.0;
        if (config_.use_uncertainty_padding) {
            const Eigen::Matrix3d Sigma_r = other_kf.covariance().block<3,3>(0,0);
            double sigma_radial = (n.transpose() * Sigma_r * n).value();
            delta_unc = config_.sigma_gain * std::sqrt(std::max(0.0, sigma_radial));
        }

        const double margin_required = config_.d_min + config_.beta * v_close_nom;// + delta_unc;
        const double h = d - margin_required;
        result.barrier_value = h;

        const double b = n.dot(v_other) - config_.alpha * h;

        if (n.dot(nominal_vel) >= b) {
            result.active = false;
            return result;
        }

        const double w_self = std::max(self.weight, 1e-6);
        const Eigen::Matrix3d Pn = n * n.transpose();
        const Eigen::Matrix3d Pt = Eigen::Matrix3d::Identity() - Pn;
        const Eigen::Matrix3d Q = w_self * (config_.q_radial * Pn + config_.q_tangential * Pt);
        const Eigen::Matrix3d Q_inv = (1.0 / w_self) * ((1.0 / config_.q_radial) * Pn + (1.0 / config_.q_tangential) * Pt);

        const double denom = n.transpose() * Q_inv * n;
        if (denom < 1e-12) {
            return result;
        }

        const double lambda = (b - n.dot(nominal_vel)) / denom;
        Eigen::Vector3d u_safe = nominal_vel + Q_inv * n * lambda;

        const double u_norm = u_safe.norm();
        if (u_norm > config_.max_velocity) {
            u_safe *= config_.max_velocity / u_norm;
        }

        result.velocity_correction = u_safe - nominal_vel;
        result.active = result.velocity_correction.norm() > 1e-9;
        return result;
    }

    std::string own_name_;
    std::shared_ptr<DeviceRegistry> registry_;
    SelfCollisionConfig config_;
    std::unordered_map<std::string, LinearMotionKF> peer_filters_;
};