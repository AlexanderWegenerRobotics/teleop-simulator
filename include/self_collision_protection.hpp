#pragma once

#include <Eigen/Dense>
#include <string>
#include <unordered_map>
#include <mutex>


struct CollisionState {
    Eigen::Vector3d ee_position  = Eigen::Vector3d::Zero();   // world frame
    Eigen::Vector3d ee_velocity  = Eigen::Vector3d::Zero();   // world frame linear vel (commanded/intended)
    double          weight       = 1.0;                       // importance (higher = protect more)
};

struct SelfCollisionConfig {
    double d_min   = 0.04;    // minimum allowed EE distance [m]
    double alpha   = 1.0;     // CBF class-K gain (higher = more aggressive)
    double beta    = 0.10;    // velocity-aware inflation [m^2/s]
    bool   enabled = true;    // global enable flag
};


struct CorrectionResult {
    Eigen::Vector3d velocity_correction = Eigen::Vector3d::Zero();  // Δx^dot to add to nominal
    double          barrier_value       = 1e6;                      // h(x), for logging/debug
    bool            active              = false;                    // true if constraint was enforced
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
    mutable std::mutex                                mtx_;
    std::unordered_map<std::string, CollisionState>      states_;
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

    CorrectionResult computeCorrection(const Eigen::Vector3d& nominal_vel, const CollisionState& own_state, double dt) const{
        if (!config_.enabled) return {};

        auto all = registry_->snapshot();

        CorrectionResult worst;
        worst.barrier_value = 1e6;

        for (const auto& [name, other] : all) {
            if (name == own_name_) continue;

            CorrectionResult cr = filterPair(nominal_vel, own_state, other, dt);
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

    CorrectionResult filterPair(const Eigen::Vector3d& nominal_vel, const CollisionState& self, const CollisionState& other, double /*dt*/) const {
        CorrectionResult result;

        const Eigen::Vector3d d   = self.ee_position - other.ee_position;
        const double          D2  = d.squaredNorm();
        const double          D   = std::sqrt(D2);

        // emergency escape if deeply inside exclusion zone
        if (D < config_.d_min * 0.5) {
            Eigen::Vector3d escape = (D > 1e-6) ? d.normalized() : Eigen::Vector3d::UnitZ();
            result.velocity_correction = escape * 0.5;
            result.barrier_value       = -1.0;
            result.active              = true;
            return result;
        }

        const Eigen::Vector3d n_hat = d / D;

        const Eigen::Vector3d rel_vel = nominal_vel - other.ee_velocity;
        const double v_approach = -n_hat.dot(rel_vel);

        const double d_min2 = config_.d_min * config_.d_min;
        const double h = D2 - d_min2 - config_.beta * std::max(0.0, v_approach);

        result.barrier_value = h;

        const Eigen::Vector3d A_self  =  2.0 * d;
        const Eigen::Vector3d A_other = -2.0 * d;

        const double other_contribution = A_other.dot(other.ee_velocity);
        const double rhs = -config_.alpha * h - other_contribution;

        // Responsibility split: how much of the correction this arm takes
        const double w_self  = self.weight;
        const double w_other = other.weight;
        const double responsibility = w_other / (w_self + w_other);

        // margin > 0 means constraint is already satisfied
        const double margin = A_self.dot(nominal_vel) - rhs;

        if (margin >= 0.0) {
            result.active = false;
            return result;
        }

        // Project onto constraint boundary (minimum-norm correction)
        const double A_norm2 = A_self.squaredNorm();
        if (A_norm2 < 1e-12) return result;

        const double lambda = (-margin / A_norm2) * responsibility;

        result.velocity_correction = lambda * A_self;
        result.active = true;

        return result;
    }

    std::string                      own_name_;
    std::shared_ptr<DeviceRegistry>  registry_;
    SelfCollisionConfig              config_;
};