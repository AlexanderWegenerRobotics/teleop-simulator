#pragma once

#include <Eigen/Dense>
#include <string>
#include <unordered_map>
#include <mutex>
#include <iostream>

// ---------------------------------------------------------------------------
// DeviceState — per-device snapshot published into the shared registry.
//               Each arm pushes its own state; reads others for collision checks.
// ---------------------------------------------------------------------------
struct DeviceState {
    Eigen::Vector3d ee_position  = Eigen::Vector3d::Zero();   // world frame
    Eigen::Vector3d ee_velocity  = Eigen::Vector3d::Zero();   // world frame linear vel
    double          weight       = 1.0;                       // importance (higher = protect more)

    // --- future extension slots (do not break ABI) ---
    // Eigen::Matrix<double,6,7> jacobian;     // for torque-level HoCBF
    // double          prediction_sigma;        // for EKF-based probabilistic barrier
};

// ---------------------------------------------------------------------------
// SelfCollisionConfig — tuneable parameters, loadable from YAML later.
// ---------------------------------------------------------------------------
struct SelfCollisionConfig {
    double d_min   = 0.04;    // minimum allowed EE distance [m]
    double alpha   = 1.0;     // CBF class-K gain (higher = more aggressive)
    double beta    = 0.10;    // velocity-aware inflation [m²/s]
    bool   enabled = true;    // global enable flag
};

// ---------------------------------------------------------------------------
// CorrectionResult — output of the filter for the calling device.
// ---------------------------------------------------------------------------
struct CorrectionResult {
    Eigen::Vector3d velocity_correction = Eigen::Vector3d::Zero();  // Δẋ to add to nominal
    double          barrier_value       = 1e6;                      // h(x), for logging/debug
    bool            active              = false;                    // true if constraint was enforced
};

// ---------------------------------------------------------------------------
// SelfCollisionProtection
//
//   Shared state registry + velocity-level CBF filter.
//   One instance per arm.  All instances share the same DeviceRegistry via
//   a std::shared_ptr so they see each other's published states.
//
//   Usage:
//     1. Avatar creates a shared DeviceRegistry.
//     2. Each ArmControl receives a shared_ptr to it + its own name.
//     3. Every state-handler tick:
//          a. arm calls  publishState(...)  with its fresh EE data
//          b. arm calls  computeCorrection(...)  before feeding the
//             interpolator, getting back the safe velocity delta
// ---------------------------------------------------------------------------

// Thread-safe registry — lives once, shared by all devices.
class DeviceRegistry {
public:
    void publish(const std::string& name, const DeviceState& state) {
        std::lock_guard<std::mutex> lock(mtx_);
        states_[name] = state;
    }

    // Returns a snapshot — caller does not hold the lock after return.
    std::unordered_map<std::string, DeviceState> snapshot() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return states_;
    }

private:
    mutable std::mutex                                mtx_;
    std::unordered_map<std::string, DeviceState>      states_;
};


class SelfCollisionProtection {
public:
    SelfCollisionProtection(const std::string& own_name,
                            std::shared_ptr<DeviceRegistry> registry,
                            const SelfCollisionConfig& config = {})
        : own_name_(own_name)
        , registry_(std::move(registry))
        , config_(config)
    {}

    // --- publish own state (call every tick, before computeCorrection) ------
    void publishState(const DeviceState& state) {
        registry_->publish(own_name_, state);
    }

    // --- core filter --------------------------------------------------------
    //  nominal_vel:  the EE velocity implied by the current interpolator target
    //  own_state:    this arm's fresh DeviceState (same data just published)
    //  dt:           tick period [s] — used to convert velocity correction → pose offset
    //
    //  Returns the correction for THIS device only.
    //  The method checks against ALL other devices in the registry.
    CorrectionResult computeCorrection(const Eigen::Vector3d& nominal_vel,
                                       const DeviceState& own_state,
                                       double dt) const
    {
        if (!config_.enabled) return {};

        auto all = registry_->snapshot();

        // Accumulate the most restrictive correction across all other devices.
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

    // --- accessors ----------------------------------------------------------
    const SelfCollisionConfig& config() const { return config_; }
    void setConfig(const SelfCollisionConfig& c) { config_ = c; }
    void setEnabled(bool e) { config_.enabled = e; }

private:

    // CBF filter for a single pair (this device vs one other).
    CorrectionResult filterPair(const Eigen::Vector3d& nominal_vel,
                                const DeviceState& self,
                                const DeviceState& other,
                                double /*dt*/) const
    {
        CorrectionResult result;

        // --- Step 1–2: geometry ---
        const Eigen::Vector3d d   = self.ee_position - other.ee_position;
        const double          D2  = d.squaredNorm();
        const double          D   = std::sqrt(D2);

        // Panic guard: if already inside d_min/2, apply maximum repulsion along d.
        if (D < config_.d_min * 0.5) {
            Eigen::Vector3d escape = (D > 1e-6) ? d.normalized() : Eigen::Vector3d::UnitZ();
            result.velocity_correction = escape * 0.5;   // strong push outward
            result.barrier_value       = -1.0;
            result.active              = true;
            return result;
        }

        const Eigen::Vector3d n_hat = d / D;

        // Approach velocity (positive = closing)
        const Eigen::Vector3d rel_vel = self.ee_velocity - other.ee_velocity;
        const double v_approach = -n_hat.dot(rel_vel);

        // --- Step 3: barrier value ---
        const double d_min2 = config_.d_min * config_.d_min;
        const double h = D2 - d_min2 - config_.beta * std::max(0.0, v_approach);

        result.barrier_value = h;

        // --- Step 4: constraint ---
        //  A_self = 2dᵀ (3-vector, the gradient w.r.t. this arm's velocity)
        //  A_other = -2dᵀ
        //  Full constraint:  A_self · ẋ_self + A_other · ẋ_other ≥ -α·h
        //
        //  Since each arm only controls its own velocity, we assume the other arm
        //  will continue at its current velocity.  So the "other" contribution is
        //  a known constant and we absorb it into the bound:
        //
        //    A_self · ẋ_self  ≥  -α·h  -  A_other · ẋ_other_current
        //                     =  -α·h  +  2dᵀ · ẋ_other_current
        //
        //  But wait — the full QP we derived jointly optimises both arms with
        //  weights.  In the decentralised version each arm does its own projection
        //  assuming the other continues at current velocity.  We use the weight
        //  ratio to scale how much THIS arm corrects vs how much it "expects"
        //  the other arm to correct:
        //
        //    responsibility = w_other / (w_self + w_other)
        //
        //  This arm only enforces (responsibility) fraction of the total
        //  required correction.  If w_self >> w_other, responsibility → 0
        //  and this arm barely moves.  If w_self << w_other, responsibility → 1
        //  and this arm takes most of the dodge.

        const Eigen::Vector3d A_self  =  2.0 * d;   // ∂h/∂ẋ_self direction
        const Eigen::Vector3d A_other = -2.0 * d;

        const double other_contribution = A_other.dot(other.ee_velocity);
        const double rhs = -config_.alpha * h + other_contribution;

        // How much of the fix is our job?
        const double w_self  = self.weight;
        const double w_other = other.weight;
        const double responsibility = w_other / (w_self + w_other);

        // Check if nominal velocity already satisfies the constraint.
        const double margin = A_self.dot(nominal_vel) - rhs;

        if (margin >= 0.0) {
            // Safe — no correction needed.
            result.active = false;
            return result;
        }

        // --- Step 5: closed-form projection ---
        //  We solve:  min  (1/2)||ẋ - ẋ_nom||²   s.t.  A_self·ẋ ≥ rhs
        //  with the correction scaled by our responsibility share.
        //
        //  λ = -margin / ||A_self||²     (scalar Lagrange multiplier)
        //  Δẋ = λ · A_self               (correction along constraint gradient)

        const double A_norm2 = A_self.squaredNorm();
        if (A_norm2 < 1e-12) return result;   // degenerate — arms coincident

        const double lambda = (-margin / A_norm2) * responsibility;

        result.velocity_correction = lambda * A_self;
        result.active = true;

        return result;
    }

    std::string                      own_name_;
    std::shared_ptr<DeviceRegistry>  registry_;
    SelfCollisionConfig              config_;
};
