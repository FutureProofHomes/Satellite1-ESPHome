/*
 * This file is part of Snapcast integration for ESPHome.
 *
 * Copyright (C) 2025 Mischa Siekmann <FutureProofHomes Inc.>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>
#include <unordered_map>
#include "freertos/semphr.h"

#include "esphome/components/audio/audio.h"

namespace esphome {
namespace snapcast {

class TimeStats {
  static constexpr size_t MAX_CONSECUTIVE_OUTLIERS = 5;

  struct OffsetSample {
    tv_t offset;  // sample offset relative to reference_offset_
    tv_t rtt;     // estimated rtt of the time sync message exchange
    tv_t t_recv;  // local clock time of received sync message response
  };

 public:
  struct Config {
    // General
    float base_smoothing = 0.02f;   // Classic EMA base alpha (also used for bias EMA)
    size_t min_valid_samples = 50;  // init: number of samples before locking initial estimate
    size_t window_size = 5;         // min-RTT selection window

    // Mode
    bool hybrid_kalman = true;  // true: Kalman offset+drift; false: adaptive EMA

    // RTT stats adaptation
    double rtt_jitter_beta = 0.05;  // EMA factor for excess RTT tracking (0.02..0.1)
    int64_t min_rtt_up_div = 400;   // slow upward creep divisor (200..1000)

    // Measurement uncertainty model (sigma_k in microseconds)
    double sigma_floor_us = 1000.0;       // baseline timestamp/app jitter floor
    double sigma_slope_us = 4;            // sigma growth per normalized RTT abnormality
    double sigma_scale_floor_us = 200.0;  // floor for normalization scale
    double sigma_z_cap = 8.0;             // cap for normalized RTT abnormality

    // Robust gating
    double gate_n_sigma = 6.0;  // outlier gate multiplier (4..8)
    bool hard_recover_on_outliers = true;

    // Adaptive forgetting (covariance inflation) on outliers (hybrid only)
    double inflate_base = 10.0;  // base inflation
    double inflate_slope = 5.0;  // extra inflation per RTT abnormality unit

    // Kalman process noise (clock behavior)
    // Offset process noise: us / sqrt(s)
    double q_offset_std_us_per_sqrt_s = 50.0;  // 20..200 typical
    // Drift random-walk: dimensionless / sqrt(s) (tiny non-zero allows drift to adapt)
    double q_drift_std_per_sqrt_s = 1e-6;  // 0..5e-6 typical

    // Initial covariance scales
    double P00_init = 1e10;
    double P11_init = 1e-10;
  };

  TimeStats() : TimeStats(Config{}) {}
  explicit TimeStats(const Config &cfg)
      : cfg_(cfg),
        smoothing_(cfg.base_smoothing),
        base_smoothing_(cfg.base_smoothing),
        min_valid_samples_(cfg.min_valid_samples),
        window_size_(cfg.window_size),
        hybrid_(cfg.hybrid_kalman) {}

  void set_request_time(uint16_t msg_id, tv_t request_time) {
    const int64_t us = request_time.to_microseconds();

    if (pending_req_send_us_.size() >= MAX_PENDING_REQ) {
      // Drop the oldest entry (simple O(n) fallback; MAX_PENDING_REQ is tiny)
      auto oldest = pending_req_send_us_.begin();
      for (auto it = pending_req_send_us_.begin(); it != pending_req_send_us_.end(); ++it) {
        if (it->second < oldest->second)
          oldest = it;
      }
      pending_req_send_us_.erase(oldest);
    }
    xSemaphoreTake(req_mu_, portMAX_DELAY);
    pending_req_send_us_[msg_id] = us;
    xSemaphoreGive(req_mu_);
  }

  bool try_get_request_send_time_us(uint32_t msg_id, int64_t *out_send_us) {
    if (!out_send_us)
      return false;
    xSemaphoreTake(req_mu_, portMAX_DELAY);
    auto it = pending_req_send_us_.find(msg_id);
    if (it == pending_req_send_us_.end()) {
      xSemaphoreGive(req_mu_);
      return false;
    }
    *out_send_us = it->second;
    pending_req_send_us_.erase(msg_id);
    xSemaphoreGive(req_mu_);
    return true;
  }

  // sample: measured offset (server - client) at approx received_time
  void add_offset(uint16_t msg_id, tv_t sample, tv_t received_time) {
    int64_t send_us = 0;
    if (!try_get_request_send_time_us(msg_id, &send_us)) {
      // No matching request -> ignore
      return;
    }

    const int64_t recv_us = received_time.to_microseconds();
    const int64_t rtt_us = recv_us - send_us;

    // Sanity clamp
    if (rtt_us <= 0 || rtt_us > 5'000'000) {  // >5s
      return;
    }
    const tv_t rtt = tv_t::from_microseconds(rtt_us);

    if (!has_reference_) {
      reference_offset_ = sample;
      has_reference_ = true;
    }

    const tv_t delta = sample - reference_offset_;

    // ---- Init: collect samples, choose min RTT one ----
    if (!has_value_) {
      pending_init_values_.push_back({delta, rtt, received_time});
      if (pending_init_values_.size() >= min_valid_samples_) {
        auto best = std::min_element(pending_init_values_.begin(), pending_init_values_.end(),
                                     [](const OffsetSample &a, const OffsetSample &b) { return a.rtt < b.rtt; });

        ema_ = best->offset;
        has_value_ = true;
        rtt_stats_reset_();
        rtt_stats_update_(best->rtt);
        pending_init_values_.clear();

        if (hybrid_)
          reset_kalman_(received_time, ema_);
      }
      return;
    }

    // ---- Bias reduction: pick min-RTT sample from a short window ----
    window_.push_back({delta, rtt});
    if (window_.size() > window_size_)
      window_.pop_front();

    auto best = std::min_element(window_.begin(), window_.end(),
                                 [](const OffsetSample &a, const OffsetSample &b) { return a.rtt < b.rtt; });

    const tv_t meas = best->offset;
    const tv_t meas_rtt = best->rtt;
    const tv_t t_meas = best->t_recv;

    // Update RTT stats (adaptive across environments)
    rtt_stats_update_(meas_rtt);

    if (!hybrid_) {
      adaptive_ema_update_(meas);
      return;
    }

    adaptive_kalman_update_(t_meas, meas, meas_rtt);
  }

  bool is_ready() const { return has_value_; }

  // Estimated offset: server_time - client_time
  tv_t get_estimate() const {
    if (!has_value_)
      return tv_t(0, 0);
    tv_t est = reference_offset_ + (hybrid_ ? last_kalman_offset_tv_ : ema_);
    if (has_bias_)
      est = est + bias_;
    return est;
  }

  void reset() {
    pending_init_values_.clear();
    window_.clear();

    reference_offset_ = tv_t(0, 0);
    ema_ = tv_t(0, 0);
    bias_ = tv_t(0, 0);
    send_request_time_ = tv_t(0, 0);

    has_value_ = false;
    has_bias_ = false;
    has_reference_ = false;
    outlier_count_ = 0;
    consecutive_outliers_ = 0;

    rtt_stats_reset_();
    reset_kalman_state_();

    smoothing_ = base_smoothing_ = cfg_.base_smoothing;
  }

  size_t outliers() const { return outlier_count_; }

  void set_hybrid(bool hybrid_kalman) {
    if (hybrid_ == hybrid_kalman)
      return;
    hybrid_ = hybrid_kalman;
    reset_kalman_state_();
  }

 public:
  // ---------------- Configuration ----------------
  Config cfg_;
  float smoothing_;
  float base_smoothing_;
  size_t min_valid_samples_;
  size_t window_size_;
  bool hybrid_;

  // ---------------- Flags/counters ----------------
  bool has_value_ = false;
  bool has_bias_ = false;
  bool has_reference_ = false;
  size_t outlier_count_ = 0;
  size_t consecutive_outliers_ = 0;

  // ---------------- Values ----------------
  tv_t ema_{0, 0};
  tv_t bias_{0, 0};
  tv_t reference_offset_{0, 0};
  tv_t send_request_time_{0, 0};

  std::deque<OffsetSample> window_;
  std::vector<OffsetSample> pending_init_values_;

  // ---------------- RTT adaptive stats ----------------
  SemaphoreHandle_t req_mu_ = xSemaphoreCreateMutex();
  std::unordered_map<uint16_t, int64_t> pending_req_send_us_;
  static constexpr size_t MAX_PENDING_REQ = 8;
  bool rtt_inited_ = false;
  int64_t min_rtt_us_ = 0;
  int64_t last_rtt_us_ = 0;
  double excess_ema_ = 0.0;  // EMA of excess RTT = RTT - minRTT
  double absdev_ema_ = 0.0;  // EMA of |excess - excess_ema|

  void rtt_stats_reset_() {
    rtt_inited_ = false;
    min_rtt_us_ = 0;
    last_rtt_us_ = 0;
    excess_ema_ = 0.0;
    absdev_ema_ = 0.0;
  }

  void rtt_stats_update_(tv_t rtt) {
    const int64_t rtt_us = std::max<int64_t>(1, rtt.to_microseconds());
    last_rtt_us_ = rtt_us;

    if (!rtt_inited_) {
      rtt_inited_ = true;
      min_rtt_us_ = rtt_us;
      excess_ema_ = 0.0;
      absdev_ema_ = 0.0;
      return;
    }

    if (rtt_us < min_rtt_us_) {
      min_rtt_us_ = rtt_us;
      return;
    }

    const int64_t gap = rtt_us - min_rtt_us_;
    const int64_t div = std::max<int64_t>(50, cfg_.min_rtt_up_div);
    min_rtt_us_ += std::max<int64_t>(0, gap / div);

    const double excess = static_cast<double>(std::max<int64_t>(0, rtt_us - min_rtt_us_));
    const double beta = std::clamp(cfg_.rtt_jitter_beta, 0.001, 0.5);
    excess_ema_ = (1.0 - beta) * excess_ema_ + beta * excess;

    const double dev = std::abs(excess - excess_ema_);
    absdev_ema_ = (1.0 - beta) * absdev_ema_ + beta * dev;
  }

  // sigma_k in microseconds
  int64_t current_sigma_us_() const {
    const double floor_sigma = std::max(10.0, cfg_.sigma_floor_us);
    if (!rtt_inited_)
      return static_cast<int64_t>(std::llround(floor_sigma));

    const double excess = static_cast<double>(std::max<int64_t>(0, last_rtt_us_ - min_rtt_us_));
    const double scale = std::max(cfg_.sigma_scale_floor_us, excess_ema_ + absdev_ema_);

    double z = excess / scale;
    if (z < 0.0)
      z = 0.0;
    if (z > cfg_.sigma_z_cap)
      z = cfg_.sigma_z_cap;

    const double sigma = floor_sigma + cfg_.sigma_slope_us * z;
    return static_cast<int64_t>(std::llround(std::max(10.0, sigma)));
  }

  // ---------------- Classic adaptive EMA ----------------
  void adaptive_ema_update_(tv_t meas) {
    const tv_t diff = meas - ema_;
    const int64_t sigma_us = current_sigma_us_();
    const int64_t gate_us = static_cast<int64_t>(std::llround(cfg_.gate_n_sigma * (double) sigma_us));
    const int64_t diff_us = diff.to_microseconds();

    if (std::llabs(diff_us) <= gate_us) {
      // RTT-adaptive effective alpha: reduce step size when RTT is abnormal
      const double excess = static_cast<double>(std::max<int64_t>(0, last_rtt_us_ - min_rtt_us_));
      const double scale = std::max(cfg_.sigma_scale_floor_us, excess_ema_ + absdev_ema_);
      double z = excess / scale;
      if (z < 0.0)
        z = 0.0;
      if (z > cfg_.sigma_z_cap)
        z = cfg_.sigma_z_cap;

      double alpha_eff = base_smoothing_ / (1.0 + 0.5 * z);
      alpha_eff = std::max(alpha_eff, static_cast<double>(base_smoothing_) * 0.25);

      ema_ = ema_ + diff * static_cast<float>(alpha_eff);
      consecutive_outliers_ = 0;
    } else {
      outlier_count_++;
      consecutive_outliers_++;
      if (cfg_.hard_recover_on_outliers && consecutive_outliers_ >= MAX_CONSECUTIVE_OUTLIERS) {
        // partial recenter for recovery
        ema_ = ema_ + (meas - ema_) * 0.5f;
        consecutive_outliers_ = 0;
      }
    }
  }

  // ---------------- Hybrid Kalman (offset + drift) ----------------
  bool kalman_init_ = false;
  int64_t kalman_last_t_us_ = 0;

  // State:
  // x_off_us_  = offset in microseconds
  // x_drift_   = drift in (us offset)/(us time) (dimensionless)
  double x_off_us_ = 0.0;
  double x_drift_ = 0.0;

  // Covariance P
  double P00_ = 1e18, P01_ = 0.0, P10_ = 0.0, P11_ = 1e12;

  tv_t last_kalman_offset_tv_{0, 0};

  void reset_kalman_state_() {
    kalman_init_ = false;
    kalman_last_t_us_ = 0;
    x_off_us_ = 0.0;
    x_drift_ = 0.0;
    P00_ = 1e18;
    P01_ = P10_ = 0.0;
    P11_ = 1e12;
    last_kalman_offset_tv_ = tv_t(0, 0);
  }

  void reset_kalman_(tv_t t_now, tv_t off_now) {
    kalman_init_ = true;
    kalman_last_t_us_ = t_now.to_microseconds();
    x_off_us_ = static_cast<double>(off_now.to_microseconds());
    x_drift_ = 0.0;

    P00_ = cfg_.P00_init;
    P01_ = P10_ = 0.0;
    P11_ = cfg_.P11_init;

    last_kalman_offset_tv_ = off_now;
  }

  void adaptive_kalman_update_(tv_t t_now, tv_t meas, tv_t rtt) {
    if (!kalman_init_) {
      reset_kalman_(t_now, meas);
      return;
    }

    const int64_t t_us = t_now.to_microseconds();
    double dt = static_cast<double>(t_us - kalman_last_t_us_);
    if (dt <= 0.0)
      dt = 1.0;

    // ---- Predict ----
    const double off_pred = x_off_us_ + x_drift_ * dt;
    const double drift_pred = x_drift_;

    // P = F P F^T + Q, F = [[1, dt], [0, 1]]
    double P00 = P00_ + (P01_ + P10_) * dt + P11_ * dt * dt;
    double P01 = P01_ + P11_ * dt;
    double P10 = P10_ + P11_ * dt;
    double P11 = P11_;

    // Process noise Q (scaled by dt in seconds)
    const double dt_s = dt / 1e6;
    const double q_off = cfg_.q_offset_std_us_per_sqrt_s * std::sqrt(std::max(0.0, dt_s));
    const double q_drift = cfg_.q_drift_std_per_sqrt_s * std::sqrt(std::max(0.0, dt_s));
    P00 += q_off * q_off;
    P11 += q_drift * q_drift;

    // ---- Measurement ----
    const double z = static_cast<double>(meas.to_microseconds());
    const double y = z - off_pred;  // residual (us)

    const double sigma = static_cast<double>(current_sigma_us_());
    const double R = sigma * sigma;

    const double gate = cfg_.gate_n_sigma * sigma;
    const bool outlier = (std::abs(y) > gate);

    if (outlier) {
      outlier_count_++;
      consecutive_outliers_++;
    } else {
      consecutive_outliers_ = 0;
    }

    // ---- Adaptive forgetting on outliers ----
    if (outlier) {
      const double rtt_us = static_cast<double>(std::max<int64_t>(1, rtt.to_microseconds()));
      const double excess = std::max(0.0, rtt_us - static_cast<double>(min_rtt_us_));
      const double scale = std::max(cfg_.sigma_scale_floor_us, excess_ema_ + absdev_ema_);

      double z_rtt = excess / scale;
      if (z_rtt < 0.0)
        z_rtt = 0.0;
      if (z_rtt > cfg_.sigma_z_cap)
        z_rtt = cfg_.sigma_z_cap;

      const double inflate = cfg_.inflate_base + cfg_.inflate_slope * z_rtt;
      P00 *= inflate;
      P01 *= inflate;
      P10 *= inflate;
      P11 *= inflate;
    }

    if (cfg_.hard_recover_on_outliers && consecutive_outliers_ >= MAX_CONSECUTIVE_OUTLIERS) {
      reset_kalman_(t_now, meas);
      consecutive_outliers_ = 0;
      return;
    }

    // ---- Update ----
    const double S = P00 + R;
    const double invS = 1.0 / S;

    const double K0 = P00 * invS;
    const double K1 = P10 * invS;

    x_off_us_ = off_pred + K0 * y;
    x_drift_ = drift_pred + K1 * y;

    // Covariance update (simple form; clamp for numeric safety)
    const double P00n = (1.0 - K0) * P00;
    const double P01n = (1.0 - K0) * P01;
    const double P10n = P10 - K1 * P00;
    const double P11n = P11 - K1 * P01;

    P00_ = std::max(0.0, P00n);
    P01_ = P01n;
    P10_ = P10n;
    P11_ = std::max(0.0, P11n);

    kalman_last_t_us_ = t_us;

    // Cache offset estimate at t_now (rounded to nearest µs)
    last_kalman_offset_tv_ = tv_t::from_microseconds(static_cast<int64_t>(std::llround(x_off_us_)));
  }
};

}  // namespace snapcast
}  // namespace esphome