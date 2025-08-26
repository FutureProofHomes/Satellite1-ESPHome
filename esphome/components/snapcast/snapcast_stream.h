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

#include <string>
#include "esp_transport.h"

#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/component.h"

#include "esphome/components/audio/chunked_ring_buffer.h"
//#include "esphome/components/audio/timed_ring_buffer.h"

#include "messages.h"



namespace esphome {
using namespace audio;
namespace snapcast {


enum class StreamState {
  DISCONNECTED,
  CONNECTING,
  CONNECTED_IDLE,      // Connected but waiting
  STREAMING,           // Receiving data
  ERROR,               // Fatal or recoverable error
  STOPPING             // Requested shutdown
};


class TimeStats {
    static constexpr size_t MAX_CONSECUTIVE_OUTLIERS = 5;
    struct OffsetSample {
        tv_t offset;
        tv_t rtt;
    };
public:
    TimeStats(float smoothing = 0.02f, int outlier_threshold_ms = 10, size_t min_valid_samples = 20)
        : smoothing_(smoothing), base_smoothing_(smoothing),
          outlier_threshold_ms_(outlier_threshold_ms),
          min_valid_samples_(min_valid_samples) {}

    
    void add_offset(tv_t sample, tv_t received_time) {
        if (!has_reference_) {
            reference_offset_ = sample;
            has_reference_ = true;
        }

        tv_t delta = sample - reference_offset_;
        tv_t rtt = received_time - send_request_time_;

        // Initialization with minimum RTT filter
        if (!has_value_) {
            pending_init_values_.push_back({delta, rtt});

            if (pending_init_values_.size() >= min_valid_samples_) {
                // Find sample with minimum RTT
                auto best = std::min_element(
                    pending_init_values_.begin(),
                    pending_init_values_.end(),
                    [](const auto &a, const auto &b) { return a.rtt < b.rtt; });

                ema_ = best->offset;
                has_value_ = true;
                pending_init_values_.clear();
#if SNAPCAST_DEBUG                
                printf("Init done: using offset with min RTT: %" PRId64 " us\n", ema_.to_microseconds());
#endif                
            }
            return;
        }

        window_.push_back({delta, rtt});
        if (window_.size() > 5) window_.pop_front();

        // Find offset with minimum RTT
        auto best = std::min_element(
            window_.begin(), window_.end(),
            [](const OffsetSample& a, const OffsetSample& b) { return a.rtt < b.rtt; });


        // After initialization: apply EMA with outlier rejection
        tv_t diff = delta - ema_;
        if (std::abs(diff.to_millis()) < outlier_threshold_ms_) {
            ema_ = ema_ + (diff * smoothing_);
            consecutive_outliers_ = 0;
            
            // Slowly decay alpha if it was bumped
            if (smoothing_ > base_smoothing_) {
                smoothing_ *= 0.99f; 
                smoothing_ = std::max(smoothing_, base_smoothing_);
            }
        } else {
            
            outlier_count_++;
            consecutive_outliers_++;
        }
#if SNAPCAST_DEBUG        
        printf( "new: %" PRId64 " AtminRTT: %" PRId64 " ema: %" PRId64 " diff: %" PRId64 " RTT: %" PRId64 " minRTT: %" PRId64 "\n", 
            delta.to_microseconds(), 
            best->offset.to_microseconds(), 
            ema_.to_microseconds(), 
            diff.to_microseconds(), 
            rtt.to_microseconds(),
            best->rtt.to_microseconds() 
        );
#endif        
    }

    // Add bias sample (from (latency_c2s - latency_s2c)/2)
    void add_bias(tv_t asymmetry) {
        if (!has_bias_) {
        bias_ = asymmetry;
        has_bias_ = true;
        return;
        }
        
        tv_t diff = asymmetry - bias_;
        if (std::abs(diff.to_millis()) < outlier_threshold_ms_) {
            bias_ = bias_ + (diff * smoothing_);
        }
    }

    // Optional: correct drift from frequent packets
    void add_drift_correction(tv_t predicted_server, tv_t received_server) {
        // Just adjust offset by tiny fraction of error
        tv_t diff = received_server - predicted_server;
        ema_ = ema_ + (diff * 0.001f); // tiny correction
    }

    void set_request_time(tv_t request_time) {
        this->send_request_time_ = request_time;
    }       
    
    bool is_ready() const {
        return has_value_;
    }

    tv_t get_estimate() const {
        return has_value_ ? reference_offset_ + ema_ + (has_bias_ ? bias_ : tv_t(0, 0)) : tv_t(0, 0);
    }

    void reset() {
        pending_init_values_.clear();
        
        reference_offset_ = tv_t(0, 0);
        ema_  = tv_t(0, 0);
        bias_ = tv_t{0,0};
        
        has_value_ = false;
        has_bias_ = false;
        has_reference_ = false;
        outlier_count_ = 0;
        consecutive_outliers_ = 0;
    }

    size_t outliers() const { return outlier_count_; }

private:
    float smoothing_;
    const float base_smoothing_;
    int outlier_threshold_ms_;
    size_t min_valid_samples_;

    bool has_value_ = false;
    bool has_bias_ = false;
    bool has_reference_ = false;
    size_t outlier_count_ = 0;
    size_t consecutive_outliers_ = 0;

    tv_t ema_{0, 0};
    tv_t bias_{0, 0};
    tv_t reference_offset_{0, 0};
    tv_t send_request_time_{0,0};

    std::deque<OffsetSample> window_;
    std::vector<OffsetSample> pending_init_values_;

    // Helper: average of tv_t deltas
    tv_t average_tv(const std::vector<tv_t>& values) const {
        if (values.empty()) return tv_t(0, 0);
        int64_t total_usec = 0;
        for (const auto& val : values) {
            total_usec += static_cast<int64_t>(val.sec) * 1'000'000 + val.usec;
        }
        total_usec /= static_cast<int64_t>(values.size());
        return tv_t(total_usec / 1'000'000, total_usec % 1'000'000);
    }
};



class SnapcastClient;

class SnapcastStream {
public:
    /// @brief Establish a connection to the Snapcast server.
    /// @param server The hostname or IP address of the Snapcast server.
    /// @param port The TCP port to connect to on the server.
    /// @return `ESP_OK` on success, or an appropriate error code.
    esp_err_t connect(std::string server, uint32_t port);

    /// @brief Disconnect from the Snapcast server.
    /// @return `ESP_OK` on success, or an appropriate error code.
    esp_err_t disconnect();

    /// @brief Start receiving and processing audio/data from the server.
    /// @param ring_buffer The buffer where received audio samples are written.
    /// @param notification_task A FreeRTOS task to be notified on status changes.
    /// @return `ESP_OK` on success, or an appropriate error code.
    esp_err_t start_with_notify(std::weak_ptr<esphome::TimedRingBuffer> ring_buffer, TaskHandle_t notification_task);

    /// @brief Stop the audio/data stream and return to an idle state.
    /// @return `ESP_OK` on success, or an appropriate error code.
    esp_err_t stop_streaming();

    /// @brief Report volume and mute status to the Snapcast server.
    /// @param volume The volume level to report (0–100).
    /// @param muted Whether the stream is muted.
    /// @return `ESP_OK` on success, or an appropriate error code.
    esp_err_t report_volume(uint8_t volume, bool muted);

    /// @brief Check if the stream is connected (either idle or actively streaming).
    /// @return `true` if connected, `false` otherwise.
    bool is_connected() {
        return this->state_ == StreamState::STREAMING || this->state_ == StreamState::CONNECTED_IDLE;
    }

    /// @brief Check if the stream is actively receiving data.
    /// @return `true` if streaming is in progress, `false` otherwise.
    bool is_running() {
        return this->state_ == StreamState::STREAMING;
    }

    /// @brief Set a callback to be invoked on stream status updates.
    /// @param cb A function receiving state, volume, and mute information.
    void set_on_status_update_callback(std::function<void(StreamState state, uint8_t volume, bool muted)> cb) {
        this->on_status_update_ = std::move(cb);
    }    

protected:
    friend SnapcastClient;
    void on_server_settings_msg_(const ServerSettingsMessage &msg);
    void on_time_msg_(MessageHeader msg, tv_t time);
    
    std::string server_;
    uint32_t port_;
    uint32_t server_buffer_size_{0};
    int32_t latency_{0};
    uint8_t volume_{0};
    bool muted_{false};
    bool reconnect_on_error_{true};
    uint32_t reconnect_counter_{0};
    
    StreamState state_{StreamState::DISCONNECTED};
    std::string error_msg_;
    
    tv_t to_local_time_(tv_t server_time) const {
        return server_time - this->est_time_diff_ + tv_t::from_millis(this->server_buffer_size_ + this->latency_);
    }
    uint32_t last_time_sync_{0};
    TimeStats time_stats_;
    tv_t est_time_diff_{0, 0};
    
    std::weak_ptr<esphome::TimedRingBuffer> write_ring_buffer_;
    
    TaskHandle_t notification_target_{nullptr};
    std::function<void(StreamState state, uint8_t volume, bool muted)> on_status_update_;

private:    
    TaskHandle_t stream_task_handle_{nullptr};
    StaticTask_t task_stack_;
    StackType_t *task_stack_buffer_{nullptr};
    
    void stream_task_();

    void start_streaming_();
    void stop_streaming_();
    void set_state_(StreamState new_state);

    void send_message_(SnapcastMessage *msg);
    void send_hello_();
    void send_report_();
    void send_time_sync_();
    
    esp_err_t read_and_process_messages_(ChunkedRingBuffer* read_ring_buffer, uint32_t timeout_ms);
    
    bool start_after_connecting_{false};
    bool codec_header_sent_{false};
};

}
}