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

#include "esphome/components/audio/timed_ring_buffer.h"

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
    static constexpr uint8_t MAX_TIMES = 100; 
public:
    void add(tv_t val) {
        times[next_insert] = val;
        next_insert = (next_insert + 1) % MAX_TIMES;
        if (count < MAX_TIMES)
            ++count;
    }

    tv_t get_median() const {
        if( count == 0 ){
            return tv_t(0,0);
        }
        std::array<tv_t, MAX_TIMES> sorted{};
        std::copy(times.begin(), times.begin() + count, sorted.begin());
        std::sort(sorted.begin(), sorted.begin() + count);
        return sorted[count / 2];
    }

    void reset() {
        this->count = 0;
        this->next_insert = 0;
    }

private:
    std::array<tv_t, MAX_TIMES> times{};
    size_t count = 0;
    size_t next_insert = 0;
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
    esp_err_t start_with_notify(std::shared_ptr<esphome::TimedRingBuffer> ring_buffer, TaskHandle_t notification_task);

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
    
    std::shared_ptr<esphome::TimedRingBuffer> write_ring_buffer_;
    
    TaskHandle_t notification_target_{nullptr};
    std::function<void(StreamState state, uint8_t volume, bool muted)> on_status_update_;

private:    
    TaskHandle_t stream_task_handle_{nullptr};    
    StaticTask_t task_stack_;
    StackType_t *task_stack_buffer_{nullptr};
    void stream_task_();

    void connect_();
    void disconnect_();
    void start_streaming_();
    void stop_streaming_();
    void set_state_(StreamState new_state);

    void send_message_(SnapcastMessage &msg);
    void send_hello_();
    void send_report_();
    void send_time_sync_();
    
    esp_err_t read_and_process_messages_(uint32_t timeout_ms);
    
    bool codec_header_sent_{false};
    esp_transport_handle_t transport_{nullptr};
};

}
}