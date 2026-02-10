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

#include "esphome/components/speaker/media_player/speaker_media_player.h"

#include "messages.h"
#include "snapcast_stream.h"
#include "snapcast_rpc.h"
#include "snapcast_schema.h"

namespace esphome {
namespace snapcast {

struct SnapcastServer {
  std::string server_ip;
  uint32_t stream_port = 1704;
  uint32_t rpc_port = 1705;
};

enum class SnapcastClientState {
  DISCONNECTED,
  CONNECTING,
  IDLE,
  PLAYING,
  DISCONNECTING,
};

/*
ESPHome Snapcast client, this component manages connections to the Snapcast server and controls the media_player
component.
*/
class SnapcastClient : public Component {
 public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION + 10; }
  void loop() override;

  error_t connect_to_server(std::string url, uint32_t stream_port = 1704, uint32_t rpc_port = 1705);
  void set_media_player(esphome::speaker::SpeakerMediaPlayer *media_player) { this->media_player_ = media_player; }
  void set_server_ip(std::string server_ip) { this->cfg_server_ip_ = server_ip; }
  SnapcastStream *get_stream() { return &this->stream_; }

  // report volume to the snapcast server via the binary stream connection
  void report_volume(float volume, bool muted);
  void on_stream_update_msg(const StreamInfo &info);
  void on_stream_state_update(StreamState state, uint8_t volume, bool muted);

  void enable() {
    this->enabled_ = true;
    this->enable_loop();
  };
  void disable() {
    this->enabled_ = false;
    this->stream_.disconnect();
    this->cntrl_session_.disconnect();
  };
  error_t connect_to_url(std::string url) { return ESP_OK; }
  bool is_snapcast_url(std::string url) { return url.starts_with("snapcast://"); }

 protected:
  bool enabled_{true};
  bool network_initialized_{false};
  void on_network_ready_();
  SnapcastClientState state_{SnapcastClientState::DISCONNECTED};
  bool play_requested_{false};

  error_t start_mdns_scan_();
  error_t mdns_task_();
  TaskHandle_t mdns_task_handle_{nullptr};
  uint32_t mdns_scan_interval_ms_{30000};
  uint32_t mdns_last_scan_{0};
  std::string found_ip_;

  std::optional<SnapcastServer> server_;

  std::string cfg_server_ip_;
  std::string client_id_;

  SnapcastUrl curr_server_url_;
  SnapcastStream stream_;
  SnapcastControlSession cntrl_session_;
  esphome::speaker::SpeakerMediaPlayer *media_player_;
};

}  // namespace snapcast
}  // namespace esphome