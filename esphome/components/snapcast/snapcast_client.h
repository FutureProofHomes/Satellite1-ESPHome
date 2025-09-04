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

/*
ESPHome Snapcast client, this component manages connections to the Snapcast server and controls the media_player component.
*/
class SnapcastClient : public Component {
public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION - 10; }

  error_t connect_to_server(std::string url, uint32_t stream_port=1704, uint32_t rpc_port=1705);
  void set_media_player(esphome::speaker::SpeakerMediaPlayer* media_player){ this->media_player_ = media_player; }  
  void set_server_ip(std::string server_ip){ this->server_ip_ = server_ip; }
  SnapcastStream* get_stream(){ return &this->stream_; }
  
  //report volume to the snapcast server via the binary stream connection
  void report_volume(float volume, bool muted);
  void on_stream_update_msg(const StreamInfo &info);
  void on_stream_state_update(StreamState state, uint8_t volume, bool muted);

  error_t connect_to_url(std::string url){ return ESP_OK; }
  bool is_snapcast_url(std::string url){ return url.starts_with("snapcast://"); }

  
protected:
  error_t connect_via_mdns();
  std::string server_ip_;
  std::string client_id_;
  TaskHandle_t mdns_task_handle_{nullptr};

  SnapcastUrl curr_server_url_;
  SnapcastStream stream_;
  SnapcastControlSession cntrl_session_;
  esphome::speaker::SpeakerMediaPlayer* media_player_;
};




}
}