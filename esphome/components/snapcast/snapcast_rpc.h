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
#include <map>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>


#include "esp_transport.h"

#include "esphome/core/defines.h"
#include "esphome/components/json/json_util.h"

namespace esphome {
namespace snapcast {

enum class RequestId : uint32_t {
  GetServerStatus = 1,
  GetGroupStatus = 2,
  // etc.
};


struct ClientState {
  std::string group_id;
  std::string stream_id;
  int32_t latency = 0;
  uint8_t volume_percent = 100;
  bool muted = false;

  bool from_groups_json(JsonArray groups, std::string client_id) {
    for (JsonObject group_obj : groups) {
      for (JsonObject client_obj : group_obj["clients"].as<JsonArray>()) {
        if (client_obj["id"].as<std::string>() == client_id) {
          group_id = group_obj["id"].as<std::string>();
          stream_id = group_obj["stream_id"].as<std::string>();
          latency = client_obj["config"]["latency"].as<int32_t>();
          volume_percent = client_obj["config"]["volume"]["percent"].as<uint8_t>();
          muted = client_obj["config"]["volume"]["muted"].as<bool>();
          return true;
        }
      }
    }
    return false; 
  }
};


struct StreamInfo {
  std::string id;
  std::string status;
  bool canPlay;
  bool canPause;
  bool canSeek;
  bool canGoNext;
  bool canGoPrevious;

  bool from_json(JsonObject stream_obj) {
    if (!stream_obj["id"].is<std::string>()) return false;

    id = stream_obj["id"].as<std::string>();
    status = stream_obj["status"].as<std::string>();
    canPlay = stream_obj["canPlay"].as<bool>();
    canPause = stream_obj["canPause"].as<bool>();
    canSeek = stream_obj["canSeek"].as<bool>();
    canGoNext = stream_obj["canGoNext"].as<bool>();
    canGoPrevious = stream_obj["canGoPrevious"].as<bool>();
    return true;
  }

  bool from_streams_json(JsonArray streams, std::string stream_id) {
    for (JsonObject stream_obj : streams) {
        if (stream_obj["id"].as<std::string>() == stream_id) {
            return this->from_json(stream_obj);
         }
    }
    return false; 
  }

  bool from_stream_properties(JsonObject properties){
    status = properties["playbackStatus"].as<std::string>();
    canPlay = properties["canPlay"].as<bool>();
    canPause = properties["canPause"].as<bool>();
    canSeek = properties["canSeek"].as<bool>();
    canGoNext = properties["canGoNext"].as<bool>();
    canGoPrevious = properties["canGoPrevious"].as<bool>();
    return true;
  }

  bool set_id(std::string stream_id){
    id = stream_id;
    status = "idle";
    return true;
  }
  
  bool set_to_default(){
    return this->set_id("default");
  }

};

class SnapcastClient;

class SnapcastControlSession {
public:
    esp_err_t connect(std::string server, uint32_t port);
    esp_err_t disconnect();
    
    void notification_loop();

    void set_on_stream_update(std::function<void(const StreamInfo &)> cb) {
        this->on_stream_update_ = std::move(cb);
    }


protected:
    friend SnapcastClient;
    void send_rpc_request_(const std::string &method, std::function<void(JsonObject)> fill_params, uint32_t id);
    void update_from_server_obj_(const JsonObject &server_obj);
    
    std::string server_;
    uint32_t port_;
    std::string client_id_;
    esp_transport_handle_t transport_{nullptr};
    bool notification_task_should_run_{false};
    TaskHandle_t notification_task_handle_{nullptr};
    std::string recv_buffer_;
    ClientState client_state_;
    std::map<std::string,StreamInfo> known_streams_;
    std::function<void(const StreamInfo &)> on_stream_update_;
};


}
}
