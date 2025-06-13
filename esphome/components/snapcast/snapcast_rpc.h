#pragma once

#include <string>
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
    if (!stream_obj.containsKey("id")) return false;

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

};


class SnapcastControlSession {
public:
    esp_err_t connect(std::string server, uint32_t port);
    esp_err_t disconnect();
    
    void notification_loop();

    void set_on_stream_update(std::function<void(const StreamInfo &)> cb) {
        this->on_stream_update_ = std::move(cb);
    }


protected:
    void send_rpc_request_(const std::string &method, std::function<void(JsonObject)> fill_params, uint32_t id);
    void update_from_server_obj_(const JsonObject &server_obj);
    
    std::string server_;
    uint32_t port_;
    esp_transport_handle_t transport_{nullptr};
    bool notification_task_should_run_{false};
    TaskHandle_t notification_task_handle_{nullptr};
    std::string recv_buffer_;
    ClientState client_state_;
   
    std::function<void(const StreamInfo &)> on_stream_update_;
};


}
}
