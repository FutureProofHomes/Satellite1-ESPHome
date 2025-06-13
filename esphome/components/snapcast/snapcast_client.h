#pragma once

#include <string>
#include "esp_transport.h"

#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/component.h"

#include "esphome/components/audio/timed_ring_buffer.h"
#include "esphome/components/speaker/media_player/speaker_media_player.h"

#include "messages.h"
#include "snapcast_stream.h"
#include "snapcast_rpc.h"


namespace esphome {
namespace snapcast {


class SnapcastClient : public Component {
public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  error_t connect_to_server(std::string url, uint32_t port);
  void set_media_player(esphome::speaker::SpeakerMediaPlayer* media_player){ this->media_player_ = media_player; }  
  void set_server_ip(std::string server_ip){ this->server_ip_ = server_ip; }
  SnapcastStream* get_stream(){ return &this->stream_; }
  
  void report_volume(float volume, bool muted);
  void on_stream_update_msg(const StreamInfo &info);
  void on_stream_state_update(StreamState state, uint8_t volume, bool muted);
protected:
  error_t connect_via_mdns();
  std::string server_ip_;
  TaskHandle_t mdns_task_handle_{nullptr};

  SnapcastStream stream_;
  SnapcastControlSession cntrl_session_;
  esphome::speaker::SpeakerMediaPlayer* media_player_;
};




}
}