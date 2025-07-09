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
#include "snapcast_stream.h"
#include "messages.h"

#include "esphome/core/log.h"

#include "esp_transport.h"
#include "esp_transport_tcp.h"

#include "esp_mac.h"


namespace esphome {
namespace snapcast {

static const char *const TAG = "snapcast_stream";

static uint8_t tx_buffer[1024];
static uint8_t rx_buffer[4096];
static uint32_t rx_bufer_length = 0; 

static const uint8_t STREAM_TASK_PRIORITY = 5;
static const uint32_t CONNECTION_TIMEOUT_MS = 2000;
static const size_t TASK_STACK_SIZE = 4 * 1024;
static const uint32_t TIME_SYNC_INTERVAL_MS =  2000;

enum class StreamCommandBits : uint32_t {
  NONE           = 0,
  CONNECT        = 1 << 0,
  DISCONNECT     = 1 << 1,
  START_STREAM   = 1 << 2,
  STOP_STREAM    = 1 << 3,
  SEND_REPORT    = 1 << 4,
};

esp_err_t SnapcastStream::connect(std::string server, uint32_t port){
    this->server_ = server;
    this->port_ = port;    
    if( this->stream_task_handle_ == nullptr ){
        ESP_LOGI(TAG, "Heap before task: %u", xPortGetFreeHeapSize());
        RAMAllocator<StackType_t> stack_allocator(RAMAllocator<StackType_t>::ALLOC_INTERNAL);
        this->task_stack_buffer_ = stack_allocator.allocate(TASK_STACK_SIZE);
        if (this->task_stack_buffer_ == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate memory.");
            return ESP_ERR_NO_MEM;
        }
        this->stream_task_handle_ =
          xTaskCreateStatic(
            [](void *param) {
                auto *stream = static_cast<SnapcastStream *>(param);
                stream->stream_task_();
                vTaskDelete(nullptr);
            } 
            , "snap_stream_task", TASK_STACK_SIZE, (void *) this,
             STREAM_TASK_PRIORITY, this->task_stack_buffer_, &this->task_stack_);
        

        if (this->stream_task_handle_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create snapcast stream task.");
            this->stream_task_handle_ = nullptr;  // Ensure it's reset
            return ESP_FAIL;
        }
        
    }
    xTaskNotify( this->stream_task_handle_, static_cast<uint32_t>(StreamCommandBits::CONNECT), eSetValueWithOverwrite);
    return ESP_OK;
}

esp_err_t SnapcastStream::disconnect(){
   xTaskNotify( this->stream_task_handle_, static_cast<uint32_t>(StreamCommandBits::DISCONNECT), eSetValueWithOverwrite);
   return ESP_OK; 
}

esp_err_t SnapcastStream::start_with_notify(std::weak_ptr<esphome::TimedRingBuffer> ring_buffer, TaskHandle_t notification_task){
    printf( "Called start_with_notify in state %d\n", static_cast<int>(this->state_));
    ESP_LOGD(TAG, "Starting stream..." );
    this->write_ring_buffer_ = ring_buffer;
    this->notification_target_ = notification_task;
    xTaskNotify( this->stream_task_handle_, static_cast<uint32_t>(StreamCommandBits::START_STREAM), eSetValueWithOverwrite);
    return ESP_OK;
}

esp_err_t SnapcastStream::stop_streaming(){
    xTaskNotify( this->stream_task_handle_, static_cast<uint32_t>(StreamCommandBits::STOP_STREAM), eSetValueWithOverwrite);
    return ESP_OK;
}

esp_err_t SnapcastStream::report_volume(uint8_t volume, bool muted){
    if( volume != this->volume_ || muted_ != this->muted_){
        this->volume_ = volume;
        this->muted_ = muted;
        xTaskNotify( this->stream_task_handle_, static_cast<uint32_t>(StreamCommandBits::SEND_REPORT), eSetValueWithOverwrite);
    }
    return ESP_OK;
}





esp_err_t SnapcastStream::read_and_process_messages_(uint32_t timeout_ms){
    auto ring_buffer = this->write_ring_buffer_.lock();
    const uint32_t timeout = millis() + timeout_ms;
    while( millis() < timeout ){
        size_t to_read = sizeof(MessageHeader) > rx_bufer_length ? sizeof(MessageHeader) - rx_bufer_length : 0;
        if( to_read > 0 ){
            int len = esp_transport_read(this->transport_, (char*) rx_buffer + rx_bufer_length, to_read, timeout_ms);
            if (len <= 0) {
                return len == 0 ? ERR_TIMEOUT : ESP_FAIL;
            } else {
                rx_bufer_length += len;
            }
        }
        if (rx_bufer_length < sizeof(MessageHeader)){
            vTaskDelay(1);
            continue;
        }
        
        MessageHeader* msg = reinterpret_cast<MessageHeader*>(rx_buffer);
        if( msg->received == tv_t(0,0) ){
            tv_t now = tv_t::now();
            msg->received = now;
#if 0            
            if( this->time_stats_.is_ready() ){
                tv_t local_est_server_time = now + this->time_stats_.get_estimate();
                this->time_stats_.add_bias( msg->sent - local_est_server_time );
            }
#endif            
        }        
        to_read = msg->getMessageSize() > rx_bufer_length ? msg->getMessageSize() - rx_bufer_length : 0;
        if ( to_read > 0 ){
            int len = esp_transport_read(this->transport_, (char*) rx_buffer + rx_bufer_length, to_read, timeout_ms);
            if (len <= 0) {
               return len == 0 ? ERR_TIMEOUT : ESP_FAIL;
            } else {
                rx_bufer_length += len;
            }
            if ( rx_bufer_length < msg->getMessageSize()){
                vTaskDelay(1);
                continue;
            }
        }
        // Now we have a complete message in rx_buffer
        // reset buffer length here, so we don't need to take care of it in the switch statement
        rx_bufer_length = 0;
        uint8_t *payload = rx_buffer + sizeof(MessageHeader); 
        size_t payload_len = msg->typed_message_size;
        switch( msg->getMessageType() ){
            case message_type::kCodecHeader:
                {
                    if (this->state_ != StreamState::STREAMING){
                        continue;
                    }
                    CodecHeaderPayloadView codec_header_payload;
                    if( !codec_header_payload.bind( payload, payload_len) ){
                        return ESP_FAIL;
                    }
                    timed_chunk_t *timed_chunk = nullptr;
                    size_t size = codec_header_payload.payload_size;
                    ring_buffer->acquire_write_chunk(&timed_chunk, sizeof(timed_chunk_t) + size, pdMS_TO_TICKS(timeout_ms));
                    if (timed_chunk == nullptr) {
                        this->error_msg_ = "Error acquiring write chunk from ring buffer";
                        return ESP_FAIL;
                    }
                    timed_chunk->stamp = tv_t(0,0);
                    if (!codec_header_payload.copyPayloadTo(timed_chunk->data, size ))
                    {
                        this->error_msg_ = "Error copying codec header payload";
                        return ESP_FAIL;
                    }
                    ring_buffer->release_write_chunk(timed_chunk, size);
                    this->codec_header_sent_ = true;
                    return ESP_OK;
                }
                break;
            case message_type::kWireChunk:
                {
                    if( this->state_ != StreamState::STREAMING || !this->codec_header_sent_ ){
                          continue;
                    }
                    WireChunkMessageView wire_chunk_msg;
                    if( !wire_chunk_msg.bind(payload, payload_len) ){
                        this->error_msg_ = "Error binding wire chunk payload";
                        return ESP_FAIL;
                    }
                    if( !this->time_stats_.is_ready() ){
                        continue;
                    }
                    tv_t time_stamp = this->to_local_time_( tv_t(wire_chunk_msg.timestamp_sec, wire_chunk_msg.timestamp_usec));
                    if( time_stamp < tv_t::now() ){
                        //chunk is in the past, ignore it
                        printf( "chunk-read: skipping full frame: delta: %lld\n", time_stamp.to_millis() - tv_t::now().to_millis());
                        printf( "server-time: sec:%d, usec:%d\n", wire_chunk_msg.timestamp_sec, wire_chunk_msg.timestamp_usec);
                        printf( "local-time: sec:%d, usec:%d\n", time_stamp.sec, time_stamp.usec);                                                
                        continue;
                    }
                    static tv_t last_time_stamp = time_stamp;
                    if( (time_stamp - last_time_stamp).to_millis() > 24 ){
                        printf( "chunk-read: packet loss, diff: %" PRId64 " ms\n", (time_stamp - last_time_stamp).to_millis()  );
                    }
                    last_time_stamp = time_stamp;
                    
                    timed_chunk_t *timed_chunk = nullptr;
                    size_t size = wire_chunk_msg.payload_size;
                    ring_buffer->acquire_write_chunk(&timed_chunk, sizeof(timed_chunk_t) + size, pdMS_TO_TICKS(timeout_ms));
                    if (timed_chunk == nullptr) {
                        this->error_msg_ = "Error acquiring write chunk from ring buffer";
                        return ESP_FAIL;
                    }
                    timed_chunk->stamp = time_stamp;
                    if (!wire_chunk_msg.copyPayloadTo(timed_chunk->data, size))
                    {
                        this->error_msg_ = "Error copying wire chunk payload";
                        return ESP_FAIL;
                    }
                    ring_buffer->release_write_chunk(timed_chunk, size);
                    return ESP_OK;
                }
                break;
            case message_type::kTime:
                {
                  tv_t stamp;
                  std::memcpy(&stamp, payload, sizeof(stamp));
                    this->on_time_msg_(*msg, stamp);
                }
                break;
            case message_type::kServerSettings:
                {
                    ServerSettingsMessage server_settings_msg(*msg, payload, payload_len);
                    this->on_server_settings_msg_(server_settings_msg);
                    server_settings_msg.print();
                }
                break;
            
            default:
                this->error_msg_ = "Unknown message type: " + to_string(msg->type);
                return ESP_FAIL;
                
        }
    } // while loop
    return ERR_TIMEOUT;
}


void SnapcastStream::stream_task_(){
    constexpr TickType_t STREAMING_WAIT = pdMS_TO_TICKS(1);     
    constexpr TickType_t IDLE_WAIT = pdMS_TO_TICKS(100);        
    
    uint32_t notify_value;
    while( true ){
        //printf("Task-SnapcastStream High Water Mark: %lu\n", uxTaskGetStackHighWaterMark(nullptr));
        TickType_t wait_time = (this->state_ == StreamState::STREAMING) ? STREAMING_WAIT : IDLE_WAIT;
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &notify_value, wait_time)) {
            if (notify_value & static_cast<uint32_t>(StreamCommandBits::CONNECT)) {
                if( this->state_ == StreamState::DISCONNECTED ){
                    this->set_state_(StreamState::CONNECTING);
                }
            }
            else if (notify_value & static_cast<uint32_t>(StreamCommandBits::DISCONNECT)) {
                this->disconnect_();
            }
            else if (notify_value & static_cast<uint32_t>(StreamCommandBits::START_STREAM)) {
                this->start_streaming_();
            }
            else if (notify_value & static_cast<uint32_t>(StreamCommandBits::STOP_STREAM)) {
                this->stop_streaming_();
            }
            else if (notify_value & static_cast<uint32_t>(StreamCommandBits::SEND_REPORT)) {
                this->send_report_();
            }
        }
        switch (this->state_) {
            case StreamState::CONNECTING:
                this->connect_();
                break;
            case StreamState::CONNECTED_IDLE:
            case StreamState::STREAMING:
                this->send_time_sync_();
                if( this->read_and_process_messages_(200) == ESP_FAIL){
                    // if( this->reconnect_on_error_ ){
                    //      this->start_after_connecting_ = true;
                    // }
                    this->set_state_(StreamState::ERROR);
                }      
                break;
            case StreamState::ERROR:
                // Handle error state, possibly reconnect
                this->disconnect_();
                if( this->reconnect_on_error_ ){
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    this->set_state_(StreamState::CONNECTING);
                }
                break;
            default:
                break;
        }
    }
}


void SnapcastStream::set_state_(StreamState new_state){
    this->state_= new_state;
    if( this->notification_target_ != nullptr ){
        xTaskNotify(this->notification_target_, static_cast<uint32_t>(this->state_), eSetValueWithOverwrite);
    }
    if (this->on_status_update_) {
        this->on_status_update_(this->state_, 255, false); // 255 for volume means do not set
    } 
}

void SnapcastStream::connect_(){
    if( this->transport_ == nullptr ){
        this->transport_ = esp_transport_tcp_init();
        if (this->transport_ == nullptr) {
            this->error_msg_ = "Error while trying to initiate esp_transport";
            this->set_state_(StreamState::ERROR);
            return;
        }
    }
    esp_transport_keep_alive_t keep_alive_config = {
        .keep_alive_enable = true,
        .keep_alive_idle = 10000, // 10 seconds
        .keep_alive_interval = 5000, // 5 seconds
        .keep_alive_count = 5, // Number of keep-alive probes to send before considering the connection dead
    };
    esp_transport_tcp_set_keep_alive(this->transport_, &keep_alive_config);
    error_t err = esp_transport_connect(this->transport_, this->server_.c_str(), this->port_, CONNECTION_TIMEOUT_MS);
    if (err != ESP_OK) {
        if( this->reconnect_counter_ < 5 ){
            this->reconnect_counter_++;
            vTaskDelay(pdMS_TO_TICKS(5000));
            return;
        }
        this->error_msg_ = "Error while trying to connect to server";
        this->set_state_(StreamState::ERROR);
        return;
    }
    this->time_stats_.reset();
    this->send_hello_();
    this->set_state_(StreamState::CONNECTED_IDLE);
    if( this->start_after_connecting_ ){
        this->start_streaming_();
    }
}


void SnapcastStream::disconnect_(){
    if( this->transport_ != nullptr ){
        esp_transport_close(this->transport_);
        esp_transport_destroy(this->transport_);
        this->transport_ = nullptr;
    }
    this->set_state_(StreamState::DISCONNECTED);
    return;
}

void SnapcastStream::start_streaming_(){
    if( this->state_ == StreamState::STREAMING ){
        //this->error_msg_ = "Called start streaming in " + to_string(static_cast<int>(this->state_)) +  " state";
        //this->set_state_(StreamState::ERROR);
        printf( "Called start streaming in STREAMING state...\n");
        return;
    }
    if( this->state_ != StreamState::CONNECTED_IDLE ){
        this->start_after_connecting_ = true;
        return;
    }
    auto rb = this->write_ring_buffer_.lock();
    if( !rb ){
        this->error_msg_ = "Ringer buffer not set yet, but trying to start streaming...";
        this->set_state_(StreamState::ERROR);
        return;
    }
    this->codec_header_sent_=false;
    this->send_hello_();
    rb->reset();
    this->start_after_connecting_ = false;
    this->set_state_(StreamState::STREAMING);
    return;
}

void SnapcastStream::stop_streaming_(){
    if( this->state_ != StreamState::STREAMING ){
        //this->error_msg_ = "Called stop streaming in " + to_string(static_cast<int>(this->state_)) +  " state";
        //this->set_state_(StreamState::ERROR);
        return;
    }
    this->start_after_connecting_ = false;
    printf( "called stop streaming!\n");
    this->set_state_(StreamState::CONNECTED_IDLE);
}

void SnapcastStream::send_message_(SnapcastMessage &msg){
    assert( msg.getMessageSize() <= sizeof(tx_buffer));
    msg.set_send_time();
    msg.toBytes(tx_buffer);
    printf( "Sending message in state: %d\n",  static_cast<int>(this->state_));
    msg.print();
    int bytes_written = esp_transport_write( this->transport_, (char*) tx_buffer, msg.getMessageSize(), 0);
    if (bytes_written < 0) {
        this->error_msg_ = (
             "Error occurred during sending: esp_transport_write() returned " + to_string(bytes_written)
           + " bytes_written, errno " + to_string(errno)
        );
    }
}

void SnapcastStream::send_hello_(){
    HelloMessage hello_msg;
    this->send_message_(hello_msg);
}

void SnapcastStream::send_report_(){
    ClientInfoMessage msg(this->volume_, this->muted_);
    this->send_message_(msg);
}

void SnapcastStream::send_time_sync_(){
    uint32_t sync_interval = TIME_SYNC_INTERVAL_MS;
    if( !this->time_stats_.is_ready() ){
        sync_interval = 100;
    }
    if (millis() - this->last_time_sync_ > sync_interval){
        TimeMessage time_sync_msg; 
        this->send_message_(time_sync_msg);
        this->last_time_sync_ = millis();
    }
}

void SnapcastStream::on_time_msg_(MessageHeader msg, tv_t latency_c2s){
    //latency_c2s = t_server-recv - t_client-sent + t_network-latency
    //latency_s2c = t_client-recv - t_server-sent + t_network_latency
    //time diff between server and client as (latency_c2s - latency_s2c) / 2
    tv_t latency_s2c = tv_t::now() - msg.sent;
    time_stats_.add_offset( (latency_c2s - latency_s2c) / 2 );
    this->est_time_diff_ = time_stats_.get_estimate();
    
#if 1
    printf( "msg.sent: sec %d, usec: %d\n", msg.sent.sec, msg.sent.usec );
    printf( "latencey_c2s: %lld\n", latency_c2s.to_millis());
    printf( "latency_s2c: %lld = %lld - %lld\n", latency_s2c.to_millis(), tv_t::now().to_millis(), msg.sent.to_millis());    
    
    const int64_t server_time = (tv_t::now() + this->est_time_diff_).to_millis();
    static int64_t last_server_time = server_time;
    static uint32_t last_call = millis();
    
    int64_t expected_server_time = last_server_time + (millis() - last_call);
    int64_t server_diff = server_time - expected_server_time;

    last_server_time = server_time;
    last_call = millis();

    printf("New server time: %" PRId64 " (delta: %" PRId64 ") , expected %" PRId64 ", diff: %" PRId64 "\n",
         server_time, this->est_time_diff_.to_millis(), expected_server_time, server_diff);
#endif
    
}

void SnapcastStream::on_server_settings_msg_(const ServerSettingsMessage &msg){
    this->server_buffer_size_ = msg.buffer_ms_;
    this->latency_ = msg.latency_;
    this->volume_ = msg.volume_;
    this->muted_ = msg.muted_;
    if (this->on_status_update_) {
        this->on_status_update_(this->state_, this->volume_, this->muted_);
    } 
}




}
}