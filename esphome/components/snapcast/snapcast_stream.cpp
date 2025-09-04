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

extern "C" {
  #include <sys/select.h>
  #include <sys/socket.h>
  #include <sys/time.h>
  #include <sys/types.h>
  #include <unistd.h>
  #include <lwip/sockets.h>
}



#include "esp_mac.h"


namespace esphome {
namespace snapcast {

static const char *const TAG = "snapcast_stream";

static uint8_t tx_buffer[1024];
static uint8_t rx_buffer[4096];
static uint32_t rx_buffer_length = 0; 

static const uint8_t STREAM_TASK_PRIORITY = 14;
static const uint32_t CONNECTION_TIMEOUT_MS = 2000;
static const size_t TASK_STACK_SIZE = 4 * 1024;
static const uint32_t TIME_SYNC_INTERVAL_MS =  2000;

QueueHandle_t outgoing_queue = nullptr;

enum StreamTaskBits : uint32_t {
  // Command sent by the main controller logic to the transport task.
  // Tells the transport task to attempt a TCP connection.
  CONNECT_BIT               = (1 << 0),

  // Command sent by the main controller logic.
  // Tells the transport task to close the connection and stop.
  STOP_BIT                  = (1 << 1),
  
  // TASK is closing.
  TASK_CLOSING_BIT          = (1 << 2),
  
  // Status signal set by the transport task.
  // Notifies the stream task that the TCP connection was successfully established.
  CONNECTION_ESTABLISHED_BIT = (1 << 3),

  // Status signal set by the transport task.
  // Notifies the stream task that the transport task failed to establish a TCP connection.
  CONNECTION_FAILED_BIT     = (1 << 4),

  // Status signal set by the transport task.
  // Notifies the stream task that an existing connection was dropped or closed.
  CONNECTION_DROPPED_BIT    = (1 << 5),

  // Command sent by the controller logic.
  // Tells the stream task to begin the audio streaming loop.
  START_STREAM_BIT          = (1 << 6),

  // Command sent by the controller logic.
  // Tells the stream task to stop the audio streaming loop.
  STOP_STREAM_BIT           = (1 << 7),

  // Command sent by the controller logic.
  // Tells the stream task to immediately send a status report or sync message.
  SEND_REPORT_BIT           = (1 << 8),

  // Optional: Command sent by the controller logic.
  // Tells the stream task to perform a manual disconnect sequence.
  DISCONNECT_BIT            = (1 << 9),

  CONNECTION_CLOSED_BIT     = (1 << 10),  
  
};


typedef struct {
      void *data;
} esp_transport_item_t;

typedef struct {
        int sock;
} esp_transport_tcp_t;

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
          xTaskCreateStatic([](void *param) {
                auto *stream = static_cast<SnapcastStream *>(param);
                stream->stream_task_();
                stream->stream_task_handle_ = nullptr;
                vTaskDelete(nullptr);
            }, 
            "snap_stream_task", 
            TASK_STACK_SIZE, 
            (void *) this,
            STREAM_TASK_PRIORITY, 
            this->task_stack_buffer_, 
            &this->task_stack_
        );

        if (this->stream_task_handle_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create snapcast stream task.");
            this->stream_task_handle_ = nullptr;  // Ensure it's reset
            return ESP_FAIL;
        }
        
    }
    xTaskNotify( this->stream_task_handle_, CONNECT_BIT, eSetValueWithOverwrite);
    return ESP_OK;
}

esp_err_t SnapcastStream::disconnect(){
   // close connection and stop all running tasks
   xTaskNotify( this->stream_task_handle_, STOP_BIT, eSetValueWithOverwrite);
   return ESP_OK; 
}

esp_err_t SnapcastStream::start_with_notify(std::weak_ptr<esphome::TimedRingBuffer> ring_buffer, TaskHandle_t notification_task){
    ESP_LOGD(TAG, "Starting stream..." );
    this->write_ring_buffer_ = ring_buffer;
    this->notification_target_ = notification_task;
    xTaskNotify( this->stream_task_handle_, START_STREAM_BIT, eSetValueWithOverwrite);
    return ESP_OK;
}

esp_err_t SnapcastStream::stop_streaming(){
    xTaskNotify( this->stream_task_handle_, STOP_STREAM_BIT, eSetValueWithOverwrite);
    return ESP_OK;
}

esp_err_t SnapcastStream::report_volume(uint8_t volume, bool muted){
    if( volume != this->volume_ || muted_ != this->muted_){
        this->volume_ = volume;
        this->muted_ = muted;
        xTaskNotify( this->stream_task_handle_, SEND_REPORT_BIT, eSetValueWithOverwrite);
    }
    return ESP_OK;
}



static void transport_task_( 
                std::string server, 
                uint32_t port, 
                std::shared_ptr<ChunkedRingBuffer> ring_buffer, 
                TaskHandle_t stream_task_handle,
                TimeStats* time_stats ){
    
    constexpr size_t HEADER_SIZE = sizeof(MessageHeader);
    
    while( true ){
        uint32_t notify_value = 0;
        xTaskNotifyWait(0, CONNECT_BIT | STOP_BIT, &notify_value, portMAX_DELAY);
        if (notify_value & STOP_BIT) break;
        if (!(notify_value & CONNECT_BIT)) continue;
        
        
        // === Create socket and connect ===
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (sock < 0) {
            ESP_LOGE("transport", "Failed to create socket: errno %d", errno);
            xTaskNotify(stream_task_handle, CONNECTION_FAILED_BIT, eSetBits);
            continue;
        }
        
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(server.c_str());  // Make sure it's a numeric IP
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(port);
        
        int err = connect(sock, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE("transport", "Socket unable to connect: errno %d", errno);
            close(sock);
            xTaskNotify(stream_task_handle, CONNECTION_FAILED_BIT, eSetBits);
            continue;
        }
        
        // TCP_NODELAY
        int one = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
#ifdef TCP_KEEPIDLE
        int idle=30, intvl=5, cnt=3;
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE,  &idle, sizeof(idle));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT,   &cnt,   sizeof(cnt));
#endif
        int flags = lwip_fcntl(sock, F_GETFL, 0);
        lwip_fcntl(sock, F_SETFL, flags | O_NONBLOCK);

        // Notify the RX task that the connection is established
        xTaskNotify(stream_task_handle, CONNECTION_ESTABLISHED_BIT, eSetBits);
        rx_buffer_length = 0;
        bool time_set = false;
        while (true) {
            // Check for shutdown signal
            uint32_t notify_value;
            if (xTaskNotifyWait( 0, DISCONNECT_BIT, &notify_value, 0) > 0) {
                break;
            }

            if (!ring_buffer) {
                break;
            }
            size_t to_read = 0;

            // Determine what we need next
            if (rx_buffer_length < HEADER_SIZE) {
                to_read = HEADER_SIZE - rx_buffer_length;
            } else {
                MessageHeader* msg = reinterpret_cast<MessageHeader*>(rx_buffer);
                to_read = msg->getMessageSize() - rx_buffer_length;
            }

            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(sock, &read_fds);
            struct timeval timeout = {0, 500000};  // 500ms
            int sel = lwip_select(sock + 1, &read_fds, NULL, NULL, &timeout);
            if (sel < 0) {
                ESP_LOGE("transport", "select() error: errno %d", errno);
                break;
            } else if (sel == 0) {
                // Timeout, nothing ready to read
            } else if (FD_ISSET(sock, &read_fds)) {
                tv_t now = tv_t::now();

                int len = recv(sock, (char*)rx_buffer + rx_buffer_length, to_read, 0);
                if (len <= 0) {
                    // Handle reconnect or exit
                    xTaskNotify(stream_task_handle, CONNECTION_DROPPED_BIT, eSetBits);
                    break;
                }

                rx_buffer_length += len;

                // Do we have a full header yet?
                if (rx_buffer_length >= HEADER_SIZE) {
                    MessageHeader* msg = reinterpret_cast<MessageHeader*>(rx_buffer);

                    if( !time_set ){
                        msg->received = now;
                        time_set = true;
                    }

                    // Do we have a full message yet?
                    if (rx_buffer_length >= msg->getMessageSize()) {
                        // At this point, rx_buffer has a complete message

                        // Push raw message to ring buffer or queue
                        uint8_t* chunk = nullptr;
                        size_t total_size = msg->getMessageSize();
                        ring_buffer->acquire_write_chunk(&chunk, total_size, 0);
                        if (chunk) {
                            memcpy(chunk, rx_buffer, total_size);
                            ring_buffer->release_write_chunk(chunk, total_size);
                        } else {
                            // Ring buffer full! Dropping packet.
                        }
                        // Reset for next message
                        rx_buffer_length = 0;
                        time_set = false;
                    }
                }
            }

            SnapcastMessage* msg;
            if (xQueueReceive(outgoing_queue, &msg, 0) == pdPASS) {
                msg->set_send_time();
                msg->toBytes(tx_buffer);
                int bytes_written = send(sock, (char*) tx_buffer, msg->getMessageSize(), 0);
                if (bytes_written > 0 && msg->getMessageType() == message_type::kTime) {
                    time_stats->set_request_time(tv_t::now());
                }
                delete msg;  // Free the message after serialization
                
            }
        }
        close(sock);
        xTaskNotify(stream_task_handle, CONNECTION_CLOSED_BIT, eSetBits);
    }
    
    xTaskNotify(stream_task_handle, TASK_CLOSING_BIT, eSetBits);
}





esp_err_t SnapcastStream::read_and_process_messages_(ChunkedRingBuffer* read_ring_buffer, uint32_t timeout_ms){
    auto timed_ring_buffer = this->write_ring_buffer_.lock();    
    const uint32_t timeout = millis() + timeout_ms;
    
    if (!read_ring_buffer) {
        this->error_msg_ = "Read ring buffer is not available";
        return ESP_FAIL;
    }
    while( millis() < timeout ){
        size_t len = 0;
        uint8_t *chunk = read_ring_buffer->get_next_chunk(len, timeout_ms);
        if( len < sizeof(MessageHeader) || chunk == nullptr ){
            // invalid chunk, drop it
            if( chunk != nullptr ){
                read_ring_buffer->release_read_chunk(chunk);
            }
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        
        MessageHeader *msg = reinterpret_cast<MessageHeader *>(chunk);
        if( len < msg->getMessageSize() ){
            // incomplete message, drop it
            read_ring_buffer->release_read_chunk(chunk);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        
        uint8_t *payload = chunk + sizeof(MessageHeader); 
        size_t payload_len = msg->typed_message_size;
        switch( msg->getMessageType() ){
            case message_type::kCodecHeader:
                {
                    if (this->state_ != StreamState::STREAMING){
                        read_ring_buffer->release_read_chunk(chunk);
                        continue;
                    }
                    CodecHeaderPayloadView codec_header_payload;
                    if( !codec_header_payload.bind( payload, payload_len) ){
                        read_ring_buffer->release_read_chunk(chunk);
                        return ESP_FAIL;
                    }
                    timed_chunk_t *timed_chunk = nullptr;
                    size_t size = codec_header_payload.payload_size;
                    timed_ring_buffer->acquire_write_chunk(&timed_chunk, sizeof(timed_chunk_t) + size, pdMS_TO_TICKS(timeout_ms));
                    if (timed_chunk == nullptr) {
                        this->error_msg_ = "Error acquiring write chunk from ring buffer";
                        read_ring_buffer->release_read_chunk(chunk);
                        return ESP_FAIL;
                    }
                    timed_chunk->stamp = tv_t(0,0);
                    if (!codec_header_payload.copyPayloadTo(timed_chunk->data, size ))
                    {
                        timed_ring_buffer->release_write_chunk(timed_chunk, size);
                        this->error_msg_ = "Error copying codec header payload";
                        read_ring_buffer->release_read_chunk(chunk);
                        return ESP_FAIL;
                    }
                    timed_ring_buffer->release_write_chunk(timed_chunk, size);
                    this->codec_header_sent_ = true;
                    read_ring_buffer->release_read_chunk(chunk);
                    return ESP_OK;
                }
                break;
            case message_type::kWireChunk:
                {
                    if( this->state_ != StreamState::STREAMING || !this->codec_header_sent_ ){
                        read_ring_buffer->release_read_chunk(chunk);  
                        vTaskDelay(pdMS_TO_TICKS(5));
                        continue;
                    }
                    WireChunkMessageView wire_chunk_msg;
                    if( !wire_chunk_msg.bind(payload, payload_len) ){
                        this->error_msg_ = "Error binding wire chunk payload";
                        read_ring_buffer->release_read_chunk(chunk);
                        return ESP_FAIL;
                    }
                    if( !this->time_stats_.is_ready() ){
                        read_ring_buffer->release_read_chunk(chunk);
                        vTaskDelay(pdMS_TO_TICKS(5));
                        return ESP_OK; // wait for time stats to be ready, return for allowing to send sync requests
                    }
                    
                    tv_t time_stamp = this->to_local_time_( tv_t(wire_chunk_msg.timestamp_sec, wire_chunk_msg.timestamp_usec));
#if SNAPCAST_DEBUG                    
                    static tv_t last_time_stamp = time_stamp;
                    if( (time_stamp - last_time_stamp).to_millis() > 24 ){
                        printf( "chunk-read: stamp diff to last package: %" PRId64 " ms\n", (time_stamp - last_time_stamp).to_millis()  );
                    }
                    last_time_stamp = time_stamp;
#endif                    
                    if( time_stamp < tv_t::now() ){
                        //chunk is in the past, ignore it
#if SNAPCAST_DEBUG                        
                        printf( "chunk-read: skipping full frame: delta: %lld\n", time_stamp.to_millis() - tv_t::now().to_millis());
                        printf( "server-time: sec:%d, usec:%d\n", wire_chunk_msg.timestamp_sec, wire_chunk_msg.timestamp_usec);
                        printf( "local-time: sec:%d, usec:%d\n", time_stamp.sec, time_stamp.usec);
#endif                                                
                        read_ring_buffer->release_read_chunk(chunk);
                        vTaskDelay(pdMS_TO_TICKS(1));
                        continue;
                    }
                    
                    timed_chunk_t *timed_chunk = nullptr;
                    size_t size = wire_chunk_msg.payload_size;
                    timed_ring_buffer->acquire_write_chunk(&timed_chunk, sizeof(timed_chunk_t) + size, pdMS_TO_TICKS(timeout_ms));
                    if (timed_chunk == nullptr) {
                        this->error_msg_ = "Error acquiring write chunk from ring buffer";
                        read_ring_buffer->release_read_chunk(chunk);
                        return ESP_FAIL;
                    }
                    timed_chunk->stamp = time_stamp;
                    if (!wire_chunk_msg.copyPayloadTo(timed_chunk->data, size))
                    {
                        timed_ring_buffer->release_write_chunk(timed_chunk, size);
                        this->error_msg_ = "Error copying wire chunk payload";
                        read_ring_buffer->release_read_chunk(chunk);
                        return ESP_FAIL;
                    }
                    timed_ring_buffer->release_write_chunk(timed_chunk, size);
                    read_ring_buffer->release_read_chunk(chunk);
                    return ESP_OK;
                }
                break;
            case message_type::kTime:
                {
                  tv_t stamp;
                  std::memcpy(&stamp, payload, sizeof(stamp));
                  this->on_time_msg_(*msg, stamp);
                }
                read_ring_buffer->release_read_chunk(chunk);
                continue;
            case message_type::kServerSettings:
                {
                    ServerSettingsMessage server_settings_msg(*msg, payload, payload_len);
                    this->on_server_settings_msg_(server_settings_msg);
                    //server_settings_msg.print();
                }
                read_ring_buffer->release_read_chunk(chunk);
                continue;
            
            default:
                this->error_msg_ = "Unknown message type: " + to_string(msg->type);
                read_ring_buffer->release_read_chunk(chunk);
                return ESP_FAIL;
                
        }
    } // while loop
    return ERR_TIMEOUT;
}


void SnapcastStream::stream_task_(){
    
    std::shared_ptr<ChunkedRingBuffer> stream_package_buffer = ChunkedRingBuffer::create(256 * 1024);
    
    if( stream_package_buffer == nullptr ){
        ESP_LOGE(TAG, "Failed to create stream package buffer!");
        return;
    }

    // For example: allow up to 10 pending messages
    outgoing_queue = xQueueCreate(10, sizeof(SnapcastMessage*));

    if (outgoing_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create outgoing queue!");
        return;
    }

    struct ReadTaskArgs {
        std::shared_ptr<ChunkedRingBuffer> buffer;
        std::string server;
        uint32_t port;
        TaskHandle_t stream_task_handle;
        TimeStats *time_stats_; 
    };

    ReadTaskArgs *args = new ReadTaskArgs();
    args->buffer = stream_package_buffer;
    args->server = this->server_;
    args->port = this->port_;
    args->stream_task_handle = xTaskGetCurrentTaskHandle();
    args->time_stats_ = &this->time_stats_; // Pass the time stats reference

    TaskHandle_t transport_task_handle = nullptr;
    BaseType_t result = xTaskCreatePinnedToCore([](void *param) {
            auto *args = static_cast<ReadTaskArgs*>(param);
            transport_task_(args->server, args->port, args->buffer, args->stream_task_handle, args->time_stats_);
            delete args;
            vTaskDelete(nullptr);  // Task cleans itself up after loop exits
        },
        "snapcast_tx_rx",      // Task name
        4096,                  // Stack size in words
        args,                  // param: your 'this' pointer
        16,                    // Priority
        &transport_task_handle, // Store handle if you want to kill it later
        1
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Snapcast RX/TX task!");
        return;
    }

    constexpr TickType_t STREAMING_WAIT = pdMS_TO_TICKS(5);     
    constexpr TickType_t IDLE_WAIT = pdMS_TO_TICKS(100);        
    
    uint32_t notify_value;
    while( true ){
        TickType_t wait_time = (this->state_ == StreamState::STREAMING) ? STREAMING_WAIT : IDLE_WAIT;
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &notify_value, wait_time)) {
            if (notify_value & CONNECT_BIT) {
                // Tell transport to connect
                this->set_state_(StreamState::CONNECTING);     
                xTaskNotify(transport_task_handle, CONNECT_BIT, eSetBits);
            }

            if (notify_value & STOP_BIT) {
                this->set_state_(StreamState::STOPPING);     
                xTaskNotify(transport_task_handle, STOP_BIT, eSetBits);
            }
            
            if (notify_value & START_STREAM_BIT) {
                this->start_streaming_();
            }
            
            if (notify_value & STOP_STREAM_BIT) {
                this->stop_streaming_();
            }
            
            if (notify_value & SEND_REPORT_BIT) {
                if( this->state_ != StreamState::DISCONNECTED && outgoing_queue != nullptr) {
                    this->send_report_();
                }
            }
            
            if (notify_value & CONNECTION_ESTABLISHED_BIT) {
                this->send_hello_();
                this->time_stats_.reset();
                set_state_(StreamState::CONNECTED_IDLE);
                if( this->start_after_connecting_ ){
                    this->start_streaming_();
                }
            }
            
            if( notify_value & CONNECTION_CLOSED_BIT) {
                this->set_state_(StreamState::DISCONNECTED);
            }
            
            if (notify_value & TASK_CLOSING_BIT) {
                // tx_rx task is closing, we can exit the loop
                this->set_state_(StreamState::DISCONNECTED);
                break;
            } 

            if (notify_value & CONNECTION_FAILED_BIT || notify_value & CONNECTION_DROPPED_BIT) {
                if( this->reconnect_on_error_ ){
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    this->set_state_(StreamState::CONNECTING);
                    xTaskNotify(transport_task_handle, CONNECT_BIT, eSetBits);
                } else {
                    this->error_msg_ = "Failed to connect or connection dropped";
                    this->set_state_(StreamState::ERROR);
                }
            }
        }
        
        if(this->state_ == StreamState::CONNECTED_IDLE || this->state_ == StreamState::STREAMING){
            this->send_time_sync_();
            if(this->read_and_process_messages_(stream_package_buffer.get(), 500) == ESP_FAIL){
                if( this->reconnect_on_error_ ){
                    this->start_after_connecting_ = true;
                    xTaskNotify(transport_task_handle, DISCONNECT_BIT | CONNECT_BIT, eSetBits);  
                } else {
                    this->set_state_(StreamState::ERROR);
                    this->error_msg_ = "Error reading or processing messages";
                }
            }      
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


void SnapcastStream::start_streaming_(){
    if( this->state_ == StreamState::STREAMING ){
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
        return;
    }
    this->start_after_connecting_ = false;
    this->set_state_(StreamState::CONNECTED_IDLE);
}

void SnapcastStream::send_message_(SnapcastMessage *msg){
    assert( msg->getMessageSize() <= sizeof(tx_buffer));
    if (xQueueSend(outgoing_queue, &msg, 0) != pdPASS) {
        delete msg; // Clean up if failed
    }
}

void SnapcastStream::send_hello_(){
    SnapcastMessage* hello_msg = new HelloMessage();
    this->send_message_(hello_msg);
}

void SnapcastStream::send_report_(){
    SnapcastMessage* msg = new ClientInfoMessage(this->volume_, this->muted_);
    this->send_message_(msg);
}

void SnapcastStream::send_time_sync_(){
    uint32_t sync_interval = TIME_SYNC_INTERVAL_MS;
    if( !this->time_stats_.is_ready() ){
        sync_interval = 100;
    }
    if (millis() - this->last_time_sync_ > sync_interval){
        SnapcastMessage* time_sync_msg = new TimeMessage(); 
        this->send_message_(time_sync_msg);
        this->last_time_sync_ = millis();
    }
}

void SnapcastStream::on_time_msg_(MessageHeader msg, tv_t latency_c2s){
    //latency_c2s = t_server-recv - t_client-sent + t_network-latency
    //latency_s2c = t_client-recv - t_server-sent + t_network_latency
    //time diff between server and client as (latency_c2s - latency_s2c) / 2
    tv_t latency_s2c = msg.received - msg.sent;
    time_stats_.add_offset( (latency_c2s - latency_s2c) / 2, msg.received );
    this->est_time_diff_ = time_stats_.get_estimate();
    
#if SNAPCAST_DEBUG
    printf( "msg.sent: sec %d, usec: %d\n", msg.sent.sec, msg.sent.usec );
    printf( "latencey_c2s: %lld\n", latency_c2s.to_millis());
    printf( "latency_s2c: %lld = %lld - %lld\n", latency_s2c.to_millis(), msg.received.to_millis(), msg.sent.to_millis());    
    
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