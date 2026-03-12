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
#include "esphome/core/helpers.h"

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

static const int32_t TX_BUFFER_SIZE = 1024;
static const int32_t RX_BUFFER_SIZE = 4096;

static const uint8_t STREAM_TASK_PRIORITY = 14;
static const uint32_t CONNECTION_TIMEOUT_MS = 2000;
static const size_t TASK_STACK_SIZE = 4 * 1024;
static const uint32_t TIME_SYNC_INTERVAL_MS = 2000;

enum StreamTaskBits : uint32_t {
  // Command sent by the main controller logic to the transport task.
  // Tells the transport task to attempt a TCP connection.
  CONNECT_BIT = (1 << 0),

  // Command sent by the main controller logic.
  // Tells the transport task to close the connection and stop.
  STOP_BIT = (1 << 1),

  // TASK is closing.
  TASK_CLOSING_BIT = (1 << 2),

  // Status signal set by the transport task.
  // Notifies the stream task that the TCP connection was successfully established.
  CONNECTION_ESTABLISHED_BIT = (1 << 3),

  // Status signal set by the transport task.
  // Notifies the stream task that the transport task failed to establish a TCP connection.
  CONNECTION_FAILED_BIT = (1 << 4),

  // Status signal set by the transport task.
  // Notifies the stream task that an existing connection was dropped or closed.
  CONNECTION_DROPPED_BIT = (1 << 5),

  // Command sent by the controller logic.
  // Tells the stream task to begin the audio streaming loop.
  START_STREAM_BIT = (1 << 6),

  // Command sent by the controller logic.
  // Tells the stream task to stop the audio streaming loop.
  STOP_STREAM_BIT = (1 << 7),

  // Command sent by the controller logic.
  // Tells the stream task to immediately send a status report or sync message.
  SEND_REPORT_BIT = (1 << 8),

  // Optional: Command sent by the controller logic.
  // Tells the stream task to perform a manual disconnect sequence.
  DISCONNECT_BIT = (1 << 9),

  CONNECTION_CLOSED_BIT = (1 << 10),

};

typedef struct {
  void *data;
} esp_transport_item_t;

typedef struct {
  int sock;
} esp_transport_tcp_t;

esp_err_t SnapcastStream::init() {
  if (this->stream_task_handle_ != nullptr) {
    if (!this->stream_task_exiting_) {
      // stream task already running
      return ESP_OK;
    }
    if (!this->finalize_termination()) {
      ESP_LOGW(TAG, "Exiting stream task still in progress");
      return ESP_ERR_INVALID_STATE;
    }
  }

  if (this->task_stack_buffer_ == nullptr) {
    RAMAllocator<StackType_t> stack_allocator(RAMAllocator<StackType_t>::ALLOC_INTERNAL);
    this->task_stack_buffer_ = stack_allocator.allocate(TASK_STACK_SIZE);
    if (this->task_stack_buffer_ == nullptr) {
      this->set_state_(StreamState::ERROR);
      return ESP_ERR_NO_MEM;
    }
  }

  if (!this->config_mutex_) {
    this->config_mutex_ = xSemaphoreCreateMutex();
    if (!this->config_mutex_)
      return ESP_FAIL;
  }

  this->set_state_(StreamState::STARTING);

  this->stream_task_handle_ = xTaskCreateStatic(
      [](void *param) {
        auto *stream = static_cast<SnapcastStream *>(param);
        stream->stream_task_();
        vTaskDelete(nullptr);
      },
      "snap_stream_task", TASK_STACK_SIZE, (void *) this, STREAM_TASK_PRIORITY, this->task_stack_buffer_,
      &this->task_stack_);
  if (this->stream_task_handle_ == nullptr) {
    this->set_state_(StreamState::ERROR);
    return ESP_FAIL;
  }

  this->stream_task_exiting_ = false;
  return ESP_OK;
}

esp_err_t SnapcastStream::terminate() {
  this->want_connected_ = false;

  // close connection and stop all running tasks
  if (this->stream_task_handle_) {
    this->stream_task_exiting_ = true;
    this->set_state_(StreamState::STOPPING);
    xTaskNotify(this->stream_task_handle_, STOP_BIT, eSetBits);
  }
  return ESP_OK;
}

bool SnapcastStream::finalize_termination() {
  if (this->stream_task_handle_ == nullptr) {
    return true;
  }
  auto st = eTaskGetState(this->stream_task_handle_);
  if (st != eDeleted && st != eInvalid) {
    // task hasn't terminated yet
    return false;
  }

  // TODO: also free stack memory

  this->stream_task_handle_ = nullptr;
  this->stream_task_exiting_ = false;
  this->set_state_(StreamState::DESTROYED);
  return true;
}

esp_err_t SnapcastStream::connect(const std::string &server, uint32_t port) {
  if (!this->stream_task_handle_ || this->stream_task_exiting_) {
    return ESP_ERR_INVALID_STATE;
  }

  {
    if (xSemaphoreTake(this->config_mutex_, 0) != pdTRUE) {
      return ESP_ERR_TIMEOUT;
    }
    this->server_ = server;
    this->port_ = port;
    this->want_connected_ = true;
    xSemaphoreGive(this->config_mutex_);
  }

  xTaskNotify(this->stream_task_handle_, CONNECT_BIT, eSetBits);
  return ESP_OK;
}

esp_err_t SnapcastStream::disconnect() {
  if (!this->stream_task_handle_ || this->stream_task_exiting_) {
    return ESP_ERR_INVALID_STATE;
  }

  this->want_connected_ = false;
  xTaskNotify(this->stream_task_handle_, DISCONNECT_BIT, eSetBits);
  return ESP_OK;
}

esp_err_t SnapcastStream::start_with_notify(std::weak_ptr<esphome::TimedRingBuffer> ring_buffer,
                                            TaskHandle_t notification_task) {
  if (!this->stream_task_handle_ || this->stream_task_exiting_) {
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGD(TAG, "Starting stream...");
  this->write_ring_buffer_ = ring_buffer;
  this->notification_target_ = notification_task;
  xTaskNotify(this->stream_task_handle_, START_STREAM_BIT, eSetBits);
  return ESP_OK;
}

esp_err_t SnapcastStream::stop_streaming() {
  if (!this->stream_task_handle_ || this->stream_task_exiting_) {
    return ESP_ERR_INVALID_STATE;
  }
  ESP_LOGD(TAG, "Stop streaming called...");
  xTaskNotify(this->stream_task_handle_, STOP_STREAM_BIT, eSetBits);
  return ESP_OK;
}

esp_err_t SnapcastStream::report_volume(uint8_t volume, bool muted) {
  if (!this->stream_task_handle_ || this->stream_task_exiting_) {
    return ESP_ERR_INVALID_STATE;
  }
  const uint8_t old_vol = this->volume_.load(std::memory_order_relaxed);
  const bool old_muted = this->muted_.load(std::memory_order_relaxed);

  if (volume != old_vol || muted != old_muted) {
    this->volume_.store(volume, std::memory_order_relaxed);
    this->muted_.store(muted, std::memory_order_relaxed);
    xTaskNotify(this->stream_task_handle_, SEND_REPORT_BIT, eSetBits);
  }
  return ESP_OK;
}

static void transport_task_(SnapcastStream *self, std::shared_ptr<ChunkedRingBuffer> ring_buffer,
                            TaskHandle_t stream_task_handle, TimeStats *time_stats, QueueHandle_t outgoing_queue) {
  constexpr size_t HEADER_SIZE = sizeof(MessageHeader);
  constexpr size_t MAX_ALLOWED_MSG_SIZE = 8 * 1024;
  RAMAllocator<uint8_t> psram_allocator;
  auto *tx_buffer = psram_allocator.allocate(TX_BUFFER_SIZE);
  if (!tx_buffer) {
    ESP_LOGE("transport", "Failed to allocate buffers");
    xTaskNotify(stream_task_handle, TASK_CLOSING_BIT, eSetBits);
    return;
  }

  // Small header staging buffer (only header lives outside ringbuffer)
  uint8_t header_buf[HEADER_SIZE];
  size_t header_have = 0;

  // Current message assembly state
  uint8_t *rb_chunk = nullptr;
  size_t msg_size = 0;  // total bytes (header+payload)
  size_t msg_have = 0;  // bytes already written into rb_chunk
  bool have_rx_stamp = false;
  tv_t rx_stamp;  // captured as early as possible per message

  // If we can't store a message (oversize / ringbuffer full), we discard bytes.
  bool discarding = false;
  size_t discard_remaining = 0;
  uint8_t discard_buf[128];

  bool stop_requested = false;
  bool reconnect_requested = false;
  while (!stop_requested) {
    uint32_t notify_value = 0;
    xTaskNotifyWait(0, 0xFFFFFFFFUL, &notify_value, portMAX_DELAY);
    if (notify_value & STOP_BIT) {
      break;
    }
    if (!(notify_value & CONNECT_BIT)) {
      continue;
    }

    std::string server;
    uint32_t port;
    self->get_target_snapshot_(server, port);

    if (server.empty() || port == 0) {
      // nothing to connect to yet
      continue;
    }

    // === Create socket and connect ===
    int sock = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
      ESP_LOGE("transport", "Failed to create socket: errno %d", errno);
      xTaskNotify(stream_task_handle, CONNECTION_CLOSED_BIT, eSetBits);
      continue;
    }

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(server.c_str());  // Make sure it's a numeric IP
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);

    int err = connect(sock, (struct sockaddr *) &dest_addr, sizeof(dest_addr));
    if (err != 0) {
      ESP_LOGE("transport", "Socket unable to connect to %s: errno %d", server.c_str(), errno);
      close(sock);
      xTaskNotify(stream_task_handle, CONNECTION_CLOSED_BIT, eSetBits);
      continue;
    }

    // TCP_NODELAY
    int one = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));

    int flags = lwip_fcntl(sock, F_GETFL, 0);
    lwip_fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    // Notify the RX task that the connection is established
    xTaskNotify(stream_task_handle, CONNECTION_ESTABLISHED_BIT, eSetBits);

    header_have = 0;
    rb_chunk = nullptr;
    msg_size = 0;
    msg_have = 0;
    discarding = false;
    discard_remaining = 0;
    have_rx_stamp = false;

    while (true) {
      // Check for shutdown signal
      uint32_t notify_value = 0;
      if (xTaskNotifyWait(0, DISCONNECT_BIT | STOP_BIT | CONNECT_BIT, &notify_value, 0) == pdTRUE) {
        if (notify_value & STOP_BIT) {
          stop_requested = true;
          break;
        }
        if (notify_value & DISCONNECT_BIT) {
          break;
        }
        if (notify_value & CONNECT_BIT) {
          std::string new_server;
          uint32_t new_port;
          self->get_target_snapshot_(new_server, new_port);

          if (new_server != server || new_port != port) {
            ESP_LOGI("transport", "Target changed, reconnecting to %s:%lu", new_server.c_str(),
                     (unsigned long) new_port);
            reconnect_requested = true;
            break;
          }
        }
      }

      if (!ring_buffer) {
        ESP_LOGE("transport", "ring_buffer_not_set");
        break;
      }

      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(sock, &read_fds);
      struct timeval timeout = {0, 500000};  // 500ms
      int sel = lwip_select(sock + 1, &read_fds, NULL, NULL, &timeout);
      if (sel < 0) {
        ESP_LOGE("transport", "select() error: errno %d", errno);
        break;
      }

      if (sel > 0 && FD_ISSET(sock, &read_fds)) {
        // Capture timestamp ASAP when we learn "readable"
        if (!have_rx_stamp) {
          rx_stamp = tv_t::now();
          have_rx_stamp = true;
        }
        int64_t t0 = esp_timer_get_time();  // us
        const int64_t budget_us = 2000;     // 2ms
        while (true) {
          if (esp_timer_get_time() - t0 > budget_us) {
            break;  // time budget is up, leave inner loop even if more data is queued
          }
          // If we are discarding a too-large / unbufferable message, just drain bytes.
          if (discarding) {
            if (discard_remaining == 0) {
              discarding = false;
              // reset for next header
              header_have = 0;
              have_rx_stamp = false;
              continue;
            }

            const size_t want = std::min(discard_remaining, sizeof(discard_buf));
            int r = lwip_recv(sock, (char *) discard_buf, want, 0);
            if (r > 0) {
              discard_remaining -= (size_t) r;
              continue;
            }
            if (r == 0) {
              ESP_LOGI("transport", "recv() EOF (peer closed)");
              goto connection_done;
            }
            int e = errno;
            if (e == EAGAIN || e == EWOULDBLOCK)
              break;
            ESP_LOGE("transport", "recv(discard) error: errno=%d (%s)", e, strerror(e));
            goto connection_done;
          }

          // If we don't have a ringbuffer chunk yet, we must be reading header.
          if (rb_chunk == nullptr) {
            if (header_have < HEADER_SIZE) {
              int r = lwip_recv(sock, (char *) (header_buf + header_have), HEADER_SIZE - header_have, 0);
              if (r > 0) {
                header_have += (size_t) r;
                // keep looping; maybe more data available
              } else if (r == 0) {
                ESP_LOGI("transport", "recv() EOF (peer closed)");
                goto connection_done;
              } else {
                int e = errno;
                if (e == EAGAIN || e == EWOULDBLOCK)
                  break;  // no more for now
                ESP_LOGE("transport", "recv(header) error: errno=%d (%s)", e, strerror(e));
                goto connection_done;
              }
            }

            // Header complete? Acquire ringbuffer chunk for whole message.
            if (header_have == HEADER_SIZE) {
              auto *hdr = reinterpret_cast<MessageHeader *>(header_buf);
              msg_size = hdr->getMessageSize();

              // Basic sanity (avoid absurd sizes / corrupted header)
              if (msg_size < HEADER_SIZE || msg_size > MAX_ALLOWED_MSG_SIZE) {
                ESP_LOGW("transport", "Invalid msg_size=%u, discarding", (unsigned) msg_size);
                // Header seems to be corrupted, reconnect
                goto connection_done;
              }

              // Acquire a chunk of exactly msg_size
              ring_buffer->acquire_write_chunk(&rb_chunk, msg_size, 0);
              if (!rb_chunk) {
                // ringbuffer full; discard the remainder of this message (body bytes)
                ESP_LOGW("transport", "Ringbuffer full, discarding message (%u bytes)", (unsigned) msg_size);
                discarding = true;
                discard_remaining = msg_size - HEADER_SIZE;
                header_have = 0;
                have_rx_stamp = false;
                continue;
              }

              // Copy header into ringbuffer chunk
              memcpy(rb_chunk, header_buf, HEADER_SIZE);
              msg_have = HEADER_SIZE;

              // Apply the captured "readable" timestamp to the header
              auto *out_hdr = reinterpret_cast<MessageHeader *>(rb_chunk);
              out_hdr->received = rx_stamp;

              // Clear header staging for next message
              header_have = 0;
              // Keep have_rx_stamp true until we finish this message
            }

            // If we just acquired chunk, fall through and start reading body
            if (rb_chunk == nullptr) {
              // still collecting header
              continue;
            }
          }

          // We have rb_chunk => read body directly into it
          const size_t remaining = msg_size - msg_have;
          if (remaining == 0) {
            // Completed message; publish it
            ring_buffer->release_write_chunk(rb_chunk, msg_size);
            rb_chunk = nullptr;
            msg_size = 0;
            msg_have = 0;
            have_rx_stamp = false;  // next message gets its own stamp
            continue;
          }

          int r = lwip_recv(sock, (char *) (rb_chunk + msg_have), remaining, 0);
          if (r > 0) {
            msg_have += (size_t) r;
            if (msg_have == msg_size) {
              ring_buffer->release_write_chunk(rb_chunk, msg_size);
              rb_chunk = nullptr;
              msg_size = 0;
              msg_have = 0;
              have_rx_stamp = false;
            }
            continue;  // maybe more data still available
          }

          if (r == 0) {
            ESP_LOGI("transport", "recv() EOF (peer closed)");
            goto connection_done;
          }

          int e = errno;
          if (e == EAGAIN || e == EWOULDBLOCK) {
            // no more data right now; keep partial message state and return to select()
            break;
          }

          int so_err = 0;
          socklen_t sl = sizeof(so_err);
          getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_err, &sl);
          ESP_LOGE("transport", "recv(body) errno=%d (%s), SO_ERROR=%d (%s)", errno, strerror(errno), so_err,
                   strerror(so_err));
          goto connection_done;
        }
        taskYIELD();
      }

      SnapcastMessage *msg;
      if (xQueueReceive(outgoing_queue, &msg, 0) == pdPASS) {
        msg->set_send_time();
        msg->toBytes(tx_buffer);
        const size_t want = msg->getMessageSize();
        int bytes_written = send(sock, (char *) tx_buffer, want, 0);
        if (bytes_written < 0) {
          int e = errno;
          ESP_LOGE("transport", "send() failed: errno=%d (%s)", e, strerror(e));
          if (msg->getMessageType() == message_type::kHello) {
            // todo resend hello
          }
          delete msg;
          if (e == EAGAIN || e == EWOULDBLOCK) {
            // couldn't send right now;
            goto connection_done;
          }
          goto connection_done;  // fatal send error
        }
        if ((size_t) bytes_written != want) {
          ESP_LOGE("transport", "partial send: %d/%u -> reconnect", bytes_written, (unsigned) want);
          delete msg;
          goto connection_done;
        }

        if (bytes_written > 0 && msg->getMessageType() == message_type::kTime) {
          time_stats->set_request_time(msg->id(), msg->send_time());
        }
        delete msg;  // Free the message after serialization
      }
    }  // connected loop
  connection_done:
    if (rb_chunk) {
      auto *h = reinterpret_cast<MessageHeader *>(rb_chunk);
      h->type = static_cast<uint16_t>(message_type::kBase);  // receiver will drop
      h->typed_message_size = 0;
      ring_buffer->release_write_chunk(rb_chunk, msg_size);
      rb_chunk = nullptr;
    }
    close(sock);
    xTaskNotify(stream_task_handle, CONNECTION_CLOSED_BIT, eSetBits);
    if (stop_requested)
      break;
    if (reconnect_requested) {
      reconnect_requested = false;
      vTaskDelay(pdMS_TO_TICKS(100));
      xTaskNotify(xTaskGetCurrentTaskHandle(), CONNECT_BIT, eSetBits);
    }
  }
  psram_allocator.deallocate(tx_buffer, TX_BUFFER_SIZE);
  xTaskNotify(stream_task_handle, TASK_CLOSING_BIT, eSetBits);
}

esp_err_t SnapcastStream::read_and_process_messages_(ChunkedRingBuffer *read_ring_buffer, uint32_t timeout_ms) {
  auto timed_ring_buffer = this->write_ring_buffer_.lock();
  const uint32_t timeout = millis() + timeout_ms;

  if (!read_ring_buffer) {
    this->error_msg_ = "Read ring buffer is not available";
    return ESP_FAIL;
  }

  while (millis() < timeout) {
    size_t len = 0;
    uint8_t *chunk = read_ring_buffer->get_next_chunk(len, pdMS_TO_TICKS(timeout_ms));
    if (len < sizeof(MessageHeader) || chunk == nullptr) {
      // invalid chunk, drop it
      if (chunk != nullptr) {
        read_ring_buffer->release_read_chunk(chunk);
      }
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    MessageHeader *msg = reinterpret_cast<MessageHeader *>(chunk);
    if (msg->getMessageType() == message_type::kBase || len < msg->getMessageSize()) {
      // incomplete message, drop it
      read_ring_buffer->release_read_chunk(chunk);
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    uint8_t *payload = chunk + sizeof(MessageHeader);
    size_t payload_len = msg->typed_message_size;
    switch (msg->getMessageType()) {
      case message_type::kCodecHeader: {
        CodecHeaderPayloadView codec_header_payload;
        if (!codec_header_payload.bind(payload, payload_len)) {
          read_ring_buffer->release_read_chunk(chunk);
          return ESP_FAIL;
        }
        if (this->codec_header_) {
          free(this->codec_header_);
        }
        size_t size = codec_header_payload.payload_size;
        RAMAllocator<uint8_t> psram_allocator;
        this->codec_header_ = psram_allocator.allocate(size);
        this->codec_header_size_ = size;
        if (!codec_header_payload.copyPayloadTo(this->codec_header_, size)) {
          free(this->codec_header_);
          this->codec_header_ = nullptr;
          this->error_msg_ = "Error copying codec header payload";
          read_ring_buffer->release_read_chunk(chunk);
          return ESP_FAIL;
        }
        read_ring_buffer->release_read_chunk(chunk);
        return ESP_OK;
      } break;
      case message_type::kWireChunk: {
        if (this->state_ == StreamState::STREAMING && this->codec_header_ && !this->codec_header_sent_) {
          timed_chunk_t *timed_chunk = nullptr;
          timed_ring_buffer->acquire_write_chunk(&timed_chunk, sizeof(timed_chunk_t) + this->codec_header_size_,
                                                 pdMS_TO_TICKS(timeout_ms));
          if (timed_chunk == nullptr) {
            this->error_msg_ = "Error acquiring write chunk from ring buffer";
            read_ring_buffer->release_read_chunk(chunk);
            return ESP_FAIL;
          }
          timed_chunk->stamp = tv_t(0, 0);
          if (!std::memcpy(timed_chunk->data, this->codec_header_, this->codec_header_size_)) {
            timed_ring_buffer->release_write_chunk(timed_chunk, this->codec_header_size_);
            this->error_msg_ = "Error copying codec header payload";
            read_ring_buffer->release_read_chunk(chunk);
            return ESP_FAIL;
          }
          timed_ring_buffer->release_write_chunk(timed_chunk, this->codec_header_size_);
          this->codec_header_sent_ = true;
        }
        if (this->state_ != StreamState::STREAMING || !this->codec_header_sent_ || !this->time_stats_.is_ready()) {
          read_ring_buffer->release_read_chunk(chunk);
          vTaskDelay(pdMS_TO_TICKS(5));
          continue;
        }
        WireChunkMessageView wire_chunk_msg;
        if (!wire_chunk_msg.bind(payload, payload_len)) {
          this->error_msg_ = "Error binding wire chunk payload";
          read_ring_buffer->release_read_chunk(chunk);
          return ESP_FAIL;
        }
        if (!this->time_stats_.is_ready()) {
          read_ring_buffer->release_read_chunk(chunk);
          vTaskDelay(pdMS_TO_TICKS(5));
          return ESP_OK;  // wait for time stats to be ready, return for allowing to send sync requests
        }

        tv_t time_stamp = this->to_local_time_(tv_t(wire_chunk_msg.timestamp_sec, wire_chunk_msg.timestamp_usec));
#if SNAPCAST_DEBUG
        static tv_t last_time_stamp = time_stamp;
        if ((time_stamp - last_time_stamp).to_millis() > 24) {
          printf("chunk-read: stamp diff to last package: %" PRId64 " ms\n",
                 (time_stamp - last_time_stamp).to_millis());
        }
        last_time_stamp = time_stamp;
#endif
        if (time_stamp < tv_t::now()) {
          // chunk is in the past, ignore it
#if SNAPCAST_DEBUG
          printf("chunk-read: skipping full frame: delta: %lld\n", time_stamp.to_millis() - tv_t::now().to_millis());
          printf("server-time: sec:%d, usec:%d\n", wire_chunk_msg.timestamp_sec, wire_chunk_msg.timestamp_usec);
          printf("local-time: sec:%d, usec:%d\n", time_stamp.sec, time_stamp.usec);
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
        if (!wire_chunk_msg.copyPayloadTo(timed_chunk->data, size)) {
          timed_ring_buffer->release_write_chunk(timed_chunk, size);
          this->error_msg_ = "Error copying wire chunk payload";
          read_ring_buffer->release_read_chunk(chunk);
          return ESP_FAIL;
        }
        timed_ring_buffer->release_write_chunk(timed_chunk, size);
        read_ring_buffer->release_read_chunk(chunk);
        return ESP_OK;
      } break;
      case message_type::kTime: {
        tv_t stamp;
        std::memcpy(&stamp, payload, sizeof(stamp));
        this->on_time_msg_(*msg, stamp);
      }
        read_ring_buffer->release_read_chunk(chunk);
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
      case message_type::kServerSettings: {
        ServerSettingsMessage server_settings_msg(*msg, payload, payload_len);
        this->on_server_settings_msg_(server_settings_msg);
        read_ring_buffer->release_read_chunk(chunk);
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
      }

      default:
        this->error_msg_ = "Unknown message type: " + to_string(msg->type);
        read_ring_buffer->release_read_chunk(chunk);
        return ESP_FAIL;
    }
  }  // while loop
  return ERR_TIMEOUT;
}

void SnapcastStream::stream_task_() {
  std::shared_ptr<ChunkedRingBuffer> stream_package_buffer = ChunkedRingBuffer::create(256 * 1024);

  auto on_exit_task = [&]() {
    this->stream_task_exiting_ = true;
    ESP_LOGW(TAG, "TASK EXITING FLAG SET");
    if (this->outgoing_queue_) {
      // Drain and delete any pending messages so they don't leak.
      SnapcastMessage *m = nullptr;
      while (xQueueReceive(this->outgoing_queue_, &m, 0) == pdPASS) {
        delete m;
      }
      vQueueDelete(this->outgoing_queue_);
      this->outgoing_queue_ = nullptr;
    }
  };

  if (stream_package_buffer == nullptr) {
    ESP_LOGE(TAG, "Failed to create stream package buffer!");
    this->set_state_(StreamState::ERROR);
    on_exit_task();
    return;
  }

  this->outgoing_queue_ = xQueueCreate(10, sizeof(SnapcastMessage *));
  if (this->outgoing_queue_ == NULL) {
    ESP_LOGE(TAG, "Failed to create outgoing queue!");
    this->set_state_(StreamState::ERROR);
    on_exit_task();
    return;
  }

  struct TransportTaskArgs {
    SnapcastStream *self;
    std::shared_ptr<ChunkedRingBuffer> buffer;
    TaskHandle_t stream_task_handle;
    TimeStats *time_stats;
    QueueHandle_t outgoing_queue;
  };

  TransportTaskArgs *args = new TransportTaskArgs();
  args->self = this;
  args->buffer = stream_package_buffer;
  args->stream_task_handle = xTaskGetCurrentTaskHandle();
  args->time_stats = &this->time_stats_;  // Pass the time stats reference
  args->outgoing_queue = this->outgoing_queue_;

  TaskHandle_t transport_task_handle = nullptr;
  BaseType_t result = xTaskCreatePinnedToCore(
      [](void *param) {
        auto *args = static_cast<TransportTaskArgs *>(param);
        transport_task_(args->self, args->buffer, args->stream_task_handle, args->time_stats, args->outgoing_queue);
        delete args;
        vTaskDelete(nullptr);  // Task cleans itself up after loop exits
      },
      "snapcast_tx_rx",        // Task name
      4096,                    // Stack size in words
      args,                    // param: your 'this' pointer
      16,                      // Priority
      &transport_task_handle,  // Store handle if you want to kill it later
      1);

  if (result != pdPASS) {
    delete args;
    ESP_LOGE(TAG, "Failed to create Snapcast RX/TX task!");
    this->set_state_(StreamState::ERROR);
    on_exit_task();
    return;
  }

  const uint32_t retry_window_ms = 5000;  // retry for 5 seconds then give up
  const uint32_t max_backoff_ms = 10000;

  bool desired_streaming = false;
  bool destroy_requested = false;

  uint32_t reconnect_start_ms = 0;  // 0 means not currently retrying
  uint32_t backoff_ms = 250;

  auto request_destroy = [&]() {
    if (destroy_requested)
      return;
    destroy_requested = true;
    desired_streaming = false;
    this->stream_task_exiting_ = true;
    this->set_state_(StreamState::STOPPING);

    // Tell transport to stop and exit
    xTaskNotify(transport_task_handle, STOP_BIT, eSetBits);
  };

  constexpr TickType_t STREAMING_WAIT = pdMS_TO_TICKS(5);
  constexpr TickType_t IDLE_WAIT = pdMS_TO_TICKS(10);

  uint32_t last_status_change_repeat = millis();

  uint32_t notify_value;
  this->set_state_(StreamState::DISCONNECTED);
  while (true) {
#if SNAPCAST_DEBUG
    static uint32_t last_log = 0;
    if (millis() - last_log > 10000) {
      ESP_LOGI(TAG, "state=%d destroy=%d desired_conn=%d", this->state_, destroy_requested, this->want_connected_);
      ESP_LOGI(TAG, "stream stack watermark=%u", uxTaskGetStackHighWaterMark(NULL));
      if (transport_task_handle) {
        auto st = eTaskGetState(transport_task_handle);
        ESP_LOGI(TAG, "transport task state=%d", (int) st);
        ESP_LOGI(TAG, "transport stack watermark=%u", uxTaskGetStackHighWaterMark(transport_task_handle));
      }
      if (this->notification_target_) {
        auto st = eTaskGetState(this->notification_target_);
        ESP_LOGI(TAG, "Notification target state=%d", (int) st);
        ESP_LOGI(TAG, "Notification target watermark=%u", uxTaskGetStackHighWaterMark(this->notification_target_));
      }
      last_log = millis();
    }

    // if (millis() - last_status_change_repeat > 60000) {
    //   ESP_LOGW(TAG, "DEBUG FORCE DISCONNECTING!!!");
    //   last_status_change_repeat = millis();
    //   this->set_state_(StreamState::RECONNECTING);
    //   xTaskNotify(transport_task_handle, DISCONNECT_BIT, eSetBits);
    //   // this->set_state_( StreamState::ERROR );
    //   // on_exit_task();
    //   return;
    // }
#endif

    TickType_t wait_time = (this->state_ == StreamState::STREAMING) ? STREAMING_WAIT : IDLE_WAIT;
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &notify_value, wait_time)) {
      ESP_LOGI(TAG, "stream notify=0x%08" PRIx32, notify_value);

      if (notify_value & CONNECT_BIT) {
        destroy_requested = false;

        reconnect_start_ms = 0;
        backoff_ms = 250;
        if (this->state_ == StreamState::DISCONNECTED) {
          // Tell transport to connect
          this->set_state_(StreamState::CONNECTING);
        }
        xTaskNotify(transport_task_handle, CONNECT_BIT, eSetBits);
      }

      if (notify_value & DISCONNECT_BIT) {
        xTaskNotify(transport_task_handle, DISCONNECT_BIT, eSetBits);
      }

      if (notify_value & STOP_BIT) {
        request_destroy();
      }

      if (notify_value & START_STREAM_BIT) {
        desired_streaming = true;
        this->start_streaming_();
      }

      if (notify_value & STOP_STREAM_BIT) {
        desired_streaming = false;
        this->stop_streaming_();
      }

      if (notify_value & SEND_REPORT_BIT) {
        if (this->state_ == StreamState::CONNECTED_IDLE || this->state_ == StreamState::STREAMING) {
          this->send_report_();
        }
      }

      if (notify_value & CONNECTION_ESTABLISHED_BIT) {
        reconnect_start_ms = 0;
        backoff_ms = 250;
        this->send_hello_();
        this->time_stats_.reset();
        if (desired_streaming) {
          this->start_streaming_();
        } else {
          set_state_(StreamState::CONNECTED_IDLE);
        }
      }

      if (notify_value & TASK_CLOSING_BIT) {
        // tx_rx task is closing, we can exit the loop
        if (desired_streaming) {
          this->set_state_(StreamState::ERROR);
        }
        break;
      }

      if (notify_value & CONNECTION_CLOSED_BIT) {
        this->codec_header_sent_ = false;
        if (destroy_requested) {
          // pass
        } else if (!this->want_connected_) {
          this->set_state_(StreamState::DISCONNECTED);
        } else {
          uint32_t now = millis();
          // start retry window if first failure
          if (reconnect_start_ms == 0)
            reconnect_start_ms = now;
          // budget check
          if ((now - reconnect_start_ms) > retry_window_ms) {
            ESP_LOGW(TAG, "Reconnect budget exceeded.");
            stream_package_buffer->reset();
            this->want_connected_ = false;
            this->set_state_(StreamState::DISCONNECTED);
          } else {
            // backoff and retry
            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
            xTaskNotify(transport_task_handle, CONNECT_BIT, eSetBits);
            backoff_ms = std::min(backoff_ms * 2u, max_backoff_ms);
            this->set_state_(StreamState::RECONNECTING);
          }
        }
      }
    }

    if (this->state_ == StreamState::CONNECTED_IDLE || this->state_ == StreamState::STREAMING) {
      this->send_time_sync_();
      const uint32_t timeout = this->time_stats_.is_ready() ? 500 : 10;
      if (this->read_and_process_messages_(stream_package_buffer.get(), timeout) == ESP_FAIL) {
        this->set_state_(StreamState::RECONNECTING);
        xTaskNotify(transport_task_handle, DISCONNECT_BIT, eSetBits);
        ESP_LOGW(TAG, "read_and_process_messages failed, curr_state: %d\n", this->state_);
        ESP_LOGW(TAG, "msg: %s\n", this->error_msg_.c_str());
      }
    }
  }

  on_exit_task();
}

void SnapcastStream::set_state_(StreamState new_state) {
  this->state_ = new_state;
  if (this->notification_target_ != nullptr) {
    xTaskNotify(this->notification_target_, static_cast<uint32_t>(this->state_), eSetValueWithOverwrite);
  }
  if (this->on_status_update_) {
    this->on_status_update_(this->state_, 255, false);  // 255 for volume means do not set
  }
}

void SnapcastStream::get_target_snapshot_(std::string &server, uint32_t &port) {
  xSemaphoreTake(this->config_mutex_, portMAX_DELAY);
  server = this->server_;
  port = this->port_;
  xSemaphoreGive(this->config_mutex_);
}

void SnapcastStream::start_streaming_() {
  if (this->state_ == StreamState::STREAMING) {
    // notify listeners that already in streaming state
    this->set_state_(StreamState::STREAMING);
    return;
  }
  if (this->state_ == StreamState::CONNECTED_IDLE || this->state_ == StreamState::RECONNECTING ||
      this->state_ == StreamState::CONNECTING) {
    auto rb = this->write_ring_buffer_.lock();
    if (!rb) {
      this->error_msg_ = "Ring-buffer not set yet, but trying to start streaming...";
      this->set_state_(StreamState::ERROR);
      return;
    }
    rb->reset();
    this->codec_header_sent_ = false;
    this->send_hello_();
    this->set_state_(StreamState::STREAMING);
    return;
  }
}

void SnapcastStream::stop_streaming_() {
  if (this->state_ != StreamState::STREAMING) {
    return;
  }
  this->set_state_(StreamState::CONNECTED_IDLE);
}

void SnapcastStream::send_message_(SnapcastMessage *msg) {
  auto q = this->outgoing_queue_;
  if (!q || msg->getMessageSize() > TX_BUFFER_SIZE) {
    delete msg;
    return;
  }
  if (xQueueSend(q, &msg, 0) != pdPASS) {
    delete msg;  // Clean up if failed
  }
}

void SnapcastStream::send_hello_() {
  SnapcastMessage *hello_msg = new HelloMessage();
  this->send_message_(hello_msg);
}

void SnapcastStream::send_report_() {
  const uint8_t vol = this->volume_.load(std::memory_order_relaxed);
  const bool muted = this->muted_.load(std::memory_order_relaxed);
  SnapcastMessage *msg = new ClientInfoMessage(vol, muted);
  this->send_message_(msg);
}

void SnapcastStream::send_time_sync_() {
  uint32_t sync_interval = TIME_SYNC_INTERVAL_MS;
  if (!this->time_stats_.is_ready()) {
    sync_interval = 500;
    SnapcastMessage *time_sync_msg0 = new TimeMessage();
    this->send_message_(time_sync_msg0);
  }
  if (millis() - this->last_time_sync_ > sync_interval) {
    SnapcastMessage *time_sync_msg = new TimeMessage();
    this->send_message_(time_sync_msg);
    this->last_time_sync_ = millis();
  }
}

void SnapcastStream::on_time_msg_(MessageHeader msg, tv_t latency_c2s) {
  // latency_c2s = t_server-recv - t_client-sent + t_network-latency
  // latency_s2c = t_client-recv - t_server-sent + t_network_latency
  // time diff between server and client as (latency_c2s - latency_s2c) / 2
  tv_t latency_s2c = msg.received - msg.sent;
  const tv_t offset_sample = (latency_c2s - latency_s2c) / 2;
  time_stats_.add_offset(msg.refersTo, offset_sample, msg.received);
  const tv_t est_offset = time_stats_.get_estimate();
  this->est_time_diff_.store(est_offset, std::memory_order_relaxed);
#if SNAPCAST_DEBUG
  printf("msg.id: %d, msg.sent: %lld, msg.received: %lld sample: %lld\n", msg.id, msg.sent.to_microseconds(),
         msg.received.to_microseconds(), est_offset.to_microseconds());
  printf("latency_c2s: %lld at-curr-est: %lld\n", latency_c2s.to_microseconds(),
         (latency_c2s - est_offset).to_microseconds());
  printf("latency_s2c: %lld = %lld - %lld at-curr-est: %lld\n", latency_s2c.to_microseconds(),
         msg.received.to_microseconds(), msg.sent.to_microseconds(), (latency_s2c + est_offset).to_microseconds());
  const tv_t now = tv_t::now();
  const int64_t server_time = (now + est_offset).to_microseconds();
  static int64_t last_server_time = server_time;
  static int64_t last_call = now.to_microseconds();

  int64_t expected_server_time = last_server_time + (now.to_microseconds() - last_call);
  int64_t server_diff = server_time - expected_server_time;

  last_server_time = server_time;
  last_call = now.to_microseconds();

  printf("New server time: %" PRId64 " (delta: %" PRId64 ") , expected %" PRId64 ", diff: %" PRId64 "\n", server_time,
         est_offset.to_microseconds(), expected_server_time, server_diff);
#endif
}

void SnapcastStream::on_server_settings_msg_(const ServerSettingsMessage &msg) {
  this->server_buffer_size_ = msg.buffer_ms_;
  this->latency_ = msg.latency_;
  this->volume_.store(msg.volume_, std::memory_order_relaxed);
  this->muted_.store(msg.muted_, std::memory_order_relaxed);

  if (this->on_status_update_) {
    this->on_status_update_(this->state_, this->volume_.load(std::memory_order_relaxed),
                            this->muted_.load(std::memory_order_relaxed));
  }
}

}  // namespace snapcast
}  // namespace esphome