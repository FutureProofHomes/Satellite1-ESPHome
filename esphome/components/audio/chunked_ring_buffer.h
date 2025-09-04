#pragma once

#ifdef USE_ESP32

#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>

#include <cinttypes>
#include <memory>

#include "esphome/core/hal.h"
#include "audio.h"

namespace esphome {
namespace audio {


class ChunkedRingBuffer {
 public:
  ChunkedRingBuffer() = default;
  ChunkedRingBuffer(size_t len) : size_(len) {}
  virtual ~ChunkedRingBuffer();

  /**
   * @brief Reads from the ring buffer, waiting up to a specified number of ticks if necessary.
   *
   * Available bytes are read into the provided data pointer. If not enough bytes are available,
   * the function will wait up to `ticks_to_wait` FreeRTOS ticks before reading what is available.
   *
   * @param data Pointer to copy read data into
   * @param max_len Number of bytes to read
   * @param ticks_to_wait Maximum number of FreeRTOS ticks to wait (default: 0)
   * @return Number of bytes read
   */
  int32_t read(void *data, size_t len, TickType_t ticks_to_wait = 0);
  
  
  // reads min(max_len, bytes_available in current or next chunk)
  int32_t readUpTo(void *data, size_t max_len, TickType_t ticks_to_wait = 0);

  uint8_t* get_next_chunk(size_t &len, TickType_t ticks_to_wait);
  void release_read_chunk(uint8_t *chunk);
  
  /**
   * @brief Writes to the ring buffer without overwriting oldest data.
   *
   * The provided data is written to the ring buffer. If not enough space is available,
   * the function will wait up to `ticks_to_wait` FreeRTOS ticks before writing as much as possible.
   *
   * @param data Pointer to data for writing
   * @param len Number of bytes to write
   * @param ticks_to_wait Maximum number of FreeRTOS ticks to wait (default: 0)
   * @return Number of bytes written
   */
  virtual size_t write_without_replacement(const void *data, size_t len, TickType_t ticks_to_wait = 0);

  /**
   * @brief Writes to the ring buffer without overwriting oldest data.
   *
   * The provided data is written to the ring buffer. If not enough space is available,
   * the function will wait up to `ticks_to_wait` FreeRTOS ticks before writing as much as possible.
   *
   * @param ppvItem [out] Double pointer to memory acquired (set to NULL if no memory were retrieved)
   * @param len Number of bytes to allocate (excluding time header)
   * @param ticks_to_wait Maximum number of FreeRTOS ticks to wait (default: 0)
   * @return pointer to allocated write_chunk
   */
  error_t acquire_write_chunk(uint8_t **write_chunk, size_t len, TickType_t ticks_to_wait = 0, bool discard_first = false);

  error_t release_write_chunk(uint8_t *write_chunk, size_t len = 0);

  
  /**
   * @brief Returns the number of available bytes in the ring buffer.
   *
   * This function provides the number of bytes that can be read from the ring buffer
   * without blocking the calling FreeRTOS task.
   *
   * @return Number of available chunks
   */
  size_t chunks_available() const;

  /**
   * @brief Returns the number of free bytes in the ring buffer.
   *
   * This function provides the number of bytes that can be written to the ring buffer
   * without overwriting data or blocking the calling FreeRTOS task.
   *
   * @return Number of free bytes
   */
  size_t free() const;

  /**
   * @brief Resets the ring buffer, discarding all stored data.
   *
   * @return pdPASS if successful, pdFAIL otherwise
   */
  virtual BaseType_t reset();

  static std::shared_ptr<ChunkedRingBuffer> create(size_t len);

  size_t bytes_available() const { return this->bytes_available_; }

 protected:
  esp_err_t init();

  /// @brief Discards data from the ring buffer.
  /// @param discard_chunks amount of chunks to discard
  /// @return True if all bytes were successfully discarded, false otherwise
  bool discard_chunks_(size_t discard_bytes);

  RingbufHandle_t handle_{nullptr};
  StaticRingbuffer_t structure_;
  uint8_t *storage_{nullptr};
  size_t chunk_header_size_{0};
  size_t size_{0};
  size_t bytes_available_{0};
  size_t bytes_waiting_in_chunk{0};
  uint8_t *curr_chunk{nullptr};
  uint8_t *curr_chunk_read_pos{nullptr};
  
};

#pragma pack(push, 1)
  struct timed_chunk_t {
    tv_t stamp;  // Timestamp of the chunk
    uint8_t data[0];  // Variable length data
  };
#pragma pack(pop)

class TimedRingBuffer : public ChunkedRingBuffer {
 public:
  TimedRingBuffer() : chunk_header_size_(sizeof(timed_chunk_t)) {}
  TimedRingBuffer(size_t len) : ChunkedRingBuffer(len), chunk_header_size_(sizeof(timed_chunk_t)) {}
  
  static std::shared_ptr<TimedRingBuffer> create(size_t len);
  
  int32_t read(void *data, size_t max_len, tv_t &stamp, TickType_t ticks_to_wait = 0);

  error_t acquire_write_chunk(timed_chunk_t **write_chunk, size_t len, TickType_t ticks_to_wait = 0);
  
  error_t release_write_chunk(timed_chunk_t *write_chunk, size_t len = 0);

  size_t write_without_replacement(const void *data, size_t len, TickType_t ticks_to_wait = 0) override;

  BaseType_t reset() override;
private:
  size_t chunk_header_size_{0};
  timed_chunk_t *curr_chunk{nullptr};
};


} // namespace audio
}  // namespace esphome

#endif
