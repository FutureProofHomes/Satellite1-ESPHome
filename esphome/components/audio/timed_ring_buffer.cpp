#include "timed_ring_buffer.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

//#include "helpers.h"

namespace esphome {
namespace audio {

static constexpr uint32_t MAX_CHUNK_SIZE = 9200;   

static const char *const TAG = "timed_ring_buffer";

TimedRingBuffer::~TimedRingBuffer() {
  if (this->handle_ != nullptr) {
    vRingbufferDelete(this->handle_);
    RAMAllocator<uint8_t> allocator(RAMAllocator<uint8_t>::ALLOW_FAILURE);
    allocator.deallocate(this->storage_, this->size_);
  }
}

std::unique_ptr<TimedRingBuffer> TimedRingBuffer::create(size_t len) {
  std::unique_ptr<TimedRingBuffer> rb = make_unique<TimedRingBuffer>();

  rb->size_ = len;

  RAMAllocator<uint8_t> allocator(RAMAllocator<uint8_t>::ALLOW_FAILURE);
  rb->storage_ = allocator.allocate(rb->size_);
  if (rb->storage_ == nullptr) {
    return nullptr;
  }

  rb->handle_ = xRingbufferCreateStatic(rb->size_, RINGBUF_TYPE_NOSPLIT, rb->storage_, &rb->structure_);
  ESP_LOGD(TAG, "Created ring buffer with size %u", len);

  return rb;
}

int32_t TimedRingBuffer::read(void *data, size_t max_len, TickType_t ticks_to_wait) {
 tv_t stamp;
 return this->read(data, max_len, stamp, ticks_to_wait);
}


int32_t TimedRingBuffer::read(void *data, size_t max_len, tv_t &stamp, TickType_t ticks_to_wait) {
  if( this->curr_chunk != nullptr ){
    if( max_len >= this->bytes_waiting_in_chunk ){
      std::memcpy(data, this->curr_chunk->data, this->bytes_waiting_in_chunk);
      stamp = this->curr_chunk->stamp;  // Copy the timestamp from the current chunk
      vRingbufferReturnItem(this->handle_, this->curr_chunk);
      this->curr_chunk = nullptr;
      this->bytes_available_ -= this->bytes_waiting_in_chunk; 
      return this->bytes_waiting_in_chunk;
    }
    else {
      //printf("pre: TimedRingBuffer: max length %d bytes, bytes in chunk: %d \n", max_len, this->bytes_waiting_in_chunk);
      return -1;
    }
  }
  
  this->curr_chunk = (timed_chunk_t*) xRingbufferReceive(this->handle_, &this->bytes_waiting_in_chunk, ticks_to_wait);
  if (curr_chunk == nullptr) {
    //printf("TimedRingBuffer: Failed to receive chunk, timeout or no data available\n");
    return 0;
  }
  // if new chunk is the first chunk with time stamp, return it in the next call
  if( stamp == tv_t(0,0) && this->curr_chunk->stamp > tv_t(0,0) ){
    return -1;
  }
  this->bytes_waiting_in_chunk -= sizeof(timed_chunk_t);  // Adjust for the size of the time header
  if( max_len >= this->bytes_waiting_in_chunk){
      std::memcpy(data, this->curr_chunk->data, this->bytes_waiting_in_chunk);
      vRingbufferReturnItem(this->handle_, this->curr_chunk);
      stamp = this->curr_chunk->stamp;  // Copy the timestamp from the current chunk
      this->curr_chunk = nullptr;
      this->bytes_available_ -= this->bytes_waiting_in_chunk; 
      return this->bytes_waiting_in_chunk;
  }
  //printf("post: TimedRingBuffer: max length %d bytes, bytes in chunk: %d \n", max_len, this->bytes_waiting_in_chunk);
  return -1;

#if 0
  if (bytes_read < len) {
    // Data may have wrapped around, so read a second time to receive the remainder
    size_t follow_up_bytes_read = 0;
    size_t bytes_remaining = len - bytes_read;

    buffer_data = xRingbufferReceiveUpTo(this->handle_, &follow_up_bytes_read, 0, bytes_remaining);

    if (buffer_data == nullptr) {
      return bytes_read;
    }

    std::memcpy((void *) ((uint8_t *) (data) + bytes_read), buffer_data, follow_up_bytes_read);

    vRingbufferReturnItem(this->handle_, buffer_data);
    bytes_read += follow_up_bytes_read;
  }
#endif
}


size_t TimedRingBuffer::write_without_replacement(const void *data, size_t len, TickType_t ticks_to_wait) {
  timed_chunk_t *chunk;
  if( this->handle_ == nullptr )
  {
    ESP_LOGE(TAG, "Ring buffer handle is null, cannot write data");
    return 0;
  }
  if (data == nullptr || len == 0) {
    ESP_LOGE(TAG, "Invalid data or length for writing to ring buffer");
    return 0;
  }
  size_t chunk_data_size = len > MAX_CHUNK_SIZE ? MAX_CHUNK_SIZE : len; 
  UBaseType_t res = xRingbufferSendAcquire(this->handle_, (void**) &chunk, chunk_data_size + sizeof(timed_chunk_t), ticks_to_wait);
  if (chunk == nullptr) {
    //ESP_LOGE(TAG, "TimedRingBuffer: Failed to acquire write chunk, timeout or no space available");
    printf( "requested: chunk_data_size: %d, len: %d, available: %d, max_item_size: %d \n", chunk_data_size, len, this->free(), xRingbufferGetMaxItemSize(this->handle_));
    return 0;
  }
  std::memcpy(chunk->data, data, chunk_data_size);
  chunk->stamp.sec = 0;  // Set time to zero, as we don't have a time value here
  chunk->stamp.usec = 0; // Set time to zero, as we don't have a time value here
  
  res = xRingbufferSendComplete(this->handle_, chunk);
  if (res != pdTRUE) {
      printf("Failed to send item\n");
      return 0;
  }
  this->bytes_available_ += chunk_data_size;
  return chunk_data_size;
}

error_t TimedRingBuffer::acquire_write_chunk(timed_chunk_t **write_chunk, size_t len, TickType_t ticks_to_wait){
  if (write_chunk == nullptr || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  UBaseType_t res = xRingbufferSendAcquire(this->handle_, (void**) write_chunk, len, ticks_to_wait);
  if (*write_chunk == nullptr) {
    return ESP_ERR_TIMEOUT;
  }

  // Return the size of the available write chunk
  return ESP_OK;
}

error_t TimedRingBuffer::release_write_chunk(timed_chunk_t *write_chunk, size_t len){
  if (write_chunk == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  UBaseType_t res = xRingbufferSendComplete(this->handle_, write_chunk);
  if (res != pdTRUE) {
      printf("Failed to send item\n");
  }
  this->bytes_available_ += len;
  return ESP_OK;
}


size_t TimedRingBuffer::chunks_available() const {
  UBaseType_t ux_items_waiting = 0;
  vRingbufferGetInfo(this->handle_, nullptr, nullptr, nullptr, nullptr, &ux_items_waiting);
  return ux_items_waiting;
}

size_t TimedRingBuffer::free() const { return xRingbufferGetCurFreeSize(this->handle_); }

BaseType_t TimedRingBuffer::reset() {
  // Discards all the available data
  if( this->curr_chunk != nullptr ){
    vRingbufferReturnItem(this->handle_, this->curr_chunk);
    this->curr_chunk = nullptr;
    this->bytes_available_ -= this->bytes_waiting_in_chunk; 
    this->bytes_waiting_in_chunk = 0;
  }
  return this->discard_chunks_(this->chunks_available());
}

bool TimedRingBuffer::discard_chunks_(size_t discard_chunks) {
  size_t bytes_discarded = 0;
  for(size_t cnt=0; cnt < discard_chunks; cnt++){
    size_t bytes_in_chunk = 0;
    void *buffer_data = xRingbufferReceive(this->handle_, &bytes_in_chunk, 0);
    if (buffer_data == nullptr){
      return bytes_discarded;
    }
    vRingbufferReturnItem(this->handle_, buffer_data);
    bytes_discarded += bytes_in_chunk;
    this->bytes_available_ -= bytes_in_chunk; 
  }
  return bytes_discarded > 0;
}

} // namespace audio
}  // namespace esphome

#endif
