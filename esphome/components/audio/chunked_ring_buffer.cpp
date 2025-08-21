#include "chunked_ring_buffer.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

namespace esphome {
namespace audio {

static constexpr uint32_t MAX_CHUNK_SIZE = 9200;   

static const char *const TAG = "timed_ring_buffer";

ChunkedRingBuffer::~ChunkedRingBuffer() {
  if (this->handle_ != nullptr) {
    vRingbufferDelete(this->handle_);
    RAMAllocator<uint8_t> allocator;
    allocator.deallocate(this->storage_, this->size_);
    this->storage_ = nullptr;
    this->handle_ = nullptr;
  }
}

esp_err_t ChunkedRingBuffer::init() {
  if (this->handle_ != nullptr) {
    ESP_LOGE(TAG, "ChunkedRingBuffer already initialized");
    return ESP_FAIL;
  }

  RAMAllocator<uint8_t> allocator;
  this->storage_ = allocator.allocate(this->size_);
  if (this->storage_ == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate storage of size %zu", this->size_);
    return ESP_ERR_NO_MEM;
  }

  this->handle_ = xRingbufferCreateStatic(this->size_, RINGBUF_TYPE_NOSPLIT, this->storage_, &this->structure_);
  
  if (this->handle_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create ring buffer with size %zu", this->size_);
    allocator.deallocate(this->storage_, this->size_);
    this->storage_ = nullptr;
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}


std::shared_ptr<ChunkedRingBuffer> ChunkedRingBuffer::create(size_t len) {
  auto rb = std::make_shared<ChunkedRingBuffer>(len);
  
  if (rb->init() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create ChunkedRingBuffer with size %zu", len);
    return nullptr;
  }

  return rb;
}


int32_t ChunkedRingBuffer::readUpTo(void *data, size_t max_len, TickType_t ticks_to_wait) {
  if( this->curr_chunk != nullptr ){
    if( max_len >= this->bytes_waiting_in_chunk ){
      std::memcpy(data, this->curr_chunk_read_pos, this->bytes_waiting_in_chunk);
      vRingbufferReturnItem(this->handle_, this->curr_chunk);
      this->curr_chunk = nullptr;
      this->curr_chunk_read_pos = nullptr; 
      this->bytes_available_ -= this->bytes_waiting_in_chunk; 
      return this->bytes_waiting_in_chunk;
    }
    else {
      std::memcpy(data, this->curr_chunk_read_pos, max_len);
      this->bytes_waiting_in_chunk -= max_len;
      this->bytes_available_ -= max_len;
      this->curr_chunk_read_pos += max_len; 
      return max_len;
    }
  }
  
  this->curr_chunk = (uint8_t*) xRingbufferReceive(this->handle_, &this->bytes_waiting_in_chunk, ticks_to_wait);
  if ( this->curr_chunk == nullptr) {
    return 0;
  }
  
  if( max_len >= this->bytes_waiting_in_chunk){
      std::memcpy(data, this->curr_chunk, this->bytes_waiting_in_chunk);
      vRingbufferReturnItem(this->handle_, this->curr_chunk);
      this->curr_chunk = nullptr;
      this->bytes_available_ -= this->bytes_waiting_in_chunk; 
      return this->bytes_waiting_in_chunk;
  }
  std::memcpy(data, this->curr_chunk, max_len);
  this->bytes_waiting_in_chunk -= max_len;
  this->bytes_available_ -= max_len;
  this->curr_chunk_read_pos = this->curr_chunk + max_len; 
  return max_len;
}

int32_t ChunkedRingBuffer::read(void *data, size_t len, TickType_t ticks_to_wait) {
  size_t bytes_read = 0;
  TickType_t start_tick = xTaskGetTickCount();
   while( bytes_read < len ){
        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed = now - start_tick;
        if (elapsed >= ticks_to_wait) {
            break; // Time budget exhausted
        }

        TickType_t remaining_ticks = ticks_to_wait - elapsed;
        int32_t r = this->readUpTo(static_cast<uint8_t*>(data) + bytes_read, len - bytes_read, remaining_ticks);
        if (r <= 0) {
            break;
        }
        bytes_read += r;
   }
   return bytes_read;
}

uint8_t* ChunkedRingBuffer::get_next_chunk(size_t &len, TickType_t ticks_to_wait) {
  return (uint8_t*) xRingbufferReceive(this->handle_, &len, ticks_to_wait);
}

void ChunkedRingBuffer::release_read_chunk(uint8_t *chunk) {
  if (chunk == nullptr) {
    return;
  }
  vRingbufferReturnItem(this->handle_, chunk);
}



size_t ChunkedRingBuffer::write_without_replacement(const void *data, size_t len, TickType_t ticks_to_wait) {
  if( this->handle_ == nullptr )
  {
    ESP_LOGE(TAG, "Ring buffer handle is null, cannot write data");
    return 0;
  }
  if (data == nullptr || len == 0) {
    ESP_LOGE(TAG, "Invalid data or length for writing to ring buffer");
    return 0;
  }
  uint8_t *chunk;
  size_t chunk_data_size = len > MAX_CHUNK_SIZE ? MAX_CHUNK_SIZE : len; 
  UBaseType_t res = xRingbufferSendAcquire(this->handle_, (void**) &chunk, chunk_data_size, ticks_to_wait);
  if (chunk == nullptr) {
    ESP_LOGE( TAG, "requested: chunk_data_size: %d, len: %d, available: %d, max_item_size: %d", chunk_data_size, len, this->free(), xRingbufferGetMaxItemSize(this->handle_));
    return 0;
  }
  std::memcpy(chunk, data, chunk_data_size);
  
  res = xRingbufferSendComplete(this->handle_, chunk);
  if (res != pdTRUE) {
      return 0;
  }
  this->bytes_available_ += chunk_data_size;
  return chunk_data_size;   
}

error_t ChunkedRingBuffer::acquire_write_chunk(uint8_t **write_chunk, size_t len, TickType_t ticks_to_wait, bool discard_first){
  if (write_chunk == nullptr || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }
  UBaseType_t res = xRingbufferSendAcquire(this->handle_, (void**) write_chunk, len, ticks_to_wait);
  if (*write_chunk == nullptr) {
    if( discard_first ){
      this->discard_chunks_(1);
      return this->acquire_write_chunk(write_chunk, len, ticks_to_wait, false);
    }
    return ESP_ERR_TIMEOUT;
  }

  // Return the size of the available write chunk
  return ESP_OK;
}


error_t ChunkedRingBuffer::release_write_chunk(uint8_t *write_chunk, size_t len){
  if (write_chunk == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  UBaseType_t res = xRingbufferSendComplete(this->handle_, write_chunk);
  if (res != pdTRUE) {
      return res;
  }
  this->bytes_available_ += len;
  return ESP_OK;
}



size_t ChunkedRingBuffer::chunks_available() const {
  UBaseType_t ux_items_waiting = 0;
  vRingbufferGetInfo(this->handle_, nullptr, nullptr, nullptr, nullptr, &ux_items_waiting);
  return ux_items_waiting;
}

size_t ChunkedRingBuffer::free() const { return xRingbufferGetCurFreeSize(this->handle_); }

BaseType_t ChunkedRingBuffer::reset() {
  // Discards all the available data
  if( this->curr_chunk != nullptr ){
    vRingbufferReturnItem(this->handle_, this->curr_chunk);
    this->curr_chunk = nullptr;
    this->bytes_available_ -= this->bytes_waiting_in_chunk; 
    this->bytes_waiting_in_chunk = 0;
  }
  return this->discard_chunks_(this->chunks_available());
}

bool ChunkedRingBuffer::discard_chunks_(size_t discard_chunks) {
  size_t bytes_discarded = 0;
  for(size_t cnt=0; cnt < discard_chunks; cnt++){
    size_t bytes_in_chunk = 0;
    void *buffer_data = xRingbufferReceive(this->handle_, &bytes_in_chunk, 0);
    if (buffer_data == nullptr){
      return bytes_discarded;
    }
    vRingbufferReturnItem(this->handle_, buffer_data);
    bytes_discarded += bytes_in_chunk; 
    this->bytes_available_ -= bytes_in_chunk - this->chunk_header_size_; 
  }
  return bytes_discarded > 0;
}



std::shared_ptr<TimedRingBuffer> TimedRingBuffer::create(size_t len) {
  auto rb = std::make_shared<TimedRingBuffer>(len);
  
  if (rb->init() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create ChunkedRingBuffer with size %zu", len);
    return nullptr;
  }

  return rb;
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
      return -1;
    }
  }
  
  this->curr_chunk = (timed_chunk_t*) xRingbufferReceive(this->handle_, &this->bytes_waiting_in_chunk, ticks_to_wait);
  if (curr_chunk == nullptr) {
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
  return -1;
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
      return res;
  }
  this->bytes_available_ += len;
  return ESP_OK;
}

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



} // namespace audio
}  // namespace esphome

#endif
