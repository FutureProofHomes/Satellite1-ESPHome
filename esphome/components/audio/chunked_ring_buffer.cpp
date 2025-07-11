#include "chunked_ring_buffer.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

//#include "helpers.h"

namespace esphome {
namespace audio {

static constexpr uint32_t MAX_CHUNK_SIZE = 9200;   

static const char *const TAG = "timed_ring_buffer";

ChunkedRingBuffer::~ChunkedRingBuffer() {
  if (this->handle_ != nullptr) {
    vRingbufferDelete(this->handle_);
    RAMAllocator<uint8_t> allocator(RAMAllocator<uint8_t>::ALLOW_FAILURE);
    allocator.deallocate(this->storage_, this->size_);
    this->storage_ = nullptr;
    this->handle_ = nullptr;
  }
}


std::shared_ptr<ChunkedRingBuffer> ChunkedRingBuffer::create(size_t len) {
  auto rb = std::make_shared<ChunkedRingBuffer>();

  rb->size_ = len;

  RAMAllocator<uint8_t> allocator(RAMAllocator<uint8_t>::ALLOW_FAILURE);
  rb->storage_ = allocator.allocate(rb->size_);
  if (rb->storage_ == nullptr) {
    printf("ChunkedRingBuffer: Failed to allocate storage of size %zu\n", rb->size_);
    return nullptr;
  }

  rb->handle_ = xRingbufferCreateStatic(rb->size_, RINGBUF_TYPE_NOSPLIT, rb->storage_, &rb->structure_);
  
  if (rb->handle_ == nullptr) {
    printf("ChunkedRingBuffer: Failed to create ring buffer with size %zu\n", rb->size_);
    allocator.deallocate(rb->storage_, rb->size_);
    rb->storage_ = nullptr;
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
    //printf("ChunkedRingBuffer: Failed to receive chunk, timeout or no data available\n");
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
  // uint32_t now = millis();
  // static uint32_t last_call = now; 
  // printf( "Time between read calls: %d ms\n", millis() - last_call );
  // last_call = now;

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
  return 0;
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
      printf("Failed to send item\n");
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
  }
  return bytes_discarded > 0;
}

} // namespace audio
}  // namespace esphome

#endif
