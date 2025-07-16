#include "audio_transfer_buffer.h"

#ifdef USE_ESP32

#include "esphome/core/helpers.h"

namespace esphome {
namespace audio {

AudioTransferBuffer::~AudioTransferBuffer() { this->deallocate_buffer_(); };

std::unique_ptr<AudioSinkTransferBuffer> AudioSinkTransferBuffer::create(size_t buffer_size) {
  std::unique_ptr<AudioSinkTransferBuffer> sink_buffer = make_unique<AudioSinkTransferBuffer>();

  if (!sink_buffer->allocate_buffer_(buffer_size)) {
    return nullptr;
  }

  return sink_buffer;
}

std::unique_ptr<AudioSourceTransferBuffer> AudioSourceTransferBuffer::create(size_t buffer_size) {
  std::unique_ptr<AudioSourceTransferBuffer> source_buffer = make_unique<AudioSourceTransferBuffer>();

  if (!source_buffer->allocate_buffer_(buffer_size)) {
    return nullptr;
  }

  return source_buffer;
}

size_t AudioTransferBuffer::free() const {
  if (this->buffer_size_ == 0) {
    return 0;
  }
  return this->buffer_size_ - (this->buffer_length_ + (this->data_start_ - this->buffer_));
}

void AudioTransferBuffer::decrease_buffer_length(size_t bytes) {
  this->buffer_length_ -= bytes;
  if (this->buffer_length_ > 0) {
    this->data_start_ += bytes;
  } else {
    // All the data in the buffer has been consumed, reset the start pointer
    this->data_start_ = this->buffer_;
  }
}

void AudioTransferBuffer::increase_buffer_length(size_t bytes) { this->buffer_length_ += bytes; }

void AudioTransferBuffer::clear_buffered_data() {
  this->buffer_length_ = 0;
  if (this->ring_buffer_.use_count() > 0) {
    this->ring_buffer_->reset();
  }
}

void AudioSinkTransferBuffer::clear_buffered_data() {
  this->buffer_length_ = 0;
  if (this->ring_buffer_.use_count() > 0) {
    this->ring_buffer_->reset();
  }
#ifdef USE_SPEAKER
  if (this->speaker_ != nullptr) {
    this->speaker_->stop();
  }
#endif
}

bool AudioTransferBuffer::has_buffered_data() const {
  if (this->ring_buffer_.use_count() > 0) {
    return ((this->ring_buffer_->available() > 0) || (this->available() > 0));
  }
  return (this->available() > 0);
}

bool AudioTransferBuffer::reallocate(size_t new_buffer_size) {
  if (this->buffer_length_ > 0) {
    // Buffer currently has data, so reallocation is impossible
    return false;
  }
  this->deallocate_buffer_();
  return this->allocate_buffer_(new_buffer_size);
}

bool AudioTransferBuffer::allocate_buffer_(size_t buffer_size) {
  this->buffer_size_ = buffer_size;

  RAMAllocator<uint8_t> allocator(ExternalRAMAllocator<uint8_t>::ALLOW_FAILURE);

  this->buffer_ = allocator.allocate(this->buffer_size_);
  if (this->buffer_ == nullptr) {
    return false;
  }

  this->data_start_ = this->buffer_;
  this->buffer_length_ = 0;

  return true;
}

void AudioTransferBuffer::deallocate_buffer_() {
  if (this->buffer_ != nullptr) {
    RAMAllocator<uint8_t> allocator(ExternalRAMAllocator<uint8_t>::ALLOW_FAILURE);
    allocator.deallocate(this->buffer_, this->buffer_size_);
    this->buffer_ = nullptr;
    this->data_start_ = nullptr;
  }

  this->buffer_size_ = 0;
  this->buffer_length_ = 0;
}

size_t AudioSourceTransferBuffer::transfer_data_from_source(TickType_t ticks_to_wait, bool pre_shift) {
  if (pre_shift) {
    // Shift data in buffer to start
    if (this->buffer_length_ > 0) {
      memmove(this->buffer_, this->data_start_, this->buffer_length_);
    }
    this->data_start_ = this->buffer_;
  }

  size_t bytes_to_read = this->free();
  size_t bytes_read = 0;
  if (bytes_to_read > 0) {
    if (this->ring_buffer_.use_count() > 0) {
      bytes_read = this->ring_buffer_->read((void *) this->get_buffer_end(), bytes_to_read, ticks_to_wait);
    }

    this->increase_buffer_length(bytes_read);
  }
  return bytes_read;
}

size_t AudioSinkTransferBuffer::transfer_data_to_sink(TickType_t ticks_to_wait, bool post_shift) {
  size_t bytes_written = 0;
  if (this->available()) {
#ifdef USE_SPEAKER
    if (this->speaker_ != nullptr) {
      bytes_written = this->speaker_->play(this->data_start_, this->available(), ticks_to_wait);
    } else
#endif
        if (this->ring_buffer_.use_count() > 0) {
      bytes_written =
          this->ring_buffer_->write_without_replacement((void *) this->data_start_, this->available(), ticks_to_wait);
    }

    this->decrease_buffer_length(bytes_written);
  }

  if (post_shift) {
    // Shift unwritten data to the start of the buffer
    memmove(this->buffer_, this->data_start_, this->buffer_length_);
    this->data_start_ = this->buffer_;
  }

  return bytes_written;
}

bool AudioSinkTransferBuffer::has_buffered_data() const {
#ifdef USE_SPEAKER
  if (this->speaker_ != nullptr) {
    return (this->speaker_->has_buffered_data() || (this->available() > 0));
  }
#endif
  if (this->ring_buffer_.use_count() > 0) {
    return ((this->ring_buffer_->available() > 0) || (this->available() > 0));
  }
  return (this->available() > 0);
}


std::unique_ptr<TimedAudioSourceTransferBuffer> TimedAudioSourceTransferBuffer::create(size_t buffer_size) {
  std::unique_ptr<TimedAudioSourceTransferBuffer> source_buffer = make_unique<TimedAudioSourceTransferBuffer>();
  
  if (!source_buffer->allocate_buffer_(buffer_size)) {
    return nullptr;
  }

  return source_buffer;
}


size_t TimedAudioSourceTransferBuffer::transfer_data_from_source(TickType_t ticks_to_wait, bool pre_shift) {
  if (pre_shift) {
    // Shift data in buffer to start
    if (this->buffer_length_ > 0) {
      memmove(this->buffer_, this->data_start_, this->buffer_length_);
    }
    this->data_start_ = this->buffer_;
  }
  
  if (this->ring_buffer_.use_count() == 0) {
    return 0;
  }
  
  size_t bytes_to_read = this->free();
  int32_t bytes_read = 0;
  int32_t read_now = 0;
  TickType_t start_ticks = xTaskGetTickCount();
  
  //chunks with time stamps are only return as long new_time_stamp is not (0,0)
  tv_t new_time_stamp = tv_t(-1,-1); 
  while (bytes_to_read > 0 ) {
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed = now - start_ticks;
    if (elapsed >= ticks_to_wait) break;
    
    read_now = this->ring_buffer_->read((void *) this->get_buffer_end(), bytes_to_read, new_time_stamp, ticks_to_wait - elapsed);
    if( read_now <= 0 ){
      // next chunk doesn't fit into free space
      return bytes_read;
    }
    
    bytes_read += read_now;
    bytes_to_read -= read_now;
    this->increase_buffer_length(read_now);
    
    if (new_time_stamp > tv_t(0,0)){
#if SNAPCAST_DEBUG      
      if( (new_time_stamp - this->current_time_stamp_ ).to_microseconds() > 24000 ){
          printf( "transfer-from-source: packet loss, diff: %" PRId64 " us\n", (new_time_stamp - this->current_time_stamp_ ).to_microseconds() );
          printf( "transfer-from-source: read: %d, avialable: %d\n", read_now, this->available() );
      }
#endif      
      if( this->available() == read_now ){
        this->current_time_stamp_ = new_time_stamp;
      }
      break; // Process only one chunk if timestamp present
    }
  }
  
  return bytes_read;
}

std::unique_ptr<TimedAudioSinkTransferBuffer> TimedAudioSinkTransferBuffer::create(size_t buffer_size) {
  std::unique_ptr<TimedAudioSinkTransferBuffer> sink_buffer = make_unique<TimedAudioSinkTransferBuffer>();
  
  if (!sink_buffer->allocate_buffer_(buffer_size)) {
    return nullptr;
  }

  return sink_buffer;
}

esp_err_t TimedAudioSinkTransferBuffer::transfer_data_to_sink(TickType_t ticks_to_wait, uint32_t &skip_next_frames, bool post_shift) {
  size_t bytes_written = 0;
  if (this->available()) {
#ifdef USE_SPEAKER
    if (this->speaker_ != nullptr) {

      bytes_written = this->speaker_->play(this->data_start_, this->available(), ticks_to_wait);
    } else
#endif
    return AudioSinkTransferBuffer::transfer_data_to_sink(ticks_to_wait, post_shift);
    this->decrease_buffer_length(bytes_written);
  } // if available()

  if (post_shift) {
    // Shift unwritten data to the start of the buffer
    memmove(this->buffer_, this->data_start_, this->buffer_length_);
    this->data_start_ = this->buffer_;
  }

  return bytes_written;
}
bool TimedAudioSinkTransferBuffer::has_buffered_data() const {
#ifdef USE_SPEAKER
  if (this->speaker_ != nullptr) {
    return (this->speaker_->has_buffered_data() || (this->available() > 0));
  }
#endif
  return false;
}

}  // namespace audio
}  // namespace esphome

#endif
