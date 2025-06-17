#include "audio_transfer_buffer.h"

#ifdef USE_ESP32

#include "esphome/core/helpers.h"

namespace esphome {
namespace audio {

AudioTransferBuffer::~AudioTransferBuffer() { this->deallocate_buffer_(); };

std::unique_ptr<AudioSinkTransferBuffer> AudioSinkTransferBuffer::create(size_t buffer_size, std::string name) {
  std::unique_ptr<AudioSinkTransferBuffer> sink_buffer = make_unique<AudioSinkTransferBuffer>(name);
  
  
  if (!sink_buffer->allocate_buffer_(buffer_size)) {
    return nullptr;
  }

  return sink_buffer;
}

std::unique_ptr<AudioSourceTransferBuffer> AudioSourceTransferBuffer::create(size_t buffer_size, std::string name) {
  std::unique_ptr<AudioSourceTransferBuffer> source_buffer = make_unique<AudioSourceTransferBuffer>(name);
  
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


std::unique_ptr<TimedAudioSourceTransferBuffer> TimedAudioSourceTransferBuffer::create(size_t buffer_size, std::string name) {
  std::unique_ptr<TimedAudioSourceTransferBuffer> source_buffer = make_unique<TimedAudioSourceTransferBuffer>(name);
  
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

  size_t bytes_to_read = this->free();
  int32_t bytes_read = 0;
  if (bytes_to_read > 0) {
    tv_t new_time_stamp;
    if (this->ring_buffer_.use_count() > 0) {
      bytes_read = this->ring_buffer_->read((void *) this->get_buffer_end(), bytes_to_read, new_time_stamp, ticks_to_wait);
      //printf( "TransferBuffer: free %d, read %d\n", bytes_to_read, bytes_read );
    } else {
      printf( "use-count is zero!!\n");
    }
    if( bytes_read <= 0 ){
      return 0;
    }
    this->current_time_stamp_ = new_time_stamp;  // Update the current timestamp
    this->increase_buffer_length(bytes_read);
  }
  return bytes_read;
}



std::unique_ptr<TimedAudioSinkTransferBuffer> TimedAudioSinkTransferBuffer::create(size_t buffer_size, std::string name) {
  std::unique_ptr<TimedAudioSinkTransferBuffer> sink_buffer = make_unique<TimedAudioSinkTransferBuffer>(name);
  
  if (!sink_buffer->allocate_buffer_(buffer_size)) {
    return nullptr;
  }

  return sink_buffer;
}
static constexpr uint32_t MAX_CHUNK_SIZE = 9200;  


esp_err_t TimedAudioSinkTransferBuffer::transfer_data_to_sink(TickType_t ticks_to_wait, bool post_shift) {
  size_t bytes_written = 0;
  if (this->available()) {
#ifdef USE_SPEAKER
    if (this->speaker_ != nullptr) {
#if 1      
       
      uint32_t playout_in_ms = this->get_unwritten_audio_ms() - this->correction_ms_;
      static uint32_t last_playout = playout_in_ms;
      if( playout_in_ms > 0 && (this->current_time_stamp_.sec != 0 || this->current_time_stamp_.usec != 0) ){
        const uint32_t desired_playout_time_ms = this->current_time_stamp_.to_millis();
        static uint32_t last_desired_time = desired_playout_time_ms;
        
        int32_t  delta_ms =  desired_playout_time_ms - (millis() + playout_in_ms);
        audio::AudioStreamInfo audio_stream_info = this->speaker_->get_audio_stream_info();
        size_t frame_size = audio_stream_info.frames_to_bytes(1);
        if( delta_ms > 0 ){
            // uint32_t to_padd = std::min(audio_stream_info.ms_to_bytes(delta_ms), size_t(this->free()/frame_size) * frame_size);
            // if( to_padd > 0 ){
            //   std::memset(this->data_start_ + this->buffer_length_, 0, to_padd);
            //   this->increase_buffer_length(to_padd);
            // }
            printf( "detla_ms %d, padded with %d zeros \n", delta_ms, 0);
            printf( "TimeStamp: last: %d, now: %d\n", last_desired_time, desired_playout_time_ms);
            printf( "playout_in_ms differecnce...: %d\n", playout_in_ms - last_playout );
            last_desired_time = desired_playout_time_ms;
            last_playout = playout_in_ms;
            this->speaker_->play_silence( std::min(delta_ms, (int32_t) 1000) );
            return 0;  
        }
#if 1         
        else if ( delta_ms < - 10 ){
            uint32_t drop_frames = audio_stream_info.ms_to_frames( -1 * delta_ms );
            uint32_t drop_bytes = std::min(audio_stream_info.frames_to_bytes(drop_frames), size_t(this->available()/frame_size) * frame_size);
            this->buffer_length_ -= drop_bytes;
            this->correction_ms_ = audio_stream_info.bytes_to_ms(drop_bytes);
            printf( "detla_ms %d, dropped %d bytes, post-delta: %d \n", delta_ms, drop_bytes, delta_ms + this->correction_ms_);
            printf( "TimeStamp: last: %d, now: %d\n", last_desired_time, desired_playout_time_ms);
            printf( "playout_in_ms differecnce...: %d\n", playout_in_ms - last_playout );
            last_playout = playout_in_ms;
            last_desired_time = desired_playout_time_ms;  
            if( this->available() == 0 ){
              this->correction_ms_ = 0;
              return 0;
            }
        }
#endif
      }
#endif
      bytes_written = this->speaker_->play(this->data_start_, this->available(), ticks_to_wait);
      if( bytes_written != this->available() ){
        //printf( "%s: speaker wrote %d bytes, remaining %lu\n", this->name_.c_str(), bytes_written, this->available()- bytes_written);
        //return -1;
      } else {
        this->correction_ms_ = 0;
      }

    } else
#endif
    if (this->ring_buffer_.use_count() > 0) {
      uint32_t to_write = this->available() > MAX_CHUNK_SIZE ? MAX_CHUNK_SIZE : this->available();
      timed_chunk_t *timed_chunk = nullptr;
      this->ring_buffer_->acquire_write_chunk(&timed_chunk, sizeof(timed_chunk_t) + to_write, ticks_to_wait); 
      if (timed_chunk == nullptr) {
        printf("%s: no ring buffer available to write to\n", this->name_.c_str());
        return 0;
      }
      timed_chunk->stamp = this->current_time_stamp_;  // Set the timestamp for the chunk
      std::memcpy(timed_chunk->data, this->data_start_, to_write);
      this->ring_buffer_->release_write_chunk(timed_chunk);
      bytes_written = to_write;
    } else {
      printf("%s: no ring buffer available to write to\n", this->name_.c_str());
      return 0;
    }
    this->decrease_buffer_length(bytes_written);
  } else {
    //printf( "no data available\n");
  }

  if (post_shift) {
    // Shift unwritten data to the start of the buffer
    memmove(this->buffer_, this->data_start_, this->buffer_length_);
    this->data_start_ = this->buffer_;
  }

  return bytes_written;
}

uint32_t TimedAudioSinkTransferBuffer::get_unwritten_audio_ms() const {
  if( this->speaker_ != nullptr ){
    return this->speaker_->get_unwritten_audio_ms();
  }
}



void TimedAudioSinkTransferBuffer::clear_buffered_data() {
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

bool TimedAudioSinkTransferBuffer::has_buffered_data() const {
#ifdef USE_SPEAKER
  if (this->speaker_ != nullptr) {
    return (this->speaker_->has_buffered_data() || (this->available() > 0));
  }
#endif
  if (this->ring_buffer_.use_count() > 0) {
    return ((this->ring_buffer_->chunks_available() > 0) || (this->available() > 0));
  }
  return (this->available() > 0);
}




}  // namespace audio
}  // namespace esphome

#endif
