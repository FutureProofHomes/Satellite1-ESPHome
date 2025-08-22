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

size_t AudioSinkTransferBuffer::transfer_data_to_sink(TickType_t ticks_to_wait, bool post_shift, bool write_partial) {
  size_t bytes_written = 0;
  if (this->available()) {
#ifdef USE_SPEAKER
    if (this->speaker_ != nullptr) {
      bytes_written = this->speaker_->play(this->data_start_, this->available(), ticks_to_wait, write_partial);
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
  
  if (!ring_buffer_) return 0;
  
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

#if USE_SNAPCAST
      audio::AudioStreamInfo audio_stream_info = this->speaker_->get_audio_stream_info();      
      const int64_t desired_playout_time_us = this->current_time_stamp_.to_microseconds();    
      static int64_t last_time_stamp = desired_playout_time_us;
      
#if SNAPCAST_DEBUG      
      bool stamp_off = false;
      if( desired_playout_time_us - last_time_stamp > 24000 ){
        printf( "packet stamp off by %" PRId64 " us\n", desired_playout_time_us - last_time_stamp );
        stamp_off = true;
      }
      last_time_stamp = desired_playout_time_us;
#endif      
      
      const int64_t playout_at = this->speaker_->get_playout_time(0);
      
      const uint32_t ms_since_last_adjustment = static_cast<uint32_t>(desired_playout_time_us / 1000) - last_adjustment_at_;
      
      if(playout_at > 0 && desired_playout_time_us && ms_since_last_adjustment > 20){
        int64_t now = esp_timer_get_time();  
        int64_t delta_us = desired_playout_time_us - playout_at;
        
        const size_t frame_size = audio_stream_info.frames_to_bytes(1);
        const size_t total_frames = audio_stream_info.bytes_to_frames(this->available());
        if( delta_us > 1000 ){
            last_adjustment_at_ = static_cast<uint32_t>(desired_playout_time_us / 1000);
#if 0            
            if( delta_us > 10000 ){
              printf( "detla_us %" PRId64 " (Now: %" PRId64 ")\n", delta_us, now);
              printf( "TimeStamp: %" PRId64 ", in %" PRId64 " us %s \n", 
                desired_playout_time_us, 
                desired_playout_time_us - now,
                stamp_off ? "(off)" :""
              );
            }
#endif
            if( delta_us >= 24 * 1000 ){
              this->speaker_->play_silence( std::min( static_cast<int32_t>(delta_us / 1000), (int32_t) 1000) );
  
            } else if (this->free() >= frame_size && total_frames > 3) {
              size_t insert_frame = 1 + (rand() % (total_frames-2));  // don't allow insertion at beginning and the end
              size_t insert_offset = insert_frame * frame_size;
              size_t bytes_after = this->buffer_length_ - insert_offset;
              std::memmove(this->data_start_ + insert_offset + frame_size, this->data_start_ + insert_offset, bytes_after);
              uint8_t channels = audio_stream_info.get_channels();
              if( audio_stream_info.get_bits_per_sample() == 16 ){
                int16_t* prev = reinterpret_cast<int16_t*>(this->data_start_ + insert_offset);
                int16_t* next = reinterpret_cast<int16_t*>(this->data_start_ + insert_offset + 2 * frame_size);
                int16_t* out  = reinterpret_cast<int16_t*>(this->data_start_ + insert_offset + frame_size);
                for( int ch=0; ch < channels; ch++ ){
                  int16_t prev_sample = prev[ch];
                  int16_t next_sample = next[ch];
                  out[ch] = (prev_sample + next_sample) / 2;
                }
              } else {
                std::memset(this->data_start_ + insert_offset, 0, frame_size);
              }
              this->increase_buffer_length(frame_size);
            }
        }
        else if( desired_playout_time_us <= now ){
#if SNAPCAST_DEBUG          
          printf( "transfer-buffer: skipping full frame: delta: %" PRId64 "\n", desired_playout_time_us - now);
#endif          
          size_t available = this->available();
          this->decrease_buffer_length(available);
          return available;
        } 
        else if ( delta_us < -1000 ){
          last_adjustment_at_ = static_cast<uint32_t>(desired_playout_time_us / 1000);
          size_t drop_frames = 1;
          if( delta_us < -50 * 1000) {
            drop_frames = audio_stream_info.ms_to_frames( -1 * delta_us / 1000 );
          }
          drop_frames = std::min(drop_frames, total_frames);
          if( drop_frames == total_frames ){
            size_t available = this->available();
            this->decrease_buffer_length(available);
#if SNAPCAST_DEBUG            
            printf( "detla_us %" PRId64 " (Now: %" PRId64 ")\n", delta_us, now);
            printf( "TimeStamp: %" PRId64 ", in %" PRId64 " us\n", desired_playout_time_us, desired_playout_time_us - now);
            printf( "dropped full frame \n");
#endif
            return available;
          }
          uint32_t drop_bytes = audio_stream_info.frames_to_bytes(drop_frames);
          this->buffer_length_ -= drop_bytes;
          this->current_time_stamp_ += tv_t::from_microseconds(audio_stream_info.bytes_to_us(drop_bytes));
#if 0
          if( delta_us < - 10000 ){
            printf( "detla_us %" PRId64 " (Now: %" PRId64 ")\n", delta_us, now);
            printf( "TimeStamp: %" PRId64 ", in %" PRId64 " us\n", desired_playout_time_us, desired_playout_time_us - now);
          }
#endif          
        }
      }

#endif      

      bytes_written = this->speaker_->play(this->data_start_, this->available(), ticks_to_wait);
      if( bytes_written > 0){
        while(!this->speaker_->update_buffer_states(bytes_written)){}
     }
#if SNAPCAST_DEBUG      
      if( bytes_written && bytes_written != this->available() ){
        printf( "wrote %d bytes to speaker, remaining %lu\n", bytes_written, this->available()- bytes_written);
      }
#endif      
    } else

#endif // USE_SPEAKER
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

bool TimedAudioSourceTransferBuffer::has_buffered_data() const {
  if (this->ring_buffer_.use_count() > 0) {
    return ((this->ring_buffer_->chunks_available() > 0) || (this->available() > 0));
  }
  return (this->available() > 0);
}

}  // namespace audio
}  // namespace esphome

#endif
