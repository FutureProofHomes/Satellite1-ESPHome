#pragma once

#ifdef USE_ESP_IDF

#include "audio.h"
#include "audio_transfer_buffer.h"
#include "chunked_ring_buffer.h"

#if USE_SNAPCAST
#include "esphome/components/snapcast/snapcast_stream.h"
#endif

#include "esp_err.h"

#include <esp_http_client.h>

namespace esphome {
namespace audio {

enum class AudioReaderState : uint8_t {
  READING = 0,  // More data is available to read
  FINISHED,     // All data has been read and transferred
  FAILED,       // Encountered an error
};

class AudioReader {
  /*
   * @brief Class that facilitates reading a raw audio file.
   * Files can be read from flash (stored in a AudioFile struct) or from an http source.
   * The file data is sent to a ring buffer sink.
   */
 public:
  /// @brief Constructs an AudioReader object.
  /// The transfer buffer isn't allocated here, but only if necessary (an http source) in the start function.
  /// @param buffer_size Transfer buffer size in bytes.
  AudioReader(size_t buffer_size, std::weak_ptr<TimedRingBuffer> output_ring_buffer)
      : buffer_size_(buffer_size), output_ring_buffer_(output_ring_buffer) {}
  ~AudioReader();

  /// @brief Starts reading an audio file from an http source. The transfer buffer is allocated here.
  /// @param uri Web url to the http file.
  /// @param file_type AudioFileType variable passed-by-reference indicating the type of file being read.
  /// @return ESP_OK if successful, an ESP_ERR* code otherwise.
  esp_err_t start(const std::string &uri, AudioFileType &file_type);

  /// @brief Starts reading an audio file from flash. No transfer buffer is allocated.
  /// @param audio_file AudioFile struct containing the file.
  /// @param file_type AudioFileType variable passed-by-reference indicating the type of file being read.
  /// @return ESP_OK
  esp_err_t start(AudioFile *audio_file, AudioFileType &file_type);

#if USE_SNAPCAST
  /// @brief Starts reading an audio file from flash. No transfer buffer is allocated.
  /// @param stream Pointer to a snapcast stream
  /// @param file_type AudioFileType variable passed-by-reference indicating the type of file being read.
  /// @return ESP_OK
  esp_err_t start(snapcast::SnapcastStream *stream, AudioFileType &file_type);
#endif

  /// @brief Reads new file data from the source and sends to the ring buffer sink.
  /// @return AudioReaderState
  AudioReaderState read();

  esp_err_t stop();

 protected:
  /// @brief Monitors the http client events to attempt determining the file type from the Content-Type header
  static esp_err_t http_event_handler(esp_http_client_event_t *evt);

  /// @brief Determines the audio file type from the http header's Content-Type key
  /// @param content_type string with the Content-Type key
  /// @return AudioFileType of the url, if it can be determined. If not, return AudioFileType::NONE.
  static AudioFileType get_audio_type(const char *content_type);

  AudioReaderState file_read_();
  AudioReaderState http_read_();
#if USE_SNAPCAST
  AudioReaderState snapcast_read_();
#endif

  std::weak_ptr<TimedRingBuffer> output_ring_buffer_;
  timed_chunk_t *current_timed_chunk_{nullptr};
  size_t bytes_in_chunk_{0};  // Number of bytes currently in the chunk being read
  void cleanup_connection_();

  size_t buffer_size_;
  uint32_t last_data_read_ms_;

  esp_http_client_handle_t client_{nullptr};

  AudioFile *current_audio_file_{nullptr};
  AudioFileType audio_file_type_{AudioFileType::NONE};
  const uint8_t *file_current_{nullptr};

#if USE_SNAPCAST
  snapcast::SnapcastStream *snapcast_stream_{nullptr};
#endif
};
}  // namespace audio
}  // namespace esphome

#endif
