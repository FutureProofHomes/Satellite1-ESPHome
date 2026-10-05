#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "esphome/core/defines.h"

#ifdef USE_SAT1_MIC_MONITOR

#include <esp_http_server.h>

#include "esphome/components/microphone/microphone.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace satellite1_web_ui {

/// Frames in the PSRAM ring: 2.048 s of 16 kHz stereo int16, 128 KB. A power of two so the writer
/// masks instead of dividing on the microphone task.
static constexpr uint32_t MM_RING_FRAMES = 32768;
/// How close a listener may fall to being lapped before frames are skipped and counted as dropped.
/// A quarter second is far more than one copy takes, so a copy never reads frames being rewritten.
static constexpr uint32_t MM_RING_GUARD = 4000;
/// The most frames one message carries: 125 ms, so one message is about 8 KB.
static constexpr uint32_t MM_MAX_FRAMES = 2000;
static constexpr size_t MM_HEADER_LEN = 20;
/// The HTTP chunk around a message: "XXXX\r\n" before, "\r\n" after.
static constexpr size_t MM_CHUNK_OVERHEAD = 8;
static constexpr size_t MM_BUF_LEN = MM_CHUNK_OVERHEAD + MM_HEADER_LEN + MM_MAX_FRAMES * 4;

/// Header flag bits, as the browser reads them.
static constexpr uint8_t MM_FLAG_MUTED = 0x01;
static constexpr uint8_t MM_FLAG_MIC_IDLE = 0x02;
static constexpr uint8_t MM_FLAG_XMOS_NOT_READY = 0x04;

/**
 * Developer builds only: streams what the microphones hear to a browser, both channels at once.
 *
 * GET /api/sat1/mic answers 200 with a chunked application/octet-stream that never ends. Each chunk
 * is one message: a 20-byte little-endian header (magic "S1M1", sample index, frame count, dropped
 * frames, flags, assistant phase, wake sequence and slot, three wake word scores) and then
 * frame_count interleaved int16 pairs, [assistant channel, wake word channel]. The assistant
 * channel is what voice_assistant gets (s >> 16); the wake word channel is what micro_wake_word
 * gets after its gain_factor. While the mute switch is on both are zeros, so the stream never
 * carries audio the device was told not to hear. With no audio flowing a header-only keepalive goes
 * out every 250 ms.
 *
 * Three tasks touch this. The microphone task writes the ring from the data callback registered in
 * setup(). The httpd task answers the request, takes the socket over and parks the listener on a
 * pending list. The main loop adopts pending listeners and pumps the ring to every socket with a
 * non-blocking send, so a slow browser costs only its own dropped frames.
 *
 * A listener's memory is only freed by loop(), and only after httpd has called free_ctx for its
 * session: httpd_sess_trigger_close is asynchronous, so a listener deleted any earlier would be
 * written to by that later free_ctx.
 */
class MicMonitor {
 public:
  void set_microphone(microphone::Microphone *mic) { this->mic_ = mic; }
  /// micro_wake_word's gain_factor, so the wake word channel matches what the model hears.
  void set_wake_word_gain(uint8_t gain) { this->gain_ = gain; }
  void set_max_listeners(uint8_t n) { this->max_listeners_ = n; }
  void set_xmos_ready_fn(std::function<bool()> fn) { this->xmos_ready_fn_ = std::move(fn); }
  void set_muted_fn(std::function<bool()> fn) { this->muted_fn_ = std::move(fn); }

  /// Registers the data callback. Setup only: CallbackManager is not safe to grow while the
  /// microphone task is calling it.
  void setup();
  /// Main loop: adopts pending listeners, pumps the ring and reaps closed sessions. `phase` and
  /// `scores` (two wake word slots and the stop word, 0-255) go into every header sent this pass.
  void loop(uint8_t phase, const uint8_t scores[3]);
  /// httpd task. Takes the socket over on success; answers 409 when every listener slot is taken
  /// and 503 when internal RAM is too low to start.
  void handle_request(AsyncWebServerRequest *request);

  /// Main loop, from push_wake_detection. `slot` is the firing track (0-1, 2 for the stop word) or
  /// 0xFF when unknown.
  void note_wake(uint8_t slot);

  uint8_t max_listeners() const { return this->max_listeners_; }
  bool streaming() const { return this->active_.load(std::memory_order_relaxed) != 0; }
  bool xmos_not_ready() const { return this->xmos_not_ready_; }

 protected:
  struct Listener {
    httpd_handle_t hd{nullptr};
    int fd{-1};
    /// Set by free_ctx on the httpd task once the session is gone; loop() may then delete.
    std::atomic<bool> gone{false};
    /// loop() asked httpd to close the session, and stops pumping it.
    bool closing{false};
    uint32_t read_idx{0};
    uint32_t sample_index{0};
    uint32_t dropped{0};
    uint8_t *buf{nullptr};
    size_t len{0};
    size_t off{0};
    uint32_t last_progress_ms{0};
    uint32_t last_sent_ms{0};
  };

  static void free_listener_ctx_(void *ctx);
  void on_audio_(const std::vector<uint8_t> &data);
  /// Builds the next message into `l->buf`. False when there is nothing to send yet.
  bool compose_(Listener *l, uint32_t now, uint8_t phase, const uint8_t scores[3]);
  uint8_t flags_(uint32_t now) const;
  void close_(Listener *l);
  void destroy_(Listener *l);

  microphone::Microphone *mic_{nullptr};
  uint8_t gain_{1};
  uint8_t max_listeners_{2};
  std::function<bool()> xmos_ready_fn_{};
  std::function<bool()> muted_fn_{};

  /// Written by the microphone task, read by the main loop.
  std::atomic<int16_t *> ring_{nullptr};
  std::atomic<uint32_t> write_idx_{0};
  std::atomic<uint32_t> last_audio_ms_{0};
  std::atomic<bool> audio_seen_{false};
  /// Listeners being pumped; the microphone task skips all work while it is 0.
  std::atomic<uint8_t> active_{0};
  /// Mirrors of main-loop state, refreshed each loop() for the microphone task and the httpd task.
  std::atomic<bool> muted_{false};
  bool xmos_not_ready_{false};
  bool usable_{false};
  size_t bytes_per_sample_{4};
  uint8_t channels_{2};

  uint8_t wake_seq_{0};
  uint8_t wake_slot_{0xFF};

  /// Guards pending_ and the size of listeners_; the vectors' contents are main-loop owned.
  Mutex lock_;
  std::vector<Listener *> pending_;
  std::vector<Listener *> listeners_;
};

}  // namespace satellite1_web_ui
}  // namespace esphome

#endif  // USE_SAT1_MIC_MONITOR
