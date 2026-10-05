#include "mic_monitor.h"

#ifdef USE_SAT1_MIC_MONITOR

#include <algorithm>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstring>

#include <esp_heap_caps.h>
#include <sys/socket.h>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace satellite1_web_ui {

static const char *const TAG_MM = "web_ui.mic";

/// Below this much free internal RAM a new stream is refused: lwIP copies every send into internal
/// pbufs, and a stream that starts the device swapping heap for audio is the wrong trade.
static constexpr size_t MM_MIN_INTERNAL_FREE = 32 * 1024;
/// A listener whose socket has taken nothing for this long is closed so its slot frees up.
static constexpr uint32_t MM_STALL_MS = 5000;
static constexpr uint32_t MM_KEEPALIVE_MS = 250;
/// No audio for this long reads as "microphone idle" in the header.
static constexpr uint32_t MM_IDLE_MS = 250;
static constexpr uint32_t MM_MAGIC = 0x314D3153;  // "S1M1" little-endian

namespace {

/// web_server_idf's nonblocking_send lives in an anonymous namespace there, so this is a copy: the
/// main loop must never block on a browser that stopped reading.
int mm_nonblocking_send(httpd_handle_t hd, int sockfd, const char *buf, size_t buf_len, int flags) {
  if (buf == nullptr)
    return HTTPD_SOCK_ERR_INVALID;
  const int ret = send(sockfd, buf, buf_len, flags | MSG_DONTWAIT);
  if (ret < 0) {
    const int err = errno;
    if (err == EAGAIN || err == EWOULDBLOCK)
      return HTTPD_SOCK_ERR_TIMEOUT;
    return HTTPD_SOCK_ERR_FAIL;
  }
  return ret;
}

void put_u16(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}

void put_u32(uint8_t *p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
  p[2] = static_cast<uint8_t>(v >> 16);
  p[3] = static_cast<uint8_t>(v >> 24);
}

/// Raw httpd responses reach past web_server_base's default headers, so each one carries the CORS
/// header by hand; the strings are literals because httpd stores the pointers until the send.
void send_raw(AsyncWebServerRequest *request, const char *status, const char *body) {
  httpd_req_t *req = *request;
  httpd_resp_set_status(req, status);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

}  // namespace

void MicMonitor::setup() {
  if (this->mic_ == nullptr)
    return;
  this->mic_->add_data_callback([this](const std::vector<uint8_t> &data) { this->on_audio_(data); });
  this->usable_ = true;
}

void MicMonitor::on_audio_(const std::vector<uint8_t> &data) {
  int16_t *ring = this->ring_.load(std::memory_order_acquire);
  if (ring == nullptr || this->active_.load(std::memory_order_relaxed) == 0)
    return;
  const audio::AudioStreamInfo info = this->mic_->get_audio_stream_info();
  const size_t bps = info.samples_to_bytes(1);
  const uint8_t channels = info.get_channels();
  if (bps == 0 || channels == 0)
    return;
  const size_t frame_bytes = bps * channels;
  const size_t frames = data.size() / frame_bytes;
  const bool muted = this->muted_.load(std::memory_order_relaxed);
  const int32_t gain = this->gain_;
  // MicrophoneSource::process_audio_'s arithmetic for micro_wake_word: Q31 to Q25, times the gain
  // (at most 64, so no overflow), clamped to Q25, back to Q31, top 16 bits. Collapsed:
  // clamp(q25 * gain) >> 10.
  constexpr int32_t q25_max = (1 << 25) - 1;
  constexpr int32_t q25_min = -(1 << 25);

  uint32_t w = this->write_idx_.load(std::memory_order_relaxed);
  const uint8_t *p = data.data();
  for (size_t i = 0; i < frames; i++, p += frame_bytes, w++) {
    int16_t *out = ring + (w & (MM_RING_FRAMES - 1)) * 2;
    if (muted) {
      out[0] = 0;
      out[1] = 0;
      continue;
    }
    const int32_t stt = audio::unpack_audio_sample_to_q31(p, bps);
    const int32_t ww = channels > 1 ? audio::unpack_audio_sample_to_q31(p + bps, bps) : stt;
    out[0] = static_cast<int16_t>(stt >> 16);
    const int32_t boosted = std::clamp((ww >> 6) * gain, q25_min, q25_max);
    out[1] = static_cast<int16_t>(boosted >> 10);
  }
  this->write_idx_.store(w, std::memory_order_release);
  this->last_audio_ms_.store(millis(), std::memory_order_relaxed);
  this->audio_seen_.store(true, std::memory_order_relaxed);
}

void MicMonitor::handle_request(AsyncWebServerRequest *request) {
  if (!this->usable_) {
    request->send(404, "application/json", "{\"ok\":0}");
    return;
  }
  if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) < MM_MIN_INTERNAL_FREE) {
    send_raw(request, "503 Service Unavailable", R"({"ok":0,"low_memory":1})");
    return;
  }

  LockGuard guard{this->lock_};
  if (this->pending_.size() + this->listeners_.size() >= this->max_listeners_) {
    send_raw(request, "409 Conflict", R"({"ok":0,"busy":1})");
    return;
  }
  if (this->ring_.load(std::memory_order_relaxed) == nullptr) {
    RAMAllocator<int16_t> alloc(RAMAllocator<int16_t>::ALLOC_EXTERNAL);
    int16_t *ring = alloc.allocate(MM_RING_FRAMES * 2);
    if (ring == nullptr) {
      send_raw(request, "503 Service Unavailable", R"({"ok":0,"low_memory":1})");
      return;
    }
    memset(ring, 0, MM_RING_FRAMES * 2 * sizeof(int16_t));
    this->ring_.store(ring, std::memory_order_release);
  }
  RAMAllocator<uint8_t> buf_alloc(RAMAllocator<uint8_t>::ALLOC_EXTERNAL);
  uint8_t *buf = buf_alloc.allocate(MM_BUF_LEN);
  if (buf == nullptr) {
    send_raw(request, "503 Service Unavailable", R"({"ok":0,"low_memory":1})");
    return;
  }

  httpd_req_t *req = *request;
  httpd_resp_set_status(req, HTTPD_200);
  httpd_resp_set_type(req, "application/octet-stream");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "X-Content-Type-Options", "nosniff");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  // The first chunk is a keepalive, so the browser has the headers and a first header to read
  // before any audio arrives. Phase and scores are zero until the main loop takes over.
  uint8_t hello[MM_HEADER_LEN] = {};
  put_u32(hello, MM_MAGIC);
  hello[12] = this->flags_(millis());
  hello[16] = 0xFF;
  if (httpd_resp_send_chunk(req, reinterpret_cast<const char *>(hello), sizeof(hello)) != ESP_OK) {
    buf_alloc.deallocate(buf, MM_BUF_LEN);
    return;
  }

  auto *l = new Listener();  // NOLINT(cppcoreguidelines-owning-memory)
  l->hd = req->handle;
  l->fd = httpd_req_to_sockfd(req);
  l->buf = buf;
  req->sess_ctx = l;
  req->free_ctx = MicMonitor::free_listener_ctx_;
  httpd_sess_set_send_override(l->hd, l->fd, mm_nonblocking_send);
  this->pending_.push_back(l);
  ESP_LOGD(TAG_MM, "Listener connected (fd %d)", l->fd);
}

void MicMonitor::free_listener_ctx_(void *ctx) {
  static_cast<Listener *>(ctx)->gone.store(true, std::memory_order_release);
}

void MicMonitor::note_wake(uint8_t slot) {
  this->wake_seq_++;
  this->wake_slot_ = slot;
}

uint8_t MicMonitor::flags_(uint32_t now) const {
  uint8_t flags = 0;
  if (this->muted_.load(std::memory_order_relaxed))
    flags |= MM_FLAG_MUTED;
  if (!this->audio_seen_.load(std::memory_order_relaxed) ||
      now - this->last_audio_ms_.load(std::memory_order_relaxed) > MM_IDLE_MS)
    flags |= MM_FLAG_MIC_IDLE;
  if (this->xmos_not_ready_)
    flags |= MM_FLAG_XMOS_NOT_READY;
  return flags;
}

bool MicMonitor::compose_(Listener *l, uint32_t now, uint8_t phase, const uint8_t scores[3]) {
  int16_t *ring = this->ring_.load(std::memory_order_acquire);
  if (ring == nullptr)
    return false;
  const uint32_t w = this->write_idx_.load(std::memory_order_acquire);
  uint32_t avail = w - l->read_idx;
  if (avail > MM_RING_FRAMES - MM_RING_GUARD) {
    // Lapped, or about to be: jump to half a ring behind the writer and say how much was lost.
    const uint32_t skip = avail - MM_RING_FRAMES / 2;
    l->read_idx += skip;
    l->sample_index += skip;
    l->dropped += skip;
    avail = MM_RING_FRAMES / 2;
  }
  const uint32_t n = std::min(avail, MM_MAX_FRAMES);
  if (n == 0 && now - l->last_sent_ms < MM_KEEPALIVE_MS)
    return false;

  uint8_t *msg = l->buf + 6;
  put_u32(msg, MM_MAGIC);
  put_u32(msg + 4, l->sample_index);
  put_u16(msg + 8, static_cast<uint16_t>(n));
  put_u16(msg + 10, static_cast<uint16_t>(std::min<uint32_t>(l->dropped, 0xFFFF)));
  msg[12] = this->flags_(now);
  msg[13] = phase;
  msg[14] = this->wake_seq_;
  msg[15] = this->wake_slot_;
  msg[16] = scores[0];
  msg[17] = scores[1];
  msg[18] = scores[2];
  msg[19] = 0;
  l->dropped = 0;

  const uint32_t start = l->read_idx & (MM_RING_FRAMES - 1);
  const uint32_t first = std::min(n, MM_RING_FRAMES - start);
  memcpy(msg + MM_HEADER_LEN, ring + start * 2, first * 4);
  if (n > first)
    memcpy(msg + MM_HEADER_LEN + first * 4, ring, (n - first) * 4);
  l->read_idx += n;
  l->sample_index += n;

  const size_t payload = MM_HEADER_LEN + n * 4;
  char head[7];
  snprintf(head, sizeof(head), "%04X\r\n", static_cast<unsigned>(payload));
  memcpy(l->buf, head, 6);
  l->buf[6 + payload] = '\r';
  l->buf[7 + payload] = '\n';
  l->len = payload + MM_CHUNK_OVERHEAD;
  l->off = 0;
  l->last_sent_ms = now;
  return true;
}

void MicMonitor::close_(Listener *l) {
  if (l->closing)
    return;
  l->closing = true;
  httpd_sess_trigger_close(l->hd, l->fd);
}

void MicMonitor::destroy_(Listener *l) {
  RAMAllocator<uint8_t> alloc(RAMAllocator<uint8_t>::ALLOC_EXTERNAL);
  alloc.deallocate(l->buf, MM_BUF_LEN);
  delete l;  // NOLINT(cppcoreguidelines-owning-memory)
}

void MicMonitor::loop(uint8_t phase, const uint8_t scores[3]) {
  if (!this->usable_)
    return;
  this->muted_.store(this->muted_fn_ && this->muted_fn_(), std::memory_order_relaxed);
  this->xmos_not_ready_ = this->xmos_ready_fn_ && !this->xmos_ready_fn_();

  {
    LockGuard guard{this->lock_};
    for (Listener *l : this->pending_) {
      // Starts at "now": the ring holds whatever the last listener heard.
      l->read_idx = this->write_idx_.load(std::memory_order_acquire);
      l->last_progress_ms = millis();
      l->last_sent_ms = l->last_progress_ms;
      this->listeners_.push_back(l);
      this->active_.fetch_add(1, std::memory_order_relaxed);
    }
    this->pending_.clear();
  }
  if (this->listeners_.empty())
    return;

  const uint32_t now = millis();
  bool reap = false;
  for (Listener *l : this->listeners_) {
    if (l->gone.load(std::memory_order_acquire)) {
      reap = true;
      continue;
    }
    if (l->closing)
      continue;
    if (l->off >= l->len && !this->compose_(l, now, phase, scores))
      continue;
    const int sent =
        httpd_socket_send(l->hd, l->fd, reinterpret_cast<const char *>(l->buf + l->off), l->len - l->off, 0);
    if (sent > 0) {
      l->off += sent;
      l->last_progress_ms = now;
    } else if (sent == HTTPD_SOCK_ERR_TIMEOUT) {
      if (now - l->last_progress_ms > MM_STALL_MS) {
        ESP_LOGW(TAG_MM, "Listener stalled for %" PRIu32 " ms, closing (fd %d)", MM_STALL_MS, l->fd);
        this->close_(l);
      }
    } else {
      this->close_(l);
    }
  }
  if (!reap)
    return;

  LockGuard guard{this->lock_};
  auto it = std::remove_if(this->listeners_.begin(), this->listeners_.end(), [this](Listener *l) {
    if (!l->gone.load(std::memory_order_acquire))
      return false;
    ESP_LOGD(TAG_MM, "Listener closed (fd %d)", l->fd);
    this->destroy_(l);
    this->active_.fetch_sub(1, std::memory_order_relaxed);
    return true;
  });
  this->listeners_.erase(it, this->listeners_.end());
}

}  // namespace satellite1_web_ui
}  // namespace esphome

#endif  // USE_SAT1_MIC_MONITOR
