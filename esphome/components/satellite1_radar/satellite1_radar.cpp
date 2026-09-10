#include "satellite1_radar.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cstring>

namespace esphome {
namespace satellite1_radar {

static const char *const TAG = "satellite1_radar";
static constexpr size_t MAX_DETECT_BYTES_PER_LOOP = 128;
static constexpr size_t MAX_DETECT_DRAIN_BYTES = 256;

static const uint8_t LD2410_FRAME_HEADER[] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t LD2450_FRAME_HEADER[] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t LD24XX_DISABLE_CONFIG[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};

/// How long after the last /api/v1/ld2410/live poll the LD2410 stays in engineering mode. The
/// tuner page polls every 500ms, so this is six missed polls: long enough that a slow render or a
/// blocked request does not flap the mode, short enough that closing the tab stops the extra
/// per-gate parsing within a few seconds.
static constexpr uint32_t TUNER_LIVE_IDLE_MS = 3000;

static void drain_uart_bytes_(uart::UARTDevice *uart, size_t max_bytes) {
  size_t drained = 0;
  while (uart->available() && drained < max_bytes) {
    uint8_t discard;
    uart->read_byte(&discard);
    drained++;
  }
}

void Satellite1Radar::setup() {
  // Registered here rather than after detection, and deliberately before web_server can register
  // anything: add_handler appends to a vector that init() later replays in order, first match
  // winning, and this component's setup_priority::DATA (800) runs ahead of web_server's
  // WIFI - 1.0f (249). Nothing currently depends on winning a shared path - the tuner has its own
  // - but registering late would silently give that ordering away.
  //
  // add_handler, not add_handler_without_auth: the tuner can factory-reset the radar and reboot
  // the device, so it belongs behind the same credentials as everything else.
  web_server_base::global_web_server_base->add_handler(&this->tuner_handler_);

  ESP_LOGI(TAG, "Starting mmWave radar auto-detection (%.1fs timeout)...", DETECT_TIMEOUT_MS / 1000.0f);
  if (this->radar_type_text_sensor_ != nullptr)
    this->radar_type_text_sensor_->publish_state("UNKNOWN");
  this->pre_detect_recovery_pending_ = true;
  this->pre_detect_recovery_sent_ms_ = 0;
  this->detection_started_ = true;
  this->detection_complete_ = false;
  this->detected_type_ = RadarType::UNKNOWN;
  this->detect_start_ms_ = millis();
  this->detect_ring_pos_ = 0;
  memset(this->detect_ring_buf_, 0, sizeof(this->detect_ring_buf_));
}

void Satellite1Radar::loop() {
  if (!this->detection_complete_) {
    this->process_detection_();
    return;
  }

  // Normal operation after detection
  if (detected_type_ == RadarType::LD2410 && ld2410_ != nullptr) {
    ld2410_->loop();
    if (this->write_config_pending_.exchange(false)) {
      ld2410_->apply_backend_config();
    }
    // Engineering mode follows whoever is watching the tuner's gate chart. Both the enable and
    // the disable push onto the LD2410's command queue, which only this task may touch, which is
    // why the httpd task records a timestamp and the decision is taken here.
    const uint32_t last_poll = this->tuner_handler_.ld2410_live_poll_ms();
    const bool wanted = last_poll != 0 && (millis() - last_poll) < TUNER_LIVE_IDLE_MS;
    if (wanted != this->ld2410_engineering_on_) {
      if (wanted) {
        ld2410_->enable_engineering_mode();
      } else {
        ld2410_->disable_engineering_mode();
      }
      this->ld2410_engineering_on_ = wanted;
    }
  } else if (detected_type_ == RadarType::LD2450 && ld2450_ != nullptr) {
    ld2450_->loop();
  }
}

void Satellite1Radar::process_detection_() {
  if (!this->detection_started_ || this->detection_complete_) {
    return;
  }

  if (this->pre_detect_recovery_pending_) {
    if (this->pre_detect_recovery_sent_ms_ == 0) {
      drain_uart_bytes_(this, MAX_DETECT_DRAIN_BYTES);
      this->write_array(LD24XX_DISABLE_CONFIG, sizeof(LD24XX_DISABLE_CONFIG));
      this->flush();
      this->pre_detect_recovery_sent_ms_ = millis();
      ESP_LOGD(TAG, "Sent pre-detect disable-config recovery frame");
      return;
    }

    if (millis() - this->pre_detect_recovery_sent_ms_ < PRE_DETECT_RECOVERY_SETTLE_MS) {
      return;
    }

    drain_uart_bytes_(this, MAX_DETECT_DRAIN_BYTES);
    this->pre_detect_recovery_pending_ = false;
  }

  size_t bytes_processed = 0;
  while (this->available() && bytes_processed < MAX_DETECT_BYTES_PER_LOOP) {
    uint8_t byte;
    if (!this->read_byte(&byte))
      break;
    bytes_processed++;

    this->detect_ring_buf_[this->detect_ring_pos_ % 4] = byte;
    this->detect_ring_pos_++;

    if (this->detect_ring_pos_ < 4)
      continue;

    uint8_t seq[4];
    for (int i = 0; i < 4; i++) {
      seq[i] = this->detect_ring_buf_[(this->detect_ring_pos_ - 4 + static_cast<size_t>(i)) % 4];
    }

    if (memcmp(seq, LD2410_FRAME_HEADER, 4) == 0) {
      ESP_LOGI(TAG, "Detected LD2410 radar (frame header 0xF4F3F2F1)");
      drain_uart_bytes_(this, MAX_DETECT_DRAIN_BYTES);
      this->finalize_detection_(RadarType::LD2410);
      return;
    }

    if (memcmp(seq, LD2450_FRAME_HEADER, 4) == 0) {
      ESP_LOGI(TAG, "Detected LD2450 radar (frame header 0xAAFF0300)");
      drain_uart_bytes_(this, MAX_DETECT_DRAIN_BYTES);
      this->finalize_detection_(RadarType::LD2450);
      return;
    }
  }

  if (millis() - this->detect_start_ms_ >= DETECT_TIMEOUT_MS) {
    ESP_LOGI(TAG, "No mmWave radar detected within timeout");
    this->finalize_detection_(RadarType::NONE);
  }
}

void Satellite1Radar::finalize_detection_(RadarType type) {
  this->detected_type_ = type;
  this->detection_complete_ = true;

  if (radar_type_text_sensor_ != nullptr) {
    switch (type) {
      case RadarType::LD2410:
        radar_type_text_sensor_->publish_state("LD2410");
        break;
      case RadarType::LD2450:
        radar_type_text_sensor_->publish_state("LD2450");
        break;
      default:
        radar_type_text_sensor_->publish_state("None");
        break;
    }
  }

  if (type == RadarType::NONE) {
    ESP_LOGI(TAG, "No radar detected, disabling loop");
    this->disable_loop();
    return;
  }

  if (type == RadarType::LD2410) {
    ld2410_ = std::unique_ptr<LD2410Handler>(new LD2410Handler(*this));
    ld2450_.reset();
    ld2410_->set_device_class_indices(this->device_class_meta_);
    ld2410_->set_unit_indices(this->unit_meta_);
    ld2410_->set_icon_indices(this->icon_meta_);
    ld2410_->create_and_register_entities();
    ld2410_->setup();
  } else if (type == RadarType::LD2450) {
    ld2450_ = std::unique_ptr<LD2450Handler>(new LD2450Handler(*this));
    ld2410_.reset();
    ld2450_->set_device_class_indices(this->device_class_meta_);
    ld2450_->set_unit_indices(this->unit_meta_);
    ld2450_->set_icon_indices(this->icon_meta_);
    ld2450_->setup();
    ld2450_->create_and_register_entities();
  }

  this->attach_tuner_();
}

/// Point the already-registered handler at the radar we actually found. Until this runs the
/// handler is mounted but empty, and answers 404 for every radar endpoint - the same answer the
/// old server gave for the radar that was not present.
void Satellite1Radar::attach_tuner_() {
  this->tuner_handler_.clear_registrations();

  if (detected_type_ == RadarType::LD2450) {
    this->tuner_handler_.set_html_content(ld2450_html_gz_, ld2450_html_gz_len_);
    this->tuner_handler_.set_ld2450_handler(ld2450_.get());

    // Left registered for the life of the device rather than gated on someone watching, unlike
    // the LD2410's engineering mode: this callback is two float stores per frame on a path that
    // already ran, so the bookkeeping to switch it off would cost more than it saves.
    if (ld2450_ != nullptr) {
      ld2450_->on_target_update = [this](int target, float x, float y) {
        this->tuner_handler_.update_target(target, x, y);
      };
    }
  } else if (detected_type_ == RadarType::LD2410) {
    this->tuner_handler_.set_html_content(ld2410_html_gz_, ld2410_html_gz_len_);
    this->tuner_handler_.set_ld2410_handler(ld2410_.get());
    this->tuner_handler_.set_ld2410_apply_callback([this]() { this->write_config_pending_.store(true); });
  }
}

void Satellite1Radar::dump_config() {
  ESP_LOGCONFIG(TAG, "Satellite1 Radar:");
  if (!detection_complete_) {
    ESP_LOGCONFIG(TAG, "  Detection: in progress...");
    return;
  }
  switch (detected_type_) {
    case RadarType::LD2410:
      ESP_LOGCONFIG(TAG, "  Detected sensor: LD2410");
      break;
    case RadarType::LD2450:
      ESP_LOGCONFIG(TAG, "  Detected sensor: LD2450");
      break;
    case RadarType::NONE:
      ESP_LOGCONFIG(TAG, "  Detected sensor: None");
      break;
    default:
      ESP_LOGCONFIG(TAG, "  Detected sensor: Unknown");
      break;
  }
  if (detected_type_ == RadarType::LD2410 || detected_type_ == RadarType::LD2450) {
    // The tuner used to need a switch turned on before it existed anywhere, so its address was
    // never worth logging. Now it is always mounted and this is the only place that says where.
    ESP_LOGCONFIG(TAG, "  Tuner UI: %s", RT_URL_ROOT);
  }
}

}  // namespace satellite1_radar
}  // namespace esphome
