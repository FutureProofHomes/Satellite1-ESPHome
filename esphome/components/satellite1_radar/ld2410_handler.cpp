#include "ld2410_handler.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <cmath>
#include <cstring>

namespace esphome {
namespace satellite1_radar {

static const char *const TAG_LD2410 = "satellite1_radar.ld2410";
static constexpr size_t MAX_UART_BYTES_PER_LOOP = 128;
static constexpr uint8_t LD2410_NO_BT_MAC[6] = {0x08, 0x05, 0x04, 0x03, 0x02, 0x01};

void LD2410Handler::setup() {
  ESP_LOGI(TAG_LD2410, "Initializing LD2410 handler");
  if (!buf_) {
    buf_ = std::unique_ptr<uint8_t[]>(new uint8_t[MAX_BUF]());
  }
  config_pref_ = global_preferences->make_preference<LD2410BackendConfig>(fnv1_hash("sat1.ld2410.config"));
  LD2410BackendConfig loaded;
  if (config_pref_.load(&loaded)) {
    this->set_backend_config(loaded);
  }
  // Runs on every boot, but only the first one - the one with nothing in NVS yet - is allowed to
  // take its answer as configuration. After that it is a consistency check; see
  // config_authoritative_.
  query_params();
}

void LD2410Handler::create_and_register_entities() {
  if (this->presence_sensor == nullptr) {
    this->runtime_presence_binary_sensor_.reset(new Satellite1RadarDynamicBinarySensor());
    this->runtime_presence_binary_sensor_->configure_dynamic("Room Presence", ENTITY_CATEGORY_NONE, false,
                                                             this->device_class_meta_.occupancy);
    App.register_binary_sensor(this->runtime_presence_binary_sensor_.get());
    this->presence_sensor = this->runtime_presence_binary_sensor_.get();
  }

  if (this->version_text_sensor == nullptr) {
    this->runtime_radar_firmware_text_sensor_.reset(new Satellite1RadarDynamicTextSensor());
    this->runtime_radar_firmware_text_sensor_->configure_dynamic("Radar Firmware", ENTITY_CATEGORY_DIAGNOSTIC, false,
                                                                 this->icon_meta_.chip);
    App.register_text_sensor(this->runtime_radar_firmware_text_sensor_.get());
    this->version_text_sensor = this->runtime_radar_firmware_text_sensor_.get();
  }

  if (this->target_state_text_sensor == nullptr) {
    this->runtime_target_state_text_sensor_.reset(new Satellite1RadarDynamicTextSensor());
    this->runtime_target_state_text_sensor_->configure_dynamic("Radar Target", ENTITY_CATEGORY_NONE, false,
                                                               this->icon_meta_.motion_sensor);
    App.register_text_sensor(this->runtime_target_state_text_sensor_.get());
    this->target_state_text_sensor = this->runtime_target_state_text_sensor_.get();
  }

  if (this->moving_distance == nullptr) {
    this->runtime_moving_distance_sensor_.reset(new Satellite1RadarDynamicSensor());
    this->runtime_moving_distance_sensor_->configure_dynamic(
        "Radar Moving Distance", ENTITY_CATEGORY_NONE, false, this->device_class_meta_.distance,
        this->unit_meta_.centimeter, 0, true, sensor::STATE_CLASS_MEASUREMENT, 0);
    App.register_sensor(this->runtime_moving_distance_sensor_.get());
    this->moving_distance = this->runtime_moving_distance_sensor_.get();
  }

  if (this->still_distance == nullptr) {
    this->runtime_still_distance_sensor_.reset(new Satellite1RadarDynamicSensor());
    this->runtime_still_distance_sensor_->configure_dynamic(
        "Radar Still Distance", ENTITY_CATEGORY_NONE, false, this->device_class_meta_.distance,
        this->unit_meta_.centimeter, 0, true, sensor::STATE_CLASS_MEASUREMENT, 0);
    App.register_sensor(this->runtime_still_distance_sensor_.get());
    this->still_distance = this->runtime_still_distance_sensor_.get();
  }

  if (this->moving_energy == nullptr) {
    this->runtime_moving_energy_sensor_.reset(new Satellite1RadarDynamicSensor());
    this->runtime_moving_energy_sensor_->configure_dynamic("Radar Moving Energy", ENTITY_CATEGORY_DIAGNOSTIC, true, 0,
                                                           this->unit_meta_.percent, this->icon_meta_.signal, true,
                                                           sensor::STATE_CLASS_MEASUREMENT, 0);
    App.register_sensor(this->runtime_moving_energy_sensor_.get());
    this->moving_energy = this->runtime_moving_energy_sensor_.get();
  }

  if (this->still_energy == nullptr) {
    this->runtime_still_energy_sensor_.reset(new Satellite1RadarDynamicSensor());
    this->runtime_still_energy_sensor_->configure_dynamic("Radar Still Energy", ENTITY_CATEGORY_DIAGNOSTIC, true, 0,
                                                          this->unit_meta_.percent, this->icon_meta_.signal, true,
                                                          sensor::STATE_CLASS_MEASUREMENT, 0);
    App.register_sensor(this->runtime_still_energy_sensor_.get());
    this->still_energy = this->runtime_still_energy_sensor_.get();
  }

  if (this->detection_distance == nullptr) {
    this->runtime_detection_distance_sensor_.reset(new Satellite1RadarDynamicSensor());
    this->runtime_detection_distance_sensor_->configure_dynamic(
        "Radar Detection Distance", ENTITY_CATEGORY_DIAGNOSTIC, true, this->device_class_meta_.distance,
        this->unit_meta_.centimeter, 0, true, sensor::STATE_CLASS_MEASUREMENT, 0);
    App.register_sensor(this->runtime_detection_distance_sensor_.get());
    this->detection_distance = this->runtime_detection_distance_sensor_.get();
  }

  if (this->light_sensor == nullptr) {
    this->runtime_light_sensor_.reset(new Satellite1RadarDynamicSensor());
    this->runtime_light_sensor_->configure_dynamic("Radar Illuminance", ENTITY_CATEGORY_DIAGNOSTIC, true,
                                                   this->device_class_meta_.illuminance, 0, 0, true,
                                                   sensor::STATE_CLASS_MEASUREMENT);
    App.register_sensor(this->runtime_light_sensor_.get());
    this->light_sensor = this->runtime_light_sensor_.get();
  }

  if (this->runtime_factory_reset_button_ == nullptr) {
    this->runtime_factory_reset_button_.reset(new Satellite1RadarButton());
    this->runtime_factory_reset_button_->configure_dynamic("Radar Factory Reset", ENTITY_CATEGORY_DIAGNOSTIC, false,
                                                           this->icon_meta_.factory);
    this->runtime_factory_reset_button_->add_on_press_callback([this]() { this->factory_reset(); });
    App.register_button(this->runtime_factory_reset_button_.get());
  }

  if (this->runtime_restart_button_ == nullptr) {
    this->runtime_restart_button_.reset(new Satellite1RadarButton());
    this->runtime_restart_button_->configure_dynamic("Radar Restart", ENTITY_CATEGORY_DIAGNOSTIC, false,
                                                     this->icon_meta_.restart);
    this->runtime_restart_button_->add_on_press_callback([this]() { this->restart(); });
    App.register_button(this->runtime_restart_button_.get());
  }

  if (this->runtime_query_params_button_ == nullptr) {
    this->runtime_query_params_button_.reset(new Satellite1RadarButton());
    this->runtime_query_params_button_->configure_dynamic("Radar Update Sensors", ENTITY_CATEGORY_DIAGNOSTIC, false,
                                                          this->icon_meta_.database_refresh);
    this->runtime_query_params_button_->add_on_press_callback([this]() { this->query_params(); });
    App.register_button(this->runtime_query_params_button_.get());
  }
}

void LD2410Handler::loop() {
  if (!buf_)
    return;

  size_t bytes_processed = 0;
  while (uart_.available() && bytes_processed < MAX_UART_BYTES_PER_LOOP) {
    uint8_t byte;
    if (!uart_.read_byte(&byte))
      break;
    bytes_processed++;

    if (buf_pos_ < MAX_BUF)
      buf_[buf_pos_++] = byte;
    else
      buf_pos_ = 0;

    if (buf_pos_ < 4)
      continue;

    bool is_data = (buf_[0] == 0xF4 && buf_[1] == 0xF3 && buf_[2] == 0xF2 && buf_[3] == 0xF1);
    bool is_cmd = (buf_[0] == 0xFD && buf_[1] == 0xFC && buf_[2] == 0xFB && buf_[3] == 0xFA);

    if (!is_data && !is_cmd) {
      memmove(buf_.get(), buf_.get() + 1, buf_pos_ - 1);
      buf_pos_--;
      continue;
    }

    if (is_data && buf_pos_ >= 8) {
      uint16_t data_len = to_uint16_(buf_[4], buf_[5]);
      size_t frame_len = 4u + 2u + data_len + 4u;

      if (buf_pos_ >= frame_len) {
        if (buf_[frame_len - 4] == 0xF8 && buf_[frame_len - 3] == 0xF7 && buf_[frame_len - 2] == 0xF6 &&
            buf_[frame_len - 1] == 0xF5) {
          parse_data_frame_(buf_.get(), frame_len);
        }
        buf_pos_ = 0;
        continue;
      }
    }

    if (is_cmd && buf_pos_ >= 8) {
      uint16_t data_len = to_uint16_(buf_[4], buf_[5]);
      size_t frame_len = 4u + 2u + data_len + 4u;

      if (buf_pos_ >= frame_len) {
        if (buf_[frame_len - 4] == 0x04 && buf_[frame_len - 3] == 0x03 && buf_[frame_len - 2] == 0x02 &&
            buf_[frame_len - 1] == 0x01) {
          handle_ack_frame_(buf_.get(), frame_len);
        }
        buf_pos_ = 0;
        continue;
      }
    }

    if (buf_pos_ >= MAX_BUF - 1) {
      buf_pos_ = 0;
    }
  }

  process_queue_();

  if (param_readback_pending_ && static_cast<int32_t>(millis() - param_readback_due_ms_) >= 0) {
    param_readback_pending_ = false;
    this->query_params();
    ESP_LOGI(TAG_LD2410, "Queued parameter readback after module restart");
  }
}

void LD2410Handler::factory_reset() {
  queue_enter_config_();
  queue_config_command_(0x00A2, nullptr, 0);
  // The datasheet is explicit that restored defaults do not take effect until the module restarts,
  // so the reset is followed by one rather than left half-applied until something else reboots it.
  queue_config_command_(0x00A3, nullptr, 0);
  queue_exit_config_();

  // The one case where the module legitimately outranks the stored configuration: the point of a
  // factory reset is to end up on the module's defaults, so stand down and let the readback below
  // adopt whatever it reports once it is back up.
  config_authoritative_ = false;
  bt_module_state_known_ = false;
  param_readback_pending_ = true;
  param_readback_due_ms_ = millis() + PARAM_READBACK_DELAY_MS;

  ESP_LOGI(TAG_LD2410, "Factory reset queued");
}

void LD2410Handler::restart() {
  queue_enter_config_();
  queue_config_command_(0x00A3, nullptr, 0);
  queue_exit_config_();
  ESP_LOGI(TAG_LD2410, "Restart queued");
}

void LD2410Handler::query_params() {
  queue_enter_config_();
  queue_config_command_(0x00A0, nullptr, 0);
  uint8_t mac_query_data[2] = {0x01, 0x00};
  queue_config_command_(0x00A5, mac_query_data, 2);
  queue_config_command_(0x0061, nullptr, 0);
  queue_config_command_(0x00AB, nullptr, 0);
  queue_exit_config_();
  ESP_LOGI(TAG_LD2410, "Parameter query queued");
}

void LD2410Handler::query_mac_address() {
  queue_enter_config_();
  uint8_t data[2] = {0x01, 0x00};
  queue_config_command_(0x00A5, data, 2);
  queue_exit_config_();
  ESP_LOGI(TAG_LD2410, "MAC query queued");
}

void LD2410Handler::write_gate_config() {
  queue_enter_config_();

  uint32_t max_move = static_cast<uint32_t>(config_.max_move_gate);
  uint32_t max_still = static_cast<uint32_t>(config_.max_still_gate);
  uint32_t tout = static_cast<uint32_t>(config_.timeout_seconds);

  uint8_t d60[18];
  d60[0] = 0x00;
  d60[1] = 0x00;
  put_uint32_le_(d60 + 2, max_move);
  d60[6] = 0x01;
  d60[7] = 0x00;
  put_uint32_le_(d60 + 8, max_still);
  d60[12] = 0x02;
  d60[13] = 0x00;
  put_uint32_le_(d60 + 14, tout);
  queue_config_command_(0x0060, d60, 18);

  for (size_t g = 0; g < NUM_GATES; g++) {
    uint32_t mv = static_cast<uint32_t>(config_.gate_move_threshold[g]);
    uint32_t sv = static_cast<uint32_t>(config_.gate_still_threshold[g]);
    uint8_t d64[10];
    put_uint16_le_(d64, static_cast<uint16_t>(g));
    put_uint32_le_(d64 + 2, mv);
    put_uint32_le_(d64 + 6, sv);
    queue_config_command_(0x0064, d64, 10);
  }

  queue_exit_config_();
  ESP_LOGI(TAG_LD2410, "Gate configuration queued");
}

void LD2410Handler::set_distance_resolution(bool fine) {
  queue_enter_config_();
  uint8_t data[2];
  put_uint16_le_(data, fine ? 0x0001 : 0x0000);
  queue_config_command_(0x00AA, data, 2);
  queue_exit_config_();
  config_.distance_resolution = fine ? 1 : 0;
  save_backend_config_();
  ESP_LOGI(TAG_LD2410, "Distance resolution change queued (%s)", fine ? "0.2m" : "0.75m");
}

void LD2410Handler::set_bluetooth_enabled(bool enabled) {
  queue_enter_config_();
  uint8_t data[2];
  put_uint16_le_(data, enabled ? 0x0001 : 0x0000);
  queue_config_command_(0x00A4, data, 2);
  queue_config_command_(0x00A3, nullptr, 0);
  queue_exit_config_();
  ESP_LOGI(TAG_LD2410, "Bluetooth %s queued with restart", enabled ? "enable" : "disable");
}

bool LD2410Handler::validate_backend_config_(LD2410BackendConfig &cfg) const {
  if (cfg.max_move_gate > 8 || cfg.max_still_gate > 8)
    return false;
  for (size_t g = 0; g < NUM_GATES; g++) {
    if (cfg.gate_move_threshold[g] > 100 || cfg.gate_still_threshold[g] > 100)
      return false;
  }
  if (cfg.distance_resolution > 1)
    return false;
  return true;
}

void LD2410Handler::save_backend_config_() {
  config_pref_.save(&config_);
  global_preferences->sync();
}

bool LD2410Handler::set_backend_config(const LD2410BackendConfig &cfg) {
  LD2410BackendConfig candidate = cfg;
  if (!validate_backend_config_(candidate))
    return false;
  config_ = candidate;
  // From here on the module's own parameters are no longer allowed to overwrite this. Covers both
  // callers: a write from the tuner, and setup() replaying a configuration out of NVS.
  config_authoritative_ = true;
  save_backend_config_();
  return true;
}

void LD2410Handler::apply_backend_config() {
  this->write_gate_config();
  this->set_distance_resolution(config_.distance_resolution == 1);

  // Deliberately conditional. set_bluetooth_enabled() queues a module restart alongside the
  // command, because the radio only changes state across one - so calling it unconditionally meant
  // every threshold tweak from the tuner rebooted the radar, dropped presence detection for about
  // a second, and scheduled a parameter readback that used to overwrite the very settings this
  // apply had just written. Only send it when the module actually disagrees; if its state has
  // never been observed, send it once to establish a known state.
  if (!bt_module_state_known_ || bt_module_enabled_ != config_.bluetooth_enabled) {
    this->set_bluetooth_enabled(config_.bluetooth_enabled);
  }
}

void LD2410Handler::enable_engineering_mode() {
  queue_enter_config_();
  queue_config_command_(0x0062, nullptr, 0);
  queue_exit_config_();
  ESP_LOGI(TAG_LD2410, "Engineering mode enable queued");
}

void LD2410Handler::disable_engineering_mode() {
  queue_enter_config_();
  queue_config_command_(0x0063, nullptr, 0);
  queue_exit_config_();
  ESP_LOGI(TAG_LD2410, "Engineering mode disable queued");
}

uint16_t LD2410Handler::to_uint16_(uint8_t lo, uint8_t hi) { return (static_cast<uint16_t>(hi) << 8) | lo; }

void LD2410Handler::process_queue_() {
  if (waiting_for_ack_) {
    if (millis() - ack_wait_start_ > ACK_TIMEOUT_MS) {
      ESP_LOGW(TAG_LD2410, "Command ACK timeout, skipping");
      waiting_for_ack_ = false;
    }
    return;
  }

  QueuedCmd cmd;
  if (!dequeue_frame_(cmd))
    return;

  uart_.write_array(cmd.frame, cmd.len);
  waiting_for_ack_ = true;
  ack_wait_start_ = millis();
}

void LD2410Handler::on_ack_received_() {
  if (!waiting_for_ack_)
    return;
  waiting_for_ack_ = false;
}

void LD2410Handler::enqueue_frame_(const QueuedCmd &cmd) {
  if (cmd_queue_count_ == MAX_CMD_QUEUE_DEPTH) {
    cmd_queue_head_ = (cmd_queue_head_ + 1) % MAX_CMD_QUEUE_DEPTH;
    cmd_queue_count_--;
    ESP_LOGW(TAG_LD2410, "Command queue full, dropping oldest");
  }

  const size_t tail = (cmd_queue_head_ + cmd_queue_count_) % MAX_CMD_QUEUE_DEPTH;
  cmd_queue_[tail] = cmd;
  cmd_queue_count_++;
}

bool LD2410Handler::dequeue_frame_(QueuedCmd &cmd) {
  if (cmd_queue_count_ == 0)
    return false;

  cmd = cmd_queue_[cmd_queue_head_];
  cmd_queue_head_ = (cmd_queue_head_ + 1) % MAX_CMD_QUEUE_DEPTH;
  cmd_queue_count_--;
  return true;
}

void LD2410Handler::queue_enter_config_() {
  QueuedCmd cmd{};
  static const uint8_t frame[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
  memcpy(cmd.frame, frame, sizeof(frame));
  cmd.len = sizeof(frame);
  enqueue_frame_(cmd);
}

void LD2410Handler::queue_exit_config_() {
  QueuedCmd cmd{};
  static const uint8_t frame[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
  memcpy(cmd.frame, frame, sizeof(frame));
  cmd.len = sizeof(frame);
  enqueue_frame_(cmd);
}

void LD2410Handler::queue_config_command_(uint16_t command, const uint8_t *data, size_t data_len) {
  QueuedCmd cmd{};
  size_t pos = 0;

  cmd.frame[pos++] = 0xFD;
  cmd.frame[pos++] = 0xFC;
  cmd.frame[pos++] = 0xFB;
  cmd.frame[pos++] = 0xFA;

  uint16_t payload_len = static_cast<uint16_t>(2 + data_len);
  cmd.frame[pos++] = payload_len & 0xFF;
  cmd.frame[pos++] = (payload_len >> 8) & 0xFF;

  cmd.frame[pos++] = command & 0xFF;
  cmd.frame[pos++] = (command >> 8) & 0xFF;

  if (data != nullptr && data_len > 0 && (pos + data_len) < (MAX_CMD_FRAME - 4)) {
    memcpy(cmd.frame + pos, data, data_len);
    pos += data_len;
  }

  cmd.frame[pos++] = 0x04;
  cmd.frame[pos++] = 0x03;
  cmd.frame[pos++] = 0x02;
  cmd.frame[pos++] = 0x01;

  cmd.len = pos;
  enqueue_frame_(cmd);
}

void LD2410Handler::parse_data_frame_(const uint8_t *buf, size_t len) {
  // Frame layout: 4(header) + 2(length) + payload_len + 4(footer)
  // payload_len_min = 11 -> len >= 21
  if (buf == nullptr || len < 21)
    return;

  uint8_t data_type = buf[6];
  uint8_t head_byte = buf[7];

  if (head_byte != 0xAA)
    return;

  uint8_t target_state = buf[8];

  bool has_moving = (target_state == 0x01 || target_state == 0x03);
  bool has_target = (target_state != 0x00);

  uint16_t move_dist = to_uint16_(buf[9], buf[10]);
  uint8_t move_energy_val = buf[11];

  uint16_t still_dist = to_uint16_(buf[12], buf[13]);
  uint8_t still_energy_val = buf[14];

  uint16_t detect_dist = to_uint16_(buf[15], buf[16]);

  if (presence_sensor != nullptr)
    presence_sensor->publish_state(has_target);

  const char *target_activity = !has_target ? "Clear" : has_moving ? "Moving" : "Still";
  debounce_target_state_(target_activity, TARGET_STATE_STREAK_THRESHOLD);

  publish_sensor_(moving_distance, static_cast<float>(move_dist));
  publish_sensor_(still_distance, static_cast<float>(still_dist));
  publish_sensor_(moving_energy, static_cast<float>(move_energy_val));
  publish_sensor_(still_energy, static_cast<float>(still_energy_val));
  publish_sensor_(detection_distance, static_cast<float>(detect_dist));

  if (data_type == 0x01 && len >= (19 + 2 * NUM_GATES + 4)) {
    size_t offset = 19;

    for (size_t g = 0; g < NUM_GATES && (offset + g) < (len - 4); g++) {
      gate_move_energy_values_[g] = static_cast<float>(buf[offset + g]);
    }
    offset += NUM_GATES;

    for (size_t g = 0; g < NUM_GATES && (offset + g) < (len - 4); g++) {
      gate_still_energy_values_[g] = static_cast<float>(buf[offset + g]);
    }
    offset += NUM_GATES;

    if (offset < len - 4) {
      publish_sensor_(light_sensor, static_cast<float>(buf[offset]));
    }
  }
}

void LD2410Handler::handle_ack_frame_(const uint8_t *buf, size_t len) {
  if (len < 10)
    return;

  uint16_t cmd_word = to_uint16_(buf[6], buf[7]);
  uint8_t status = buf[8];

  if (status != 0) {
    ESP_LOGW(TAG_LD2410, "Command 0x%04X failed (status=%u)", static_cast<unsigned int>(cmd_word),
             static_cast<unsigned int>(status));
  }

  if (cmd_word == 0x01A0 && status == 0 && len >= 18) {
    char ver[32];
    snprintf(ver, sizeof(ver), "V%u.%02X.%02X%02X%02X%02X", static_cast<unsigned int>(buf[13]),
             static_cast<unsigned int>(buf[12]), static_cast<unsigned int>(buf[17]), static_cast<unsigned int>(buf[16]),
             static_cast<unsigned int>(buf[15]), static_cast<unsigned int>(buf[14]));
    if (version_text_sensor != nullptr)
      version_text_sensor->publish_state(ver);
    ESP_LOGI(TAG_LD2410, "Firmware version: %s", ver);
  }

  if (cmd_word == 0x0161 && status == 0 && len >= PARAM_ACK_FRAME_LEN) {
    // Payload offsets, counted from the start of the frame. buf[10] is the 0xAA marker that opens
    // the parameter block and buf[11] is the module's maximum supported distance gate; neither is
    // a configured value. Reading the configured gates from 10 and 11 instead of 12 and 13 shifted
    // every field that follows by two bytes, so max_move took the value of the 0xAA marker (170,
    // clamped to 8), both threshold arrays slid two gates to the right, and the duration was
    // assembled out of the last two still sensitivities - two gates at 20 reading back as a
    // timeout of 5140 seconds. The misread was then persisted, which is what silently reverted
    // every tuner write about two seconds after it was applied.
    static constexpr size_t OFF_MAX_MOVE_GATE = 12;
    static constexpr size_t OFF_MAX_STILL_GATE = 13;
    static constexpr size_t OFF_MOVE_THRESHOLDS = 14;
    static constexpr size_t OFF_STILL_THRESHOLDS = OFF_MOVE_THRESHOLDS + NUM_GATES;
    static constexpr size_t OFF_DURATION = OFF_STILL_THRESHOLDS + NUM_GATES;

    const uint8_t max_move = buf[OFF_MAX_MOVE_GATE];
    const uint8_t max_still = buf[OFF_MAX_STILL_GATE];

    if (!config_authoritative_) {
      config_.max_move_gate = (max_move > 8) ? 8 : max_move;
      config_.max_still_gate = (max_still > 8) ? 8 : max_still;

      for (size_t g = 0; g < NUM_GATES; g++) {
        const uint8_t move_val = buf[OFF_MOVE_THRESHOLDS + g];
        const uint8_t still_val = buf[OFF_STILL_THRESHOLDS + g];
        config_.gate_move_threshold[g] = (move_val > 100) ? 100 : move_val;
        config_.gate_still_threshold[g] = (still_val > 100) ? 100 : still_val;
      }

      config_.timeout_seconds = to_uint16_(buf[OFF_DURATION], buf[OFF_DURATION + 1]);

      save_backend_config_();
      ESP_LOGI(TAG_LD2410, "Adopted module parameters: max_move=%u max_still=%u timeout=%us",
               static_cast<unsigned int>(config_.max_move_gate), static_cast<unsigned int>(config_.max_still_gate),
               static_cast<unsigned int>(config_.timeout_seconds));
    } else {
      // Purely informational from here on: the stored configuration stands. A mismatch means the
      // module has not caught up with the last apply, which is worth seeing in a log but must not
      // be written back over what the user asked for.
      const uint16_t module_timeout = to_uint16_(buf[OFF_DURATION], buf[OFF_DURATION + 1]);
      if (max_move != config_.max_move_gate || max_still != config_.max_still_gate ||
          module_timeout != config_.timeout_seconds) {
        ESP_LOGW(TAG_LD2410,
                 "Module parameters differ from stored config (module: max_move=%u max_still=%u timeout=%us; "
                 "stored: max_move=%u max_still=%u timeout=%us)",
                 static_cast<unsigned int>(max_move), static_cast<unsigned int>(max_still),
                 static_cast<unsigned int>(module_timeout), static_cast<unsigned int>(config_.max_move_gate),
                 static_cast<unsigned int>(config_.max_still_gate), static_cast<unsigned int>(config_.timeout_seconds));
      } else {
        ESP_LOGI(TAG_LD2410, "Module parameters match stored config (max_move=%u max_still=%u timeout=%us)",
                 static_cast<unsigned int>(max_move), static_cast<unsigned int>(max_still),
                 static_cast<unsigned int>(module_timeout));
      }
    }
  }

  if (cmd_word == 0x0162) {
    if (status == 0) {
      ESP_LOGI(TAG_LD2410, "Engineering mode enabled");
    } else {
      ESP_LOGW(TAG_LD2410, "Failed to enable engineering mode");
    }
  }

  if (cmd_word == 0x0163) {
    if (status == 0) {
      ESP_LOGI(TAG_LD2410, "Engineering mode disabled");
    }
  }

  if (cmd_word == 0x01AB && status == 0 && len >= 12) {
    uint16_t res = to_uint16_(buf[10], buf[11]);
    bool fine = (res == 0x0001);
    if (!config_authoritative_) {
      config_.distance_resolution = fine ? 1 : 0;
      save_backend_config_();
    }
    ESP_LOGI(TAG_LD2410, "Distance resolution: %s", fine ? "0.2m" : "0.75m");
  }

  if (cmd_word == 0x01A5 && status == 0 && len >= 16) {
    const bool has_bt_mac = memcmp(&buf[10], LD2410_NO_BT_MAC, sizeof(LD2410_NO_BT_MAC)) != 0;

    // Always recorded: this is the only way to learn what the module's radio is actually doing,
    // and apply_backend_config() needs it to decide whether the Bluetooth command - which drags a
    // module restart along with it - has to be sent at all.
    bt_module_enabled_ = has_bt_mac;
    bt_module_state_known_ = true;

    if (!config_authoritative_) {
      config_.bluetooth_enabled = has_bt_mac;
      save_backend_config_();
    }
    ESP_LOGI(TAG_LD2410, "Bluetooth state from MAC query: %s", has_bt_mac ? "enabled" : "disabled");
  }

  if (cmd_word == 0x01A4 && status == 0) {
    param_readback_pending_ = true;
    param_readback_due_ms_ = millis() + PARAM_READBACK_DELAY_MS;
    ESP_LOGI(TAG_LD2410, "Bluetooth command acknowledged, scheduling readback");
  }

  on_ack_received_();
}

void LD2410Handler::publish_sensor_(sensor::Sensor *s, float val) {
  if (s != nullptr)
    s->publish_state(val);
}

void LD2410Handler::debounce_target_state_(const std::string &raw, int threshold) {
  if (target_state_text_sensor == nullptr)
    return;
  if (threshold <= 0 || pub_target_state_.empty()) {
    pub_target_state_ = raw;
    target_state_text_sensor->publish_state(raw);
    return;
  }
  if (raw == pub_target_state_) {
    streak_target_state_ = 0;
    return;
  }
  if (raw == cand_target_state_) {
    streak_target_state_++;
  } else {
    cand_target_state_ = raw;
    streak_target_state_ = 1;
  }
  if (streak_target_state_ >= threshold) {
    pub_target_state_ = raw;
    target_state_text_sensor->publish_state(raw);
    streak_target_state_ = 0;
  }
}

void LD2410Handler::put_uint16_le_(uint8_t *buf, uint16_t val) {
  buf[0] = val & 0xFF;
  buf[1] = (val >> 8) & 0xFF;
}

void LD2410Handler::put_uint32_le_(uint8_t *buf, uint32_t val) {
  buf[0] = val & 0xFF;
  buf[1] = (val >> 8) & 0xFF;
  buf[2] = (val >> 16) & 0xFF;
  buf[3] = (val >> 24) & 0xFF;
}

}  // namespace satellite1_radar
}  // namespace esphome
