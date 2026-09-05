#include "satellite1.h"
#include "esp_rom_gpio.h"
#include "esphome/core/log.h"

#include <cstdint>

namespace esphome {
namespace satellite1 {

static const char *TAG = "Satellite1";

static inline void write_u32_le(uint8_t *dst, uint32_t value) {
  dst[0] = value & 0xFF;
  dst[1] = (value >> 8) & 0xFF;
  dst[2] = (value >> 16) & 0xFF;
  dst[3] = (value >> 24) & 0xFF;
}

static inline void write_i32_le(uint8_t *dst, int32_t value) {
  write_u32_le(dst, static_cast<uint32_t>(value));
}

static inline int32_t read_i32_le(const uint8_t *src) {
  return static_cast<int32_t>(static_cast<uint32_t>(src[0]) | (static_cast<uint32_t>(src[1]) << 8) |
                              (static_cast<uint32_t>(src[2]) << 16) | (static_cast<uint32_t>(src[3]) << 24));
}

void Satellite1::setup() {
  this->spi_setup();
  this->enable();
  this->transfer_byte(0);
  this->disable();

  if (this->xmos_rst_pin_) {
    this->xmos_rst_pin_->setup();
  }

  memset(this->xmos_fw_version, 0, 5);
  memset(this->xmos_flash_uid, 0, sizeof(this->xmos_flash_uid));
  this->dfu_get_fw_version_();
  this->dfu_get_flash_uid_();
}

void Satellite1::dump_config() {
  ESP_LOGCONFIG(TAG, "Satellite1 config:");
  if (!this->xmos_rst_pin_) {
    ESP_LOGCONFIG(TAG, "    xmos_rst_pin not set up properly.");
  }
}

void Satellite1::loop() {
  switch (this->state) {
    case SAT_DETACHED_STATE:
      if (this->connection_attempts <= MAX_CONNECTION_ATTEMPTS && (millis() - this->last_attempt_timestamp_) > 1000) {
        if (this->connection_attempts == MAX_CONNECTION_ATTEMPTS) {
          this->state_callback_.call();
        } else if (this->check_for_xmos_()) {
          this->state = SAT_XMOS_CONNECTED_STATE;
          this->connection_attempts = 0;
          this->state_callback_.call();
        }
        this->last_attempt_timestamp_ = millis();
        this->connection_attempts++;
      }
      break;
    case SAT_XMOS_CONNECTED_STATE:
    case SAT_FLASH_CONNECTED_STATE:
      break;
  }
}

static std::string prerelease_str(uint8_t pre_idx) {
  switch (pre_idx) {
    case 1:
      return "alpha";
    case 2:
      return "beta";
    case 3:
      return "rc";
    case 4:
      return "dev";
    case 0:  // fallthrough
    default:
      return "";
  }
}

std::string Satellite1::status_string() {
  switch (this->state) {
    case SAT_DETACHED_STATE:
      return "XMOS not responding";

    case SAT_XMOS_CONNECTED_STATE:
      return ("v" + std::to_string(this->xmos_fw_version[0]) + "." + std::to_string(this->xmos_fw_version[1]) + "." +
              std::to_string(this->xmos_fw_version[2]) +
              (this->xmos_fw_version[3] ? "-" + prerelease_str(this->xmos_fw_version[3]) : "") +
              (this->xmos_fw_version[4] ? "." + std::to_string(this->xmos_fw_version[4]) : ""));
    case SAT_FLASH_CONNECTED_STATE:
      return "Flashing Mode";
    default:
      return "";
  }
}

bool Satellite1::request_status_register_update() {
  bool ret = this->transfer(0, 0, NULL, 0);
  uint8_t *arr = this->dc_status_register_;
  return ret;
}

bool Satellite1::transfer(uint8_t resource_id, uint8_t command, uint8_t *payload, uint8_t payload_len) {
  if (this->spi_flash_direct_access_enabled_) {
    return false;
  }

  uint8_t send_recv_buf[256 + 3] = {0};
  const bool is_read = (command & CONTROL_CMD_READ_BIT) != 0;
  const uint8_t read_request_len = payload_len + (is_read ? 1 : 0);
  int status_report_dummies = std::max<int>(0, DC_STATUS_REGISTER::REGISTER_LEN - read_request_len - 1);

  int attempts = 3;
  do {
    send_recv_buf[0] = resource_id;
    send_recv_buf[1] = command;
    send_recv_buf[2] = read_request_len;
    if (payload_len > 0 && payload != nullptr) {
      memcpy(&send_recv_buf[3], payload, payload_len);
    }
    this->enable();
    this->transfer_array(&send_recv_buf[0], payload_len + 3 + status_report_dummies);
    this->disable();
    vTaskDelay(1);
  } while (send_recv_buf[0] == CONTROL_COMMAND_IGNORED_IN_DEVICE && attempts-- > 0);

  if (send_recv_buf[0] == CONTROL_COMMAND_IGNORED_IN_DEVICE) {
    return false;
  }

  // XMOS not responding at all
  if ((send_recv_buf[0] + send_recv_buf[1] + send_recv_buf[2]) == 0) {
    return false;
  }

  // Got status register report
  if (send_recv_buf[0] == DC_RESOURCE::CNTRL_ID && send_recv_buf[1] != DC_RET_STATUS::PAYLOAD_AVAILABLE) {
    memcpy(this->dc_status_register_, &send_recv_buf[2], DC_STATUS_REGISTER::REGISTER_LEN);
    uint8_t *arr = this->dc_status_register_;
  }

  if (is_read) {
    attempts = 10;
    do {
      const uint8_t read_len = std::max<uint8_t>(3, read_request_len);
      memset(send_recv_buf, 0, read_len);
      this->enable();
      this->transfer_array(&send_recv_buf[0], read_len);
      this->disable();

      if (send_recv_buf[0] == CONTROL_COMMAND_IGNORED_IN_DEVICE) {
        vTaskDelay(1);
        continue;
      }

      const bool payload_available = (send_recv_buf[0] == DC_RET_STATUS::PAYLOAD_AVAILABLE);
      const bool legacy_dfu_payload = (resource_id == DC_RESOURCE::DFU_CONTROLLER && command == DC_DFU_CMD::GET_VERSION &&
                                       send_recv_buf[0] == 0);
      const bool legacy_dfu_flash_uid_payload =
          (resource_id == DC_RESOURCE::DFU_CONTROLLER && command == DC_DFU_CMD::GET_FLASH_UID && send_recv_buf[0] == 0);
      const bool legacy_audio_payload = (send_recv_buf[0] == 0 &&
                                         (resource_id == DC_AUDIO_PIPELINE::MIC_OUTPUT_SETTINGS_RES_ID ||
                                          resource_id == DC_AUDIO_PIPELINE::SPEAKER_SETTINGS_RES_ID ||
                                          resource_id == DC_AUDIO_PIPELINE::MIC_INPUT_SETTINGS_RES_ID));
      if (!(payload_available || legacy_dfu_payload || legacy_dfu_flash_uid_payload || legacy_audio_payload)) {
        vTaskDelay(1);
        continue;
      }

      if (read_len < (payload_len + 1)) {
        vTaskDelay(1);
        continue;
      }

      if (payload_len > 0 && payload != nullptr) {
        memcpy(payload, &send_recv_buf[1], payload_len);
      }
      return true;
    } while (attempts-- > 0);

    return false;
  }

  return true;
}

void Satellite1::set_spi_flash_direct_access_mode(bool enable) {
  this->xmos_rst_pin_->digital_write(enable);
  if (enable) {
    this->state = SAT_FLASH_CONNECTED_STATE;
  } else if (this->spi_flash_direct_access_enabled_) {
    this->state = SAT_DETACHED_STATE;
    this->connection_attempts = 0;
  }
  this->spi_flash_direct_access_enabled_ = enable;
  this->state_callback_.call();
}

bool Satellite1::dfu_get_fw_version_() {
  uint8_t version_resp[5];
  if (!this->transfer(DC_RESOURCE::DFU_CONTROLLER, DC_DFU_CMD::GET_VERSION, version_resp, 5)) {
    ESP_LOGW(TAG, "Requesting XMOS version failed");
    return false;
  }

  memcpy(this->xmos_fw_version, version_resp, 5);
  ESP_LOGI(TAG, "XMOS Firmware Version: %s ", this->status_string().c_str());

  return true;
}

bool Satellite1::dfu_get_flash_uid_() {
  uint8_t uid_resp[8] = {0};
  if (!this->transfer(DC_RESOURCE::DFU_CONTROLLER, DC_DFU_CMD::GET_FLASH_UID, uid_resp, sizeof(uid_resp))) {
    ESP_LOGW(TAG, "Requesting XMOS flash UID failed");
    return false;
  }

  memcpy(this->xmos_flash_uid, uid_resp, sizeof(this->xmos_flash_uid));
  ESP_LOGI(TAG, "XMOS Flash UID: %02x %02x %02x %02x %02x %02x %02x %02x", this->xmos_flash_uid[0],
           this->xmos_flash_uid[1], this->xmos_flash_uid[2], this->xmos_flash_uid[3], this->xmos_flash_uid[4],
           this->xmos_flash_uid[5], this->xmos_flash_uid[6], this->xmos_flash_uid[7]);
  return true;
}

bool Satellite1::check_for_xmos_() {
  if (!this->dfu_get_fw_version_()) {
    return false;
  }
  const uint8_t compare_zeros[5] = {0};
  return (memcmp(this->xmos_fw_version, compare_zeros, 5) != 0);
}

void Satellite1::xmos_hardware_reset() {
  this->xmos_rst_pin_->digital_write(1);
  delay(100);
  this->xmos_rst_pin_->digital_write(0);
  delay(100);
}

bool Satellite1::set_mic_gain(int mic_gain) {
  uint8_t payload[20] = {0};
  write_u32_le(payload, DC_AUDIO_PIPELINE::MIC_INPUT_FIELD_MIC_GAIN);
  write_i32_le(payload + 4, static_cast<int32_t>(mic_gain));
  return this->transfer(DC_AUDIO_PIPELINE::MIC_INPUT_SETTINGS_RES_ID, DC_AUDIO_PIPELINE::CMD_SET_SETTINGS_PARTIAL,
                        payload, sizeof(payload));
}

bool Satellite1::get_mic_gain(int *mic_gain_out) {
  if (mic_gain_out == nullptr)
    return false;
  uint8_t payload[16] = {0};
  if (!this->transfer(DC_AUDIO_PIPELINE::MIC_INPUT_SETTINGS_RES_ID, DC_AUDIO_PIPELINE::CMD_GET_SETTINGS, payload,
                      sizeof(payload))) {
    return false;
  }
  *mic_gain_out = read_i32_le(payload);
  return true;
}

bool Satellite1::set_mic_output_i2s_channel_map(uint8_t left, uint8_t right) {
  uint8_t payload[16] = {0};
  write_u32_le(payload, DC_AUDIO_PIPELINE::MIC_OUTPUT_FIELD_I2S_CHANNEL_MAP);
  payload[5] = left;
  payload[6] = right;
  return this->transfer(DC_AUDIO_PIPELINE::MIC_OUTPUT_SETTINGS_RES_ID, DC_AUDIO_PIPELINE::CMD_SET_SETTINGS_PARTIAL,
                        payload, sizeof(payload));
}

bool Satellite1::get_mic_output_i2s_channel_map(uint8_t *left, uint8_t *right) {
  if (left == nullptr || right == nullptr)
    return false;
  uint8_t payload[9] = {0};
  if (!this->transfer(DC_AUDIO_PIPELINE::MIC_OUTPUT_SETTINGS_RES_ID, DC_AUDIO_PIPELINE::CMD_GET_SETTINGS, payload,
                      sizeof(payload))) {
    return false;
  }
  *left = payload[1];
  *right = payload[2];
  return true;
}

bool Satellite1::set_mic_output_pack_extra_upsample_channels(bool enabled) {
  uint8_t payload[16] = {0};
  write_u32_le(payload, DC_AUDIO_PIPELINE::MIC_OUTPUT_FIELD_PACK_EXTRA_UPSAMPLE_CHANNELS);
  payload[4] = enabled ? 1 : 0;
  return this->transfer(DC_AUDIO_PIPELINE::MIC_OUTPUT_SETTINGS_RES_ID, DC_AUDIO_PIPELINE::CMD_SET_SETTINGS_PARTIAL,
                        payload, sizeof(payload));
}

}  // namespace satellite1
}  // namespace esphome
