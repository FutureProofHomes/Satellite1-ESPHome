#include "satellite1.h"
#include <cstdio>
#include "esp_rom_gpio.h"
#include "esphome/core/log.h"

namespace esphome {
namespace satellite1 {

static const char *TAG = "Satellite1";

static const char *dc_status_to_string(uint8_t status) {
  switch (status) {
    case 0:
      return "CONTROL_SUCCESS";
    case 1:
      return "CONTROL_REGISTRATION_FAILED";
    case 2:
      return "CONTROL_BAD_COMMAND";
    case 3:
      return "CONTROL_DATA_LENGTH_ERROR";
    case 4:
      return "CONTROL_OTHER_TRANSPORT_ERROR";
    case 5:
      return "CONTROL_BAD_RESOURCE";
    case 6:
      return "CONTROL_MALFORMED_PACKET";
    case 7:
      return "CONTROL_COMMAND_IGNORED_IN_DEVICE";
    case 8:
      return "CONTROL_ERROR";
    case 64:
      return "SERVICER_COMMAND_RETRY";
    case 65:
      return "SERVICER_WRONG_COMMAND_ID";
    case 66:
      return "SERVICER_WRONG_COMMAND_LEN";
    case 67:
      return "SERVICER_WRONG_PAYLOAD";
    case 68:
      return "SERVICER_QUEUE_FULL";
    case 69:
      return "SERVICER_SPECIAL_COMMAND_ALREADY_ONGOING";
    case 70:
      return "SERVICER_SPECIAL_COMMAND_BUFFER_OVERFLOW";
    case 71:
      return "SERVICER_RESOURCE_ERROR";
    case 72:
      return "SERVICER_SPECIAL_COMMAND_WRONG_ORDER";
    case 73:
      return "SERVICER_SPECIAL_COMMAND_BUF_SIZE_ERROR";
    default:
      return "UNKNOWN_STATUS";
  }
}

static bool is_status_frame_with_error(const uint8_t *buf, size_t len, uint8_t *status) {
  if (buf == nullptr || len < 2) {
    return false;
  }
  if (buf[0] != CONTROL_RESOURCE_CNTRL_ID) {
    return false;
  }
  if (buf[1] == 0) {
    return false;
  }
  if (status != nullptr) {
    *status = buf[1];
  }
  return true;
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
  if (this->check_for_xmos_() && !this->status_register_valid_) {
    this->request_status_register_update();
  }
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
          ESP_LOGW(TAG, "XMOS did not respond after %u connection attempts",
                   static_cast<unsigned>(MAX_CONNECTION_ATTEMPTS));
          this->state_callback_.call();
        } else if (this->check_for_xmos_()) {
          this->state = SAT_XMOS_CONNECTED_STATE;
          this->connection_attempts = 0;
          ESP_LOGI(TAG, "XMOS Firmware Version: %s", this->status_string().c_str());
          this->state_callback_.call();
        }
        this->last_attempt_timestamp_ = millis();
        this->connection_attempts++;
      }
      break;
    case SAT_XMOS_CONNECTED_STATE:
      if (!this->status_refresh_attempted_ ||
          static_cast<uint32_t>(millis() - this->status_refresh_timestamp_) >= STATUS_REFRESH_INTERVAL_MS) {
        this->request_status_register_update(false);
      }
      break;
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

bool Satellite1::request_status_register_update(bool retry) {
  this->status_refresh_timestamp_ = millis();
  this->status_refresh_attempted_ = true;
  bool status_report_received = false;
  if (!this->transfer(0, 0, nullptr, 0, &status_report_received, retry)) {
    return false;
  }
  if (!status_report_received) {
    return false;
  }
  return true;
}

bool Satellite1::get_cached_dc_status(DC_STATUS_REGISTER::register_id reg, uint8_t *value) {
  assert(reg < DC_STATUS_REGISTER::REGISTER_LEN);
  assert(value != nullptr);
  if (!this->status_register_valid_) {
    return false;
  }
  *value = this->dc_status_register_[reg];
  return true;
}

bool Satellite1::transfer(uint8_t resource_id, uint8_t command, uint8_t *payload, uint8_t payload_len,
                          bool *status_report_received, bool retry) {
  if (this->spi_flash_direct_access_enabled_) {
    return false;
  }

  uint8_t send_recv_buf[256 + 3] = {0};
  int status_report_dummies = std::max<int>(0, DC_STATUS_REGISTER::REGISTER_LEN - payload_len - 1);

  for (int attempts = retry ? 3 : 0;; --attempts) {
    send_recv_buf[0] = resource_id;
    send_recv_buf[1] = command;
    send_recv_buf[2] = payload_len + !!(command & CONTROL_CMD_READ_BIT);
    if (payload_len > 0 && payload != nullptr) {
      memcpy(&send_recv_buf[3], payload, payload_len);
    }
    this->enable();
    this->transfer_array(&send_recv_buf[0], payload_len + 3 + status_report_dummies);
    this->disable();
    if (send_recv_buf[0] != CONTROL_COMMAND_IGNORED_IN_DEVICE || attempts == 0) {
      break;
    }
    vTaskDelay(1);
  }

  if (send_recv_buf[0] == CONTROL_COMMAND_IGNORED_IN_DEVICE) {
    return false;
  }

  uint8_t status_code = 0;
  if (is_status_frame_with_error(send_recv_buf, sizeof(send_recv_buf), &status_code)) {
    ESP_LOGD(TAG, "SPI status frame after cmd: res=%u cmd=0x%02X status=%u (%s)", resource_id, command, status_code,
             dc_status_to_string(status_code));
  }

  // XMOS not responding at all
  if ((send_recv_buf[0] + send_recv_buf[1] + send_recv_buf[2]) == 0) {
    ESP_LOGW(TAG, "SPI no response: res=%u cmd=0x%02X rx=[00 00 00]", resource_id, command);
    return false;
  }

  // Got status register report
  if (send_recv_buf[0] == DC_RESOURCE::CNTRL_ID && send_recv_buf[1] != DC_RET_STATUS::PAYLOAD_AVAILABLE) {
    memcpy(this->dc_status_register_, &send_recv_buf[2], DC_STATUS_REGISTER::REGISTER_LEN);
    this->status_register_valid_ = true;
    if (status_report_received != nullptr) {
      *status_report_received = true;
    }
  }

  if (command & CONTROL_CMD_READ_BIT) {
    vTaskDelay(1);
    for (int attempts = retry ? 3 : 0;; --attempts) {
      memset(send_recv_buf, 0, payload_len + 3);
      this->enable();
      this->transfer_array(&send_recv_buf[0], payload_len + 3);
      this->disable();
      if (send_recv_buf[0] != CONTROL_COMMAND_IGNORED_IN_DEVICE || attempts == 0) {
        break;
      }
      vTaskDelay(1);
    }

    if (send_recv_buf[0] == CONTROL_COMMAND_IGNORED_IN_DEVICE) {
      return false;
    }

    if (is_status_frame_with_error(send_recv_buf, sizeof(send_recv_buf), &status_code)) {
      ESP_LOGD(TAG, "SPI status frame during read: res=%u cmd=0x%02X status=%u (%s)", resource_id, command, status_code,
               dc_status_to_string(status_code));
    }

    if (send_recv_buf[0] != RET_STATUS_PAYLOAD_AVAIL) {
      ESP_LOGW(TAG, "SPI read unexpected frame: res=%u cmd=0x%02X rx=[0x%02X 0x%02X 0x%02X]", resource_id, command,
               send_recv_buf[0], send_recv_buf[1], send_recv_buf[2]);
      this->log_last_command_status_(resource_id, command, "unexpected read frame");
      return false;
    }

    if (send_recv_buf[1] != DC_RET_STATUS::CMD_SUCCESS) {
      ESP_LOGW(TAG, "SPI read command failed: res=%u cmd=0x%02X status=%u (%s)", resource_id, command, send_recv_buf[1],
               dc_status_to_string(send_recv_buf[1]));
      return false;
    }

    if (payload_len > 0 && payload != nullptr) {
      memcpy(payload, &send_recv_buf[2], payload_len);
    }
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

  return true;
}

bool Satellite1::dfu_get_flash_serial_() {
  uint8_t serial_resp[8];
  if (!this->transfer(DC_RESOURCE::DFU_CONTROLLER, DC_DFU_CMD::GET_FLASH_SERIAL, serial_resp, sizeof(serial_resp))) {
    ESP_LOGW(TAG, "Requesting XMOS flash serial failed");
    return false;
  }

  char serial_hex[17];
  snprintf(serial_hex, sizeof(serial_hex), "%02X%02X%02X%02X%02X%02X%02X%02X", serial_resp[0], serial_resp[1],
           serial_resp[2], serial_resp[3], serial_resp[4], serial_resp[5], serial_resp[6], serial_resp[7]);
  this->hat_serial_ = serial_hex;

  ESP_LOGI(TAG, "XMOS Flash UID: %s", this->hat_serial_.c_str());
  return true;
}

std::string Satellite1::get_hat_serial() {
  if (!this->hat_serial_.empty()) {
    return this->hat_serial_;
  }

  if (this->state != SAT_XMOS_CONNECTED_STATE) {
    return "";
  }

  if (!this->is_device_ready_()) {
    return "";
  }

  if (!this->dfu_get_flash_serial_()) {
    return "";
  }

  return this->hat_serial_;
}

bool Satellite1::dfu_get_image_status_() {
  uint8_t status_flags = 0;
  if (!this->transfer(DC_RESOURCE::DFU_CONTROLLER, DC_DFU_CMD::GET_IMAGE_STATUS, &status_flags, sizeof(status_flags))) {
    ESP_LOGW(TAG, "Requesting XMOS DFU image status failed");
    return false;
  }

  const bool upgrade_present = (status_flags & (1u << 0)) != 0;
  const bool data_partition_available = (status_flags & (1u << 1)) != 0;
  ESP_LOGI(TAG, "XMOS DFU image status: upgrade_present=%s data_partition_available=%s (0x%02X)",
           upgrade_present ? "true" : "false", data_partition_available ? "true" : "false", status_flags);
  return true;
}

bool Satellite1::is_device_ready_() {
  if (!this->request_status_register_update()) {
    ESP_LOGW(TAG, "XMOS status-register request failed");
    return false;
  }
  uint8_t status = 0;
  if (!this->get_cached_dc_status(DC_STATUS_REGISTER::DEVICE_STATUS, &status)) {
    ESP_LOGW(TAG, "XMOS status-register response was not cached");
    return false;
  }
  if (status != DEVICE_STATUS_READY_VALUE) {
    ESP_LOGW(TAG, "XMOS status-register not ready: 0x%02X", status);
    return false;
  }
  return true;
}

bool Satellite1::read_control_version_(uint8_t *version) {
  if (version == nullptr) {
    return false;
  }

  uint8_t resp = 0;
  if (!this->transfer(CONTROL_SPECIAL_RESID, CONTROL_GET_VERSION, &resp, sizeof(resp))) {
    return false;
  }

  *version = resp;
  return true;
}

bool Satellite1::read_last_command_status_(uint8_t *status) {
  if (status == nullptr) {
    return false;
  }

  uint8_t resp = 0;
  if (!this->transfer(CONTROL_SPECIAL_RESID, CONTROL_GET_LAST_COMMAND_STATUS, &resp, sizeof(resp))) {
    return false;
  }

  *status = resp;
  return true;
}

void Satellite1::log_last_command_status_(uint8_t resource_id, uint8_t command, const char *context) {
  if (this->status_query_in_progress_) {
    return;
  }
  if ((resource_id == CONTROL_SPECIAL_RESID) && (command == CONTROL_GET_LAST_COMMAND_STATUS)) {
    return;
  }

  this->status_query_in_progress_ = true;
  uint8_t status = 0;
  if (this->read_last_command_status_(&status)) {
    ESP_LOGW(TAG, "SPI %s: res=%u cmd=0x%02X status=%u (%s)", context, resource_id, command, status,
             dc_status_to_string(status));
  } else {
    ESP_LOGW(TAG, "SPI %s: res=%u cmd=0x%02X (failed to read last status)", context, resource_id, command);
  }
  this->status_query_in_progress_ = false;
}

bool Satellite1::check_for_xmos_() {
  if (!this->is_device_ready_()) {
    return false;
  }

  uint8_t control_version = 0;
  if (this->read_control_version_(&control_version)) {
    this->control_version_ = control_version;
    ESP_LOGI(TAG, "XMOS Control Protocol Version: 0x%02X", control_version);
  } else {
    ESP_LOGW(TAG, "Failed to read XMOS control protocol version");
  }

  if (!this->dfu_get_fw_version_()) {
    return false;
  }

  if (!this->dfu_get_flash_serial_()) {
    ESP_LOGD(TAG, "XMOS flash UID command unavailable or failed");
  }

  if (!this->dfu_get_image_status_()) {
    ESP_LOGD(TAG, "XMOS DFU image status command unavailable or failed");
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

}  // namespace satellite1
}  // namespace esphome
