#include "improv_serial_component.h"

#include "esphome/components/logger/logger.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/version.h"

namespace esphome {
namespace improv_serial {

static const char *const TAG = "improv_serial";

void ImprovSerialComponent::setup() {
  global_improv_serial_component = this;
#ifdef USE_ESP32
  this->uart_num_ = logger::global_logger->get_uart_num();
#elif defined(USE_ARDUINO)
  this->hw_serial_ = logger::global_logger->get_hw_serial();
#endif
#ifdef USE_WIFI
  if (wifi::global_wifi_component->has_sta())
    this->state_ = manufacturer_improv_ext::STATE_PROVISIONED;
  else
    wifi::global_wifi_component->start_scanning();
#endif
}

void ImprovSerialComponent::dump_config() { ESP_LOGCONFIG(TAG, "Improv Serial:"); }

optional<uint8_t> ImprovSerialComponent::read_byte_() {
  optional<uint8_t> byte;
  uint8_t data = 0;
#ifdef USE_ESP32
  switch (logger::global_logger->get_uart()) {
    case logger::UART_SELECTION_UART0:
    case logger::UART_SELECTION_UART1:
#if !defined(USE_ESP32_VARIANT_ESP32C3) && !defined(USE_ESP32_VARIANT_ESP32C6) && \
    !defined(USE_ESP32_VARIANT_ESP32C61) && !defined(USE_ESP32_VARIANT_ESP32S2) && !defined(USE_ESP32_VARIANT_ESP32S3)
    case logger::UART_SELECTION_UART2:
#endif
      if (this->uart_num_ >= 0) {
        size_t available;
        uart_get_buffered_data_len(this->uart_num_, &available);
        if (available) {
          uart_read_bytes(this->uart_num_, &data, 1, 0);
          byte = data;
        }
      }
      break;
#if defined(USE_LOGGER_USB_CDC) && defined(CONFIG_ESP_CONSOLE_USB_CDC)
    case logger::UART_SELECTION_USB_CDC:
      if (esp_usb_console_available_for_read()) {
        esp_usb_console_read_buf(reinterpret_cast<char *>(&data), 1);
        byte = data;
      }
      break;
#endif
#ifdef USE_LOGGER_USB_SERIAL_JTAG
    case logger::UART_SELECTION_USB_SERIAL_JTAG:
      if (usb_serial_jtag_read_bytes(reinterpret_cast<char *>(&data), 1, 0))
        byte = data;
      break;
#endif
    default:
      break;
  }
#elif defined(USE_ARDUINO)
  if (this->hw_serial_->available()) {
    this->hw_serial_->readBytes(&data, 1);
    byte = data;
  }
#endif
  return byte;
}

void ImprovSerialComponent::write_data_(const uint8_t *data, const size_t size) {
  this->tx_header_[TX_LENGTH_IDX] = this->tx_header_[TX_TYPE_IDX] == TYPE_RPC_RESPONSE ? size : 1;
  const bool has_data = data != nullptr && size > 0;
  const uint8_t header_checksum_len = has_data ? TX_BUFFER_SIZE - 3 : TX_BUFFER_SIZE - 2;
  const uint8_t header_tx_len = has_data ? TX_BUFFER_SIZE - 3 : TX_BUFFER_SIZE;
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < header_checksum_len; i++)
    checksum += this->tx_header_[i];
  if (has_data)
    for (size_t i = 0; i < size; i++)
      checksum += data[i];
  this->tx_header_[TX_CHECKSUM_IDX] = checksum;
#ifdef USE_ESP32
  switch (logger::global_logger->get_uart()) {
    case logger::UART_SELECTION_UART0:
    case logger::UART_SELECTION_UART1:
#if !defined(USE_ESP32_VARIANT_ESP32C3) && !defined(USE_ESP32_VARIANT_ESP32C6) && \
    !defined(USE_ESP32_VARIANT_ESP32C61) && !defined(USE_ESP32_VARIANT_ESP32S2) && !defined(USE_ESP32_VARIANT_ESP32S3)
    case logger::UART_SELECTION_UART2:
#endif
      uart_write_bytes(this->uart_num_, reinterpret_cast<const char *>(this->tx_header_), header_tx_len);
      if (has_data) {
        uart_write_bytes(this->uart_num_, reinterpret_cast<const char *>(data), size);
        uart_write_bytes(this->uart_num_, reinterpret_cast<const char *>(&this->tx_header_[TX_CHECKSUM_IDX]), 2);
      }
      break;
#if defined(USE_LOGGER_USB_CDC) && defined(CONFIG_ESP_CONSOLE_USB_CDC)
    case logger::UART_SELECTION_USB_CDC:
      esp_usb_console_write_buf(reinterpret_cast<const char *>(this->tx_header_), header_tx_len);
      if (has_data) {
        esp_usb_console_write_buf(reinterpret_cast<const char *>(data), size);
        esp_usb_console_write_buf(reinterpret_cast<const char *>(&this->tx_header_[TX_CHECKSUM_IDX]), 2);
      }
      break;
#endif
#ifdef USE_LOGGER_USB_SERIAL_JTAG
    case logger::UART_SELECTION_USB_SERIAL_JTAG:
      usb_serial_jtag_write_bytes(reinterpret_cast<const char *>(this->tx_header_), header_tx_len,
                                  20 / portTICK_PERIOD_MS);
      if (has_data) {
        usb_serial_jtag_write_bytes(reinterpret_cast<const char *>(data), size, 20 / portTICK_PERIOD_MS);
        usb_serial_jtag_write_bytes(reinterpret_cast<const char *>(&this->tx_header_[TX_CHECKSUM_IDX]), 2,
                                    20 / portTICK_PERIOD_MS);
      }
      break;
#endif
    default:
      break;
  }
#elif defined(USE_ARDUINO)
  this->hw_serial_->write(this->tx_header_, header_tx_len);
  if (has_data) {
    this->hw_serial_->write(data, size);
    this->hw_serial_->write(&this->tx_header_[TX_CHECKSUM_IDX], 2);
  }
#endif
}

void ImprovSerialComponent::loop() {
  if (this->last_read_byte_ && millis() - this->last_read_byte_ > MANUFACTURER_IMPROV_SERIAL_TIMEOUT) {
    this->last_read_byte_ = 0;
    this->rx_buffer_.clear();
    ESP_LOGV(TAG, "Timeout");
  }
  auto byte = this->read_byte_();
  while (byte.has_value()) {
    if (this->parse_improv_serial_byte_(byte.value()))
      this->last_read_byte_ = millis();
    else {
      this->last_read_byte_ = 0;
      this->rx_buffer_.clear();
    }
    byte = this->read_byte_();
  }
#ifdef USE_WIFI
  if (this->state_ == manufacturer_improv_ext::STATE_PROVISIONING && wifi::global_wifi_component->is_connected()) {
    wifi::global_wifi_component->save_wifi_sta(this->connecting_sta_.get_ssid(), this->connecting_sta_.get_password());
    this->connecting_sta_ = {};
    this->cancel_timeout("wifi-connect-timeout");
    this->set_state_(manufacturer_improv_ext::STATE_PROVISIONED);
    auto url = this->build_rpc_settings_response_(manufacturer_improv_ext::WIFI_SETTINGS);
    this->send_response_(url);
  }
#endif
}

bool ImprovSerialComponent::parse_improv_serial_byte_(uint8_t byte) {
  const size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  return manufacturer_improv_ext::parse_improv_serial_byte(
      at, byte, this->rx_buffer_.data(),
      [this](manufacturer_improv_ext::ImprovCommand command) { return this->parse_improv_payload_(command); },
      [this](manufacturer_improv_ext::Error error) { this->set_error_(error); });
}

bool ImprovSerialComponent::parse_improv_payload_(manufacturer_improv_ext::ImprovCommand &command) {
  switch (command.command) {
    case manufacturer_improv_ext::WIFI_SETTINGS:
#ifdef USE_WIFI
      this->connecting_sta_ = {};
      this->connecting_sta_.set_ssid(command.ssid.c_str());
      this->connecting_sta_.set_password(command.password.c_str());
      wifi::global_wifi_component->set_sta(this->connecting_sta_);
      wifi::global_wifi_component->start_connecting(this->connecting_sta_);
      this->set_state_(manufacturer_improv_ext::STATE_PROVISIONING);
      this->set_timeout("wifi-connect-timeout", 30000, [this]() { this->on_wifi_connect_timeout_(); });
#else
      this->set_error_(manufacturer_improv_ext::ERROR_UNKNOWN_RPC);
#endif
      return true;
    case manufacturer_improv_ext::GET_CURRENT_STATE:
      this->set_state_(this->state_);
#ifdef USE_WIFI
      if (this->state_ == manufacturer_improv_ext::STATE_PROVISIONED) {
        auto url = this->build_rpc_settings_response_(manufacturer_improv_ext::GET_CURRENT_STATE);
        this->send_response_(url);
      }
#endif
      return true;
    case manufacturer_improv_ext::GET_DEVICE_INFO: {
      auto info = this->build_version_info_();
      this->send_response_(info);
      return true;
    }
    case manufacturer_improv_ext::GET_WIFI_NETWORKS: {
#ifdef USE_WIFI
      std::vector<std::string> networks;
      for (const auto &scan : wifi::global_wifi_component->get_scan_result()) {
        if (scan.get_is_hidden())
          continue;
        const std::string &ssid = scan.get_ssid();
        bool duplicate = false;
        for (const auto &seen : networks) {
          if (seen == ssid) {
            duplicate = true;
            break;
          }
        }
        if (duplicate)
          continue;
        char rssi_buf[5];
        *int8_to_str(rssi_buf, scan.get_rssi()) = '\0';
        auto data = manufacturer_improv_ext::build_rpc_response(manufacturer_improv_ext::GET_WIFI_NETWORKS,
                                                                {ssid, rssi_buf, YESNO(scan.get_with_auth())}, false);
        this->send_response_(data);
        networks.push_back(ssid);
      }
      {
        auto data = manufacturer_improv_ext::build_rpc_response(manufacturer_improv_ext::GET_WIFI_NETWORKS, {}, false);
        this->send_response_(data);
      }
#else
      this->set_error_(manufacturer_improv_ext::ERROR_UNKNOWN_RPC);
#endif
      return true;
    }
    case manufacturer_improv_ext::TRIGGER_ACTION: {
      ExtAction action(command);
      this->defer([this, action]() { this->action_request_trigger_->trigger(action); });
      auto data =
          manufacturer_improv_ext::build_rpc_response(manufacturer_improv_ext::TRIGGER_ACTION, {"received"}, false);
      this->send_response_(data);
      return true;
    }
    default:
      this->set_error_(manufacturer_improv_ext::ERROR_UNKNOWN_RPC);
      return false;
  }
}

void ImprovSerialComponent::send_action_status(const std::string &action, int status) {
  auto data = manufacturer_improv_ext::build_rpc_response(manufacturer_improv_ext::TRIGGER_ACTION,
                                                          {"status", action, std::to_string(status)}, false);
  this->send_response_(data);
}

void ImprovSerialComponent::set_state_(manufacturer_improv_ext::State state) {
  this->state_ = state;
  this->tx_header_[TX_TYPE_IDX] = TYPE_CURRENT_STATE;
  this->tx_header_[TX_DATA_IDX] = state;
  this->write_data_();
}

void ImprovSerialComponent::set_error_(manufacturer_improv_ext::Error error) {
  this->tx_header_[TX_TYPE_IDX] = TYPE_ERROR_STATE;
  this->tx_header_[TX_DATA_IDX] = error;
  this->write_data_();
}

void ImprovSerialComponent::send_response_(std::vector<uint8_t> &response) {
  if (response.empty() || response.size() > UINT8_MAX) {
    this->set_error_(manufacturer_improv_ext::ERROR_INVALID_RPC);
    return;
  }
  this->tx_header_[TX_TYPE_IDX] = TYPE_RPC_RESPONSE;
  this->write_data_(response.data(), response.size());
}

std::vector<uint8_t> ImprovSerialComponent::build_version_info_() {
#ifdef ESPHOME_PROJECT_NAME
  return manufacturer_improv_ext::build_rpc_response(
      manufacturer_improv_ext::GET_DEVICE_INFO,
      {ESPHOME_PROJECT_NAME, ESPHOME_PROJECT_VERSION, ESPHOME_VARIANT, App.get_name()}, false);
#else
  return manufacturer_improv_ext::build_rpc_response(
      manufacturer_improv_ext::GET_DEVICE_INFO, {"ESPHome", ESPHOME_VERSION, ESPHOME_VARIANT, App.get_name()}, false);
#endif
}

#ifdef USE_WIFI
std::vector<uint8_t> ImprovSerialComponent::build_rpc_settings_response_(
    manufacturer_improv_ext::Command command) {
  std::vector<std::string> urls;
#ifdef USE_MANUFACTURER_IMPROV_SERIAL_NEXT_URL
  char url_buffer[384];
  const size_t len = this->get_formatted_next_url_(url_buffer, sizeof(url_buffer));
  if (len > 0)
    urls.emplace_back(url_buffer, len);
#endif
#ifdef USE_WEBSERVER
  for (const auto &ip : wifi::global_wifi_component->wifi_sta_ip_addresses()) {
    if (!ip.is_ip4())
      continue;
    char ip_buf[network::IP_ADDRESS_BUFFER_SIZE];
    ip.str_to(ip_buf);
    char webserver_url[7 + network::IP_ADDRESS_BUFFER_SIZE + 1 + 5 + 1];
    snprintf(webserver_url, sizeof(webserver_url), "http://%s:%u", ip_buf, USE_WEBSERVER_PORT);
    urls.emplace_back(webserver_url);
    break;
  }
#endif
  return manufacturer_improv_ext::build_rpc_response(command, urls, false);
}

void ImprovSerialComponent::on_wifi_connect_timeout_() {
  this->set_error_(manufacturer_improv_ext::ERROR_UNABLE_TO_CONNECT);
  this->set_state_(manufacturer_improv_ext::STATE_AUTHORIZED);
  wifi::global_wifi_component->clear_sta();
}
#endif

ImprovSerialComponent *global_improv_serial_component = nullptr;

}  // namespace improv_serial
}  // namespace esphome
