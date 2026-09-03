#pragma once

#include "esphome/components/improv_base/improv_base.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "improv_ext.h"

#include <vector>

#ifdef USE_WIFI
#include "esphome/components/wifi/wifi_component.h"
#endif

#ifdef USE_ESP32
#include <driver/uart.h>
#if defined(USE_ESP32_VARIANT_ESP32C3) || defined(USE_ESP32_VARIANT_ESP32C6) || defined(USE_ESP32_VARIANT_ESP32C61) || \
    defined(USE_ESP32_VARIANT_ESP32H2) || defined(USE_ESP32_VARIANT_ESP32S3)
#include <driver/usb_serial_jtag.h>
#include <hal/usb_serial_jtag_ll.h>
#endif
#if defined(USE_ESP32_VARIANT_ESP32S2) || defined(USE_ESP32_VARIANT_ESP32S3)
#include <esp_private/usb_console.h>
#endif
#elif defined(USE_ARDUINO)
#include <HardwareSerial.h>
#endif

namespace esphome {
namespace improv_serial {

static constexpr uint8_t TX_TYPE_IDX = 7;
static constexpr uint8_t TX_LENGTH_IDX = 8;
static constexpr uint8_t TX_DATA_IDX = 9;
static constexpr uint8_t TX_CHECKSUM_IDX = 10;
static constexpr uint8_t TX_BUFFER_SIZE = 12;

enum ImprovSerialType : uint8_t {
  TYPE_CURRENT_STATE = 0x01,
  TYPE_ERROR_STATE = 0x02,
  TYPE_RPC_RESPONSE = 0x04,
};

static const uint16_t MANUFACTURER_IMPROV_SERIAL_TIMEOUT = 100;
static const uint8_t MANUFACTURER_IMPROV_SERIAL_VERSION = 1;

class ExtAction {
 public:
  ExtAction() = default;
  explicit ExtAction(const manufacturer_improv_ext::ImprovCommand &command)
      : action_(command.ssid), url_(command.password) {}
  const std::string &get_action() const { return action_; }
  const optional<std::string> &get_url() const { return url_; }

 protected:
  std::string action_;
  optional<std::string> url_;
};

class ImprovSerialComponent : public Component, public improv_base::ImprovBase {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void send_action_status(const std::string &action, int status);
  Trigger<ExtAction> *get_action_request_trigger() { return this->action_request_trigger_; }

 protected:
  bool parse_improv_serial_byte_(uint8_t byte);
  bool parse_improv_payload_(manufacturer_improv_ext::ImprovCommand &command);
  void set_state_(manufacturer_improv_ext::State state);
  void set_error_(manufacturer_improv_ext::Error error);
  void send_response_(std::vector<uint8_t> &response);
#ifdef USE_WIFI
  void on_wifi_connect_timeout_();
  std::vector<uint8_t> build_rpc_settings_response_(manufacturer_improv_ext::Command command);
#endif
  std::vector<uint8_t> build_version_info_();
  optional<uint8_t> read_byte_();
  void write_data_(const uint8_t *data = nullptr, size_t size = 0);

  uint8_t tx_header_[TX_BUFFER_SIZE] = {'I', 'M', 'P', 'R', 'O', 'V', MANUFACTURER_IMPROV_SERIAL_VERSION,
                                        0,   0,   0,   0,   '\n'};
#ifdef USE_ESP32
  uart_port_t uart_num_;
#elif defined(USE_ARDUINO)
  Stream *hw_serial_{nullptr};
#endif
  std::vector<uint8_t> rx_buffer_;
  uint32_t last_read_byte_{0};
#ifdef USE_WIFI
  wifi::WiFiAP connecting_sta_;
#endif
  manufacturer_improv_ext::State state_{manufacturer_improv_ext::STATE_AUTHORIZED};
  Trigger<ExtAction> *action_request_trigger_ = new Trigger<ExtAction>();
};

extern ImprovSerialComponent *global_improv_serial_component;

template<typename... Ts> class ImprovSendActionStatusAction : public Action<Ts...> {
 public:
  explicit ImprovSendActionStatusAction(ImprovSerialComponent *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(std::string, action)
  TEMPLATABLE_VALUE(int, status)
  void play(const Ts &...x) override { this->parent_->send_action_status(action_.value(x...), status_.value(x...)); }

 protected:
  ImprovSerialComponent *parent_;
};

}  // namespace improv_serial
}  // namespace esphome
