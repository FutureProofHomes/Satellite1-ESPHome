#include "idf_iperf.h"

#include "esp_err.h"
#include "iperf.h"
#include "lwip/inet.h"

#include "esphome/core/log.h"

namespace esphome {
namespace iperf {

static const char *const TAG = "esphome_iperf";

esp_err_t start_tcp_ipv4_client(const char *remote_ip, uint16_t port, uint32_t duration_s,
                                uint32_t interval_s) {
  if (remote_ip == nullptr) {
    ESP_LOGE(TAG, "remote_ip must be set in client mode");
    return ESP_ERR_INVALID_ARG;
  }

  ip4_addr_t address;
  if (!inet_aton(remote_ip, &address)) {
    ESP_LOGE(TAG, "invalid IPv4 address: %s", remote_ip);
    return ESP_ERR_INVALID_ARG;
  }

  iperf_cfg_t config = {};
  config.flag = IPERF_FLAG_CLIENT | IPERF_FLAG_TCP;
  config.type = IPERF_IP_TYPE_IPV4;
  config.sport = port;
  config.dport = port;
  config.time = duration_s ? duration_s : IPERF_DEFAULT_TIME;
  config.interval = interval_s ? interval_s : IPERF_DEFAULT_INTERVAL;
  config.len_send_buf = IPERF_DEFAULT_TCP_TX_LEN;
  config.format = MBITS_PER_SEC;
  config.destination_ip4 = address.addr;

  ESP_LOGI(TAG, "Starting iperf client: %s:%u for %lus", remote_ip, port,
           static_cast<unsigned long>(duration_s));
  return iperf_start(&config);
}

void Iperf::start_client() {
  const auto error = start_tcp_ipv4_client(remote_ip_.c_str(), port_, duration_s_, interval_s_);
  if (error != ESP_OK) {
    ESP_LOGE("iperf", "Failed to start iperf client: %d", static_cast<int>(error));
  }
}

}  // namespace iperf
}  // namespace esphome
