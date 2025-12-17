#include "idf_iperf.h"

#include "esp_err.h"
#include "iperf.h" 
#include "lwip/inet.h"  

#include "esphome/core/log.h"
#include "esphome/components/network/util.h"

namespace esphome {
namespace iperf {

static const char *TAG = "esphome_iperf";

typedef enum {
    ESPHOME_IPERF_DIR_REMOTE_TO_ESP,  // ESP acts as server
    ESPHOME_IPERF_DIR_ESP_TO_REMOTE,  // ESP acts as client
} esphome_iperf_direction_t;


esp_err_t esphome_iperf_start_tcp_ipv4(
    esphome_iperf_direction_t dir,
    const char *remote_ip,
    uint16_t port,
    uint32_t duration_s,
    uint32_t interval_s
)
{
    iperf_cfg_t cfg = {0};

    // Flags: TCP, client/server
    if (dir == ESPHOME_IPERF_DIR_REMOTE_TO_ESP) {
        // ESP = server
        cfg.flag = IPERF_FLAG_SERVER | IPERF_FLAG_TCP;
    } else {
        // ESP = client
        cfg.flag = IPERF_FLAG_CLIENT | IPERF_FLAG_TCP;
    }

    // IP type: IPv4
    cfg.type = IPERF_IP_TYPE_IPV4;

    // Port
    cfg.sport = port ? port : IPERF_DEFAULT_PORT;
    cfg.dport = port ? port : IPERF_DEFAULT_PORT;

    // Duration / interval
    cfg.time     = duration_s ? duration_s : IPERF_DEFAULT_TIME;
    cfg.interval = interval_s ? interval_s : IPERF_DEFAULT_INTERVAL;

    // Buffers and units
    cfg.len_send_buf = IPERF_DEFAULT_TCP_TX_LEN;
    cfg.format       = MBITS_PER_SEC;

    // Destination only matters in CLIENT mode
    if (dir == ESPHOME_IPERF_DIR_ESP_TO_REMOTE) {
        if (!remote_ip) {
            ESP_LOGE(TAG, "remote_ip must be set in client mode");
            return ESP_ERR_INVALID_ARG;
        }
        ip4_addr_t addr;
        if (!inet_aton(remote_ip, &addr)) {
            ESP_LOGE(TAG, "invalid IPv4 address: %s", remote_ip);
            return ESP_ERR_INVALID_ARG;
        }
        cfg.destination_ip4 = addr.addr;
    }

    ESP_LOGI(TAG, "Starting iperf: dir=%d, port=%u, t=%u, int=%u",
             (int)dir, (unsigned)port, (unsigned)duration_s, (unsigned)interval_s);

    return iperf_start(&cfg);
}

void Iperf::loop() {
    if (!this->network_initialized_ && network::is_connected()) {
        // Perform network setup once connected
        this->start_server();
        this->network_initialized_ = true;
    }
  }


void Iperf::start_server() {
    auto err = esphome_iperf_start_tcp_ipv4(
        ESPHOME_IPERF_DIR_REMOTE_TO_ESP,
        nullptr,
        port_,
        duration_s_,
        interval_s_);
    if (err != ESP_OK) {
      ESP_LOGE("iperf", "Failed to start iperf server: %d", (int)err);
    }
  }

  // ESP as iperf client (ESP -> remote)
  void Iperf::start_client() {
    auto err = esphome_iperf_start_tcp_ipv4(
        ESPHOME_IPERF_DIR_ESP_TO_REMOTE,
        remote_ip_.c_str(),
        port_,
        duration_s_,
        interval_s_);
    if (err != ESP_OK) {
      ESP_LOGE("iperf", "Failed to start iperf client: %d", (int)err);
    }
  }

  void Iperf::stop() {
    auto err = iperf_stop();
    if (err != ESP_OK) {
      ESP_LOGE("iperf", "Failed to stop iperf: %d", (int)err);
    }
  }


}
}