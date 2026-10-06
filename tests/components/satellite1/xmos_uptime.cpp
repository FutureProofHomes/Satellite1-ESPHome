#include "satellite1.h"
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <iostream>
#include <string>

using namespace esphome;
using namespace esphome::satellite1;

#define CHECK(condition) \
  do { \
    if (!(condition)) { \
      std::cerr << __LINE__ << ": " << #condition << " failed\n"; \
      std::exit(1); \
    } \
  } while (false)

class TestSatellite : public Satellite1 {
 public:
  TestSatellite() { this->set_xmos_rst_pin(&this->pin); }
  GPIOPin pin;
  uint32_t deadline() const { return this->xmos_boot_ready_timestamp_; }
  void expect_pending(bool expected) const {
#ifdef HAS_SETTLE_PENDING
    CHECK(this->xmos_boot_settle_pending_ == expected);
#endif
  }
  void connected() {
    this->state = SAT_XMOS_CONNECTED_STATE;
    this->status_refresh_attempted_ = false;
  }
  void release() {
    this->set_spi_flash_direct_access_mode(true);
    CHECK(this->pin.level);
    this->set_spi_flash_direct_access_mode(false);
    CHECK(!this->pin.level);
    CHECK(this->state == SAT_DETACHED_STATE);
    CHECK(this->connection_attempts == 0);
    CHECK(this->xmos_booting_);
    this->expect_pending(true);
  }
  void transfer_allowed(bool expected) {
    unsigned before = host::spi_calls;
    CHECK(this->transfer(2, 0, nullptr, 0, nullptr, false) == expected);
    CHECK(host::spi_calls == before + (expected ? 1u : 0u));
  }
  void loop_allowed(bool expected) {
    // Force the connected status refresh path to be due, independent of its timer.
    this->connected();
    unsigned before = host::spi_calls;
    this->loop();
    CHECK(host::spi_calls == before + (expected ? 1u : 0u));
  }
};

int main(int argc, char **argv) {
  CHECK(argc == 2);
  const std::string test = argv[1];
  TestSatellite sat;
  if (test == "fresh_high" || test == "fresh_wrap") {
    sat.expect_pending(false);
    host::now = test == "fresh_high" ? 0x80000000u : 0xffffffffu;
    sat.transfer_allowed(true);
    sat.loop_allowed(true);
    if (test == "fresh_wrap") {
      host::now = 0;
      sat.transfer_allowed(true);
      sat.loop_allowed(true);
    }
  } else if (test == "boundary" || test == "deadline_wrap") {
    const uint32_t start = test == "boundary" ? 100u : 0xfffffff0u;
    host::now = start;
    sat.release();
    CHECK(sat.deadline() == uint32_t(start + 4000u));
    sat.transfer_allowed(false);
    sat.loop_allowed(false);
    host::now = start + 3999u;
    sat.transfer_allowed(false);
    sat.loop_allowed(false);
    sat.expect_pending(true);
    host::now = start + 4000u;
    sat.transfer_allowed(true);
    sat.expect_pending(false);
    sat.loop_allowed(true);
  } else if (test == "loop_clear" || test == "transfer_clear" || test == "long_uptime") {
    host::now = 100;
    sat.release();
    host::now = 4100;
    if (test == "loop_clear") {
      // No transfer in this state: clearing must be done by loop itself.
      sat.state = SAT_FLASH_CONNECTED_STATE;
      unsigned before = host::spi_calls;
      sat.loop();
      CHECK(host::spi_calls == before);
    } else {
      sat.transfer_allowed(true);
    }
    sat.expect_pending(false);
    if (test == "long_uptime") {
      for (uint32_t now : {4100u + 0x80000000u, 0xffffffffu, 0u, 4100u}) {
        host::now = now;
        sat.transfer_allowed(true);
        sat.loop_allowed(true);
        sat.expect_pending(false);
      }
    }
  } else if (test == "rearm") {
    host::now = 100;
    sat.release();
    host::now = 2100;
    sat.release();
    CHECK(sat.deadline() == 6100);
    host::now = 4100;  // First release's deadline must no longer permit traffic.
    sat.transfer_allowed(false);
    host::now = 6099;
    sat.loop_allowed(false);
    host::now = 6100;
    sat.transfer_allowed(true);
    sat.expect_pending(false);
    host::now = 7000;  // Re-arm after the previous pending period has cleared.
    sat.release();
    CHECK(sat.deadline() == 11000);
    host::now = 10999;
    sat.transfer_allowed(false);
    host::now = 11000;
    sat.loop_allowed(true);
    sat.expect_pending(false);
  } else if (test == "direct") {
    host::now = 100;
    sat.set_spi_flash_direct_access_mode(true);
    sat.transfer_allowed(false);
    host::now = 10000;
    sat.transfer_allowed(false);
    unsigned before = host::spi_calls;
    sat.loop();
    CHECK(host::spi_calls == before);
    sat.release();
    host::now = 14000;
    sat.transfer_allowed(true);
    sat.set_spi_flash_direct_access_mode(true);
    sat.transfer_allowed(false);
  } else if (test == "recovery") {
    host::now = 100;
    sat.release();
    sat.set_boot_recovery_pending(true);
    host::now = 4100;
    sat.loop_allowed(false);
    sat.expect_pending(true);  // Recovery guard precedes even settle bookkeeping.
    sat.set_boot_recovery_pending(false);
    sat.state = SAT_FLASH_CONNECTED_STATE;
    unsigned before = host::spi_calls;
    sat.loop();
    CHECK(host::spi_calls == before);
    sat.expect_pending(false);
    sat.loop_allowed(true);
  } else {
    CHECK(false);
  }
}
