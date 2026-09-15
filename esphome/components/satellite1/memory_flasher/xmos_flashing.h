#pragma once

#include "esphome/components/http_request/http_request.h"
#include "esphome/components/ota/ota_backend.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/md5/md5.h"
#include "esphome/core/preferences.h"

#include "esphome/components/memory_flasher/memory_flasher.h"
#include "esphome/components/satellite1/satellite1.h"

namespace esphome {
using namespace memory_flasher;
namespace satellite1 {

enum class XmosFlashState : uint8_t {
  UNKNOWN = 0,
  VERIFIED = 1,
  RECOVERY_REQUIRED = 2,
  FLASH_REQUESTED = 3,
  FULL_ERASE_FLASH_REQUESTED = 4,
};

struct XmosFlashRecord {
  uint32_t magic;
  XmosFlashState state;
  uint8_t unique_id[8];
  uint32_t image_length;
  uint32_t boot_partition_size;
  uint8_t image_md5[16];
};

class XMOSFlasher : public MemoryFlasher, public Satellite1SPIService {
 public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void loop() override;

  bool init_flasher() override;
  bool deinit_flasher() override;
  void dump_flash_info() override;

  void erase_memory() override;
  void flash_remote_image() override;
  void flash_embedded_image() override;
  void request_embedded_flash_reboot() override;
  void request_full_erase_flash_reboot() override;
  bool boot_flash_pending() const { return this->pending_boot_action_; }

  bool flash_accessible() override {
    this->parent_->set_spi_flash_direct_access_mode(true);
    bool got_id = this->read_JEDECID_();
    this->parent_->set_spi_flash_direct_access_mode(false);
    return got_id;
  }

 protected:
  bool read_JEDECID_();
  bool read_flash_identity_();
  bool read_unique_id_(uint8_t *unique_id);
  bool enable_writing_();
  bool disable_writing_();
  bool chip_erase_();
  bool erase_sector_(int sector);
  bool wait_while_flash_busy_(uint32_t timeout_ms);
  bool read_page_(uint32_t byte_addr, uint8_t *buffer);
  bool write_page_(uint32_t byte_addr, uint8_t *buffer);
  bool prepare_flash_transaction_(uint32_t erased_length);
  void start_record_verification_();
  bool verify_record_step_();
  bool load_record_();
  bool save_record_();
  void set_record_unknown_();
  bool record_is_valid_() const;
  bool record_matches_embedded_() const;
  bool set_record_md5_(const std::string &md5);
  std::string record_md5_() const;

  uint8_t manufacturerID_;
  uint8_t memoryTypeID_;
  uint8_t capacityID_;
  int32_t capacity_;
  size_t total_number_of_sectors_;

  bool init_flashing_();
  void deinit_flashing_();
  int flashing_step_();
  int erasing_step_();
  void publish_progress_() override;

  bool http_flash_{false};
  bool embedded_flash_{false};
  FlashImageReader *reader_{nullptr};
  md5::MD5Digest md5_receive_;

  uint32_t flashing_start_time_{0};
  uint32_t last_published_{0};
  size_t total_sectors_to_erase_{0};
  int current_sector_{-1};
  size_t total_number_of_bytes_{0};
  size_t bytes_remaining_;
  int page_pos_{0};

  uint8_t *reader_buffer_{nullptr};
  uint8_t *compare_buffer_{nullptr};

  ESPPreferenceObject record_pref_;
  XmosFlashRecord record_{};
  uint8_t active_unique_id_[8]{};
  uint32_t verify_position_{0};
  bool verify_content_valid_{true};
  md5::MD5Digest md5_verify_;
  bool pending_boot_action_{false};
  bool boot_recovery_active_{false};
  FlasherAction pending_boot_action_type_{ACTION_FLASH_EMBEDDED_IMAGE};
};

}  // namespace satellite1
}  // namespace esphome
