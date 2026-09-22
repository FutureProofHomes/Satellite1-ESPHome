#include "xmos_flashing.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"

#include <algorithm>
#include <endian.h>

namespace esphome {
namespace satellite1 {

static const char *const TAG = "xmos_flasher";

static const size_t FLASH_PAGE_SIZE = 256;
static const size_t FLASH_SECTOR_SIZE = 4096;
constexpr size_t FLASH_TOTAL_NUMBER_OF_SECTORS = 8388608 / FLASH_SECTOR_SIZE;
constexpr uint32_t FLASH_BOOT_PARTITION_SIZE = 0x100000;
constexpr uint32_t FLASH_RECORD_MAGIC = 0x58464C54;
constexpr uint8_t FLASH_JEDEC_ID[] = {0xEF, 0x40, 0x17};

static bool is_recovery_state(XmosFlashState state) {
  return state == XmosFlashState::RECOVERY_REQUIRED || state == XmosFlashState::FULL_ERASE_RECOVERY_REQUIRED ||
         state == XmosFlashState::FACTORY_RESET_RECOVERY_REQUIRED;
}

void XMOSFlasher::setup() {
  this->record_pref_ = global_preferences->make_preference<XmosFlashRecord>(fnv1_hash("sat1.xmos.flash_record"));
  this->load_record_();

  if (this->record_.state == XmosFlashState::FLASH_REQUESTED) {
    if (this->record_matches_embedded_()) {
      this->pending_boot_action_ = true;
      this->pending_boot_action_type_ = ACTION_FLASH_EMBEDDED_IMAGE;
    } else {
      ESP_LOGE(TAG, "Pending XMOS flash request does not match the embedded image");
      this->set_record_unknown_();
      this->save_record_();
    }
  } else if (this->record_.state == XmosFlashState::FULL_ERASE_FLASH_REQUESTED ||
             this->record_.state == XmosFlashState::FACTORY_RESET_FLASH_REQUESTED) {
    this->factory_reset_pending_ = this->record_.state == XmosFlashState::FACTORY_RESET_FLASH_REQUESTED;
    if (this->record_matches_embedded_()) {
      this->pending_boot_action_ = true;
      this->pending_boot_action_type_ = ACTION_FLASH_EMBEDDED_FULL_ERASE;
    } else {
      ESP_LOGE(TAG, "Pending full XMOS erase request does not match the embedded image");
      this->factory_reset_pending_ = false;
      this->set_record_unknown_();
      this->save_record_();
    }
  } else if (is_recovery_state(this->record_.state) && this->record_is_valid_()) {
    this->factory_reset_pending_ = this->record_.state == XmosFlashState::FACTORY_RESET_RECOVERY_REQUIRED;
    this->pending_boot_action_ = true;
    this->pending_boot_action_type_ = this->record_.state == XmosFlashState::RECOVERY_REQUIRED
                                          ? ACTION_FLASH_EMBEDDED_IMAGE
                                          : ACTION_FLASH_EMBEDDED_FULL_ERASE;
  }

  if (this->pending_boot_action_) {
    // Prevent XMOS probing and state callbacks until this persisted recovery is terminal.
    this->boot_recovery_active_ = true;
    this->parent_->set_boot_recovery_pending(true);
    this->parent_->set_spi_flash_direct_access_mode(true);
  }
}

void XMOSFlasher::loop() {
  if (this->pending_boot_action_ && this->state == FLASHER_IDLE && App.is_setup_complete()) {
    this->pending_boot_action_ = false;
    if (this->pending_boot_action_type_ == ACTION_FLASH_EMBEDDED_FULL_ERASE) {
      this->flash_attempted_this_boot_ = true;
      this->md5_expected_ = this->embedded_image_.md5;
      this->requested_action = ACTION_FLASH_EMBEDDED_FULL_ERASE;
      this->state = FLASHER_INITIALIZING;
      this->publish();
    } else {
      ESP_LOGW(TAG, "Resuming XMOS recovery with the embedded image");
      this->flash_embedded_image();
    }
  }

  switch (this->state) {
    case FLASHER_IDLE:
      break;

    case FLASHER_INITIALIZING:
      if (this->requested_action == ACTION_VERIFY_RECORD) {
        if (!this->init_flasher() ||
            memcmp(this->record_.unique_id, this->active_unique_id_, sizeof(this->active_unique_id_))) {
          this->deinit_flashing_();
          this->state = FLASHER_IDLE;
        } else {
          this->start_record_verification_();
        }
      } else if (this->init_flashing_()) {
        this->state = FLASHER_ERASING;
      } else {
        this->deinit_flashing_();
        this->state = FLASHER_ERROR_STATE;
      }
      break;

    case FLASHER_VERIFYING:
      if (this->verify_record_step_()) {
        if (this->verify_content_valid_) {
          this->record_.state = XmosFlashState::VERIFIED;
          if (!this->save_record_())
            this->error_code = INIT_FLASH_ERROR;
        }
        this->deinit_flashing_();
        this->state =
            this->verify_content_valid_ && this->error_code == FLASHER_OK ? FLASHER_SUCCESS_STATE : FLASHER_ERROR_STATE;
      }
      break;

    case FLASHER_ERASING: {
      int remaining = this->erasing_step_();
      this->publish_progress_();
      if (remaining == 0 && this->requested_action == ACTION_FULL_ERASE) {
        this->deinit_flashing_();
        this->state = FLASHER_SUCCESS_STATE;
      } else if (remaining == 0) {
        this->state = FLASHER_FLASHING;
      } else if (remaining < 0) {
        this->deinit_flashing_();
        this->state = FLASHER_ERROR_STATE;
      }
      break;
    }

    case FLASHER_FLASHING: {
      int remaining = this->flashing_step_();
      this->publish_progress_();
      if (remaining == 0) {
        this->deinit_flashing_();
        this->state = FLASHER_SUCCESS_STATE;
      } else if (remaining < 0) {
        this->deinit_flashing_();
        this->state = FLASHER_ERROR_STATE;
      }
      break;
    }

    case FLASHER_SUCCESS_STATE:
      this->publish();
      this->state = FLASHER_IDLE;
      break;

    case FLASHER_ERROR_STATE:
      this->publish();
      this->state = FLASHER_IDLE;
      break;

    default:
      break;
  }
}

void XMOSFlasher::publish_progress_() {
  uint32_t now = millis();

  if ((now - this->last_published_) > 1000) {
    const uint64_t erase_bytes = this->total_sectors_to_erase_ * FLASH_SECTOR_SIZE;
    const uint64_t erased_bytes = std::min<uint64_t>(
        erase_bytes,
        this->current_sector_ < 0 ? 0 : static_cast<uint64_t>(this->current_sector_ + 1) * FLASH_SECTOR_SIZE);
    const uint64_t written_bytes = this->total_number_of_bytes_ - this->bytes_remaining_;
    const uint64_t total_work_bytes = erase_bytes + this->total_number_of_bytes_;
    if (this->requested_action == ACTION_FLASH_EMBEDDED_IMAGE) {
      // Sector erases are much slower than page programs, so progress follows operation time rather than bytes.
      constexpr uint8_t ERASE_PROGRESS_PERCENT = 75;
      if (this->state == FLASHER_ERASING) {
        this->flashing_progress = erase_bytes == 0 ? 0 : erased_bytes * ERASE_PROGRESS_PERCENT / erase_bytes;
      } else {
        this->flashing_progress = this->total_number_of_bytes_ == 0
                                      ? ERASE_PROGRESS_PERCENT
                                      : ERASE_PROGRESS_PERCENT + written_bytes * (100 - ERASE_PROGRESS_PERCENT) /
                                                                     this->total_number_of_bytes_;
      }
    } else {
      this->flashing_progress = total_work_bytes == 0 ? 0 : (erased_bytes + written_bytes) * 100 / total_work_bytes;
    }
    this->last_published_ = now;
    ESP_LOGD(TAG, "Progress: %d%%", this->flashing_progress);
    this->publish();
  }
}

bool XMOSFlasher::init_flasher() {
  ESP_LOGD(TAG, "Setting up XMOS flasher...");
  this->parent_->set_spi_flash_direct_access_mode(true);
  if (!this->read_flash_identity_()) {
    ESP_LOGE(TAG, "Unable to read expected XMOS flash identity");
    return false;
  }
  this->dump_flash_info();
  this->total_number_of_sectors_ = FLASH_TOTAL_NUMBER_OF_SECTORS;
  return true;
}

bool XMOSFlasher::deinit_flasher() {
  ESP_LOGD(TAG, "Stopping XMOS flasher...");
  this->parent_->set_spi_flash_direct_access_mode(false);
  if (this->boot_recovery_active_) {
    this->parent_->set_boot_recovery_pending(false);
    this->boot_recovery_active_ = false;
  }
  return true;
}

void XMOSFlasher::dump_flash_info() {
  ESP_LOGCONFIG(TAG, "Satellite1-Flasher:");
  ESP_LOGCONFIG(TAG, "	JEDEC-manufacturerID %hhu", this->manufacturerID_);
  ESP_LOGCONFIG(TAG, "	JEDEC-memoryTypeID %hhu", this->memoryTypeID_);
  ESP_LOGCONFIG(TAG, "	JEDEC-capacityID %hhu", this->capacityID_);
  ESP_LOGCONFIG(TAG, "	JEDEC-capacityID %hhu", this->capacityID_);
  ESP_LOGCONFIG(TAG, "	JEDEC-capacity: %hhu", 1 << this->capacityID_);
}

void XMOSFlasher::erase_memory() {
  if (this->state != FLASHER_IDLE) {
    ESP_LOGE(TAG, "XMOS flasher is busy, can't inititate erasing");
    return;
  }

  this->factory_reset_pending_ = false;
  this->requested_action = ACTION_FULL_ERASE;
  this->state = FLASHER_INITIALIZING;
  this->publish();
}

void XMOSFlasher::request_embedded_flash_reboot() {
  if (this->state != FLASHER_IDLE || this->embedded_image_.length == 0) {
    ESP_LOGE(TAG, "XMOS flash request rejected: flasher busy or no embedded image");
    return;
  }
  this->factory_reset_pending_ = false;
  memset(&this->record_, 0, sizeof(this->record_));
  this->record_.magic = FLASH_RECORD_MAGIC;
  this->record_.state = XmosFlashState::FLASH_REQUESTED;
  this->record_.image_length = this->embedded_image_.length;
  this->record_.boot_partition_size = FLASH_BOOT_PARTITION_SIZE;
  if (!this->set_record_md5_(this->embedded_image_.md5) || !this->save_record_()) {
    ESP_LOGE(TAG, "Couldn't persist XMOS flash reboot request");
    return;
  }
  App.safe_reboot();
}

void XMOSFlasher::request_full_erase_flash_reboot() {
  this->request_full_erase_flash_reboot_(XmosFlashState::FULL_ERASE_FLASH_REQUESTED);
}

void XMOSFlasher::request_factory_reset_reboot() {
  this->request_full_erase_flash_reboot_(XmosFlashState::FACTORY_RESET_FLASH_REQUESTED);
}

void XMOSFlasher::request_full_erase_flash_reboot_(XmosFlashState request_state) {
  if (this->state != FLASHER_IDLE || this->embedded_image_.length == 0) {
    ESP_LOGE(TAG, "XMOS full erase request rejected: flasher busy or no embedded image");
    return;
  }
  this->factory_reset_pending_ = request_state == XmosFlashState::FACTORY_RESET_FLASH_REQUESTED;
  memset(&this->record_, 0, sizeof(this->record_));
  this->record_.magic = FLASH_RECORD_MAGIC;
  this->record_.state = request_state;
  this->record_.image_length = this->embedded_image_.length;
  this->record_.boot_partition_size = FLASH_BOOT_PARTITION_SIZE;
  if (!this->set_record_md5_(this->embedded_image_.md5) || !this->save_record_()) {
    ESP_LOGE(TAG, "Couldn't persist XMOS full erase reboot request");
    this->factory_reset_pending_ = false;
    return;
  }
  App.safe_reboot();
}

void XMOSFlasher::flash_remote_image() {
  if (this->state != FLASHER_IDLE) {
    ESP_LOGE(TAG, "XMOS flasher is busy, can't initiate new flash");
    return;
  }

  if (this->url_.empty()) {
    ESP_LOGE(TAG, "URL not set; cannot start flashing");
    this->error_code = BAD_URL;
    this->state = FLASHER_ERROR_STATE;
    return;
  }

  this->flash_attempted_this_boot_ = true;

  if (this->md5_expected_.empty() && !this->http_get_md5_()) {
    ESP_LOGE(TAG, "Couldn't receive expected md5 sum.");
    this->error_code = MD5_INVALID;
    this->state = FLASHER_ERROR_STATE;
    return;
  }

  this->factory_reset_pending_ = false;
  this->requested_action = ACTION_FLASH_REMOTE_IMAGE;
  this->state = FLASHER_INITIALIZING;
  this->publish();
}

void XMOSFlasher::flash_embedded_image() {
  if (this->state != FLASHER_IDLE) {
    ESP_LOGE(TAG, "XMOS flasher is busy, can't inititate new flash");
    return;
  }

  if (this->embedded_image_.length == 0) {
    ESP_LOGE(TAG, "Didn't find embedded image!");
    this->error_code = NO_EMBEDDED_IMAGE_ERROR;
    this->state = FLASHER_ERROR_STATE;
    return;
  }

  this->flash_attempted_this_boot_ = true;

  this->factory_reset_pending_ = false;
  this->md5_expected_ = this->embedded_image_.md5;
  this->requested_action = ACTION_FLASH_EMBEDDED_IMAGE;
  this->state = FLASHER_INITIALIZING;
  this->publish();
}

bool XMOSFlasher::read_JEDECID_() {
  uint8_t manufacturer = 0;
  uint8_t memoryType = 0;
  uint8_t capcacity = 0;
  this->enable();
  this->transfer_byte(0x9F);
  manufacturer = this->transfer_byte(0);
  memoryType = this->transfer_byte(0);
  capcacity = this->transfer_byte(0);
  this->disable();

  if (manufacturer == FLASH_JEDEC_ID[0] && memoryType == FLASH_JEDEC_ID[1] && capcacity == FLASH_JEDEC_ID[2]) {
    this->manufacturerID_ = manufacturer;
    this->memoryTypeID_ = memoryType;
    this->capacityID_ = capcacity;
    return true;
  }
  return false;
}

bool XMOSFlasher::read_unique_id_(uint8_t *unique_id) {
  this->enable();
  this->transfer_byte(0x4B);
  for (size_t i = 0; i < 4; i++)
    this->transfer_byte(0x00);
  for (size_t i = 0; i < 8; i++)
    unique_id[i] = this->transfer_byte(0x00);
  this->disable();
  return true;
}

bool XMOSFlasher::read_flash_identity_() {
  uint8_t first[8];
  uint8_t second[8];
  if (!this->read_JEDECID_() || !this->read_unique_id_(first) || !this->read_unique_id_(second) ||
      memcmp(first, second, sizeof(first)) != 0) {
    return false;
  }
  memcpy(this->active_unique_id_, first, sizeof(this->active_unique_id_));
  return true;
}

bool XMOSFlasher::wait_while_flash_busy_(uint32_t timeout_ms) {
  int32_t timeout_invoke = millis();
  const uint8_t WEL = 2;
  const uint8_t BUSY = 1;

  while ((millis() - timeout_invoke) < timeout_ms) {
    this->enable();
    this->transfer_byte(0x05);
    uint8_t status = this->transfer_byte(0x00);
    this->disable();
    if ((status & BUSY) == 0) {
      return true;
    }
  }
  return false;
}

bool XMOSFlasher::enable_writing_() {
  // enable writing
  this->enable();
  this->transfer_byte(0x06);
  this->disable();

  this->enable();
  this->transfer_byte(0x05);
  uint8_t status = this->transfer_byte(0x00);
  this->disable();
  const uint8_t WEL = 2;
  if (!(status & WEL)) {
    return false;
  }
  return true;
}

bool XMOSFlasher::disable_writing_() {
  // disable writing
  this->enable();
  this->transfer_byte(0x04);
  this->disable();
  return true;
}

bool XMOSFlasher::erase_sector_(int sector) {
  // erase 4kB sector
  assert(FLASH_SECTOR_SIZE == 4096);
  uint32_t u32 = htole32(sector * FLASH_SECTOR_SIZE);
  uint8_t *u8_ptr = (uint8_t *) &u32;

  if (!this->enable_writing_()) {
    return false;
  }

  this->enable();
  this->transfer_byte(0x20);
  this->transfer_byte(*(u8_ptr + 2));
  this->transfer_byte(*(u8_ptr + 1));
  this->transfer_byte(*(u8_ptr));
  this->disable();

  // this->disable_writing_();
  return true;
}

bool XMOSFlasher::chip_erase_() {
  if (!this->enable_writing_()) {
    return false;
  }

  this->enable();
  this->transfer_byte(0xc7);
  this->disable();

  // this->disable_writing_();
  return true;
}

bool XMOSFlasher::write_page_(uint32_t byte_addr, uint8_t *buffer) {
  if ((byte_addr & (FLASH_PAGE_SIZE - 1)) != 0) {
    ESP_LOGE(TAG, "Address needs to be page aligned (%zu).", FLASH_PAGE_SIZE);
    return false;
  }
  if (!this->enable_writing_()) {
    ESP_LOGE(TAG, "Couldn't enable writing");
    return false;
  }

  uint32_t u32 = htole32(byte_addr);
  uint8_t *u8_ptr = (uint8_t *) &u32;
  this->enable();
  this->transfer_byte(0x02);
  this->transfer_byte(*(u8_ptr + 2));
  this->transfer_byte(*(u8_ptr + 1));
  this->transfer_byte(*(u8_ptr));
  for (int pos = 0; pos < FLASH_PAGE_SIZE; pos++) {
    this->transfer_byte(*(buffer + pos));
  }
  this->disable();

  if (!this->wait_while_flash_busy_(15)) {
    ESP_LOGE(TAG, "Writing page timeout");
    return false;
  }
  this->disable_writing_();
  return true;
}

bool XMOSFlasher::read_page_(uint32_t byte_addr, uint8_t *buffer) {
  if ((byte_addr & (FLASH_PAGE_SIZE - 1)) != 0) {
    return false;
  }
  uint32_t u32 = htole32(byte_addr);
  uint8_t *u8_ptr = (uint8_t *) &u32;
  this->enable();
  this->transfer_byte(0x0B);
  this->transfer_byte(*(u8_ptr + 2));
  this->transfer_byte(*(u8_ptr + 1));
  this->transfer_byte(*(u8_ptr));
  this->transfer_byte(0x00);
  for (int pos = 0; pos < FLASH_PAGE_SIZE; pos++) {
    *(buffer + pos) = this->transfer_byte(0x00);
  }
  this->disable();
  return true;
}

void XMOSFlasher::set_record_unknown_() {
  memset(&this->record_, 0, sizeof(this->record_));
  this->record_.magic = FLASH_RECORD_MAGIC;
  this->record_.state = XmosFlashState::UNKNOWN;
}

bool XMOSFlasher::record_is_valid_() const {
  if (this->record_.magic != FLASH_RECORD_MAGIC || !is_recovery_state(this->record_.state) ||
      this->record_.image_length == 0 || this->record_.boot_partition_size == 0 ||
      this->record_.boot_partition_size > FLASH_BOOT_PARTITION_SIZE ||
      this->record_.boot_partition_size % FLASH_SECTOR_SIZE != 0) {
    return false;
  }
  return this->record_.image_length <= this->record_.boot_partition_size;
}

bool XMOSFlasher::record_matches_embedded_() const {
  return this->embedded_image_.length > 0 && this->record_.image_length == this->embedded_image_.length &&
         this->record_.boot_partition_size == FLASH_BOOT_PARTITION_SIZE &&
         this->record_md5_() == this->embedded_image_.md5;
}

bool XMOSFlasher::load_record_() {
  XmosFlashRecord loaded{};
  if (!this->record_pref_.load(&loaded) || loaded.magic != FLASH_RECORD_MAGIC ||
      loaded.state > XmosFlashState::FACTORY_RESET_RECOVERY_REQUIRED) {
    this->set_record_unknown_();
    return false;
  }
  this->record_ = loaded;
  return true;
}

bool XMOSFlasher::save_record_() {
  if (!this->record_pref_.save(&this->record_)) {
    ESP_LOGE(TAG, "Couldn't save XMOS flash record");
    return false;
  }
  if (!global_preferences->sync()) {
    ESP_LOGE(TAG, "Couldn't synchronize XMOS flash record");
    return false;
  }
  return true;
}

bool XMOSFlasher::set_record_md5_(const std::string &md5) {
  if (md5.size() != MD5_SIZE)
    return false;
  for (size_t i = 0; i < sizeof(this->record_.image_md5); i++) {
    const auto nibble = [](char value) -> int {
      if (value >= '0' && value <= '9')
        return value - '0';
      if (value >= 'a' && value <= 'f')
        return value - 'a' + 10;
      if (value >= 'A' && value <= 'F')
        return value - 'A' + 10;
      return -1;
    };
    const int high = nibble(md5[i * 2]);
    const int low = nibble(md5[i * 2 + 1]);
    if (high < 0 || low < 0)
      return false;
    this->record_.image_md5[i] = static_cast<uint8_t>((high << 4) | low);
  }
  return true;
}

std::string XMOSFlasher::record_md5_() const {
  static const char HEX[] = "0123456789abcdef";
  std::string md5(MD5_SIZE, '0');
  for (size_t i = 0; i < sizeof(this->record_.image_md5); i++) {
    md5[i * 2] = HEX[this->record_.image_md5[i] >> 4];
    md5[i * 2 + 1] = HEX[this->record_.image_md5[i] & 0x0F];
  }
  return md5;
}

bool XMOSFlasher::prepare_flash_transaction_(uint32_t erased_length, XmosFlashState recovery_state) {
  if (erased_length == 0 || erased_length > FLASH_BOOT_PARTITION_SIZE || erased_length % FLASH_SECTOR_SIZE != 0 ||
      this->total_number_of_bytes_ == 0 || this->total_number_of_bytes_ > erased_length ||
      !is_recovery_state(recovery_state)) {
    return false;
  }
  memset(&this->record_, 0, sizeof(this->record_));
  this->record_.magic = FLASH_RECORD_MAGIC;
  this->record_.state = recovery_state;
  memcpy(this->record_.unique_id, this->active_unique_id_, sizeof(this->record_.unique_id));
  this->record_.image_length = this->total_number_of_bytes_;
  this->record_.boot_partition_size = erased_length;
  return this->set_record_md5_(this->md5_expected_) && this->save_record_();
}

void XMOSFlasher::start_record_verification_() {
  if (this->reader_buffer_ == nullptr)
    this->reader_buffer_ = static_cast<uint8_t *>(malloc(FLASH_PAGE_SIZE));
  if (this->reader_buffer_ == nullptr) {
    this->error_code = INIT_FLASH_ERROR;
    this->state = FLASHER_ERROR_STATE;
    return;
  }
  this->verify_position_ = 0;
  this->verify_content_valid_ = true;
  this->md5_verify_.init();
  this->state = FLASHER_VERIFYING;
}

bool XMOSFlasher::verify_record_step_() {
  if (!this->read_page_(this->verify_position_, this->reader_buffer_)) {
    this->error_code = WRITE_TO_FLASH_ERROR;
    return true;
  }
  const uint32_t image_end = this->record_.image_length;
  const uint32_t page_end = this->verify_position_ + FLASH_PAGE_SIZE;
  if (this->verify_position_ < image_end) {
    const uint32_t image_stop = std::min(page_end, image_end);
    this->md5_verify_.add(this->reader_buffer_, image_stop - this->verify_position_);
  }
  const uint32_t tail_start = std::max(this->verify_position_, image_end);
  if (tail_start < page_end) {
    for (uint32_t pos = tail_start; pos < std::min(page_end, this->record_.boot_partition_size); pos++) {
      if (this->reader_buffer_[pos - this->verify_position_] != 0xFF)
        this->verify_content_valid_ = false;
    }
  }
  this->verify_position_ += FLASH_PAGE_SIZE;
  if (this->verify_position_ < this->record_.boot_partition_size)
    return false;

  std::unique_ptr<char[]> md5(new char[MD5_SIZE + 1]);
  this->md5_verify_.calculate();
  this->md5_verify_.get_hex(md5.get());
  if (strncmp(md5.get(), this->record_md5_().c_str(), MD5_SIZE) != 0)
    this->verify_content_valid_ = false;
  return true;
}

bool XMOSFlasher::init_flashing_() {
  if (!this->init_flasher()) {
    this->error_code = INIT_FLASH_ERROR;
    return false;
  }

  if (this->boot_recovery_active_ && is_recovery_state(this->record_.state) &&
      memcmp(this->record_.unique_id, this->active_unique_id_, sizeof(this->active_unique_id_)) != 0) {
    ESP_LOGE(TAG, "XMOS flash UID changed; refusing to resume interrupted recovery");
    this->error_code = INIT_FLASH_ERROR;
    return false;
  }

  this->flashing_start_time_ = millis();

  switch (this->requested_action) {
    case ACTION_FLASH_EMBEDDED_IMAGE:
    case ACTION_FLASH_EMBEDDED_FULL_ERASE:
      this->reader_ = new EmbeddedImageReader(this->embedded_image_);
      break;
    case ACTION_FLASH_REMOTE_IMAGE:
      this->reader_ = new HttpImageReader(this->http_request_, this->url_);
      break;
    case ACTION_FULL_ERASE:
      memset(&this->record_, 0, sizeof(this->record_));
      this->record_.magic = FLASH_RECORD_MAGIC;
      this->record_.state = XmosFlashState::RECOVERY_REQUIRED;
      memcpy(this->record_.unique_id, this->active_unique_id_, sizeof(this->record_.unique_id));
      this->record_.boot_partition_size = FLASH_BOOT_PARTITION_SIZE;
      if (!this->save_record_()) {
        this->error_code = INIT_FLASH_ERROR;
        return false;
      }
      this->total_sectors_to_erase_ = this->total_number_of_sectors_;
      this->total_sectors_to_erase_ = FLASH_BOOT_PARTITION_SIZE / FLASH_SECTOR_SIZE;
      this->current_sector_ = -1;
      return true;
    case ACTION_VERIFY_RECORD:
      return false;
  };

  if (!this->reader_->init_reader()) {
    this->error_code = INIT_READER_ERROR;
    return false;
  }

  this->reader_buffer_ = (uint8_t *) malloc(FLASH_PAGE_SIZE);
  if (this->reader_buffer_ == nullptr) {
    ESP_LOGE(TAG, "Couldn't allocate memory");
    this->error_code = INIT_FLASH_ERROR;
    return false;
  }

  this->compare_buffer_ = (uint8_t *) malloc(FLASH_PAGE_SIZE);
  if (this->compare_buffer_ == nullptr) {
    ESP_LOGE(TAG, "Couldn't allocate memory");
    this->error_code = INIT_FLASH_ERROR;
    return false;
  }

  ESP_LOGD(TAG, "MD5 expected: %s", this->md5_expected_.c_str());

  this->flashing_progress = 0;
  this->md5_receive_.init();

  size_t size_in_bytes = this->reader_->get_image_size();
  size_t size_in_pages = ((size_in_bytes + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE);
  size_t size_in_sectors = ((size_in_pages * FLASH_PAGE_SIZE + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE);

  this->total_number_of_bytes_ = size_in_bytes;
  this->bytes_remaining_ = size_in_bytes;
  this->page_pos_ = 0;

  const uint32_t erase_length = this->requested_action == ACTION_FLASH_EMBEDDED_FULL_ERASE
                                    ? FLASH_TOTAL_NUMBER_OF_SECTORS * FLASH_SECTOR_SIZE
                                : this->requested_action == ACTION_FLASH_EMBEDDED_IMAGE
                                    ? FLASH_BOOT_PARTITION_SIZE
                                    : static_cast<uint32_t>(size_in_sectors * FLASH_SECTOR_SIZE);
  const XmosFlashState recovery_state = this->requested_action == ACTION_FLASH_EMBEDDED_FULL_ERASE
                                            ? this->factory_reset_pending_
                                                  ? XmosFlashState::FACTORY_RESET_RECOVERY_REQUIRED
                                                  : XmosFlashState::FULL_ERASE_RECOVERY_REQUIRED
                                            : XmosFlashState::RECOVERY_REQUIRED;
  if (!this->prepare_flash_transaction_(FLASH_BOOT_PARTITION_SIZE, recovery_state)) {
    ESP_LOGE(TAG, "Couldn't persist XMOS flash transaction before erase");
    this->error_code = INIT_FLASH_ERROR;
    return false;
  }
  this->total_sectors_to_erase_ = erase_length / FLASH_SECTOR_SIZE;
  this->current_sector_ = -1;

  return true;
}

void XMOSFlasher::deinit_flashing_() {
  if (this->reader_buffer_) {
    free(this->reader_buffer_);
    this->reader_buffer_ = nullptr;
  }

  if (this->compare_buffer_) {
    free(this->compare_buffer_);
    this->compare_buffer_ = nullptr;
  }

  if (this->reader_) {
    this->reader_->deinit_reader();
    delete this->reader_;
    this->reader_ = nullptr;
  }

  this->md5_computed_.clear();
  this->md5_expected_.clear();

  delay(5);
  this->deinit_flasher();
}

int XMOSFlasher::erasing_step_() {
  if (!this->wait_while_flash_busy_(1)) {
    return this->total_sectors_to_erase_ - this->current_sector_;
  }

  this->current_sector_++;
  if (this->current_sector_ < this->total_sectors_to_erase_) {
    if (!this->erase_sector_(this->current_sector_)) {
      this->error_code = WRITE_TO_FLASH_ERROR;
      return -1;
    }
  }

  return this->total_sectors_to_erase_ - this->current_sector_;
}

int XMOSFlasher::flashing_step_() {
  // read a maximum of chunk_size bytes into buf. (real read size returned)
  int bytes_read = this->reader_->read_image_block(this->reader_buffer_, FLASH_PAGE_SIZE);
  if (bytes_read < 0) {
    ESP_LOGE(TAG, "Stream closed");
    this->error_code = CONNECTION_ERROR;
    return -1;
  }

  this->md5_receive_.add(this->reader_buffer_, bytes_read);
  this->bytes_remaining_ -= bytes_read;
  if (bytes_read != FLASH_PAGE_SIZE) {
    if (this->bytes_remaining_ != 0) {
      this->error_code = CONNECTION_ERROR;
      return -1;
    }
    // it's the last page to flash
    // The remaining boot partition must stay erased after the factory artifact.
    memset(this->reader_buffer_ + bytes_read, 0xFF, FLASH_PAGE_SIZE - bytes_read);
  }

  int page_pos = this->page_pos_;
  if (!this->write_page_(page_pos, this->reader_buffer_)) {
    ESP_LOGE(TAG, "Error while writing page %d, retrying...", page_pos);
  }

  // read back the page that has just been written
  this->read_page_(page_pos, this->compare_buffer_);

  if (memcmp(this->reader_buffer_, this->compare_buffer_, FLASH_PAGE_SIZE) != 0) {
    // not equal, give it a second try
    if (!this->write_page_(page_pos, this->reader_buffer_)) {
      ESP_LOGE(TAG, "Error while writing page %d, giving up...", page_pos);
      this->error_code = WRITE_TO_FLASH_ERROR;
      return -1;
    }

    this->read_page_(page_pos, this->compare_buffer_);
    if (memcmp(this->reader_buffer_, this->compare_buffer_, FLASH_PAGE_SIZE) != 0) {
      ESP_LOGE(TAG, "Read page mismatch, page addr: %d", page_pos);
      this->error_code = WRITE_TO_FLASH_ERROR;
      return -1;
    }
  }

  this->page_pos_ += FLASH_PAGE_SIZE;

  if (this->bytes_remaining_ == 0) {
    std::unique_ptr<char[]> md5_receive_str(new char[33]);
    this->md5_receive_.calculate();
    this->md5_receive_.get_hex(md5_receive_str.get());
    this->md5_computed_ = md5_receive_str.get();

    if (strncmp(this->md5_computed_.c_str(), this->md5_expected_.c_str(), MD5_SIZE) != 0) {
      ESP_LOGE(TAG, "MD5 computed: %s - Aborting due to MD5 mismatch", this->md5_computed_.c_str());
      this->error_code = MD5_MISMATCH_ERROR;
      return -1;
    } else {
      ESP_LOGD(TAG, "MD5 computed: %s - Matches!", this->md5_computed_.c_str());
    }
    this->record_.state =
        this->factory_reset_pending_ ? XmosFlashState::FACTORY_RESET_RECOVERY_REQUIRED : XmosFlashState::VERIFIED;
    if (!this->save_record_()) {
      this->error_code = INIT_FLASH_ERROR;
      return -1;
    }
    return 0;
  }
  return this->bytes_remaining_;
}

}  // namespace satellite1
}  // namespace esphome
