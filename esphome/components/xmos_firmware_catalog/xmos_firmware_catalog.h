#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32

#include "esphome/components/button/button.h"
#include "esphome/components/satellite1/memory_flasher/xmos_flashing.h"
#include "esphome/components/select/select.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

#include <atomic>
#include <string>
#include <vector>

namespace esphome {
namespace xmos_firmware_catalog {

static constexpr size_t MAX_ENTRIES = 20;
static constexpr size_t MAX_TOKEN_LENGTH = 159;

// Internal RAM is scarce, so catalog data lives in PSRAM.
template<class T> struct PsramAllocator {
  using value_type = T;
  PsramAllocator() = default;
  template<class U> constexpr PsramAllocator(const PsramAllocator<U> & /*other*/) {}
  T *allocate(size_t n) { return RAMAllocator<T>().allocate(n); }
  void deallocate(T *p, size_t n) { RAMAllocator<T>().deallocate(p, n); }
  bool operator==(const PsramAllocator & /*other*/) const { return true; }
  bool operator!=(const PsramAllocator & /*other*/) const { return false; }
};
template<class T> using PsramVector = std::vector<T, PsramAllocator<T>>;
using PsramString = std::basic_string<char, std::char_traits<char>, PsramAllocator<char>>;

struct FirmwareSource {
  const char *repo;
  const char *token;  // From the YAML config; empty when unset
  bool include_prereleases;
  bool is_private;
};

struct FirmwareEntry {
  char version[24];  // Display label, e.g. "v1.1.0-dev.110"
  char tag[32];      // Release tag holding the asset
  char stem[40];     // Asset name without ".factory.bin"
  char date[21];     // Asset upload time (ISO 8601), used for ordering
  uint8_t version_bytes[5];
  uint8_t source;
  bool prerelease;
  uint32_t size;
  uint64_t bin_asset_id;
  uint64_t md5_asset_id;
};

struct SourceState {
  PsramVector<FirmwareEntry> entries;
  char etag[96];
  char error[96];
};

enum class InstallState : uint8_t {
  IDLE,
  REQUESTED,    // Install trigger fired; the install automation is stopping audio
  DOWNLOADING,  // Downloading and MD5-checking the image into PSRAM
  FLASHING,
  STARTING,    // Flash finished; waiting for the XMOS to report in
  RECOVERING,  // Flashing the built-in image after a failed install
};

class XmosFirmwareCatalog;

class CatalogSelect : public select::Select, public Parented<XmosFirmwareCatalog> {
 protected:
  void control(size_t index) override;
};

class CatalogRefreshButton : public button::Button, public Parented<XmosFirmwareCatalog> {
 protected:
  void press_action() override;
};

class CatalogInstallButton : public button::Button, public Parented<XmosFirmwareCatalog> {
 protected:
  void press_action() override;
};

struct CatalogJob;

class XmosFirmwareCatalog : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_flasher(satellite1::XMOSFlasher *flasher) { this->flasher_ = flasher; }
  void set_builtin_version(const char *version) { this->builtin_version_ = version; }
  void set_max_releases(uint8_t max_releases) { this->max_releases_ = max_releases; }
  void add_source(const char *repo, bool include_prereleases, bool is_private, const char *token) {
    this->sources_.push_back({repo, token, include_prereleases, is_private});
  }
  void set_select(CatalogSelect *select) { this->select_ = select; }
  void set_catalog_text_sensor(text_sensor::TextSensor *sensor) { this->catalog_sensor_ = sensor; }
  void set_status_text_sensor(text_sensor::TextSensor *sensor) { this->status_sensor_ = sensor; }
  template<typename F> void add_on_install_request_callback(F &&callback) {
    this->install_request_callback_.add(std::forward<F>(callback));
  }
  // Fired when a refresh changes the firmware choices. Home Assistant reads select options only when
  // it connects, so the automation should ask it to reload this device's config entry. Without one,
  // the API connection is dropped so Home Assistant reconnects and reads them again - unless the
  // select is internal, which Home Assistant never sees, so nothing happens.
  template<typename F> void add_on_choices_changed_callback(F &&callback) {
    this->choices_changed_callback_.add(std::forward<F>(callback));
    this->has_choices_automation_ = true;
  }

  // Safe to call from any task; the work happens in loop().
  void request_refresh();
  // Like request_refresh(), but waits for enough free memory instead of failing.
  void request_auto_refresh();
  // "builtin" selects the image embedded in this ESP32 firmware.
  void request_install(const std::string &version);
  // Write-only: the token is persisted but never reported back. An empty token clears it.
  void set_runtime_token(const std::string &token);
  // Catalog and install state as JSON. Safe to call from any task.
  std::string snapshot_json();

  // For on_install_request, once audio is stopped: downloads, verifies and flashes the requested image.
  bool begin_install();
  void abort_install(const std::string &reason);
  bool busy() const;

  void select_option(size_t index);
  void install_selected();

 protected:
  static void task_entry_(void *arg);
  bool start_job_(CatalogJob *job);
  void finish_job_();

  void process_requests_();
  void handle_install_request_(const std::string &version);
  void start_refresh_();
  void apply_refresh_results_(CatalogJob &job);
  bool rebuild_entries_();
  void update_select_options_();
  void announce_choices_changed_();
  void on_flasher_state_();
  void update_install_progress_();
  void fail_install_(const std::string &reason);
  void finish_install_(const std::string &result, bool ok);
  void start_recovery_(const std::string &reason);
  void release_staged_image_();

  const FirmwareEntry *find_entry_(const std::string &version) const;
  bool is_builtin_request_(const std::string &version) const;
  std::string token_for_source_(size_t source_index) const;
  bool xmos_running_version_(uint8_t version[5]) const;
  std::string target_label_() const;

  void load_pin_();
  void set_pin_(const uint8_t version[5], const std::string &label);
  void clear_pin_();
  uint32_t sources_hash_() const;
  void load_cache_();
  void save_cache_();
  void load_token_();
  void save_token_(const std::string &token);

  void set_status_(const std::string &status);
  void publish_catalog_text_();
  void rebuild_snapshot_();

  satellite1::XMOSFlasher *flasher_{nullptr};
  CatalogSelect *select_{nullptr};
  text_sensor::TextSensor *catalog_sensor_{nullptr};
  text_sensor::TextSensor *status_sensor_{nullptr};
  CallbackManager<void(const std::string &)> install_request_callback_;
  CallbackManager<void()> choices_changed_callback_;
  bool has_choices_automation_{false};

  const char *builtin_version_{""};
  uint8_t builtin_bytes_[5]{};
  uint8_t max_releases_{20};
  std::vector<FirmwareSource> sources_;

  // Catalog state (main loop only).
  PsramVector<SourceState> source_state_;
  PsramVector<FirmwareEntry> entries_;  // Newest first, without the built-in version
  PsramString builtin_label_;
  PsramString runtime_token_;
  std::string list_error_;
  std::string selected_id_;
  bool list_loaded_{false};
  bool refreshing_{false};
  bool auto_refresh_waiting_{false};
  uint32_t last_auto_refresh_check_{0};
  bool last_connected_{false};
  uint8_t last_running_[5]{};
  uint32_t last_refresh_attempt_{0};
  bool snapshot_dirty_{true};
  uint32_t snapshot_built_{0};

  // Install state (main loop only).
  InstallState install_state_{InstallState::IDLE};
  bool install_saw_erase_{false};
  bool recovery_pending_{false};
  bool last_result_ok_{true};
  uint8_t last_reported_progress_{255};
  uint32_t install_state_since_{0};
  uint32_t last_progress_ms_{0};
  std::string install_target_;  // "builtin" or a catalog version
  std::string recovery_reason_;
  std::string status_;
  std::string last_result_;
  uint8_t *staged_data_{nullptr};
  size_t staged_capacity_{0};
  size_t staged_length_{0};
  std::string staged_md5_;

  ESPPreferenceObject pin_pref_;
  ESPPreferenceObject cache_pref_;
  ESPPreferenceObject token_pref_;
  // Version that boot auto-flash must keep.
  bool pin_set_{false};
  uint8_t pin_bytes_[5]{};
  std::string pin_label_;

  // Background job: the task owns *job_ until job_done_ is set.
  CatalogJob *job_{nullptr};
  std::atomic<bool> job_done_{false};

  // Requests from other tasks and the snapshot they read, both under mutex_.
  Mutex mutex_;
  bool pending_refresh_{false};
  bool pending_auto_refresh_{false};
  bool pending_install_set_{false};
  bool pending_token_set_{false};
  std::string pending_install_;
  std::string pending_token_;
  PsramString snapshot_;
};

class InstallRequestTrigger : public Trigger<std::string> {
 public:
  explicit InstallRequestTrigger(XmosFirmwareCatalog *parent) {
    parent->add_on_install_request_callback([this](const std::string &version) { this->trigger(version); });
  }
};

class ChoicesChangedTrigger : public Trigger<> {
 public:
  explicit ChoicesChangedTrigger(XmosFirmwareCatalog *parent) {
    parent->add_on_choices_changed_callback([this]() { this->trigger(); });
  }
};

}  // namespace xmos_firmware_catalog
}  // namespace esphome

#endif  // USE_ESP32
