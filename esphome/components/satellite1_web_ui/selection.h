#pragma once

#include <string>

#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace satellite1_web_ui {

/// Which media players a feature is aimed at, as the app expresses it.
///
/// Three fields rather than one list, because "the whole of this area" has to survive a speaker being
/// added to that area later - an enumerated list cannot. `excluded` is what makes the whole-area form
/// usable: carving one speaker out of a twelve-player area costs one entry rather than re-listing the
/// other eleven, which is the difference between fitting and not. The measurement that forced this:
/// eleven entity ids is about 460 characters, and the text entity this used to live in caps at 255.
///
/// The selection's string type: PSRAM-backed, internal fallback on a PSRAM-less board.
///
/// These strings live for the life of the process and grow with the installation - eleven entity
/// ids is about 460 characters, and there are six such fields - which is exactly the profile that
/// does not belong on the internal heap. Callers that need a std::string (the YAML lambdas handing
/// these to Home Assistant as Jinja variables) go through .c_str(), paying a transient copy at
/// call time instead of a resident one forever.
using SelString = std::basic_string<char, std::char_traits<char>, RAMAllocator<char>>;

/// All three are stored as comma-separated strings rather than vectors. They are handed to Home
/// Assistant as Jinja variables, which wants exactly that, and it makes the whole struct trivially
/// serializable for NVS.
struct SelectionSet {
  /// Area ids taken wholesale. Ids, not names, because `area_entities()` accepts an id directly and
  /// an id cannot change under a rename.
  SelString areas;

  /// Individually chosen entity ids, including players Home Assistant has put in no area at all -
  /// which on a real installation is most of them.
  SelString extra;

  /// Players carved back out of a wholesale area, each stored as `area_id:entity_id`.
  ///
  /// The area prefix is not for resolution - an entity belongs to at most one area, so Home Assistant
  /// can reject these globally and get the same answer. It exists so the *device* can answer "is my
  /// own area selected whole, with nothing carved out of it" without knowing area membership, which
  /// it cannot know unaided. That question is what the derived switch reads.
  SelString excluded;

  /// Nothing aimed anywhere. This is the gate the firmware uses in place of the old master switch.
  bool empty() const { return this->areas.empty() && this->extra.empty(); }

  /// Field-wise copy through pointer-and-length assign, in place of `operator=`. The synthesized
  /// assignment would invoke basic_string's copy assignment, whose allocator-propagation branch
  /// compares allocators with an operator RAMAllocator does not define - a plain `if`, not
  /// `if constexpr`, so it fails to compile even though the branch can never be taken. The
  /// (const char *, len) overload has no allocator logic at all.
  void copy_from(const SelectionSet &other) {
    this->areas.assign(other.areas.data(), other.areas.size());
    this->extra.assign(other.extra.data(), other.extra.size());
    this->excluded.assign(other.excluded.data(), other.excluded.size());
  }

  void clear() {
    this->areas.clear();
    this->extra.clear();
    this->excluded.clear();
  }
};

/// The device's own copy of what the app has chosen, persisted across reboots.
///
/// This lives in the web UI component because the app is the only thing that writes a fine-grained
/// selection. That is a slight wart - routing configuration owned by a UI component - and it holds
/// only because common/web_ui.yaml is always included. If that ever stops being true, this class is
/// deliberately self-contained enough to move out into its own component without touching callers.
class Selection {
 public:
  /// Loads from NVS. Called from the component's setup(), before anything can read.
  void setup();

  SelectionSet &routing() { return this->routing_; }
  SelectionSet &duck() { return this->duck_; }
  const SelectionSet &routing() const { return this->routing_; }
  const SelectionSet &duck() const { return this->duck_; }

  /// Whether this device speaks the answer aloud itself. Replaces the `tts_mute_local_voice` switch,
  /// and keeps its default: on, so a device that has never been configured still talks.
  bool local_speaker() const { return this->local_speaker_; }
  void set_local_speaker(bool on);

  /// This device's own Home Assistant area id, learned from the data layer and persisted.
  ///
  /// Persisted because the derived switches have to have an answer before the first sync completes,
  /// and a switch that reads "off" for the first five seconds of every boot would look like a setting
  /// that failed to restore.
  const SelString &own_area() const { return this->own_area_; }
  void set_own_area(const std::string &area_id);

  /// "My whole area, with nothing carved out of it" - the state the renamed switches project.
  bool routing_whole_own_area() const { return this->whole_own_area_(this->routing_); }
  bool duck_whole_own_area() const { return this->whole_own_area_(this->duck_); }

  /// Writes that same state. Turning it on adds this device's area and drops any carve-outs belonging
  /// to it; other areas' carve-outs are left alone, which is what the `area_id:` prefix buys.
  void set_routing_whole_own_area(bool on) { this->set_whole_own_area_(this->routing_, on); }
  void set_duck_whole_own_area(bool on) { this->set_whole_own_area_(this->duck_, on); }

  /// Replaces both selections wholesale, as the app's POST does. One call so one save and one
  /// notification, rather than six.
  void replace(const SelectionSet &routing, const SelectionSet &duck, bool local_speaker);

  /// Fires after anything changes. tts_routing.yaml hangs its re-check scripts here, in place of the
  /// `on_value` the deleted text entity used to carry.
  void add_on_change_callback(std::function<void()> &&cb) { this->change_callback_.add(std::move(cb)); }

  /// Serializes both selections for the app, into the selection's own PSRAM string type - the
  /// endpoint sends straight from it, so the body never exists on the internal heap.
  void to_json(SelString &out) const;

 protected:
  /// The stored form. One blob for everything, so a change is one NVS write rather than six, and so
  /// the parts can never be restored out of step with each other.
  ///
  /// Fixed-size because that is what ESPPreferenceObject stores. 1536 bytes covers a generous
  /// selection - five areas, ten individual players and five carve-outs is about 700 - and costs
  /// nothing at rest.
  static const size_t BLOB_MAX = 1536;
  struct Blob {
    uint16_t len;
    char data[BLOB_MAX];
  } __attribute__((packed));

  /// The staging Blob for load and save, lazily allocated in PSRAM and kept. It used to be a local,
  /// which put a 1,538-byte spike on the main task's stack every save; PSRAM once beats stack every
  /// time. Main loop only, like every caller.
  Blob *blob_scratch_();

  bool whole_own_area_(const SelectionSet &set) const;
  void set_whole_own_area_(SelectionSet &set, bool on);

  /// Writes NVS and fires the callback. Every mutator ends here.
  void save_();

  SelectionSet routing_;
  SelectionSet duck_;
  bool local_speaker_{true};
  SelString own_area_;
  Blob *blob_{nullptr};

  ESPPreferenceObject pref_;
  CallbackManager<void()> change_callback_;
};

/* ------------------------------------------------------------------ */
/* Comma-separated list helpers                                        */
/* ------------------------------------------------------------------ */

/// Whether `needle` is one of the comma-separated entries in `csv`. Compares whole entries, so
/// "kitchen" does not match "kitchen_counter". All five operate on the selection's own PSRAM string
/// type; they have no other callers.
bool csv_contains(const SelString &csv, const std::string &needle);

/// Appends `entry` unless it is already present.
void csv_add(SelString &csv, const std::string &entry);

/// Removes `entry` if present.
void csv_remove(SelString &csv, const std::string &entry);

/// Removes every entry beginning with `prefix`. Used to drop one area's carve-outs and leave the
/// rest, which is the only reason `excluded` carries an area prefix at all.
void csv_remove_prefixed(SelString &csv, const std::string &prefix);

/// Whether any entry begins with `prefix`.
bool csv_has_prefixed(const SelString &csv, const std::string &prefix);

}  // namespace satellite1_web_ui
}  // namespace esphome
