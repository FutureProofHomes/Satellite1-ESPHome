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
/// All three are stored as comma-separated strings rather than vectors. They are handed to Home
/// Assistant as Jinja variables, which wants exactly that, and it makes the whole struct trivially
/// serializable for NVS.
struct SelectionSet {
  /// Area ids taken wholesale. Ids, not names, because `area_entities()` accepts an id directly and
  /// an id cannot change under a rename.
  std::string areas;

  /// Individually chosen entity ids, including players Home Assistant has put in no area at all -
  /// which on a real installation is most of them.
  std::string extra;

  /// Players carved back out of a wholesale area, each stored as `area_id:entity_id`.
  ///
  /// The area prefix is not for resolution - an entity belongs to at most one area, so Home Assistant
  /// can reject these globally and get the same answer. It exists so the *device* can answer "is my
  /// own area selected whole, with nothing carved out of it" without knowing area membership, which
  /// it cannot know unaided. That question is what the derived switch reads.
  std::string excluded;

  /// Nothing aimed anywhere. This is the gate the firmware uses in place of the old master switch.
  bool empty() const { return this->areas.empty() && this->extra.empty(); }
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
  const std::string &own_area() const { return this->own_area_; }
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

  /// Serializes both selections for the app. Appends to `out` rather than returning, so the endpoint
  /// can stream it without building a second copy.
  void to_json(std::string &out) const;

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

  bool whole_own_area_(const SelectionSet &set) const;
  void set_whole_own_area_(SelectionSet &set, bool on);

  /// Writes NVS and fires the callback. Every mutator ends here.
  void save_();

  SelectionSet routing_;
  SelectionSet duck_;
  bool local_speaker_{true};
  std::string own_area_;

  ESPPreferenceObject pref_;
  CallbackManager<void()> change_callback_;
};

/* ------------------------------------------------------------------ */
/* Comma-separated list helpers                                        */
/* ------------------------------------------------------------------ */

/// Whether `needle` is one of the comma-separated entries in `csv`. Compares whole entries, so
/// "kitchen" does not match "kitchen_counter".
bool csv_contains(const std::string &csv, const std::string &needle);

/// Appends `entry` unless it is already present.
void csv_add(std::string &csv, const std::string &entry);

/// Removes `entry` if present.
void csv_remove(std::string &csv, const std::string &entry);

/// Removes every entry beginning with `prefix`. Used to drop one area's carve-outs and leave the
/// rest, which is the only reason `excluded` carries an area prefix at all.
void csv_remove_prefixed(std::string &csv, const std::string &prefix);

/// Whether any entry begins with `prefix`.
bool csv_has_prefixed(const std::string &csv, const std::string &prefix);

}  // namespace satellite1_web_ui
}  // namespace esphome
