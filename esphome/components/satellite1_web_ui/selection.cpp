#include "selection.h"

#include <cstring>

#include "esphome/core/log.h"

namespace esphome {
namespace satellite1_web_ui {

static const char *const TAG_SEL = "satellite1_web_ui.sel";

/* ------------------------------------------------------------------ */
/* Comma-separated list helpers                                        */
/* ------------------------------------------------------------------ */

/// Walks `csv` entry by entry, calling `fn(start, len)`. Empty entries are skipped, so a trailing or
/// doubled comma is harmless rather than something every caller has to guard against.
template<typename F> static void csv_each(const std::string &csv, F fn) {
  size_t start = 0;
  while (start < csv.size()) {
    size_t comma = csv.find(',', start);
    size_t end = comma == std::string::npos ? csv.size() : comma;
    if (end > start)
      fn(start, end - start);
    if (comma == std::string::npos)
      break;
    start = comma + 1;
  }
}

bool csv_contains(const std::string &csv, const std::string &needle) {
  if (needle.empty())
    return false;
  bool found = false;
  csv_each(csv, [&](size_t at, size_t len) {
    if (!found && len == needle.size() && csv.compare(at, len, needle) == 0)
      found = true;
  });
  return found;
}

bool csv_has_prefixed(const std::string &csv, const std::string &prefix) {
  if (prefix.empty())
    return false;
  bool found = false;
  csv_each(csv, [&](size_t at, size_t len) {
    if (!found && len >= prefix.size() && csv.compare(at, prefix.size(), prefix) == 0)
      found = true;
  });
  return found;
}

void csv_add(std::string &csv, const std::string &entry) {
  if (entry.empty() || csv_contains(csv, entry))
    return;
  if (!csv.empty())
    csv += ',';
  csv += entry;
}

/// Rebuilds the list keeping everything the predicate accepts. Rebuilding rather than splicing in
/// place, because erasing from the middle has to fix up the neighbouring separators and that is where
/// the off-by-one lives.
template<typename P> static void csv_keep(std::string &csv, P keep) {
  std::string out;
  csv_each(csv, [&](size_t at, size_t len) {
    if (!keep(csv, at, len)) {
      return;
    }
    if (!out.empty())
      out += ',';
    out.append(csv, at, len);
  });
  csv = std::move(out);
}

void csv_remove(std::string &csv, const std::string &entry) {
  if (entry.empty())
    return;
  csv_keep(csv, [&](const std::string &s, size_t at, size_t len) {
    return !(len == entry.size() && s.compare(at, len, entry) == 0);
  });
}

void csv_remove_prefixed(std::string &csv, const std::string &prefix) {
  if (prefix.empty())
    return;
  csv_keep(csv, [&](const std::string &s, size_t at, size_t len) {
    return !(len >= prefix.size() && s.compare(at, prefix.size(), prefix) == 0);
  });
}

/* ------------------------------------------------------------------ */
/* Serialization                                                       */
/* ------------------------------------------------------------------ */

/// One line per field, in a fixed order, with a version marker first.
///
/// Text rather than a packed binary struct: every field is already a string, the whole thing is
/// legible in a log line when something goes wrong, and adding a field later means appending a line
/// rather than versioning a layout. Newline is a safe separator because area ids and entity ids
/// cannot contain one.
static void blob_write(std::string &out, const SelectionSet &routing, const SelectionSet &duck, bool local,
                       const std::string &own_area) {
  out = "1\n";
  out += own_area + "\n";
  out += (local ? "1" : "0");
  out += "\n";
  out += routing.areas + "\n" + routing.extra + "\n" + routing.excluded + "\n";
  out += duck.areas + "\n" + duck.extra + "\n" + duck.excluded + "\n";
}

/// Returns false on anything it does not recognise, which leaves the defaults in place. A selection
/// that fails to parse is better lost than half-applied: half a routing target list means answers
/// going somewhere the customer did not choose.
static bool blob_read(const char *data, size_t len, SelectionSet &routing, SelectionSet &duck, bool &local,
                      std::string &own_area) {
  std::string s(data, len);
  std::string fields[9];
  size_t start = 0;
  for (auto &field : fields) {
    size_t nl = s.find('\n', start);
    if (nl == std::string::npos)
      return false;
    field = s.substr(start, nl - start);
    start = nl + 1;
  }
  if (fields[0] != "1")
    return false;

  own_area = fields[1];
  local = fields[2] != "0";
  routing.areas = fields[3];
  routing.extra = fields[4];
  routing.excluded = fields[5];
  duck.areas = fields[6];
  duck.extra = fields[7];
  duck.excluded = fields[8];
  return true;
}

/* ------------------------------------------------------------------ */
/* Selection                                                           */
/* ------------------------------------------------------------------ */

void Selection::setup() {
  this->pref_ = global_preferences->make_preference<Blob>(fnv1_hash("satellite1_web_ui_selection"));

  Blob blob{};
  if (!this->pref_.load(&blob)) {
    ESP_LOGD(TAG_SEL, "No stored selection; starting with the local speaker only");
    return;
  }
  if (blob.len > BLOB_MAX) {
    ESP_LOGW(TAG_SEL, "Stored selection claims %u bytes; ignoring it", static_cast<unsigned>(blob.len));
    return;
  }
  if (!blob_read(blob.data, blob.len, this->routing_, this->duck_, this->local_speaker_, this->own_area_)) {
    ESP_LOGW(TAG_SEL, "Stored selection did not parse; starting fresh");
    this->routing_ = SelectionSet{};
    this->duck_ = SelectionSet{};
    this->local_speaker_ = true;
    this->own_area_.clear();
    return;
  }
  ESP_LOGD(TAG_SEL, "Selection restored: routing areas [%s] extra [%s], duck areas [%s] extra [%s], local %s",
           this->routing_.areas.c_str(), this->routing_.extra.c_str(), this->duck_.areas.c_str(),
           this->duck_.extra.c_str(), YESNO(this->local_speaker_));
}

void Selection::save_() {
  std::string text;
  blob_write(text, this->routing_, this->duck_, this->local_speaker_, this->own_area_);

  if (text.size() > BLOB_MAX) {
    // Refusing rather than truncating. A truncated list is a different list, and it would be a list
    // the customer never chose - so the previous one stands and the log says why.
    ESP_LOGE(TAG_SEL, "Selection needs %u bytes and the store holds %u; keeping the previous one",
             static_cast<unsigned>(text.size()), static_cast<unsigned>(BLOB_MAX));
    return;
  }

  Blob blob{};
  blob.len = static_cast<uint16_t>(text.size());
  memcpy(blob.data, text.c_str(), text.size());
  this->pref_.save(&blob);
  this->change_callback_.call();
}

void Selection::set_local_speaker(bool on) {
  if (this->local_speaker_ == on)
    return;
  this->local_speaker_ = on;
  this->save_();
}

void Selection::set_own_area(const std::string &area_id) {
  if (this->own_area_ == area_id)
    return;
  this->own_area_ = area_id;
  this->save_();
}

bool Selection::whole_own_area_(const SelectionSet &set) const {
  if (this->own_area_.empty())
    return false;
  return csv_contains(set.areas, this->own_area_) && !csv_has_prefixed(set.excluded, this->own_area_ + ":");
}

void Selection::set_whole_own_area_(SelectionSet &set, bool on) {
  if (this->own_area_.empty()) {
    // Nothing to select. This happens when Home Assistant has never told us our area, and it is the
    // one case where the switch cannot honour a write - so it says so rather than silently no-oping.
    ESP_LOGW(TAG_SEL, "No area known for this device yet; cannot select it");
    return;
  }
  if (on) {
    csv_add(set.areas, this->own_area_);
  } else {
    csv_remove(set.areas, this->own_area_);
  }
  // Either way the carve-outs for this area are meaningless afterwards: with the area selected whole
  // there is nothing carved out, and with it deselected there is nothing to carve out of.
  csv_remove_prefixed(set.excluded, this->own_area_ + ":");
  this->save_();
}

void Selection::replace(const SelectionSet &routing, const SelectionSet &duck, bool local_speaker) {
  this->routing_ = routing;
  this->duck_ = duck;
  this->local_speaker_ = local_speaker;
  this->save_();
}

/// Emits the csv fields as JSON arrays, so the app gets a shape it can put straight into a Set rather
/// than splitting strings itself.
static void csv_to_json_array(std::string &out, const std::string &csv) {
  out += '[';
  bool first = true;
  csv_each(csv, [&](size_t at, size_t len) {
    if (!first)
      out += ',';
    first = false;
    out += '"';
    out.append(csv, at, len);
    out += '"';
  });
  out += ']';
}

static void set_to_json(std::string &out, const SelectionSet &set) {
  out += "{\"areas\":";
  csv_to_json_array(out, set.areas);
  out += ",\"extra\":";
  csv_to_json_array(out, set.extra);
  out += ",\"excluded\":";
  csv_to_json_array(out, set.excluded);
  out += '}';
}

void Selection::to_json(std::string &out) const {
  out += "{\"local\":";
  out += this->local_speaker_ ? "1" : "0";
  out += ",\"area\":\"";
  out += this->own_area_;
  out += "\",\"routing\":";
  set_to_json(out, this->routing_);
  out += ",\"duck\":";
  set_to_json(out, this->duck_);
  out += '}';
}

}  // namespace satellite1_web_ui
}  // namespace esphome
