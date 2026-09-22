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
template<typename F> static void csv_each(const SelString &csv, F fn) {
  size_t start = 0;
  while (start < csv.size()) {
    size_t comma = csv.find(',', start);
    size_t end = comma == SelString::npos ? csv.size() : comma;
    if (end > start)
      fn(start, end - start);
    if (comma == SelString::npos)
      break;
    start = comma + 1;
  }
}

// memcmp rather than basic_string::compare throughout: the haystack is the selection's PSRAM string
// type and the needle a std::string, and compare() has no cross-allocator overload.
bool csv_contains(const SelString &csv, const std::string &needle) {
  if (needle.empty())
    return false;
  bool found = false;
  csv_each(csv, [&](size_t at, size_t len) {
    if (!found && len == needle.size() && memcmp(csv.data() + at, needle.data(), len) == 0)
      found = true;
  });
  return found;
}

bool csv_has_prefixed(const SelString &csv, const std::string &prefix) {
  if (prefix.empty())
    return false;
  bool found = false;
  csv_each(csv, [&](size_t at, size_t len) {
    if (!found && len >= prefix.size() && memcmp(csv.data() + at, prefix.data(), prefix.size()) == 0)
      found = true;
  });
  return found;
}

void csv_add(SelString &csv, const std::string &entry) {
  if (entry.empty() || csv_contains(csv, entry))
    return;
  if (!csv.empty())
    csv += ',';
  csv.append(entry.data(), entry.size());
}

/// Rebuilds the list keeping everything the predicate accepts. Rebuilding rather than splicing in
/// place, because erasing from the middle has to fix up the neighbouring separators and that is where
/// the off-by-one lives.
template<typename P> static void csv_keep(SelString &csv, P keep) {
  SelString out;
  csv_each(csv, [&](size_t at, size_t len) {
    if (!keep(csv, at, len)) {
      return;
    }
    if (!out.empty())
      out += ',';
    out.append(csv, at, len);
  });
  // Pointer-and-length assign rather than a move: basic_string's move assignment compiles an
  // allocator comparison RAMAllocator does not provide - SelectionSet::copy_from explains. The
  // copy is a few hundred PSRAM bytes on a rare mutation.
  csv.assign(out.data(), out.size());
}

void csv_remove(SelString &csv, const std::string &entry) {
  if (entry.empty())
    return;
  csv_keep(csv, [&](const SelString &s, size_t at, size_t len) {
    return !(len == entry.size() && memcmp(s.data() + at, entry.data(), len) == 0);
  });
}

void csv_remove_prefixed(SelString &csv, const std::string &prefix) {
  if (prefix.empty())
    return;
  csv_keep(csv, [&](const SelString &s, size_t at, size_t len) {
    return !(len >= prefix.size() && memcmp(s.data() + at, prefix.data(), prefix.size()) == 0);
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
///
/// Serialized straight into the blob's fixed buffer rather than through a string: the pieces are
/// already in memory and their sizes known, so an intermediate copy bought nothing but a heap spike.
/// Returns the total length, or one past `cap` when it did not fit - the caller refuses, it never
/// truncates.
static size_t blob_write(char *out, size_t cap, const SelectionSet &routing, const SelectionSet &duck, bool local,
                         const SelString &own_area) {
  size_t at = 0;
  const auto put = [&](const char *data, size_t len) {
    if (at + len > cap) {
      at = cap + 1;  // Poisoned: past every valid length, so the caller's size check refuses.
      return;
    }
    memcpy(out + at, data, len);
    at += len;
  };
  const auto put_line = [&](const char *data, size_t len) {
    put(data, len);
    put("\n", 1);
  };
  put_line("1", 1);
  put_line(own_area.data(), own_area.size());
  put_line(local ? "1" : "0", 1);
  put_line(routing.areas.data(), routing.areas.size());
  put_line(routing.extra.data(), routing.extra.size());
  put_line(routing.excluded.data(), routing.excluded.size());
  put_line(duck.areas.data(), duck.areas.size());
  put_line(duck.extra.data(), duck.extra.size());
  put_line(duck.excluded.data(), duck.excluded.size());
  return at;
}

/// Returns false on anything it does not recognise, which leaves the defaults in place. A selection
/// that fails to parse is better lost than half-applied: half a routing target list means answers
/// going somewhere the customer did not choose.
///
/// Parses in place - the fields land in their PSRAM strings straight from the blob, with no working
/// copy of the whole thing.
static bool blob_read(const char *data, size_t len, SelectionSet &routing, SelectionSet &duck, bool &local,
                      SelString &own_area) {
  const char *at = data;
  const char *end = data + len;
  const char *fields[9];
  size_t field_lens[9];
  for (size_t i = 0; i < 9; i++) {
    const char *nl = static_cast<const char *>(memchr(at, '\n', static_cast<size_t>(end - at)));
    if (nl == nullptr)
      return false;
    fields[i] = at;
    field_lens[i] = static_cast<size_t>(nl - at);
    at = nl + 1;
  }
  if (field_lens[0] != 1 || fields[0][0] != '1')
    return false;

  own_area.assign(fields[1], field_lens[1]);
  local = !(field_lens[2] == 1 && fields[2][0] == '0');
  routing.areas.assign(fields[3], field_lens[3]);
  routing.extra.assign(fields[4], field_lens[4]);
  routing.excluded.assign(fields[5], field_lens[5]);
  duck.areas.assign(fields[6], field_lens[6]);
  duck.extra.assign(fields[7], field_lens[7]);
  duck.excluded.assign(fields[8], field_lens[8]);
  return true;
}

/* ------------------------------------------------------------------ */
/* Selection                                                           */
/* ------------------------------------------------------------------ */

Selection::Blob *Selection::blob_scratch_() {
  if (this->blob_ == nullptr) {
    RAMAllocator<Blob> alloc(RAMAllocator<Blob>::NONE);  // PSRAM first, internal fallback.
    this->blob_ = alloc.allocate(1);
  }
  return this->blob_;
}

void Selection::setup() {
  this->pref_ = global_preferences->make_preference<Blob>(fnv1_hash("satellite1_web_ui_selection"));

  Blob *blob = this->blob_scratch_();
  if (blob == nullptr)
    return;
  if (!this->pref_.load(blob)) {
    ESP_LOGD(TAG_SEL, "No stored selection; starting with the local speaker only");
    return;
  }
  if (blob->len > BLOB_MAX) {
    ESP_LOGW(TAG_SEL, "Stored selection claims %u bytes; ignoring it", static_cast<unsigned>(blob->len));
    return;
  }
  if (!blob_read(blob->data, blob->len, this->routing_, this->duck_, this->local_speaker_, this->own_area_)) {
    ESP_LOGW(TAG_SEL, "Stored selection did not parse; starting fresh");
    this->routing_.clear();
    this->duck_.clear();
    this->local_speaker_ = true;
    this->own_area_.clear();
    return;
  }
  ESP_LOGD(TAG_SEL, "Selection restored: routing areas [%s] extra [%s], duck areas [%s] extra [%s], local %s",
           this->routing_.areas.c_str(), this->routing_.extra.c_str(), this->duck_.areas.c_str(),
           this->duck_.extra.c_str(), YESNO(this->local_speaker_));
}

void Selection::save_() {
  Blob *blob = this->blob_scratch_();
  if (blob == nullptr)
    return;
  const size_t len = blob_write(blob->data, BLOB_MAX, this->routing_, this->duck_, this->local_speaker_,
                                this->own_area_);

  if (len > BLOB_MAX) {
    // Refusing rather than truncating. A truncated list is a different list, and it would be a list
    // the customer never chose - so the previous one stands and the log says why. The scratch holds
    // a half-written serialization now, which is fine: every use rewrites it from the start.
    ESP_LOGE(TAG_SEL, "Selection does not fit the %u byte store; keeping the previous one",
             static_cast<unsigned>(BLOB_MAX));
    return;
  }

  // The scratch is reused, so the tail past `len` still holds the previous serialization. Zeroed
  // before the save: the whole struct goes to NVS, and a garbage tail would make identical
  // selections look different to the preferences layer's are-you-unchanged check - flash wear for
  // nothing.
  memset(blob->data + len, 0, BLOB_MAX - len);
  blob->len = static_cast<uint16_t>(len);
  this->pref_.save(blob);
  this->change_callback_.call();
}

void Selection::set_local_speaker(bool on) {
  if (this->local_speaker_ == on)
    return;
  this->local_speaker_ = on;
  this->save_();
}

void Selection::set_own_area(const std::string &area_id) {
  if (this->own_area_.size() == area_id.size() && memcmp(this->own_area_.data(), area_id.data(), area_id.size()) == 0)
    return;
  this->own_area_.assign(area_id.data(), area_id.size());
  this->save_();
}

bool Selection::whole_own_area_(const SelectionSet &set) const {
  if (this->own_area_.empty())
    return false;
  // Transient std::string copies of a short area id, because the csv helpers take the needle as the
  // std::string their other callers hold. These run when a derived switch is read, not per frame.
  const std::string own(this->own_area_.c_str(), this->own_area_.size());
  return csv_contains(set.areas, own) && !csv_has_prefixed(set.excluded, own + ":");
}

void Selection::set_whole_own_area_(SelectionSet &set, bool on) {
  if (this->own_area_.empty()) {
    // Nothing to select. This happens when Home Assistant has never told us our area, and it is the
    // one case where the switch cannot honour a write - so it says so rather than silently no-oping.
    ESP_LOGW(TAG_SEL, "No area known for this device yet; cannot select it");
    return;
  }
  const std::string own(this->own_area_.c_str(), this->own_area_.size());
  if (on) {
    csv_add(set.areas, own);
  } else {
    csv_remove(set.areas, own);
  }
  // Either way the carve-outs for this area are meaningless afterwards: with the area selected whole
  // there is nothing carved out, and with it deselected there is nothing to carve out of.
  csv_remove_prefixed(set.excluded, own + ":");
  this->save_();
}

void Selection::replace(const SelectionSet &routing, const SelectionSet &duck, bool local_speaker) {
  // copy_from, not operator= - see its comment in selection.h.
  this->routing_.copy_from(routing);
  this->duck_.copy_from(duck);
  this->local_speaker_ = local_speaker;
  this->save_();
}

/// Emits the csv fields as JSON arrays, so the app gets a shape it can put straight into a Set rather
/// than splitting strings itself.
static void csv_to_json_array(SelString &out, const SelString &csv) {
  out += '[';
  bool first = true;
  csv_each(csv, [&](size_t at, size_t len) {
    if (!first)
      out += ',';
    first = false;
    out += '"';
    out.append(csv.data() + at, len);
    out += '"';
  });
  out += ']';
}

static void set_to_json(SelString &out, const SelectionSet &set) {
  out += "{\"areas\":";
  csv_to_json_array(out, set.areas);
  out += ",\"extra\":";
  csv_to_json_array(out, set.extra);
  out += ",\"excluded\":";
  csv_to_json_array(out, set.excluded);
  out += '}';
}

void Selection::to_json(SelString &out) const {
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
