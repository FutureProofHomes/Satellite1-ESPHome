// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace esphome::stt_wake_verifier {

// Same English normalization and neighboring-word matching as the bridge.
inline std::string normalize_phrase(const std::string &value) {
  std::string out;
  bool space = false;
  for (unsigned char c : value) {
    if (c >= 'A' && c <= 'Z')
      c += 'a' - 'A';
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) {
      if (space && !out.empty())
        out += ' ';
      out += static_cast<char>(c);
      space = false;
    } else {
      space = true;
    }
  }
  return out;
}

// Ratcliff/Obershelp matching blocks with the same earliest-match tie breaking
// as SequenceMatcher. Wake phrases are short, so autojunk does not apply.
inline size_t matching_characters(const std::string &a, size_t alo, size_t ahi, const std::string &b, size_t blo,
                                  size_t bhi) {
  size_t best_a = alo, best_b = blo, best = 0;
  std::vector<size_t> previous(bhi - blo + 1), current(previous.size());
  for (size_t i = alo; i < ahi; ++i) {
    std::fill(current.begin(), current.end(), 0);
    for (size_t j = blo; j < bhi; ++j) {
      if (a[i] != b[j])
        continue;
      size_t length = current[j - blo + 1] = previous[j - blo] + 1;
      if (length > best) {
        best_a = i - length + 1;
        best_b = j - length + 1;
        best = length;
      }
    }
    previous.swap(current);
  }
  if (!best)
    return 0;
  size_t count = best;
  if (alo < best_a && blo < best_b)
    count += matching_characters(a, alo, best_a, b, blo, best_b);
  if (best_a + best < ahi && best_b + best < bhi)
    count += matching_characters(a, best_a + best, ahi, b, best_b + best, bhi);
  return count;
}

inline float transcript_match_score(const std::string &transcript, const std::string &phrase) {
  const auto target = normalize_phrase(phrase);
  const auto text = normalize_phrase(transcript);
  if (target.empty() || text.empty())
    return 0;
  const size_t max_words = std::count(target.begin(), target.end(), ' ') + 2;
  float score = 0;
  for (size_t start = 0; start < text.size();) {
    size_t end = start;
    for (size_t words = 0; words < max_words && end < text.size(); ++words) {
      end = text.find(' ', end);
      if (end == std::string::npos)
        end = text.size();
      const auto candidate = text.substr(start, end - start);
      const float ratio = 2.0f * matching_characters(candidate, 0, candidate.size(), target, 0, target.size()) /
                          (candidate.size() + target.size());
      score = std::max(score, ratio);
      if (score == 1.0f)
        return score;
      if (end < text.size())
        ++end;
    }
    start = text.find(' ', start);
    if (start == std::string::npos)
      break;
    ++start;
  }
  return score;
}

// Caller owns storage and synchronization. Command queues never overwrite the
// oldest words; only the rolling wake-history buffer is allowed to overwrite.
class PcmBuffer {
 public:
  void bind(int16_t *data, size_t capacity) {
    data_ = data;
    capacity_ = capacity;
    clear();
  }
  void clear() { begin_ = count_ = 0; }
  void adopt_prefix(size_t count) {
    begin_ = 0;
    count_ = std::min(count, capacity_);
  }
  size_t size() const { return count_; }
  size_t capacity() const { return capacity_; }
  bool push(const int16_t *data, size_t count, bool overwrite = false) {
    if (!capacity_ || (!overwrite && count > capacity_ - count_))
      return false;
    for (size_t i = 0; i < count; ++i) {
      if (count_ == capacity_) {
        begin_ = (begin_ + 1) % capacity_;
        --count_;
      }
      data_[(begin_ + count_) % capacity_] = data[i];
      ++count_;
    }
    return true;
  }
  size_t tail(int16_t *out, size_t count) const {
    count = std::min(count, count_);
    for (size_t i = 0; i < count; ++i)
      out[i] = data_[(begin_ + count_ - count + i) % capacity_];
    return count;
  }
  size_t pop(int16_t *out, size_t count) {
    count = std::min(count, count_);
    for (size_t i = 0; i < count; ++i)
      out[i] = data_[(begin_ + i) % capacity_];
    begin_ = (begin_ + count) % capacity_;
    count_ -= count;
    return count;
  }

 protected:
  int16_t *data_{nullptr};
  size_t capacity_{0}, begin_{0}, count_{0};
};

inline bool deadline_expired(uint32_t now, uint32_t started, uint32_t timeout) {
  return static_cast<uint32_t>(now - started) >= timeout;
}

struct VerificationDecision {
  bool allow, reject;
  const char *label;
};

inline VerificationDecision evaluate_check(bool available, bool matched, bool observe, bool late) {
  if (late)
    return {false, false,
            available ? (matched ? "Late match — allowed after timeout" : "Late mismatch — allowed after timeout")
                      : "Timeout — allowed"};
  if (!available)
    return {true, false, nullptr};
  if (observe)
    return {true, false, matched ? "Observed match — allowed" : "Observed mismatch — allowed"};
  return {matched, !matched, matched ? "Accepted" : "Rejected"};
}

}  // namespace esphome::stt_wake_verifier
