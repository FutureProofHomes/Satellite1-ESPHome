#include "verifier_core.h"
#include <cassert>
#include <iostream>
#include <numeric>
using namespace esphome::stt_wake_verifier;

int main(int argc, char **argv) {
  if (argc == 3) {
    std::cout.precision(9);
    std::cout << transcript_match_score(argv[1], argv[2]);
    return 0;
  }
  // Every possible outcome: only a timely, available Enforce mismatch rejects.
  for (bool available : {false, true})
    for (bool matched : {false, true})
      for (bool observe : {false, true})
        for (bool late : {false, true}) {
          auto result = evaluate_check(available, matched, observe, late);
          assert(result.reject == (available && !matched && !observe && !late));
          assert(result.allow == (!late && (!available || matched || observe)));
        }
  assert(std::string(evaluate_check(true, false, true, false).label) == "Observed mismatch — allowed");
  assert(std::string(evaluate_check(true, false, false, true).label) == "Late mismatch — allowed after timeout");
  assert(!deadline_expired(1000, 900, 500));
  assert(deadline_expired(1400, 900, 500));
  assert(deadline_expired(300, UINT32_MAX - 300, 500));

  // Snapshot a rolling wake history, retain the first command samples through
  // a full 2000 ms check and 300 ms chime, then drain without gaps/duplicates.
  int16_t history_memory[32000], command_memory[96000];
  PcmBuffer history, command;
  history.bind(history_memory, 32000);
  command.bind(command_memory, 96000);
  std::vector<int16_t> first(40000);
  std::iota(first.begin(), first.end(), -20000);
  assert(history.push(first.data(), first.size(), true));
  auto count = history.tail(command_memory, 4000);
  command.adopt_prefix(count);
  std::vector<int16_t> speech(36800);
  for (size_t i = 0; i < speech.size(); ++i)
    speech[i] = static_cast<int16_t>(i);
  assert(command.push(speech.data(), speech.size()));
  std::vector<int16_t> expected(first.end() - 4000, first.end()), actual;
  expected.insert(expected.end(), speech.begin(), speech.end());
  int16_t frame[512];
  while ((count = command.pop(frame, 512)))
    actual.insert(actual.end(), frame, frame + count);
  assert(actual == expected);

  int16_t tiny[4], values[] = {1, 2, 3, 4, 5};
  command.bind(tiny, 4);
  assert(command.push(values, 4));
  assert(!command.push(values + 4, 1));  // Never discard the first word to make room.
  assert(command.pop(frame, 4) == 4 && frame[0] == 1 && frame[3] == 4);
  assert(command.push(values, 3));
  assert(command.pop(frame, 2) == 2);
  assert(command.push(values + 2, 3));
  assert(command.pop(frame, 4) == 4 && frame[0] == 3 && frame[3] == 5);
  command.clear();
  assert(command.size() == 0);

  std::cout << "Outcome, timeout, PCM retention, wraparound, overflow checks passed\n";
}
