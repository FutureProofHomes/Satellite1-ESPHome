// Collaborators only: all control decisions under test are inserted from production.
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <sys/types.h>
#define ESP_LOGW(...) ((void) 0)
#define ESP_LOGD(...) ((void) 0)

template<typename T, typename U> T remap(U value, U low, U high, T out_low, T out_high) {
  return static_cast<T>((value - low) * (out_high - out_low) / (high - low) + out_low);
}
template<typename> struct CallbackManager;
template<typename... Args> struct CallbackManager<void(Args...)> {
  std::vector<std::function<void(Args...)>> callbacks;
  template<typename F> void add(F &&f) { callbacks.emplace_back(std::forward<F>(f)); }
  void call(Args... args) { for (auto &f : callbacks) f(args...); }
};
// PRODUCTION_STATE

struct Provider : VolumeState {
  void initialize() override { publish_state_(0.5f, false); }
  bool set_volume(float v) override { return publish_state_(v, state_.muted); }
  bool set_muted(bool m) override { return publish_state_(state_.volume, m); }
  void refresh() { publish_state_(state_.volume, state_.muted, true); }
};
namespace audio_dac {
struct AudioDac {
  virtual ~AudioDac() = default;
  virtual bool set_volume(float) = 0;
  virtual bool set_mute_on() = 0;
  virtual bool set_mute_off() = 0;
  virtual float volume() = 0;
  virtual bool is_muted() = 0;
};
}
struct FakeDac : audio_dac::AudioDac {
  std::vector<std::string> calls;
  float gain{0.5f};
  bool muted{true}, fail_gain{false}, fail_mute{false};
  std::function<void()> on_gain, on_mute;
  bool set_volume(float v) override {
    calls.push_back("gain"); gain = v;
    if (on_gain) on_gain();
    return !fail_gain;
  }
  bool set_mute_on() override {
    calls.push_back("mute"); muted = true;
    if (on_mute) on_mute();
    return !fail_mute;
  }
  bool set_mute_off() override { calls.push_back("unmute"); muted = false; return true; }
  float volume() override { return gain; }
  bool is_muted() override { return muted; }
};
enum DacOutput : uint8_t { SPEAKER = 0, LINE_OUT = 1 };
// Preference I/O is deliberately in-memory/no-restore; persistence and migration are not tested.
struct ESPPreferenceObject {
  template<typename T> bool load(T *) { return false; }
  template<typename T> bool save(const T *) { return true; }
};
struct Preferences {
  template<typename T> ESPPreferenceObject make_preference(uint32_t) { return {}; }
} preferences;
auto *global_preferences = &preferences;
uint32_t fnv1_hash(const char *) { return 0; }
template<typename T> T clamp(T v, T low, T high) { return std::clamp(v, low, high); }
struct DACProxyRestoreState {
  uint8_t dac_output; float speaker_volume; bool speaker_is_muted;
  float line_out_volume; bool line_out_is_muted;
};
struct DACProxyRoutingPreferences { uint8_t dac_output{LINE_OUT}; };
class DACOutputVolumeState;
bool valid_output(int output) { return output == 0 || output == 1; }
bool valid_volume(float v) { return std::isfinite(v) && v >= 0 && v <= 1; }
class DACProxy : public audio_dac::AudioDac {
 public:
  struct AppliedState { bool mute_valid{false}, muted{true}, volume_valid{false}; float volume{0}; };
  audio_dac::AudioDac *tas2780_{nullptr}, *pcm5122_{nullptr};
  AppliedState speaker_applied_, lineout_applied_;
  DacOutput active_dac{SPEAKER}, requested_dac_{SPEAKER}, pending_output_{SPEAKER};
  bool audio_ready_{true}, target_muted_{true}, fresh_volume_received_{false}, fresh_mute_received_{false};
  bool switching_output_{false}, setup_was_called_{true};
  bool requested_output_notified_{true}, selection_pending_{false};
  DACOutputVolumeState *volume_state_{nullptr};
  ESPPreferenceObject pref_;
  DACProxyRoutingPreferences restore_state_;
  CallbackManager<void()> requested_output_callback_, output_change_callback_, selection_callback_, state_callback_;
  std::vector<std::function<void()>> deferred;
  float target_volume_{0.5f};
  // PRODUCTION_PROXY_ACCESSORS
  template<typename T> ESPPreferenceObject make_entity_preference() { return {}; }
  void send_selected_dac_() {}
  void defer(std::function<void()> f) { deferred.push_back(std::move(f)); }
  void defer(const char *, std::function<void()> f) { defer(std::move(f)); }
  void apply_volume_limits_(DacOutput) {}
  bool request_output(DacOutput);
  void notify_requested_output_();
  void select_output_(DacOutput);
  void set_audio_ready(bool);
  bool apply_mute_(audio_dac::AudioDac *, AppliedState &, bool);
  bool reconcile_();
  bool receiving_current_output_() const;
  bool set_volume(float) override;
  bool set_mute_on() override;
  bool set_mute_off() override;
  float volume() override { return target_volume_; }
  bool is_muted() override { return target_muted_; }
};
namespace speaker_source { using ::VolumeState; }
// PRODUCTION_PROVIDER
struct I2SAudioSpeaker {
  audio_dac::AudioDac *audio_dac_{nullptr};
  float volume_{0.5f}, pending_volume_{-1};
  bool mute_state_{false}, pending_mute_state_{false}, has_pending_volume_{false}, has_pending_mute_{false};
  std::atomic<int16_t> q15_volume_factor_{INT16_MAX};
  void process_volume_state_();
  void update_software_volume_();
  void set_volume(float);
  void set_mute_state(bool);
};
template<typename... Args> struct Trigger {
  std::function<void(Args...)> callback;
  void trigger(Args... args) { if (callback) callback(args...); }
};
struct Source {
  std::vector<float> volumes;
  std::vector<bool> mutes;
  void notify_volume_changed(float v) { volumes.push_back(v); }
  void notify_mute_changed(bool m) { mutes.push_back(m); }
};
struct Binding { Source *source; };
struct Output {
  I2SAudioSpeaker *backend{nullptr};
  std::vector<float> volumes;
  std::vector<bool> mutes;
  std::function<void()> on_volume;
  void set_volume(float v) {
    volumes.push_back(v);
    if (backend) backend->set_volume(v);
    if (on_volume) on_volume();
  }
  void set_mute_state(bool m) { mutes.push_back(m); if (backend) backend->set_mute_state(m); }
};
struct Pipeline {
  Output *speaker{nullptr};
  std::vector<std::unique_ptr<Binding>> sources;
  bool is_configured() const { return speaker != nullptr; }
};
struct SpeakerSourceMediaPlayer {
  VolumeState *volume_state_;
  float volume{1}, volume_min_{0.1f}, volume_max_{0.9f};
  bool is_muted_{false}, volume_state_initialized_{false}, source_volume_state_initialized_{false};
  bool completed_volume_state_initialized_{false}, volume_notifications_queued_{false};
  VolumeMuteState completed_volume_state_{1, false}, notified_volume_state_{1, false};
  std::vector<Pipeline> pipelines_{2};
  Trigger<float> volume_trigger_;
  Trigger<> mute_trigger_, unmute_trigger_;
  std::vector<std::function<void()>> deferred;
  int publications{0};
  void publish_state() { ++publications; }
  void defer(std::function<void()> f) { deferred.push_back(std::move(f)); }
  void drain() {
    for (int pass = 0; !deferred.empty(); ++pass) {
      assert(pass < 10);
      auto batch = std::move(deferred); deferred.clear();
      for (auto &f : batch) f();
    }
  }
  void handle_volume_state_(const VolumeMuteState &);
  void queue_volume_notifications_();
};
// PRODUCTION_METHODS

void test_player() {
  Provider provider;
  Output a, b;
  Source source;
  SpeakerSourceMediaPlayer player;
  player.volume_state_ = &provider;
  player.pipelines_[0].speaker = &a;
  player.pipelines_[1].speaker = &b;
  player.pipelines_[0].sources.emplace_back(new Binding{&source});
  provider.add_on_state_callback([&](const VolumeMuteState &s) { player.handle_volume_state_(s); });
  provider.initialize(); player.drain();
  for (const auto &[v, expected] : std::vector<std::pair<float, float>>{
           {0.0f, 0.0f}, {0.0009f, 0.0f}, {0.001f, 0.1008f}, {0.5f, 0.5f}}) {
    provider.set_volume(v); player.drain();
    assert(std::abs(a.volumes.back() - expected) < 0.000001f);
    assert(a.volumes.back() == b.volumes.back());
    assert(source.volumes.back() == v && !provider.state().muted && !a.mutes.back());
  }
  provider.set_muted(true); provider.set_volume(0); player.drain();
  assert(provider.state().muted && a.mutes.back() && source.mutes.back());
  const auto notifications = source.volumes.size();
  const auto forwards = a.mutes.size();
  const auto publications = player.publications;
  provider.refresh(); player.drain();
  assert(a.mutes.size() == forwards + 1 && source.volumes.size() == notifications);
  assert(player.publications == publications);
  // Superseded snapshot must stop before its mute is forwarded to either output.
  a.on_volume = [&] { provider.set_muted(true); };
  provider.set_muted(false); player.drain();
  assert(provider.state().muted && a.mutes.back() && b.mutes.back());
  a.on_volume = nullptr;
  // A volume automation must not emit the stale opposite mute notification.
  int unmutes = 0;
  player.unmute_trigger_.callback = [&] { ++unmutes; };
  player.volume_trigger_.callback = [&](float) { provider.set_muted(true); };
  provider.set_muted(false); provider.set_volume(0.6f); player.drain();
  assert(unmutes == 0 && provider.state().muted);
}

void test_software() {
  I2SAudioSpeaker speaker;
  speaker.set_volume(0); assert(speaker.q15_volume_factor_ == 0);
  speaker.set_volume(0.0009f); assert(speaker.q15_volume_factor_ == 0);
  speaker.set_volume(1); assert(speaker.q15_volume_factor_ == INT16_MAX);
  speaker.set_mute_state(true); speaker.set_volume(0.5f); assert(speaker.q15_volume_factor_ == 0);
  speaker.set_mute_state(false); assert(speaker.q15_volume_factor_ > 0);
  const auto gain = speaker.q15_volume_factor_.load();
  speaker.set_volume(NAN); speaker.set_volume(-1); speaker.set_volume(2);
  assert(speaker.q15_volume_factor_ == gain);
}

#ifdef USE_AUDIO_DAC
void test_backend() {
  FakeDac dac;
  I2SAudioSpeaker speaker;
  speaker.audio_dac_ = &dac;
  speaker.set_volume(0); assert(dac.calls.empty());
  speaker.process_volume_state_();
  assert((dac.calls == std::vector<std::string>{"mute", "gain"}));
  assert(!speaker.mute_state_);
  speaker.set_mute_state(false); speaker.process_volume_state_(); assert(dac.muted);
  speaker.set_volume(0.0009f); speaker.process_volume_state_(); assert(dac.muted);
  dac.calls.clear(); speaker.set_volume(0.001f); speaker.process_volume_state_();
  assert((dac.calls == std::vector<std::string>{"gain", "unmute"}));
  speaker.set_mute_state(true); speaker.set_volume(0.8f); speaker.process_volume_state_(); assert(dac.muted);
  dac.calls.clear(); dac.fail_gain = true;
  speaker.set_mute_state(false); speaker.set_volume(0.7f); speaker.process_volume_state_();
  assert((dac.calls == std::vector<std::string>{"gain"}));
  speaker.process_volume_state_(); assert(dac.calls.size() == 1); // No implicit retry.
  dac.fail_gain = false;
  dac.on_gain = [&] { speaker.set_volume(0); };
  dac.calls.clear(); speaker.set_volume(0.9f); speaker.process_volume_state_();
  assert((dac.calls == std::vector<std::string>{"gain"}));
  dac.on_gain = nullptr; speaker.process_volume_state_(); assert(dac.muted && dac.gain == 0);
  // Even continuously reentrant gain updates consume only one snapshot per pass.
  dac.on_gain = [&] { speaker.set_volume(0.6f); };
  dac.calls.clear(); speaker.set_volume(0.5f); speaker.process_volume_state_();
  assert(dac.calls.size() == 1);
  dac.on_gain = nullptr; speaker.process_volume_state_(); assert(!dac.muted && dac.gain == 0.6f);
  // A newer audible gain requested during mute must remain queued until the next pass.
  dac.on_mute = [&] { speaker.set_volume(0.8f); };
  dac.calls.clear(); speaker.set_volume(0); speaker.process_volume_state_();
  assert((dac.calls == std::vector<std::string>{"mute", "gain"}));
  assert(dac.muted && speaker.has_pending_volume_);
  dac.on_mute = nullptr; speaker.process_volume_state_(); assert(!dac.muted && dac.gain == 0.8f);
  dac.fail_mute = true; dac.calls.clear(); speaker.set_volume(0); speaker.process_volume_state_();
  assert((dac.calls == std::vector<std::string>{"mute", "gain"}));
}

void test_proxy() {
  FakeDac selected, inactive;
  DACProxy proxy;
  proxy.tas2780_ = &selected; proxy.pcm5122_ = &inactive;
  I2SAudioSpeaker speaker; speaker.audio_dac_ = &proxy;
  speaker.set_volume(0); speaker.set_mute_state(false); speaker.process_volume_state_();
  assert(selected.muted && inactive.muted && proxy.target_muted_);
  selected.fail_gain = true; selected.calls.clear();
  speaker.set_volume(0.5f); speaker.process_volume_state_();
  assert(std::count(selected.calls.begin(), selected.calls.end(), "gain") == DAC_WRITE_ATTEMPTS);
  assert(std::count(selected.calls.begin(), selected.calls.end(), "unmute") == 0);
  // set_mute_off itself reconciles gain; failed writes must still gate physical unmute.
  selected.calls.clear(); assert(!proxy.set_mute_off());
  assert(std::count(selected.calls.begin(), selected.calls.end(), "unmute") == 0);
  selected.fail_gain = false; selected.calls.clear();
  speaker.set_volume(0.6f); speaker.process_volume_state_();
  assert((selected.calls == std::vector<std::string>{"gain", "unmute"}));
  assert(inactive.muted);
  proxy.audio_ready_ = false; proxy.reconcile_(); assert(selected.muted);
  proxy.audio_ready_ = true; proxy.fresh_mute_received_ = false;
  proxy.reconcile_(); assert(selected.muted); // Route refresh requires both fields.
  proxy.set_volume(0.7f); assert(selected.muted);
  proxy.set_mute_off(); assert(!selected.muted);
  proxy.requested_dac_ = LINE_OUT; selected.calls.clear(); proxy.set_volume(0.8f);
  assert(std::count(selected.calls.begin(), selected.calls.end(), "gain") == 0);
}

void test_profile_routes() {
  FakeDac internal, lineout;
  DACProxy proxy;
  proxy.tas2780_ = &internal; proxy.pcm5122_ = &lineout;
  DACOutputVolumeState provider;
  provider.set_parent(&proxy);
  I2SAudioSpeaker speaker;
  speaker.audio_dac_ = &proxy;
  Output output;
  output.backend = &speaker;
  SpeakerSourceMediaPlayer player;
  player.volume_state_ = &provider;
  player.pipelines_[0].speaker = &output;
  provider.add_on_state_callback([&](const VolumeMuteState &state) { player.handle_volume_state_(state); });
  provider.initialize();
  auto flush = [&] { player.drain(); speaker.process_volume_state_(); };
  flush();
  assert(!internal.muted && lineout.muted);
  provider.set_volume(0); flush();
  assert(internal.muted && !provider.state().muted);

  // Production request/selection/publication performs routing; this helper only drives
  // the two asynchronous phases and checks the physical outputs at their boundaries.
  auto switch_to = [&](DacOutput destination, float logical, float physical, bool muted) {
    assert(proxy.request_output(destination));
    assert(internal.muted && lineout.muted);
    assert(provider.state().volume == logical && provider.state().muted == muted);
    // Deliver the request-time pair before physical selection: it must not count
    // as a destination refresh or program destination gain onto the old output.
    const float old_gain = proxy.active_dac == SPEAKER ? internal.gain : lineout.gain;
    flush();
    assert(!proxy.fresh_volume_received_ && !proxy.fresh_mute_received_);
    assert((proxy.active_dac == SPEAKER ? internal.gain : lineout.gain) == old_gain);
    proxy.select_output_(destination);
    assert(proxy.active_dac == destination);
    assert(!proxy.fresh_volume_received_ && !proxy.fresh_mute_received_);
    assert(internal.muted && lineout.muted);
    flush();
    assert(proxy.fresh_volume_received_ && proxy.fresh_mute_received_);
    auto &selected = destination == SPEAKER ? internal : lineout;
    auto &inactive = destination == SPEAKER ? lineout : internal;
    assert(std::abs(selected.gain - physical) < 0.000001f);
    assert(selected.muted == (muted || logical == 0));
    assert(inactive.muted && provider.state().muted == muted);
    assert(player.volume == logical && player.is_muted_ == muted);
  };
  switch_to(LINE_OUT, 0.5f, 0.5f, false); // Silent speaker -> audible line-out.
  switch_to(SPEAKER, 0.0f, 0.0f, false);
  provider.set_volume(0.5f); flush();
  switch_to(LINE_OUT, 0.5f, 0.5f, false); // Identical logical pair still needs fresh fields.
  provider.set_volume(0); flush();
  switch_to(SPEAKER, 0.5f, 0.5f, false); // Silent line-out -> audible speaker.
  switch_to(LINE_OUT, 0.0f, 0.0f, false);
  provider.set_muted(true); flush();
  switch_to(SPEAKER, 0.5f, 0.5f, true); // Destination gain never clears explicit mute.
  provider.set_volume(0.7f); flush(); assert(internal.muted);
  switch_to(LINE_OUT, 0.0f, 0.0f, true);
  provider.set_muted(false); flush(); assert(lineout.muted); // Zero is still silence.
  switch_to(SPEAKER, 0.7f, 0.66f, false);

  proxy.set_audio_ready(false);
  assert(internal.muted && lineout.muted);
  assert(provider.state().volume == 0.7f && !provider.state().muted);
  provider.set_volume(0.8f); flush(); assert(internal.muted && lineout.muted);
  internal.calls.clear();
  proxy.set_audio_ready(true);
  assert((internal.calls == std::vector<std::string>{"gain", "unmute"}));
  assert(!internal.muted && lineout.muted && std::abs(internal.gain - 0.74f) < 0.000001f);
  assert(provider.state().volume == 0.8f && !provider.state().muted);
  assert(proxy.deferred.empty()); // These non-reentrant transitions complete without scheduler retries.
}
#endif

int main() {
  test_player(); test_software();
#ifdef USE_AUDIO_DAC
  test_backend(); test_proxy(); test_profile_routes();
  std::cout << "PASS: 5 groups: player, software, deferred DAC, proxy, integrated profiles (8 switches)\n";
#else
  std::cout << "PASS: 2 groups: player, software (USE_AUDIO_DAC disabled)\n";
#endif
}
