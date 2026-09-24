#pragma once

// Before any USE_* test, for the reason web_ui_handler.h gives: the guards below must see the
// generated define set, not an empty one.
#include "esphome/core/defines.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace satellite1_web_ui {

/// How long a pairing window stays open, and how long its outcome stays readable afterwards.
///
/// 30 seconds (owner decision, September 23 2026, down from 60) is the whole budget for walking to
/// the device or hearing the spoken code out - a watchdog downgrade resets it, so a fallback still
/// gets a full window. The grace periods exist because the browser polls every half-second to
/// second, so an approval or a failure must outlive the moment it happened by at least a few polls
/// or the page would show "expired" for a sign-in that actually succeeded.
static constexpr uint32_t SG_WINDOW_MS = 30000;
static constexpr uint32_t SG_APPROVED_GRACE_MS = 30000;
static constexpr uint32_t SG_ENDED_GRACE_MS = 15000;

/// The offline challenge's length, in wake-word symbols over the three-model alphabet. Six (owner
/// decision, September 23 2026, up from three): 3^6 = 729 combinations per window, against the
/// voice lockout's exponential backoff. The page shows the words as chips, so a longer answer
/// costs the honest user seconds, not memory.
static constexpr uint8_t SG_SEQ_LEN = 6;

/// The quiet period that closes an offline wake-word answer that is anything short of perfect. A
/// perfect answer approves the moment its final symbol lands (consume_login_wake) - the match is
/// anchored to the first SG_SEQ_LEN symbols heard, so that is one judgment either way and a
/// count-up gains nothing. This period judges the rest: wrong or short answers wait it out and are
/// denied in one verdict, so their timing never reveals which symbol missed.
static constexpr uint32_t SG_SEQ_QUIET_MS = 8000;

/// Password-login rate limit: this many consecutive failures lock the password path for the
/// window below. Scoped to the password path alone - the button and voice approvals stay
/// available, so a brute-force attempt cannot deny the owner their own front door.
static constexpr uint8_t SG_PW_FAILS = 5;
static constexpr uint32_t SG_PW_LOCKOUT_MS = 60000;

/// New-password bounds for the authenticated change endpoint. 31 because the NVS global that
/// persists it caps at max_restore_data_length: 32 (common/web_ui.yaml) - a longer password would
/// silently fail to restore and lock everyone out on the next boot. 8 is the floor.
static constexpr size_t SG_PW_MIN = 8;
static constexpr size_t SG_PW_MAX = 31;

/// Voice-approval lockout: after this many failed voice windows the voice paths close and new
/// windows open button-only, for a period that doubles per failure up to the cap. The offline
/// challenge is a 1-in-729 guess per window; the backoff is what turns that into hours per try.
static constexpr uint8_t SG_VOICE_FAILS = 3;
static constexpr uint32_t SG_VOICE_LOCKOUT_BASE_MS = 60000;
static constexpr uint32_t SG_VOICE_LOCKOUT_MAX_MS = 3600000;

/// Minimum gap between two pairing-window opens, so a hostile client cannot hold the window open
/// perpetually by reopening it - which would also deny sign-in to everyone else.
static constexpr uint32_t SG_START_GAP_MS = 3000;

/// How many code mismatches close an online-mode window.
static constexpr uint8_t SG_CODE_ATTEMPTS = 3;

/// The pairing window's approval channels, reported to the login page through login/poll so it can
/// show matching instructions. BUTTON also covers "voice locked out" and "mic muted".
enum class LoginMode : uint8_t { BUTTON = 0, CODE = 1, SEQ = 2 };

/// What ended a window, for the close trigger YAML hangs the LED/chime handling on. CANCELLED is
/// the browser's own Cancel button; it clears the window immediately rather than through the usual
/// grace period, so a person who cancels and immediately retries is not told the device is busy
/// with their own abandoned request.
enum class LoginResult : uint8_t { APPROVED = 0, EXPIRED = 1, DENIED = 2, CANCELLED = 3 };

/**
 * The session gate: cookie auth in front of every handler on the shared web server, plus the
 * login endpoints that mint the cookie.
 *
 * Registered from generated code in main.cpp, which runs before any component's setup() - so it
 * lands at position 0 of web_server_base's handler vector, ahead of satellite1_radar (which
 * registers at setup priority 800) and everything after. That position is what makes it a gate:
 * AsyncWebServer walks the vector in order and first canHandle wins, and init() replays the same
 * vector after a listener restart, so the ordering survives the Web UI switch's deinit/init cycle.
 *
 * canHandle answers true only for requests the gate itself must answer: its own login endpoints,
 * and any non-exempt request that carries neither a valid session cookie nor valid digest
 * credentials. Authorized traffic and the public surface fall through untouched, so the gate costs
 * an in-RAM string compare per request and nothing else.
 *
 * The session token is one device-wide value: HMAC-SHA256 over an NVS salt, keyed by the password
 * and a generation counter. Changing the password invalidates every session; bumping the counter
 * (the app's "Sign out everywhere" action) does the same without touching the password, which is
 * the only revocation path for a leaked sign-in link that does not also break the digest/curl
 * users. The cookie's Max-Age only expires the browser's copy - the value itself stays valid until
 * one of those two changes, which is why the regenerate action exists.
 *
 * Wire security, stated honestly: this is plain HTTP. The challenge-response login keeps the
 * password itself off the wire (the property digest auth had), but the cookie and the sign-in key
 * are bearer tokens a LAN sniffer could replay. That is unfixable without TLS, which ESPHome's
 * web server does not offer and which self-signed certificates would make worse, not better.
 */
class SessionGate : public AsyncWebHandler {
 public:
  /// Loads the salt and generation from NVS (creating them on first boot) and computes the token.
  /// Called from the component's setup(), which runs after the on_boot block that installs the
  /// credentials and before the listener starts accepting - so no request ever races the token.
  void setup();

  /// The digest/basic fallback credentials and the token's password half. `username` is a string
  /// literal from YAML; the password is copied. Called from the on_boot lambda in
  /// common/web_ui.yaml at priority 600, before setup() above. `fixed` says the password is the
  /// YAML substitution re-applied on every boot (a fleet-shared password) - the change endpoint
  /// refuses then, because a change would silently revert on restart.
  void set_credentials(const char *username, const std::string &password, bool fixed = false) {
    this->username_ = username;
    this->password_ = password;
    this->password_fixed_ = fixed;
  }

  /// Whether the password is pinned by YAML - read by the state payload (`pw_fixed`) so the app
  /// hides the change-password form instead of offering one that cannot work.
  bool password_fixed() const { return this->password_fixed_; }

  /// Whether the microphones can hear a voice approval right now. Read at window-open time to pick
  /// the mode; a muted mic opens the window button-only, and the login page says why.
  void set_mic_available_fn(std::function<bool()> fn) { this->mic_available_fn_ = std::move(fn); }

  /// Whether this build compiled the wake-word models the offline challenge is made of.
  void set_seq_available(bool available) { this->seq_available_ = available; }

  /// The actions-checkbox verdict tts_routing's probe stores on the handler (0 unknown, 1 allowed,
  /// 2 blocked, 3 Home Assistant too old to say). Read at window-open time: an API connection alone
  /// used to be enough to pick the spoken code, but with the checkbox off Home Assistant drops
  /// assist_satellite.start_conversation without an error - the user waits for a code that never
  /// plays, the window expires, and their button press falls through to the assistant. Verdicts 0
  /// and 2 therefore route to the offline challenge instead.
  void set_ha_actions_fn(std::function<int()> fn) { this->ha_actions_fn_ = std::move(fn); }

  /// The session token as the sign-in link carries it. Read by the /api/sat1/state handler so the
  /// Diagnostics Launch section can render the link and its QR code - behind the gate, so only an
  /// already-authenticated browser ever sees it.
  const char *session_key() const { return this->token_hex_; }

  /// Bumps the generation counter: every cookie and sign-in link everywhere dies instantly. The
  /// caller stays signed in because the endpoint answers with a fresh cookie.
  void regenerate();

  /* ---- The device-presence approvals, called from YAML on the main loop. ---- */

  /// The action button, pressed while a window is pending. Returns true when a window was approved
  /// - which is the caller's signal to consume the press instead of running its normal action.
  bool approve_pending_login();

  /// The announcement watchdog's fallback, called from YAML when a spoken-code window's
  /// announcement never started playing (the actions checkbox off mid-probe, a missing
  /// assist_satellite entity, a Home Assistant hiccup - all of which drop the call without an
  /// error). Switches a pending CODE window to the offline challenge (or button-only when the
  /// models are unavailable), resets the deadline so the user gets a full window after the local
  /// announcement, and re-queues the open event so YAML announces the new challenge. Returns true
  /// when a downgrade happened.
  bool downgrade_login_window();

  /// A finished STT transcript, offered to a pending online-mode window. Returns true when the
  /// text was consumed - match or mismatch - so the caller suppresses the transcript ring and
  /// stops the pipeline; the code must never reach the conversation agent as a nonsense query.
  bool consume_login_transcript(const std::string &text);

  /// A wake-word firing, offered to a pending offline-mode window as one challenge symbol.
  /// Returns true when consumed, so the caller records a symbol instead of starting the assistant.
  bool consume_login_wake(const std::string &phrase);

  /// Whether the last consumed symbol kept the answer on track (the whole prefix heard so far
  /// matches the challenge). Read-and-clear, from the same on_wake_word_detected automation that
  /// called consume_login_wake - YAML blips the ring green on it, the in-room twin of the page's
  /// un-bolding chips. Progress it reveals to a bystander is progress the blip is *for*: the
  /// symbols just spoken aloud were right, which the speaker knows.
  bool take_login_symbol_ok() { return this->ev_symbol_ok_.exchange(false); }

  /// Whether a window is pending at all, and pending in code mode specifically - the second is
  /// what the YAML re-listen loop tests before reopening the mic for another attempt.
  bool login_pending();
  bool login_voice_pending();

  /* ---- Fired from the component's loop(), where triggers are allowed to start actions. ---- */

  /// Window lifecycle: expiry, the offline quiet-period judgement, and state cleanup.
  void tick(uint32_t now);

  /// Hands out the one queued open event: mode as a string ("button"/"code"/"seq") and the secret
  /// ("4271", or the sequence as symbol digits "012"). True at most once per window.
  bool take_open_event(std::string &mode, std::string &secret);

  /// Hands out the one queued close event ("approved"/"expired"/"denied").
  bool take_close_event(std::string &result);

  /// Hands out the one queued password-change event (the accepted new password), for the
  /// on_password_change automation that assigns the NVS global and republishes the Web UI
  /// Password sensor. HTTP handlers run on the httpd task and must not fire automations, so the
  /// endpoint queues and loop() drains - the same shape as the open/close events above.
  bool take_password_change(std::string &new_password);

  /* ---- AsyncWebHandler ---- */
  // NOLINTNEXTLINE(readability-identifier-naming)
  bool canHandle(AsyncWebServerRequest *request) const override;
  // NOLINTNEXTLINE(readability-identifier-naming)
  void handleRequest(AsyncWebServerRequest *request) override;

 protected:
  /// What the gate does with one request. PASS falls through to the real handlers; DENY answers
  /// 401; the rest are the gate's own endpoints. Match and dispatch share this so the two can
  /// never disagree - the same shape WebUIHandler::Route takes.
  enum class GateRoute : uint8_t {
    PASS = 0,
    DENY,
    NONCE,       // GET  /api/sat1/login/nonce
    LOGIN,       // POST /api/sat1/login
    LOGOUT,      // POST /api/sat1/logout
    LOGOUT_ALL,  // POST /api/sat1/logout_all  (requires a valid session)
    START,       // POST /api/sat1/login/start
    POLL,        // GET  /api/sat1/login/poll
    CANCEL,      // POST /api/sat1/login/cancel (requires the window's own pair cookie)
    WHOAMI,      // GET  /api/sat1/whoami
    PASSWORD,    // POST /api/sat1/password (requires a valid session AND the current password)
    PREFLIGHT,   // OPTIONS /api/sat1/...
  };

  GateRoute route_(AsyncWebServerRequest *request) const;

  /// The public surface: the SPA and the assets a browser needs before it has a session. The
  /// bundle is public on GitHub, so serving it unauthenticated gives nothing away; everything with
  /// state in it stays behind the gate - including /api/sat1/state, whose sensor readings and
  /// transcripts are exactly what a DNS-rebinding page could otherwise read (a rebound hostname
  /// never carries the host-bound cookie, so the unauthenticated surface is its whole reach).
  static bool exempt_(const char *url, size_t len);

  /// A valid session cookie, a valid ?key= bearer (the same token the cookie carries, for a peer
  /// Satellite1's page remote-controlling this device cross-origin, where cookies cannot follow
  /// and EventSource can set no headers), or valid digest credentials - the last is what keeps
  /// curl and the radar tuner scripts working exactly as they did when web_server owned the auth.
  bool authorized_(AsyncWebServerRequest *request) const;
  bool cookie_valid_(AsyncWebServerRequest *request) const;

  /// Whether the Host header is one this device answers to: any IP literal, or the device's own
  /// mDNS name (bare or .local). Checked on the pairing endpoints only - the DNS-rebinding
  /// defense, see the comment at the definition.
  bool pairing_host_ok_(AsyncWebServerRequest *request) const;

  /// Strict dotted-quad parse; the guard origin_is_lan_ and pairing_host_ok_ share.
  static bool parse_ipv4_(const std::string &host, unsigned o[4]);

  void handle_nonce_(AsyncWebServerRequest *request);
  void handle_login_(AsyncWebServerRequest *request);
  void handle_logout_(AsyncWebServerRequest *request);
  void handle_logout_all_(AsyncWebServerRequest *request);
  void handle_start_(AsyncWebServerRequest *request);
  void handle_poll_(AsyncWebServerRequest *request);
  void handle_cancel_(AsyncWebServerRequest *request);
  void handle_whoami_(AsyncWebServerRequest *request);
  void handle_password_(AsyncWebServerRequest *request);
  void handle_preflight_(AsyncWebServerRequest *request);
  void deny_(AsyncWebServerRequest *request);

  /// Answers through the raw httpd API rather than ESPHome's response path, for two reasons that
  /// are both load-bearing. init_response_ stamps every response with the Access-Control-Allow-
  /// Origin: * default header web_server_base installs, and a wildcard is exactly what a browser
  /// rejects on a credentialed cross-origin call - which the dual-origin cookie priming is. And
  /// httpd_resp_set_hdr stores pointers rather than copying, so the Set-Cookie value needs storage
  /// that outlives the call; the members below provide it, which is safe because esp_http_server
  /// runs one request at a time on one task.
  ///
  /// CORS: when the request carries an Origin header naming one of this device's own two origins
  /// (the .local name and the IP, on the server's port), it is echoed back with credentials
  /// allowed; any other origin gets no CORS headers at all and the browser withholds the response.
  void send_json_(AsyncWebServerRequest *request, const char *status, const char *body, bool set_session,
                  bool clear_session, const char *pair_nonce, bool lan_cors = false);

  /// True when `origin` is one of this device's own two web origins.
  bool origin_is_self_(const std::string &origin) const;

  /// True when `origin` is plain HTTP to a private-range host (RFC1918 or .local) - the fleet's
  /// whole address space. Granted CORS only on the nonce and login endpoints, and only so a peer
  /// Satellite1's page can run the silent cross-sign-in the device switcher does: fetch a nonce,
  /// answer the challenge with the peer password Home Assistant already publishes, and receive the
  /// cookie. Those endpoints are built to face unauthenticated callers - the challenge and the rate
  /// limits are the guard, not the origin check - so widening who may *read* their responses adds
  /// no new attack that a direct LAN client did not already have.
  static bool origin_is_lan_(const std::string &origin);

  /// Recomputes token_hex_ from the salt, the password and the generation counter.
  void compute_token_();

  /// Opens a window (caller holds win_lock_): picks the mode, generates the secret, queues the
  /// open event. `nonce_out` receives the pairing nonce for the Set-Cookie.
  void open_window_(uint32_t now);

  /// Closes a pending window with `result` and queues the close event. Caller holds win_lock_.
  void close_window_(LoginResult result, uint32_t now);

  /// One voice failure: counts toward the lockout and applies its backoff.
  void voice_failed_(uint32_t now);

  std::string username_;
  std::string password_;
  bool password_fixed_{false};
  std::function<bool()> mic_available_fn_;
  std::function<int()> ha_actions_fn_;
  bool seq_available_{false};

  /// The salt and generation, together in one blob so they can never restore out of step.
  struct SessionSecret {
    uint8_t salt[16];
    uint32_t generation;
  } __attribute__((packed));
  SessionSecret secret_{};
  ESPPreferenceObject pref_;

  /// The one token every cookie and sign-in link carries, as lowercase hex. Computed at setup and
  /// on password/generation change, compared constant-time per request.
  char token_hex_[65]{};

  /// The outstanding login nonce: single slot, single use, short TTL. A slot younger than a few
  /// seconds is handed out again rather than overwritten, so a hostile client polling the nonce
  /// endpoint cannot burn a legitimate login's nonce out from under it; any attempt against the
  /// slot still consumes it.
  char nonce_hex_[33]{};
  uint32_t nonce_at_{0};

  /// Password-path rate limiting. Touched only on the httpd task. The bool carries "was a lockout
  /// set", so the timestamp math stays wrap-safe signed differences with no zero sentinel.
  uint8_t pw_fails_{0};
  bool pw_locked_{false};
  uint32_t pw_lock_until_{0};

  /// Voice-path lockout, shared by both voice modes. Touched under win_lock_.
  uint8_t voice_fails_{0};
  bool voice_locked_{false};
  uint32_t voice_lock_until_{0};

  /// The pairing window. One at a time by design: the LED ring and the announcement are singular,
  /// and a second concurrent request is exactly the confused-deputy case the refusal copy warns
  /// about. Guarded by win_lock_ because the endpoints run on the httpd task while the approvals
  /// and tick() run on the main loop.
  struct Window {
    uint8_t state{0};  // 0 none, 1 pending, 2 approved, 3 expired, 4 denied
    LoginMode mode{LoginMode::BUTTON};
    /// Button-only because the hardware mute slider is on - reported through the poll so the login
    /// page can say "slide the mute off for voice sign-in" instead of leaving the silence a mystery.
    bool hw_muted{false};
    /// The spoken code was unavailable (or failed mid-window) because Home Assistant is connected
    /// but not allowed to perform actions - reported through the poll so the login page can explain
    /// why it is hearing the wake-word challenge instead of a code.
    bool hab{false};
    char nonce[33]{};
    char code[5]{};
    uint8_t seq[SG_SEQ_LEN]{};
    uint8_t heard[SG_SEQ_LEN + 6]{};
    uint8_t heard_len{0};
    uint8_t attempts{0};
    uint32_t deadline{0};
    uint32_t state_at{0};
    uint32_t last_symbol{0};
  };
  Window win_;
  uint32_t last_start_{0};
  Mutex win_lock_;

  /// The open/close events, one slot each - a window opens once and closes once, and loop() drains
  /// between the two. Payloads guarded by win_lock_; the flags say whether a payload is waiting.
  std::atomic<bool> ev_open_{false};
  std::atomic<bool> ev_close_{false};
  /// One correct-so-far challenge symbol, for the green blip - see take_login_symbol_ok.
  std::atomic<bool> ev_symbol_ok_{false};
  char ev_mode_[8]{};
  char ev_secret_[SG_SEQ_LEN + 2]{};
  char ev_result_[12]{};
  /// The password-change event: the accepted new password, waiting for loop() to hand it to the
  /// on_password_change automation. Same lock discipline as the payloads above.
  std::atomic<bool> ev_password_{false};
  char ev_new_password_[SG_PW_MAX + 1]{};

  /// Stable storage for the headers send_json_ sets - see its comment. One request at a time on
  /// the httpd task is what makes members sufficient.
  /// Sized for the ingress-proxied worst case: name + 64-hex token + a 96-char X-Ingress-Path
  /// cookie Path + Max-Age + HttpOnly + SameSite + Secure (see send_json_'s cookie block).
  char session_cookie_buf_[256]{};
  char pair_cookie_buf_[224]{};
  char origin_buf_[80]{};
};

}  // namespace satellite1_web_ui
}  // namespace esphome
