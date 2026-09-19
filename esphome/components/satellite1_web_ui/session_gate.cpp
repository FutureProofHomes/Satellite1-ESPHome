#include "session_gate.h"

#include <cctype>
#include <cstdio>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "esphome/components/network/ip_address.h"
#include "esphome/components/network/util.h"

#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif

#include <esp_http_server.h>
#include <esp_random.h>
#include <mbedtls/md.h>
#include <mbedtls/sha256.h>

namespace esphome {
namespace satellite1_web_ui {

static const char *const TAG_SG = "web_ui.auth";

/// The NVS slot for the salt and generation. A fixed constant rather than a hash of anything
/// runtime, so the same device keeps the same sessions across renames and reflashes.
static constexpr uint32_t SG_PREF_HASH = 0x53415431;  // "SAT1"

static const char *const SESSION_COOKIE = "sat1_session";
static const char *const PAIR_COOKIE = "sat1_pair";

/* ---- Small crypto and string helpers ----------------------------------------------------- */

static void to_hex_(const uint8_t *in, size_t n, char *out) {
  static const char HEX[] = "0123456789abcdef";
  for (size_t i = 0; i < n; i++) {
    out[i * 2] = HEX[in[i] >> 4];
    out[i * 2 + 1] = HEX[in[i] & 0x0F];
  }
  out[n * 2] = '\0';
}

static void hmac_sha256_(const uint8_t *key, size_t key_len, const uint8_t *msg, size_t msg_len, uint8_t out[32]) {
  const mbedtls_md_info_t *md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  mbedtls_md_hmac(md, key, key_len, msg, msg_len, out);
}

/// Constant-time equality over NUL-terminated strings, the same fold-the-length-in construction
/// ESPHome's own Basic-auth check uses: no early exit, so a mismatch costs the same time wherever
/// it is.
static bool ct_equal_(const char *expected, const char *provided) {
  const size_t elen = strlen(expected);
  const size_t plen = strlen(provided);
  volatile size_t acc = elen ^ plen;
  for (size_t i = 0; i < elen; i++) {
    const char p = i < plen ? provided[i] : 0;
    acc |= static_cast<uint8_t>(expected[i] ^ p);
  }
  return acc == 0;
}

/// Pulls one cookie's value out of a Cookie header. Boundary-aware: "sat1_session" must not match
/// inside some other cookie's name or value.
static bool cookie_value_(const std::string &header, const char *name, std::string &out) {
  const size_t name_len = strlen(name);
  size_t at = 0;
  while (at < header.size()) {
    // Skip separators and whitespace to the start of a pair.
    while (at < header.size() && (header[at] == ';' || header[at] == ' '))
      at++;
    const size_t eq = header.find('=', at);
    if (eq == std::string::npos)
      return false;
    const size_t end = std::min(header.find(';', eq), header.size());
    if (eq - at == name_len && header.compare(at, name_len, name) == 0) {
      out = header.substr(eq + 1, end - eq - 1);
      return true;
    }
    at = end;
  }
  return false;
}

/// Digits out of a transcript: ASCII digits pass through, and the spoken forms map to them, so
/// "four two seven one", "4271" and "42 71" all normalize to "4271". Everything else is dropped.
/// The homophones are deliberate: STT engines write "for" and "to" often enough that rejecting
/// them would fail real people saying the right code.
static void normalize_digits_(const std::string &text, std::string &out) {
  static const struct {
    const char *word;
    char digit;
  } WORDS[] = {
      {"zero", '0'},  {"oh", '0'},    {"one", '1'},  {"won", '1'},  {"two", '2'},  {"to", '2'},
      {"too", '2'},   {"three", '3'}, {"four", '4'}, {"for", '4'},  {"five", '5'}, {"six", '6'},
      {"seven", '7'}, {"eight", '8'}, {"ate", '8'},  {"nine", '9'},
  };
  out.clear();
  std::string word;
  const auto flush = [&]() {
    if (word.empty())
      return;
    for (const auto &entry : WORDS) {
      if (word == entry.word) {
        out.push_back(entry.digit);
        break;
      }
    }
    word.clear();
  };
  for (const char raw : text) {
    const char c = static_cast<char>(std::tolower(static_cast<unsigned char>(raw)));
    if (c >= '0' && c <= '9') {
      flush();
      out.push_back(c);
    } else if (c >= 'a' && c <= 'z') {
      word.push_back(c);
    } else {
      flush();
    }
  }
  flush();
}

/* ---- Setup and the token ------------------------------------------------------------------ */

void SessionGate::setup() {
  this->pref_ = global_preferences->make_preference<SessionSecret>(SG_PREF_HASH);

  bool have = this->pref_.load(&this->secret_);
  if (have) {
    // A blob of zeros is not a salt; treat it as absent rather than minting every device on a
    // wiped partition the same token.
    bool all_zero = true;
    for (const uint8_t b : this->secret_.salt)
      all_zero &= b == 0;
    have = !all_zero;
  }
  if (!have) {
    esp_fill_random(this->secret_.salt, sizeof(this->secret_.salt));
    this->secret_.generation = 1;
    this->pref_.save(&this->secret_);
    ESP_LOGI(TAG_SG, "Generated a new session salt");
  }

  this->compute_token_();
}

void SessionGate::compute_token_() {
  // The generation folds into the message rather than the key, so bumping it is exactly as strong
  // as changing the password: both change every bit of the HMAC input. A std::string rather than
  // a fixed buffer because the password's length is the user's choice - a silent truncation would
  // mean this token and the digest fallback disagree about what the password is.
  std::string msg = this->password_;
  msg += '\n';
  msg += std::to_string(this->secret_.generation);
  uint8_t mac[32];
  hmac_sha256_(this->secret_.salt, sizeof(this->secret_.salt), reinterpret_cast<const uint8_t *>(msg.data()),
               msg.size(), mac);
  to_hex_(mac, sizeof(mac), this->token_hex_);
}

void SessionGate::regenerate() {
  this->secret_.generation++;
  this->pref_.save(&this->secret_);
  this->compute_token_();
  ESP_LOGI(TAG_SG, "Session generation bumped to %u; every cookie and sign-in link is now invalid",
           static_cast<unsigned>(this->secret_.generation));
}

/* ---- Routing ------------------------------------------------------------------------------ */

bool SessionGate::exempt_(const char *url, size_t len) {
  // The SPA itself and its pre-login assets. Everything else - /events, the entity REST API, the
  // radar tuner, every /api/sat1/* read - stays gated.
  static const char *const EXEMPT[] = {
      "/",
      "/ui",
      "/ui/",
      "/ui/no-sensor.webp",
      "/manifest.webmanifest",
      "/ui/icon-192.png",
      "/ui/icon-512.png",
      "/apple-touch-icon.png",
  };
  for (const char *path : EXEMPT) {
    if (strcmp(url, path) == 0)
      return true;
  }
  // The device-served sounds, fetched by third-party speakers playing a mirrored timer ring or
  // wake chime (web_ui_handler.cpp's SOUND route). A Sonos or Cast has no session and never will,
  // and the content is a chime, not a secret. Reached only under HTTP_GET - route_ tests exempt_
  // in its GET branch alone - so nothing here widens what a POST can do.
  if (strncmp(url, "/api/sat1/sounds/", 17) == 0)
    return true;
  (void) len;
  return false;
}

SessionGate::GateRoute SessionGate::route_(AsyncWebServerRequest *request) const {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  const StringRef url = request->url_to(url_buf);
  const auto method = request->method();

  if (method == HTTP_OPTIONS) {
    // Only our own API namespace; anything else keeps whatever behaviour it has today.
    if (strncmp(url_buf, "/api/sat1/", 10) == 0)
      return GateRoute::PREFLIGHT;
    return GateRoute::PASS;
  }

  if (method == HTTP_POST) {
    if (url == "/api/sat1/login")
      return GateRoute::LOGIN;
    if (url == "/api/sat1/login/start")
      return GateRoute::START;
    if (url == "/api/sat1/login/cancel")
      return GateRoute::CANCEL;
    if (url == "/api/sat1/logout")
      return GateRoute::LOGOUT;
    if (url == "/api/sat1/logout_all")
      return GateRoute::LOGOUT_ALL;
  } else if (method == HTTP_GET) {
    if (url == "/api/sat1/login/nonce")
      return GateRoute::NONCE;
    if (url == "/api/sat1/login/poll")
      return GateRoute::POLL;
    if (url == "/api/sat1/whoami")
      return GateRoute::WHOAMI;
    if (exempt_(url_buf, url.size()))
      return GateRoute::PASS;
  }

  return this->authorized_(request) ? GateRoute::PASS : GateRoute::DENY;
}

bool SessionGate::cookie_valid_(AsyncWebServerRequest *request) const {
  const auto header = request->get_header("Cookie");
  if (!header.has_value())
    return false;
  std::string value;
  if (!cookie_value_(header.value(), SESSION_COOKIE, value))
    return false;
  return ct_equal_(this->token_hex_, value.c_str());
}

bool SessionGate::authorized_(AsyncWebServerRequest *request) const {
  if (this->cookie_valid_(request))
    return true;
  // The bearer fallback, for a caller that cannot carry this device's cookie: a peer Satellite1's
  // page remote-controlling this device cross-origin (single-origin device switching). Safari
  // blocks third-party cookies outright, and EventSource can set no headers, so the query string
  // is the one channel that fetch, EventSource and the entity REST posts all share. The value is
  // the same token the cookie carries, obtained the same way (the login challenge), checked with
  // the same constant-time compare the login endpoint's bearer path uses - so this widens where
  // the secret may ride, not who can mint one.
  if (auto *key = request->getParam("key"); key != nullptr && ct_equal_(this->token_hex_, key->value().c_str()))
    return true;
#ifdef USE_WEBSERVER_AUTH
  // The digest fallback: exactly the check AuthMiddlewareHandler ran when web_server owned the
  // auth block, against the same credentials, so curl and the radar tuner scripts keep working
  // unchanged. Only consulted when there is an Authorization header to check, which browsers on
  // the cookie path never send.
  if (!this->username_.empty() && request->get_header("Authorization").has_value() &&
      request->authenticate(this->username_.c_str(), this->password_.c_str())) {
    return true;
  }
#endif
  return false;
}

bool SessionGate::canHandle(AsyncWebServerRequest *request) const {
  // PASS is the only route the gate does not answer; everything else - its own endpoints and the
  // deny - it claims here and dispatches in handleRequest.
  return this->route_(request) != GateRoute::PASS;
}

void SessionGate::handleRequest(AsyncWebServerRequest *request) {
  switch (this->route_(request)) {
    case GateRoute::NONCE:
      this->handle_nonce_(request);
      break;
    case GateRoute::LOGIN:
      this->handle_login_(request);
      break;
    case GateRoute::LOGOUT:
      this->handle_logout_(request);
      break;
    case GateRoute::LOGOUT_ALL:
      this->handle_logout_all_(request);
      break;
    case GateRoute::START:
      this->handle_start_(request);
      break;
    case GateRoute::POLL:
      this->handle_poll_(request);
      break;
    case GateRoute::CANCEL:
      this->handle_cancel_(request);
      break;
    case GateRoute::WHOAMI:
      this->handle_whoami_(request);
      break;
    case GateRoute::PREFLIGHT:
      this->handle_preflight_(request);
      break;
    case GateRoute::DENY:
      this->deny_(request);
      break;
    case GateRoute::PASS:
      break;
  }
}

/* ---- Responses ----------------------------------------------------------------------------- */

bool SessionGate::origin_is_self_(const std::string &origin) const {
  const uint16_t port = web_server_base::global_web_server_base->get_port();
  char candidate[80];

  // The mDNS origin. App.get_name() is the hostname mDNS advertises.
  if (port == 80) {
    snprintf(candidate, sizeof(candidate), "http://%s.local", App.get_name().c_str());
  } else {
    snprintf(candidate, sizeof(candidate), "http://%s.local:%u", App.get_name().c_str(), port);
  }
  if (origin == candidate)
    return true;

  // The IP origins, one per interface.
  for (auto &addr : network::get_ip_addresses()) {
    if (!addr.is_set())
      continue;
    char ip_buf[network::IP_ADDRESS_BUFFER_SIZE];
    addr.str_to(ip_buf);
    if (port == 80) {
      snprintf(candidate, sizeof(candidate), "http://%s", ip_buf);
    } else {
      snprintf(candidate, sizeof(candidate), "http://%s:%u", ip_buf, port);
    }
    if (origin == candidate)
      return true;
  }
  return false;
}

/// True when `host` is a dotted-quad IPv4 literal, octets parsed into `o`. The literal test is
/// what keeps a *registered public domain* like "10.evil.example" or "192.168.attacker.tld" from
/// passing a prefix check: such names contain letters, and a pure digits-and-dots name cannot be
/// registered in public DNS (browsers parse it as an address before ever resolving it).
bool SessionGate::parse_ipv4_(const std::string &host, unsigned o[4]) {
  for (const char c : host) {
    if ((c < '0' || c > '9') && c != '.')
      return false;
  }
  char tail;
  if (sscanf(host.c_str(), "%u.%u.%u.%u%c", &o[0], &o[1], &o[2], &o[3], &tail) != 4)
    return false;
  return o[0] <= 255 && o[1] <= 255 && o[2] <= 255 && o[3] <= 255;
}

/// The private-origin test the header explains: http:// plus an RFC1918 literal or a .local name.
bool SessionGate::origin_is_lan_(const std::string &origin) {
  if (origin.rfind("http://", 0) != 0)
    return false;
  std::string host = origin.substr(7);
  const size_t colon = host.find(':');
  if (colon != std::string::npos)
    host = host.substr(0, colon);
  if (host.size() > 6 && host.compare(host.size() - 6, 6, ".local") == 0)
    return true;
  unsigned o[4];
  if (!parse_ipv4_(host, o))
    return false;
  // RFC1918: 10/8, 192.168/16, 172.16/12.
  return o[0] == 10 || (o[0] == 192 && o[1] == 168) || (o[0] == 172 && o[1] >= 16 && o[1] <= 31);
}

void SessionGate::send_json_(AsyncWebServerRequest *request, const char *status, const char *body, bool set_session,
                             bool clear_session, const char *pair_nonce, bool lan_cors) {
  httpd_resp_set_status(*request, status);
  httpd_resp_set_type(*request, "application/json");
  httpd_resp_set_hdr(*request, "Cache-Control", "no-store");

  const auto origin = request->get_header("Origin");
  if (origin.has_value() && (this->origin_is_self_(origin.value()) || (lan_cors && origin_is_lan_(origin.value())))) {
    snprintf(this->origin_buf_, sizeof(this->origin_buf_), "%s", origin.value().c_str());
    httpd_resp_set_hdr(*request, "Access-Control-Allow-Origin", this->origin_buf_);
    httpd_resp_set_hdr(*request, "Access-Control-Allow-Credentials", "true");
    httpd_resp_set_hdr(*request, "Vary", "Origin");
  }

  if (set_session) {
    snprintf(this->session_cookie_buf_, sizeof(this->session_cookie_buf_),
             "%s=%s; Path=/; Max-Age=7776000; HttpOnly; SameSite=Lax", SESSION_COOKIE, this->token_hex_);
    httpd_resp_set_hdr(*request, "Set-Cookie", this->session_cookie_buf_);
  } else if (clear_session) {
    snprintf(this->session_cookie_buf_, sizeof(this->session_cookie_buf_),
             "%s=; Path=/; Max-Age=0; HttpOnly; SameSite=Lax", SESSION_COOKIE);
    httpd_resp_set_hdr(*request, "Set-Cookie", this->session_cookie_buf_);
  }
  if (pair_nonce != nullptr) {
    snprintf(this->pair_cookie_buf_, sizeof(this->pair_cookie_buf_),
             "%s=%s; Path=/; Max-Age=180; HttpOnly; SameSite=Lax", PAIR_COOKIE, pair_nonce);
    httpd_resp_set_hdr(*request, "Set-Cookie", this->pair_cookie_buf_);
  }

  httpd_resp_send(*request, body, HTTPD_RESP_USE_STRLEN);
}

void SessionGate::deny_(AsyncWebServerRequest *request) {
  // The digest challenge goes only to non-browser clients. Digest cannot be spoken without it -
  // curl needs the realm and a nonce from the server before it can compute a response, so a bare
  // 401 would lock every script out. But handed to a browser, the same header pops the native
  // credentials dialog over fetch() calls, which is the exact prompt this design removes.
  //
  // The tell is the User-Agent: every browser's starts with "Mozilla/", and no CLI tool's does
  // (curl/8, Wget/1, python-requests/2). Sec-Fetch-Mode would be the principled signal, but the
  // Fetch Metadata spec sends it only to potentially-trustworthy origins - HTTPS or localhost -
  // and this device's whole context is plain HTTP on a LAN, where no browser sends it at all
  // (learned the hard way: the boot probe's same-origin fetch carries neither Sec-Fetch-Mode nor
  // Origin here, so header-based detection called Safari a script and iOS popped the native
  // dialog over a blank page). Both header checks stay as accelerators for whatever grows them
  // back someday, but the UA is what decides. The challenge matches requestAuthentication()
  // upstream, and verification is stateless (check_digest_auth uses the nonce the client echoes
  // back), so nothing here needs to remember what it issued.
  const auto ua = request->get_header("User-Agent");
  const bool browser = (ua.has_value() && ua.value().rfind("Mozilla/", 0) == 0) ||
                       request->get_header("Sec-Fetch-Mode").has_value() || request->get_header("Origin").has_value();
  httpd_resp_set_status(*request, "401 Unauthorized");
  httpd_resp_set_type(*request, "application/json");
  // Stack storage is safe here for the same reason upstream uses it: httpd_resp_set_hdr stores
  // the pointer, and httpd_resp_send below happens before this frame unwinds.
  char header[96];
  if (!browser) {
    uint8_t raw[16];
    char nonce[33];
    esp_fill_random(raw, sizeof(raw));
    to_hex_(raw, sizeof(raw), nonce);
    snprintf(header, sizeof(header), R"(Digest realm="Login Required", qop="auth", nonce="%s")", nonce);
    httpd_resp_set_hdr(*request, "WWW-Authenticate", header);
  }
  // CORS on the refusal, for LAN origins only: a peer Satellite1's page remote-controlling this
  // device needs to *read* the 401 to tell "your key is stale" apart from "the device is gone" -
  // without the header, the browser reports both as the same opaque network error. The body is
  // {"ok":0,"auth":0} either way, so a readable refusal discloses nothing new. origin_buf_ is the
  // member send_json_ uses for the same purpose; safe here for the same reason (one request at a
  // time on one task, and httpd_resp_send below happens before this frame unwinds).
  const auto origin = request->get_header("Origin");
  if (origin.has_value() && (this->origin_is_self_(origin.value()) || origin_is_lan_(origin.value()))) {
    snprintf(this->origin_buf_, sizeof(this->origin_buf_), "%s", origin.value().c_str());
    httpd_resp_set_hdr(*request, "Access-Control-Allow-Origin", this->origin_buf_);
    httpd_resp_set_hdr(*request, "Vary", "Origin");
  }
  const char *body = R"({"ok":0,"auth":0})";
  httpd_resp_send(*request, body, HTTPD_RESP_USE_STRLEN);
}

void SessionGate::handle_whoami_(AsyncWebServerRequest *request) {
  // Name only, and public by design: the smart redirect needs the mDNS hostname before any login
  // exists. The plan's index-embedding variant would expose exactly the same string to exactly the
  // same callers (the bundle is served unauthenticated either way), and cannot be done without
  // abandoning the pre-gzipped PROGMEM bundle - so this endpoint is the same disclosure at a
  // fraction of the cost. Nothing else rides along; state stays gated.
  char body[80];
  snprintf(body, sizeof(body), R"({"name":"%s"})", App.get_name().c_str());
  this->send_json_(request, "200 OK", body, false, false, nullptr);
}

void SessionGate::handle_preflight_(AsyncWebServerRequest *request) {
  // Defensive completeness: the login POSTs are form-encoded "simple requests" that never
  // preflight, but a future fetch with a JSON body would, and answering here costs nothing.
  httpd_resp_set_status(*request, "204 No Content");
  const auto origin = request->get_header("Origin");
  if (origin.has_value() && (this->origin_is_self_(origin.value()) || origin_is_lan_(origin.value()))) {
    snprintf(this->origin_buf_, sizeof(this->origin_buf_), "%s", origin.value().c_str());
    httpd_resp_set_hdr(*request, "Access-Control-Allow-Origin", this->origin_buf_);
    httpd_resp_set_hdr(*request, "Access-Control-Allow-Credentials", "true");
    httpd_resp_set_hdr(*request, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(*request, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_set_hdr(*request, "Access-Control-Max-Age", "600");
    httpd_resp_set_hdr(*request, "Vary", "Origin");
  }
  httpd_resp_send(*request, nullptr, 0);
}

/* ---- Password and key login ----------------------------------------------------------------- */

void SessionGate::handle_nonce_(AsyncWebServerRequest *request) {
  // A fresh nonce only when the slot is empty (burned by an attempt) or a few seconds stale.
  // Within that window every caller reads the same value, which changes nothing for security -
  // answering it still takes the password, and any attempt burns it - but means a hostile client
  // polling this endpoint cannot yank a just-issued nonce out from under a legitimate login. The
  // race that remains needs the login's fetch-answer round trip to lose to a 3-second timer,
  // which on a LAN it does not.
  const uint32_t now = millis();
  if (this->nonce_hex_[0] == '\0' || now - this->nonce_at_ > 3000) {
    uint8_t raw[16];
    esp_fill_random(raw, sizeof(raw));
    to_hex_(raw, sizeof(raw), this->nonce_hex_);
    this->nonce_at_ = now;
  }

  char body[64];
  snprintf(body, sizeof(body), R"({"n":"%s"})", this->nonce_hex_);
  // lan_cors: a peer Satellite1's page fetches this during the switcher's cross-sign-in.
  this->send_json_(request, "200 OK", body, false, false, nullptr, true);
}

void SessionGate::handle_login_(AsyncWebServerRequest *request) {
  const uint32_t now = millis();
  // The flag says whether a lockout was ever set; the signed difference of unsigned timestamps
  // says whether it still holds. Together they read correctly on either side of the ~49.7-day
  // millis() wrap, where a bare `now < until` comparison would not. Same idiom for every deadline
  // in this file.
  if (this->pw_locked_) {
    const int32_t left = static_cast<int32_t>(this->pw_lock_until_ - now);
    if (left > 0) {
      char body[64];
      snprintf(body, sizeof(body), R"({"ok":0,"locked":1,"retry":%u})", static_cast<unsigned>(left) / 1000 + 1);
      this->send_json_(request, "429 Too Many Requests", body, false, false, nullptr, true);
      return;
    }
    this->pw_locked_ = false;
  }

  bool ok = false;

  if (auto *key = request->getParam("key"); key != nullptr) {
    // The bearer path: the sign-in link's token, or a priming call carrying what a poll or an
    // earlier login handed back.
    ok = ct_equal_(this->token_hex_, key->value().c_str());
  } else {
    auto *n = request->getParam("n");
    auto *r = request->getParam("r");
    if (n == nullptr || r == nullptr) {
      this->send_json_(request, "400 Bad Request", R"({"ok":0})", false, false, nullptr, true);
      return;
    }
    // Single use and short-lived, burned on any attempt against it - a replayed answer meets a
    // nonce that no longer exists.
    const bool nonce_ok = this->nonce_hex_[0] != '\0' && ct_equal_(this->nonce_hex_, n->value().c_str()) &&
                          now - this->nonce_at_ <= 60000;
    this->nonce_hex_[0] = '\0';
    if (nonce_ok) {
      // The browser answers HMAC-SHA256(SHA-256(password), nonce_hex). Keying by the hash rather
      // than the password itself costs nothing here and means the JS never holds the password
      // longer than the submit handler. The message is the caller's copy of the nonce, which the
      // constant-time check above proved identical to the one just burned.
      uint8_t pw_hash[32];
      mbedtls_sha256(reinterpret_cast<const uint8_t *>(this->password_.c_str()), this->password_.size(), pw_hash, 0);
      uint8_t mac[32];
      hmac_sha256_(pw_hash, sizeof(pw_hash), reinterpret_cast<const uint8_t *>(n->value().c_str()), n->value().size(),
                   mac);
      char expected[65];
      to_hex_(mac, sizeof(mac), expected);
      ok = ct_equal_(expected, r->value().c_str());
    }
  }

  if (!ok) {
    if (++this->pw_fails_ >= SG_PW_FAILS) {
      this->pw_fails_ = 0;
      this->pw_locked_ = true;
      this->pw_lock_until_ = now + SG_PW_LOCKOUT_MS;
      ESP_LOGW(TAG_SG, "Too many failed logins; password path locked for %us",
               static_cast<unsigned>(SG_PW_LOCKOUT_MS / 1000));
    }
    this->send_json_(request, "401 Unauthorized", R"({"ok":0})", false, false, nullptr, true);
    return;
  }

  this->pw_fails_ = 0;
  // The key rides the body so the app can prime the device's other origin (.local vs IP) with one
  // CORS call - the same value the Set-Cookie carries, to a caller who by definition now holds it.
  // lan_cors also lets a peer Satellite1's page read it during the switcher's cross-sign-in; the
  // peer then lands here with ?key= so the cookie is set first-party, out of reach of third-party
  // cookie blocking.
  //
  // The name rides along for that same cross-sign-in: it lets the switcher jump straight to this
  // device's .local origin instead of paying the IP-then-redirect double load. It is the identical
  // string whoami serves unauthenticated, so nothing new is disclosed - and putting it here rather
  // than widening whoami's CORS costs the peer's page one fewer round trip.
  char body[160];
  snprintf(body, sizeof(body), R"({"ok":1,"key":"%s","name":"%s"})", this->token_hex_, App.get_name().c_str());
  this->send_json_(request, "200 OK", body, true, false, nullptr, true);
}

void SessionGate::handle_logout_(AsyncWebServerRequest *request) {
  // No auth required: all this does is expire the caller's own cookie, which anyone can do to
  // themselves by clearing site data anyway.
  this->send_json_(request, "200 OK", R"({"ok":1})", false, true, nullptr);
}

void SessionGate::handle_logout_all_(AsyncWebServerRequest *request) {
  // This one changes everyone's sessions, so it demands one.
  if (!this->authorized_(request)) {
    this->deny_(request);
    return;
  }
  this->regenerate();
  // The caller stays signed in on the new generation - "sign out everywhere else" would be the
  // truthful button label, and the fresh cookie plus the fresh key are what the page re-renders
  // its Launch section from.
  //
  // lan_cors: a peer Satellite1's page remote-controlling this device presses this button too, and
  // it must be able to read the fresh key or the regenerate strands its own session mid-use. The
  // caller just proved a session (the authorized_ check above), so echoing CORS to a LAN origin
  // hands the new key only to someone the old key already vouched for.
  char body[96];
  snprintf(body, sizeof(body), R"({"ok":1,"key":"%s"})", this->token_hex_);
  this->send_json_(request, "200 OK", body, true, false, nullptr, true);
}

/* ---- The pairing window --------------------------------------------------------------------- */

void SessionGate::open_window_(uint32_t now) {
  // Mode selection: voice needs a live microphone and no active lockout; the spoken code
  // additionally needs Home Assistant for TTS and STT; the wake-word sequence needs the compiled
  // models. Anything less falls back to the button, and the login page's copy explains which.
  const bool mic = this->mic_available_fn_ ? this->mic_available_fn_() : false;
  if (this->voice_locked_ && static_cast<int32_t>(this->voice_lock_until_ - now) <= 0)
    this->voice_locked_ = false;
  const bool voice_ok = mic && !this->voice_locked_;
#ifdef USE_API
  const bool ha = api::global_api_server != nullptr && api::global_api_server->is_connected();
#else
  const bool ha = false;
#endif

  this->win_ = Window{};
  this->win_.state = 1;
  this->win_.deadline = now + SG_WINDOW_MS;
  this->win_.state_at = now;

  uint8_t raw[16];
  esp_fill_random(raw, sizeof(raw));
  to_hex_(raw, sizeof(raw), this->win_.nonce);

  if (voice_ok && ha) {
    this->win_.mode = LoginMode::CODE;
    uint32_t rnd = esp_random();
    for (int i = 0; i < 4; i++) {
      this->win_.code[i] = static_cast<char>('0' + rnd % 10);
      rnd /= 10;
    }
    this->win_.code[4] = '\0';
  } else if (voice_ok && this->seq_available_) {
    this->win_.mode = LoginMode::SEQ;
    const uint32_t rnd = esp_random();
    for (int i = 0; i < 3; i++)
      this->win_.seq[i] = static_cast<uint8_t>((rnd >> (i * 8)) % 3);
  } else {
    this->win_.mode = LoginMode::BUTTON;
    // mic_available_fn_ reports the hardware mute slider, the one obstacle software cannot clear.
    // The flag rides the poll so the login page can tell the user to slide it off.
    this->win_.hw_muted = !mic;
  }

  // Queue the open event for loop(): mode name plus the secret, which YAML turns into the
  // announcement. The secret exists only as audio in the room and in this struct - the poll never
  // carries it, which is the entire point.
  const char *mode_name = this->win_.mode == LoginMode::CODE  ? "code"
                          : this->win_.mode == LoginMode::SEQ ? "seq"
                                                              : "button";
  snprintf(this->ev_mode_, sizeof(this->ev_mode_), "%s", mode_name);
  if (this->win_.mode == LoginMode::CODE) {
    snprintf(this->ev_secret_, sizeof(this->ev_secret_), "%s", this->win_.code);
  } else if (this->win_.mode == LoginMode::SEQ) {
    snprintf(this->ev_secret_, sizeof(this->ev_secret_), "%c%c%c", '0' + this->win_.seq[0], '0' + this->win_.seq[1],
             '0' + this->win_.seq[2]);
  } else {
    this->ev_secret_[0] = '\0';
  }
  this->ev_open_.store(true);

  ESP_LOGI(TAG_SG, "Pairing window open for %us, mode %s", static_cast<unsigned>(SG_WINDOW_MS / 1000), mode_name);
}

void SessionGate::close_window_(LoginResult result, uint32_t now) {
  // CANCELLED clears the window outright: the owner gave up on purpose, so nothing waits around in
  // a terminal state, and a fresh start right afterwards is not refused as busy.
  this->win_.state = result == LoginResult::APPROVED    ? 2
                     : result == LoginResult::EXPIRED   ? 3
                     : result == LoginResult::CANCELLED ? 0
                                                        : 4;
  this->win_.state_at = now;
  const char *name = result == LoginResult::APPROVED    ? "approved"
                     : result == LoginResult::EXPIRED   ? "expired"
                     : result == LoginResult::CANCELLED ? "cancelled"
                                                        : "denied";
  snprintf(this->ev_result_, sizeof(this->ev_result_), "%s", name);
  this->ev_close_.store(true);
  if (result == LoginResult::APPROVED)
    this->voice_fails_ = 0;
  ESP_LOGI(TAG_SG, "Pairing window %s", name);
}

void SessionGate::voice_failed_(uint32_t now) {
  this->voice_fails_++;
  if (this->voice_fails_ >= SG_VOICE_FAILS) {
    uint32_t lockout = SG_VOICE_LOCKOUT_BASE_MS << (this->voice_fails_ - SG_VOICE_FAILS);
    if (lockout > SG_VOICE_LOCKOUT_MAX_MS)
      lockout = SG_VOICE_LOCKOUT_MAX_MS;
    this->voice_locked_ = true;
    this->voice_lock_until_ = now + lockout;
    ESP_LOGW(TAG_SG, "Voice sign-in locked for %us after %u failed windows", static_cast<unsigned>(lockout / 1000),
             this->voice_fails_);
  }
}

bool SessionGate::pairing_host_ok_(AsyncWebServerRequest *request) const {
  // The DNS-rebinding defense for the window endpoints: a rebound page necessarily carries the
  // attacker's *registered hostname* in Host - rebinding is a DNS trick, and dotted-quad or
  // .local names cannot be registered in public DNS. So any IP-literal Host passes (that is
  // direct LAN access, or a reverse proxy dialing the device by address), and DNS names must be
  // this device's own: its mDNS name, bare or as <name>.local. Only the pairing endpoints check
  // this, because they are the ones that mint a session from nothing but a button press - the
  // password and key logins already demand a secret a rebound page does not have.
  const auto header = request->get_header("Host");
  if (!header.has_value())
    return false;
  std::string host = header.value();
  if (!host.empty() && host[0] == '[')
    return true;  // An IPv6 literal - an address, not a registrable name.
  const size_t colon = host.find(':');
  if (colon != std::string::npos)
    host = host.substr(0, colon);
  unsigned o[4];
  if (parse_ipv4_(host, o))
    return true;
  for (char &c : host)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  const std::string &name = App.get_name();
  if (host == name)
    return true;
  return host.size() == name.size() + 6 && host.compare(0, name.size(), name) == 0 &&
         host.compare(name.size(), 6, ".local") == 0;
}

void SessionGate::handle_start_(AsyncWebServerRequest *request) {
  if (!this->pairing_host_ok_(request)) {
    this->send_json_(request, "403 Forbidden", R"({"ok":0,"host":0})", false, false, nullptr);
    return;
  }
  const uint32_t now = millis();
  LockGuard guard{this->win_lock_};

  if (this->win_.state == 1) {
    // One at a time, whoever asked. The refusal is the page's cue for the confused-deputy copy:
    // "another sign-in is already waiting on this device - if that isn't you, don't press the
    // button."
    this->send_json_(request, "409 Conflict", R"({"ok":0,"pending":1})", false, false, nullptr);
    return;
  }
  if (now - this->last_start_ < SG_START_GAP_MS) {
    this->send_json_(request, "429 Too Many Requests", R"({"ok":0,"retry":3})", false, false, nullptr);
    return;
  }
  this->last_start_ = now;

  this->open_window_(now);

  char body[80];
  snprintf(body, sizeof(body), R"({"ok":1,"mode":"%s","left":%u,"hw":%d})", this->ev_mode_,
           static_cast<unsigned>(SG_WINDOW_MS / 1000), this->win_.hw_muted ? 1 : 0);
  this->send_json_(request, "200 OK", body, false, false, this->win_.nonce);
}

void SessionGate::handle_cancel_(AsyncWebServerRequest *request) {
  if (!this->pairing_host_ok_(request)) {
    this->send_json_(request, "403 Forbidden", R"({"ok":0,"host":0})", false, false, nullptr);
    return;
  }
  // Only the window's owner can cancel it - proven by the pair cookie, the same way the poll
  // proves whose window it is. Cancelling also forgets the start-gap timestamp, so the follow-up
  // "sign in" click the cancel usually precedes is not refused as too soon.
  std::string nonce;
  {
    const auto header = request->get_header("Cookie");
    if (header.has_value())
      cookie_value_(header.value(), PAIR_COOKIE, nonce);
  }

  LockGuard guard{this->win_lock_};
  if (this->win_.state == 1 && !nonce.empty() && ct_equal_(this->win_.nonce, nonce.c_str())) {
    this->close_window_(LoginResult::CANCELLED, millis());
    this->last_start_ = 0;
  }
  this->send_json_(request, "200 OK", R"({"ok":1})", false, false, nullptr);
}

void SessionGate::handle_poll_(AsyncWebServerRequest *request) {
  if (!this->pairing_host_ok_(request)) {
    this->send_json_(request, "403 Forbidden", R"({"ok":0,"host":0})", false, false, nullptr);
    return;
  }
  std::string nonce;
  {
    const auto header = request->get_header("Cookie");
    if (header.has_value())
      cookie_value_(header.value(), PAIR_COOKIE, nonce);
  }

  const uint32_t now = millis();
  LockGuard guard{this->win_lock_};

  if (this->win_.state == 0) {
    this->send_json_(request, "200 OK", R"({"s":"none"})", false, false, nullptr);
    return;
  }
  if (nonce.empty() || !ct_equal_(this->win_.nonce, nonce.c_str())) {
    // A window exists but it is not this browser's. Says so without saying anything else.
    this->send_json_(request, "200 OK", R"({"s":"busy"})", false, false, nullptr);
    return;
  }

  switch (this->win_.state) {
    case 1: {
      char body[80];
      const int32_t left_ms = static_cast<int32_t>(this->win_.deadline - now);
      const uint32_t left = left_ms > 0 ? static_cast<uint32_t>(left_ms) / 1000 : 0;
      const char *mode = this->win_.mode == LoginMode::CODE  ? "code"
                         : this->win_.mode == LoginMode::SEQ ? "seq"
                                                             : "button";
      snprintf(body, sizeof(body), R"({"s":"pending","mode":"%s","left":%u,"hw":%d})", mode,
               static_cast<unsigned>(left), this->win_.hw_muted ? 1 : 0);
      this->send_json_(request, "200 OK", body, false, false, nullptr);
      return;
    }
    case 2: {
      // Approved: the cookie, plus the key once in the body so the page can prime the device's
      // other origin - the pairing flows have no other secret in JS to prime with. The window is
      // spent the moment this is delivered.
      this->win_.state = 0;
      char body[96];
      snprintf(body, sizeof(body), R"({"s":"ok","key":"%s"})", this->token_hex_);
      this->send_json_(request, "200 OK", body, true, false, nullptr);
      return;
    }
    case 3:
      this->send_json_(request, "200 OK", R"({"s":"expired"})", false, false, nullptr);
      return;
    default:
      this->send_json_(request, "200 OK", R"({"s":"denied"})", false, false, nullptr);
      return;
  }
}

/* ---- Approvals, called from YAML on the main loop ------------------------------------------- */

bool SessionGate::approve_pending_login() {
  LockGuard guard{this->win_lock_};
  if (this->win_.state != 1)
    return false;
  this->close_window_(LoginResult::APPROVED, millis());
  return true;
}

bool SessionGate::consume_login_transcript(const std::string &text) {
  LockGuard guard{this->win_lock_};
  if (this->win_.state != 1 || this->win_.mode != LoginMode::CODE)
    return false;

  std::string digits;
  normalize_digits_(text, digits);
  const uint32_t now = millis();

  if (digits == this->win_.code) {
    this->close_window_(LoginResult::APPROVED, now);
  } else if (++this->win_.attempts >= SG_CODE_ATTEMPTS) {
    this->close_window_(LoginResult::DENIED, now);
    this->voice_failed_(now);
  } else {
    ESP_LOGD(TAG_SG, "Spoken code mismatch %u of %u", this->win_.attempts, SG_CODE_ATTEMPTS);
  }
  // Consumed either way: the transcript was an answer to us, so it must reach neither the
  // transcript ring nor the conversation agent.
  return true;
}

bool SessionGate::consume_login_wake(const std::string &phrase) {
  LockGuard guard{this->win_lock_};
  if (this->win_.state != 1 || this->win_.mode != LoginMode::SEQ)
    return false;

  std::string lower;
  lower.reserve(phrase.size());
  for (const char c : phrase)
    lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));

  int symbol = -1;
  if (lower.find("jarvis") != std::string::npos) {
    symbol = 0;
  } else if (lower.find("nabu") != std::string::npos) {
    symbol = 1;
  } else if (lower.find("stop") != std::string::npos) {
    symbol = 2;
  }

  // Anything the models fired on during the window belongs to the challenge, recognised or not -
  // returning true is what keeps a firing from starting the assistant mid-answer.
  if (symbol >= 0 && this->win_.heard_len < sizeof(this->win_.heard)) {
    this->win_.heard[this->win_.heard_len++] = static_cast<uint8_t>(symbol);
    this->win_.last_symbol = millis();
    ESP_LOGD(TAG_SG, "Challenge symbol %d heard (%u so far)", symbol, this->win_.heard_len);
  }
  return true;
}

bool SessionGate::login_pending() {
  LockGuard guard{this->win_lock_};
  return this->win_.state == 1;
}

bool SessionGate::login_voice_pending() {
  LockGuard guard{this->win_lock_};
  return this->win_.state == 1 && this->win_.mode == LoginMode::CODE;
}

/* ---- The main-loop tick ---------------------------------------------------------------------- */

void SessionGate::tick(uint32_t now) {
  LockGuard guard{this->win_lock_};
  switch (this->win_.state) {
    case 1:
      if (static_cast<int32_t>(now - this->win_.deadline) >= 0) {
        // An unanswered offline challenge at expiry is a failed guess, not a timeout: symbols were
        // heard and they were wrong, or the count-up attacker ran out of window mid-count.
        if (this->win_.mode == LoginMode::SEQ && this->win_.heard_len > 0) {
          this->close_window_(LoginResult::DENIED, now);
          this->voice_failed_(now);
        } else {
          this->close_window_(LoginResult::EXPIRED, now);
        }
        return;
      }
      // The quiet-period judgement: exact sequence, judged only once the speaker has finished.
      // Overshooting fails, which is what makes counting upward through the answer worthless.
      if (this->win_.mode == LoginMode::SEQ && this->win_.heard_len > 0 &&
          now - this->win_.last_symbol >= SG_SEQ_QUIET_MS) {
        const bool match = this->win_.heard_len == 3 && memcmp(this->win_.heard, this->win_.seq, 3) == 0;
        if (match) {
          this->close_window_(LoginResult::APPROVED, now);
        } else {
          this->close_window_(LoginResult::DENIED, now);
          this->voice_failed_(now);
        }
      }
      return;
    case 2:
      if (now - this->win_.state_at >= SG_APPROVED_GRACE_MS)
        this->win_.state = 0;
      return;
    case 3:
    case 4:
      if (now - this->win_.state_at >= SG_ENDED_GRACE_MS)
        this->win_.state = 0;
      return;
    default:
      return;
  }
}

bool SessionGate::take_open_event(std::string &mode, std::string &secret) {
  if (!this->ev_open_.exchange(false))
    return false;
  LockGuard guard{this->win_lock_};
  mode = this->ev_mode_;
  secret = this->ev_secret_;
  return true;
}

bool SessionGate::take_close_event(std::string &result) {
  if (!this->ev_close_.exchange(false))
    return false;
  LockGuard guard{this->win_lock_};
  result = this->ev_result_;
  return true;
}

}  // namespace satellite1_web_ui
}  // namespace esphome
