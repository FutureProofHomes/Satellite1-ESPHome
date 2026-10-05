#include "xmos_firmware_catalog.h"

#ifdef USE_ESP32

#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif
#include "esphome/components/json/json_util.h"
#include "esphome/components/md5/md5.h"
#include "esphome/components/network/util.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include <esp_heap_caps.h>
#include <esp_http_client.h>
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include <esp_crt_bundle.h>
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/idf_additions.h>
#include <freertos/task.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <functional>
#include <new>

namespace esphome {
namespace xmos_firmware_catalog {

using memory_flasher::FLASHER_ERASING;
using memory_flasher::FLASHER_ERROR_STATE;
using memory_flasher::FLASHER_FLASHING;
using memory_flasher::FLASHER_IDLE;
using memory_flasher::FLASHER_SUCCESS_STATE;

static const char *const TAG = "xmos_catalog";
static const char *const BUILTIN_ID = "builtin";
static const char *const USER_AGENT = "Satellite1-XMOS-Catalog";
static const char *const GITHUB_API_VERSION = "2022-11-28";
static const char *const BIN_SUFFIX = ".factory.bin";
static const char *const MD5_SUFFIX = ".factory.md5";
// Official releases name the image after the project; the version is the release tag.
static const char *const GENERIC_STEM = "satellite1_xmos";

static constexpr size_t MAX_IMAGE_SIZE = 0xFF000;
static constexpr size_t MAX_LIST_BODY = 512 * 1024;
static constexpr size_t MAX_MD5_BODY = 256;
static constexpr size_t READ_CHUNK = 1024;
// In PSRAM, which is only safe because the task never touches flash: it does HTTP, TLS, JSON
// parsing and MD5, while NVS writes and flashing stay on the main loop.
static constexpr uint32_t TASK_STACK_SIZE = 10240;
static constexpr int HTTP_RX_BUFFER = 1024;
// Redirected asset URLs are signed and close to 1 KB long.
static constexpr int HTTP_TX_BUFFER = 2048;
static constexpr int HTTP_TIMEOUT_MS = 20000;
static constexpr int MAX_REDIRECTS = 5;
// esp_http_client and esp_tls state comes from internal RAM. While BLE is still enabled after boot,
// starting playback alone leaves only a few KB, so a job then could abort the device or stop the
// speaker task from starting. Once BLE is off, over 50 KB stays free even with music playing.
static constexpr size_t JOB_MIN_INTERNAL_FREE = 24 * 1024;
static constexpr size_t JOB_MIN_INTERNAL_BLOCK = 10 * 1024;
static constexpr uint32_t MIN_REFRESH_INTERVAL_MS = 2000;
static constexpr uint32_t REQUEST_TIMEOUT_MS = 30000;
static constexpr uint32_t START_TIMEOUT_MS = 45000;
static constexpr uint32_t PROGRESS_PUBLISH_INTERVAL_MS = 1000;
static constexpr uint32_t SNAPSHOT_MIN_INTERVAL_MS = 250;
static constexpr size_t CATALOG_TEXT_MAX = 250;

static constexpr uint32_t PIN_MAGIC = 0x58504E31;
static constexpr uint32_t CACHE_MAGIC = 0x58434332;
static constexpr uint32_t TOKEN_MAGIC = 0x58544B31;

struct PinRecord {
  uint32_t magic;
  uint8_t version[5];
  char label[27];
};

struct CatalogCache {
  uint32_t magic;
  uint32_t sources_hash;
  uint32_t count;
  FirmwareEntry entries[MAX_ENTRIES];
};

struct TokenRecord {
  uint32_t magic;
  char token[MAX_TOKEN_LENGTH + 1];
};

enum class JobType : uint8_t { REFRESH, STAGE };

struct JobSource {
  const char *repo;
  std::string token;
  bool include_prereleases;
  bool is_private;
  char etag[96];
};

struct SourceResult {
  bool ok{false};
  bool not_modified{false};
  char etag[96]{};
  char error[96]{};
  PsramVector<FirmwareEntry> entries;
};

// Lives in PSRAM for the duration of one background job.
struct CatalogJob {
  JobType type{JobType::REFRESH};
  uint8_t max_releases{20};
  PsramVector<JobSource> sources;
  PsramVector<SourceResult> results;
  FirmwareEntry entry{};
  bool ok{false};
  std::string error;
  uint8_t *data{nullptr};
  size_t capacity{0};
  size_t length{0};
  char md5[33]{};
  std::atomic<uint8_t> progress{0};
};

static CatalogJob *new_job() {
  void *memory = RAMAllocator<CatalogJob>().allocate(1);
  return memory == nullptr ? nullptr : new (memory) CatalogJob();
}

static void delete_job(CatalogJob *job) {
  if (job == nullptr)
    return;
  job->~CatalogJob();
  RAMAllocator<CatalogJob>().deallocate(job, 1);
}

static void free_image(uint8_t *data, size_t capacity) {
  if (data == nullptr)
    return;
  RAMAllocator<uint8_t> allocator(RAMAllocator<uint8_t>::ALLOC_EXTERNAL);
  allocator.deallocate(data, capacity);
}

static size_t internal_free() { return heap_caps_get_free_size(MALLOC_CAP_INTERNAL); }

static bool internal_ram_available() {
  return internal_free() >= JOB_MIN_INTERNAL_FREE &&
         heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) >= JOB_MIN_INTERNAL_BLOCK;
}

// ---------------------------------------------------------------------------
// Strings and versions

// Copies into a fixed-size field; false if it doesn't fit.
template<size_t N> static bool set_field(char (&dest)[N], const char *src, size_t length) {
  if (length >= N)
    return false;
  memcpy(dest, src, length);
  dest[length] = '\0';
  return true;
}

template<size_t N> static bool set_field(char (&dest)[N], const char *src) { return set_field(dest, src, strlen(src)); }

template<size_t N> static void set_truncated(char (&dest)[N], const char *src) {
  const size_t length = std::min(strlen(src), N - 1);
  memcpy(dest, src, length);
  dest[length] = '\0';
}

static bool read_version_number(const char *text, size_t &pos, uint8_t &out) {
  const size_t start = pos;
  uint32_t value = 0;
  while (std::isdigit(static_cast<unsigned char>(text[pos]))) {
    value = value * 10 + (text[pos] - '0');
    if (value > 255)
      return false;
    pos++;
  }
  if (pos == start)
    return false;
  out = static_cast<uint8_t>(value);
  return true;
}

// Same grammar as memory_flasher's image_version: v?MAJOR.MINOR.PATCH[-(alpha|beta|rc|dev)[.N]]
static bool parse_version(const char *text, uint8_t out[5]) {
  static const char *const PRE_RELEASE_NAMES[] = {"alpha", "beta", "rc", "dev"};
  uint8_t version[5]{};
  size_t pos = 0;
  if (text[pos] == 'v' || text[pos] == 'V')
    pos++;
  for (int i = 0; i < 3; i++) {
    if (i > 0) {
      if (text[pos] != '.')
        return false;
      pos++;
    }
    if (!read_version_number(text, pos, version[i]))
      return false;
  }
  if (text[pos] != '\0') {
    if (text[pos] != '-')
      return false;
    pos++;
    for (uint8_t i = 0; i < 4 && version[3] == 0; i++) {
      const size_t length = strlen(PRE_RELEASE_NAMES[i]);
      if (strncasecmp(text + pos, PRE_RELEASE_NAMES[i], length) == 0 &&
          (text[pos + length] == '\0' || text[pos + length] == '.')) {
        version[3] = i + 1;
        pos += length;
      }
    }
    if (version[3] == 0)
      return false;
    if (text[pos] != '\0') {
      pos++;
      if (!read_version_number(text, pos, version[4]) || text[pos] != '\0')
        return false;
    }
  }
  memcpy(out, version, sizeof(version));
  return true;
}

// Matches Satellite1::status_string() so labels line up with the "XMOS Firmware" sensor.
static std::string format_version(const uint8_t version[5]) {
  static const char *const PRE_RELEASE_NAMES[] = {"", "-alpha", "-beta", "-rc", "-dev"};
  char text[32];
  const int length = snprintf(text, sizeof(text), "v%u.%u.%u%s", version[0], version[1], version[2],
                              version[3] <= 4 ? PRE_RELEASE_NAMES[version[3]] : "");
  if (version[4] > 0 && length > 0 && static_cast<size_t>(length) < sizeof(text))
    snprintf(text + length, sizeof(text) - length, ".%u", version[4]);
  return text;
}

static bool init_entry_version(FirmwareEntry &entry, const char *text) {
  if (!parse_version(text, entry.version_bytes))
    return false;
  const char *rest = (text[0] == 'v' || text[0] == 'V') ? text + 1 : text;
  const size_t length = strlen(rest);
  if (length + 1 >= sizeof(entry.version))
    return false;
  entry.version[0] = 'v';
  memcpy(entry.version + 1, rest, length + 1);
  return true;
}

static bool same_version(const uint8_t a[5], const uint8_t b[5]) { return memcmp(a, b, 5) == 0; }

static bool same_entry(const FirmwareEntry &a, const FirmwareEntry &b) {
  return strcmp(a.version, b.version) == 0 && strcmp(a.tag, b.tag) == 0 && strcmp(a.stem, b.stem) == 0 &&
         strcmp(a.date, b.date) == 0 && a.bin_asset_id == b.bin_asset_id && a.md5_asset_id == b.md5_asset_id &&
         a.source == b.source;
}

static bool is_url_safe(const char *text) {
  if (text[0] == '\0')
    return false;
  for (const char *c = text; *c != '\0'; c++) {
    if (!std::isalnum(static_cast<unsigned char>(*c)) && *c != '.' && *c != '-' && *c != '_')
      return false;
  }
  return true;
}

static std::string asset_url(const JobSource &source, const FirmwareEntry &entry, bool md5) {
  if (source.is_private) {
    return std::string("https://api.github.com/repos/") + source.repo + "/releases/assets/" +
           std::to_string(md5 ? entry.md5_asset_id : entry.bin_asset_id);
  }
  return std::string("https://github.com/") + source.repo + "/releases/download/" + entry.tag + "/" + entry.stem +
         (md5 ? MD5_SUFFIX : BIN_SUFFIX);
}

// Accepts "<md5>" or "<md5>  <file name>", as written by md5 -q and md5sum.
static bool parse_md5(const char *text, char out[33]) {
  while (std::isspace(static_cast<unsigned char>(*text)))
    text++;
  for (int i = 0; i < 32; i++) {
    if (!std::isxdigit(static_cast<unsigned char>(text[i])))
      return false;
    out[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(text[i])));
  }
  out[32] = '\0';
  return text[32] == '\0' || std::isspace(static_cast<unsigned char>(text[32]));
}

// ---------------------------------------------------------------------------
// HTTP (runs on the catalog task)

namespace {

struct HttpHeader {
  const char *name;
  std::string value;
};

struct HttpResult {
  int status{-1};
  char etag[96]{};
  std::string error;
};

using BodyBeginFn = std::function<bool(int64_t content_length)>;
using BodyChunkFn = std::function<bool(const uint8_t *data, size_t length)>;

esp_err_t http_event_handler(esp_http_client_event_t *evt) {
  if (evt->event_id == HTTP_EVENT_ON_HEADER && evt->user_data != nullptr && evt->header_key != nullptr &&
      evt->header_value != nullptr && strcasecmp(evt->header_key, "ETag") == 0) {
    // An ETag that doesn't fit only costs the next refresh its 304.
    auto *result = static_cast<HttpResult *>(evt->user_data);
    if (!set_field(result->etag, evt->header_value))
      result->etag[0] = '\0';
  }
  return ESP_OK;
}

bool is_redirect(int status) {
  return status == 301 || status == 302 || status == 303 || status == 307 || status == 308;
}

// Content length is -1 when the response is chunked.
HttpResult http_get(const std::string &url, const std::vector<HttpHeader> &headers, const BodyBeginFn &on_begin,
                    const BodyChunkFn &on_chunk) {
  HttpResult result;
  esp_http_client_config_t config = {};
  config.url = url.c_str();
  config.method = HTTP_METHOD_GET;
  config.timeout_ms = HTTP_TIMEOUT_MS;
  config.buffer_size = HTTP_RX_BUFFER;
  config.buffer_size_tx = HTTP_TX_BUFFER;
  config.user_agent = USER_AGENT;
  config.keep_alive_enable = false;
  config.event_handler = http_event_handler;
  config.user_data = &result;
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
  config.crt_bundle_attach = esp_crt_bundle_attach;
#endif

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (client == nullptr) {
    result.error = "Not enough memory for HTTP";
    return result;
  }
  for (const auto &header : headers)
    esp_http_client_set_header(client, header.name, header.value.c_str());

  esp_err_t err = esp_http_client_open(client, 0);
  int redirects = 0;
  while (err == ESP_OK) {
    const int64_t content_length = esp_http_client_fetch_headers(client);
    result.status = esp_http_client_get_status_code(client);
    if (is_redirect(result.status) && redirects < MAX_REDIRECTS) {
      redirects++;
      esp_http_client_flush_response(client, nullptr);
      // Signed CDN URLs reject requests that also carry GitHub credentials.
      esp_http_client_delete_header(client, "Authorization");
      esp_http_client_delete_header(client, "If-None-Match");
      result.etag[0] = '\0';
      err = esp_http_client_set_redirection(client);
      if (err == ESP_OK)
        err = esp_http_client_open(client, 0);
      continue;
    }
    if (result.status != 200 || !on_chunk)
      break;

    const int64_t length = esp_http_client_is_chunked_response(client) ? -1 : content_length;
    if (on_begin && !on_begin(length)) {
      result.status = -1;
      result.error = "Download rejected";
      break;
    }
    uint8_t buffer[READ_CHUNK];
    while (true) {
      const int read = esp_http_client_read(client, reinterpret_cast<char *>(buffer), sizeof(buffer));
      if (read < 0) {
        result.status = -1;
        result.error = "Download interrupted";
        break;
      }
      if (read == 0) {
        if (!esp_http_client_is_complete_data_received(client)) {
          result.status = -1;
          result.error = "Download incomplete";
        }
        break;
      }
      if (!on_chunk(buffer, read)) {
        result.status = -1;
        result.error = "Download rejected";
        break;
      }
    }
    break;
  }
  if (err != ESP_OK) {
    result.status = -1;
    result.error = std::string("Connection failed (") + esp_err_to_name(err) + ")";
  }
  esp_http_client_close(client);
  esp_http_client_cleanup(client);
  return result;
}

std::string describe_http_failure(const HttpResult &result, bool has_token) {
  if (result.status < 0)
    return result.error.empty() ? "Connection failed" : result.error;
  switch (result.status) {
    case 401:
      return "GitHub rejected the token";
    case 403:
    case 429:
      return "GitHub rate limit reached or access denied; try again later";
    case 404:
      return has_token ? "Not found; check the repository name and token access"
                       : "Not found; private repositories need a token";
    default:
      return "HTTP " + std::to_string(result.status);
  }
}

// Growable PSRAM buffer for the release list (~120 KB for 20 releases).
class PsramBuffer {
 public:
  ~PsramBuffer() {
    if (this->data_ != nullptr)
      this->allocator_.deallocate(this->data_, this->capacity_);
  }
  bool append(const uint8_t *data, size_t length, size_t limit) {
    if (this->length_ + length > limit)
      return false;
    if (this->length_ + length > this->capacity_) {
      size_t capacity = std::max<size_t>(this->capacity_ * 2, 32 * 1024);
      while (capacity < this->length_ + length)
        capacity *= 2;
      capacity = std::min(capacity, limit);
      uint8_t *grown = this->allocator_.reallocate(this->data_, capacity);
      if (grown == nullptr)
        return false;
      this->data_ = grown;
      this->capacity_ = capacity;
    }
    memcpy(this->data_ + this->length_, data, length);
    this->length_ += length;
    return true;
  }
  const uint8_t *data() const { return this->data_; }
  size_t length() const { return this->length_; }

 protected:
  RAMAllocator<uint8_t> allocator_{RAMAllocator<uint8_t>::ALLOC_EXTERNAL};
  uint8_t *data_{nullptr};
  size_t capacity_{0};
  size_t length_{0};
};

bool parse_releases(const uint8_t *data, size_t length, uint8_t source_index, const JobSource &source,
                    PsramVector<FirmwareEntry> &entries) {
  if (data == nullptr || length == 0)
    return false;

#ifdef USE_PSRAM
  json::SpiRamAllocator allocator;
  JsonDocument filter(&allocator);
  JsonDocument doc(&allocator);
#else
  JsonDocument filter;
  JsonDocument doc;
#endif
  filter[0]["tag_name"] = true;
  filter[0]["draft"] = true;
  filter[0]["prerelease"] = true;
  filter[0]["published_at"] = true;
  filter[0]["assets"][0]["id"] = true;
  filter[0]["assets"][0]["name"] = true;
  filter[0]["assets"][0]["size"] = true;
  filter[0]["assets"][0]["updated_at"] = true;

  const DeserializationError error =
      deserializeJson(doc, reinterpret_cast<const char *>(data), length, DeserializationOption::Filter(filter));
  if (error || !doc.is<JsonArray>()) {
    ESP_LOGW(TAG, "Couldn't parse the release list from %s: %s", source.repo, error.c_str());
    return false;
  }

  const size_t suffix_length = strlen(BIN_SUFFIX);
  for (JsonObject release : doc.as<JsonArray>()) {
    if (release["draft"] | false)
      continue;
    const bool prerelease = release["prerelease"] | false;
    if (prerelease && !source.include_prereleases)
      continue;
    const char *tag = release["tag_name"] | "";
    if (!is_url_safe(tag))
      continue;
    const char *published = release["published_at"] | "";
    JsonArray assets = release["assets"];
    for (JsonObject asset : assets) {
      const char *name = asset["name"] | "";
      const size_t name_length = strlen(name);
      if (name_length <= suffix_length || strcmp(name + name_length - suffix_length, BIN_SUFFIX) != 0)
        continue;
      FirmwareEntry entry{};
      if (!set_field(entry.stem, name, name_length - suffix_length) || !is_url_safe(entry.stem) ||
          !set_field(entry.tag, tag))
        continue;
      char md5_name[sizeof(entry.stem) + 16];
      snprintf(md5_name, sizeof(md5_name), "%s%s", entry.stem, MD5_SUFFIX);
      for (JsonObject other : assets) {
        if (strcmp(md5_name, other["name"] | "") == 0) {
          entry.md5_asset_id = other["id"] | static_cast<uint64_t>(0);
          break;
        }
      }
      entry.size = asset["size"] | static_cast<uint32_t>(0);
      if (entry.md5_asset_id == 0 || entry.size == 0 || entry.size > MAX_IMAGE_SIZE)
        continue;
      if (!init_entry_version(entry, strcmp(entry.stem, GENERIC_STEM) == 0 ? entry.tag : entry.stem))
        continue;
      set_truncated(entry.date, asset["updated_at"] | published);
      entry.bin_asset_id = asset["id"] | static_cast<uint64_t>(0);
      entry.source = source_index;
      entry.prerelease = prerelease;
      entries.push_back(entry);
    }
  }
  return true;
}

void run_refresh_job(CatalogJob &job) {
  job.results.resize(job.sources.size());
  for (size_t i = 0; i < job.sources.size(); i++) {
    const JobSource &source = job.sources[i];
    SourceResult &result = job.results[i];
    const std::string url = std::string("https://api.github.com/repos/") + source.repo +
                            "/releases?per_page=" + std::to_string(job.max_releases);
    std::string token = source.token;
    for (int attempt = 0; attempt < 2; attempt++) {
      std::vector<HttpHeader> headers = {{"Accept", "application/vnd.github+json"},
                                         {"X-GitHub-Api-Version", GITHUB_API_VERSION}};
      if (!token.empty())
        headers.push_back({"Authorization", "Bearer " + token});
      // 304 responses don't count against GitHub's unauthenticated rate limit.
      if (source.etag[0] != '\0')
        headers.push_back({"If-None-Match", source.etag});

      PsramBuffer body;
      bool too_large = false;
      HttpResult http = http_get(
          url, headers, [](int64_t length) { return length < 0 || static_cast<size_t>(length) <= MAX_LIST_BODY; },
          [&body, &too_large](const uint8_t *data, size_t length) {
            too_large = !body.append(data, length, MAX_LIST_BODY);
            return !too_large;
          });

      if (http.status == 401 && !token.empty() && !source.is_private) {
        // A bad token must not hide a public repository.
        token.clear();
        continue;
      }
      if (http.status == 304) {
        result.ok = true;
        result.not_modified = true;
        memcpy(result.etag, source.etag, sizeof(result.etag));
      } else if (http.status == 200) {
        result.ok = parse_releases(body.data(), body.length(), static_cast<uint8_t>(i), source, result.entries);
        if (result.ok) {
          memcpy(result.etag, http.etag, sizeof(result.etag));
        } else {
          set_truncated(result.error, "Unexpected response from GitHub");
        }
      } else {
        set_truncated(result.error,
                      too_large ? "Release list is too large" : describe_http_failure(http, !token.empty()).c_str());
      }
      break;
    }
    if (result.not_modified) {
      ESP_LOGD(TAG, "%s: unchanged", source.repo);
    } else if (result.ok) {
      ESP_LOGD(TAG, "%s: %u images", source.repo, static_cast<unsigned>(result.entries.size()));
    } else {
      ESP_LOGW(TAG, "%s: %s", source.repo, result.error);
    }
  }
}

void run_stage_job(CatalogJob &job) {
  const JobSource &source = job.sources[0];
  const FirmwareEntry &entry = job.entry;
  const bool use_token = source.is_private && !source.token.empty();
  std::vector<HttpHeader> headers = {{"Accept", "application/octet-stream"}};
  if (use_token) {
    headers.push_back({"Authorization", "Bearer " + source.token});
    headers.push_back({"X-GitHub-Api-Version", GITHUB_API_VERSION});
  }

  char md5_text[MAX_MD5_BODY + 1];
  size_t md5_length = 0;
  HttpResult http = http_get(asset_url(source, entry, true), headers, nullptr,
                             [&md5_text, &md5_length](const uint8_t *data, size_t length) {
                               if (md5_length + length > MAX_MD5_BODY)
                                 return false;
                               memcpy(md5_text + md5_length, data, length);
                               md5_length += length;
                               return true;
                             });
  if (http.status != 200) {
    job.error = "Checksum download failed: " + describe_http_failure(http, use_token);
    return;
  }
  md5_text[md5_length] = '\0';
  if (!parse_md5(md5_text, job.md5)) {
    job.error = "Checksum file is invalid";
    return;
  }

  RAMAllocator<uint8_t> allocator(RAMAllocator<uint8_t>::ALLOC_EXTERNAL);
  size_t expected = 0;
  bool too_large = false;
  bool out_of_memory = false;
  md5::MD5Digest digest;
  digest.init();
  http = http_get(
      asset_url(source, entry, false), headers,
      [&](int64_t content_length) {
        if (content_length == 0 || content_length > static_cast<int64_t>(MAX_IMAGE_SIZE)) {
          too_large = content_length > 0;
          return false;
        }
        expected = content_length > 0 ? static_cast<size_t>(content_length) : entry.size;
        job.capacity = content_length > 0 ? static_cast<size_t>(content_length) : MAX_IMAGE_SIZE;
        job.data = allocator.allocate(job.capacity);
        out_of_memory = job.data == nullptr;
        return !out_of_memory;
      },
      [&](const uint8_t *chunk, size_t chunk_length) {
        if (job.length + chunk_length > job.capacity) {
          too_large = true;
          return false;
        }
        memcpy(job.data + job.length, chunk, chunk_length);
        job.length += chunk_length;
        digest.add(chunk, chunk_length);
        if (expected > 0)
          job.progress.store(static_cast<uint8_t>(std::min<size_t>(99, job.length * 100 / expected)));
        return true;
      });

  if (http.status != 200 || job.length == 0) {
    if (too_large) {
      job.error = "Image is larger than the XMOS boot partition";
    } else if (out_of_memory) {
      job.error = "Not enough memory to stage the image";
    } else {
      job.error = "Download failed: " + describe_http_failure(http, use_token);
    }
    return;
  }

  char computed[33];
  digest.calculate();
  digest.get_hex(computed);
  if (strncmp(computed, job.md5, 32) != 0) {
    ESP_LOGW(TAG, "MD5 mismatch: expected %s, computed %s", job.md5, computed);
    job.error = "Checksum mismatch; nothing was flashed";
    return;
  }
  job.ok = true;
  job.progress.store(100);
}

}  // namespace

// ---------------------------------------------------------------------------
// Entities

void CatalogSelect::control(size_t index) { this->parent_->select_option(index); }
void CatalogRefreshButton::press_action() { this->parent_->request_refresh(); }
void CatalogInstallButton::press_action() { this->parent_->install_selected(); }

// ---------------------------------------------------------------------------
// Component

void XmosFirmwareCatalog::setup() {
  parse_version(this->builtin_version_, this->builtin_bytes_);
  this->builtin_label_.append("Built-in (").append(this->builtin_version_).append(")");
  this->source_state_.resize(this->sources_.size());
  this->selected_id_ = BUILTIN_ID;

  this->load_token_();
  this->load_pin_();
  this->load_cache_();
  if (this->pin_set_) {
    if (const FirmwareEntry *entry = this->find_entry_(this->pin_label_))
      this->selected_id_ = entry->version;
  }
  this->update_select_options_();
  this->publish_catalog_text_();
  this->set_status_("Idle");

  this->flasher_->add_on_state_callback([this]() { this->on_flasher_state_(); });
}

void XmosFirmwareCatalog::dump_config() {
  ESP_LOGCONFIG(TAG,
                "XMOS Firmware Catalog:\n"
                "  Built-in image: %s\n"
                "  Releases per source: %u",
                this->builtin_version_, this->max_releases_);
  for (const auto &source : this->sources_) {
    ESP_LOGCONFIG(TAG, "  Source: %s%s%s%s", source.repo, source.is_private ? " (private)" : "",
                  source.include_prereleases ? "" : " (releases only)", source.token[0] != '\0' ? " [token]" : "");
  }
  ESP_LOGCONFIG(TAG, "  Runtime token: %s", this->runtime_token_.empty() ? "not set" : "set");
  if (this->pin_set_)
    ESP_LOGCONFIG(TAG, "  Kept across reboots: %s", this->pin_label_.c_str());
}

void XmosFirmwareCatalog::loop() {
  this->process_requests_();

  if (this->job_done_.load(std::memory_order_acquire))
    this->finish_job_();

  const uint32_t now = millis();
  if (this->auto_refresh_waiting_ && now - this->last_auto_refresh_check_ >= 1000) {
    this->last_auto_refresh_check_ = now;
    if (this->job_ == nullptr && this->install_state_ == InstallState::IDLE && network::is_connected() &&
        internal_ram_available())
      this->start_refresh_();
  }

  if (this->recovery_pending_ && this->flasher_->state == FLASHER_IDLE) {
    this->recovery_pending_ = false;
    ESP_LOGW(TAG, "Restoring built-in XMOS firmware %s", this->builtin_version_);
    this->flasher_->flash_embedded_image();
  }

  this->update_install_progress_();

  uint8_t running[5]{};
  const bool connected = this->xmos_running_version_(running);
  if (connected != this->last_connected_ || (connected && !same_version(running, this->last_running_))) {
    this->last_connected_ = connected;
    memcpy(this->last_running_, running, sizeof(running));
    this->snapshot_dirty_ = true;
  }

  if (this->snapshot_dirty_ && millis() - this->snapshot_built_ >= SNAPSHOT_MIN_INTERVAL_MS)
    this->rebuild_snapshot_();
}

// ---------------------------------------------------------------------------
// Requests (any task)

void XmosFirmwareCatalog::request_refresh() {
  LockGuard lock(this->mutex_);
  this->pending_refresh_ = true;
}

void XmosFirmwareCatalog::request_auto_refresh() {
  LockGuard lock(this->mutex_);
  this->pending_auto_refresh_ = true;
}

void XmosFirmwareCatalog::request_install(const std::string &version) {
  LockGuard lock(this->mutex_);
  this->pending_install_ = version;
  this->pending_install_set_ = true;
}

void XmosFirmwareCatalog::set_runtime_token(const std::string &token) {
  LockGuard lock(this->mutex_);
  this->pending_token_ = token;
  this->pending_token_set_ = true;
}

std::string XmosFirmwareCatalog::snapshot_json() {
  LockGuard lock(this->mutex_);
  return std::string(this->snapshot_.data(), this->snapshot_.size());
}

void XmosFirmwareCatalog::process_requests_() {
  bool refresh = false;
  bool install = false;
  bool token = false;
  std::string install_version;
  std::string new_token;
  {
    LockGuard lock(this->mutex_);
    refresh = this->pending_refresh_;
    this->pending_refresh_ = false;
    if (this->pending_auto_refresh_) {
      this->auto_refresh_waiting_ = true;
      this->pending_auto_refresh_ = false;
    }
    if (this->pending_token_set_) {
      token = true;
      new_token = std::move(this->pending_token_);
      this->pending_token_.clear();
      this->pending_token_set_ = false;
    }
    // An install waits for a running list refresh so it sees the newest entries.
    if (this->pending_install_set_ && (this->job_ == nullptr || this->job_->type != JobType::REFRESH)) {
      install = true;
      install_version = std::move(this->pending_install_);
      this->pending_install_.clear();
      this->pending_install_set_ = false;
    }
  }

  if (token) {
    new_token.erase(0, new_token.find_first_not_of(" \t\r\n"));
    new_token.erase(new_token.find_last_not_of(" \t\r\n") + 1);
    bool valid = new_token.size() <= MAX_TOKEN_LENGTH;
    for (char c : new_token)
      valid = valid && c > ' ' && c < 0x7F;
    if (!valid) {
      ESP_LOGW(TAG, "Ignoring GitHub token: it must be at most %u printable characters",
               static_cast<unsigned>(MAX_TOKEN_LENGTH));
    } else {
      this->save_token_(new_token);
      this->runtime_token_.assign(new_token.data(), new_token.size());
      // Visibility depends on the token, so cached ETags no longer apply.
      for (auto &state : this->source_state_)
        state.etag[0] = '\0';
      ESP_LOGI(TAG, "GitHub token %s", this->runtime_token_.empty() ? "cleared" : "updated");
      this->snapshot_dirty_ = true;
      refresh = true;
    }
  }
  if (install)
    this->handle_install_request_(install_version);
  if (refresh)
    this->start_refresh_();
}

void XmosFirmwareCatalog::select_option(size_t index) {
  if (this->select_ == nullptr || index > this->entries_.size())
    return;
  this->selected_id_ = index == 0 ? BUILTIN_ID : this->entries_[index - 1].version;
  this->select_->publish_state(index);
  this->snapshot_dirty_ = true;
}

void XmosFirmwareCatalog::install_selected() { this->request_install(this->selected_id_); }

// ---------------------------------------------------------------------------
// Refresh

void XmosFirmwareCatalog::start_refresh_() {
  if (this->job_ != nullptr || this->install_state_ != InstallState::IDLE) {
    ESP_LOGD(TAG, "Refresh skipped: busy");
    return;
  }
  const uint32_t now = millis();
  if (this->last_refresh_attempt_ != 0 && now - this->last_refresh_attempt_ < MIN_REFRESH_INTERVAL_MS)
    return;
  this->last_refresh_attempt_ = now | 1;

  if (!network::is_connected()) {
    this->list_error_ = "No network connection";
    this->publish_catalog_text_();
    this->snapshot_dirty_ = true;
    return;
  }
  if (!internal_ram_available()) {
    ESP_LOGW(TAG, "Refresh refused: only %u B internal RAM free", static_cast<unsigned>(internal_free()));
    this->list_error_ = "Not enough free memory; try again in a minute";
    this->publish_catalog_text_();
    this->set_status_("Refresh failed: not enough free memory; try again in a minute");
    return;
  }

  CatalogJob *job = new_job();
  if (job != nullptr) {
    job->type = JobType::REFRESH;
    job->max_releases = this->max_releases_;
    for (size_t i = 0; i < this->sources_.size(); i++) {
      JobSource source{};
      source.repo = this->sources_[i].repo;
      source.token = this->token_for_source_(i);
      source.include_prereleases = this->sources_[i].include_prereleases;
      source.is_private = this->sources_[i].is_private;
      memcpy(source.etag, this->source_state_[i].etag, sizeof(source.etag));
      job->sources.push_back(std::move(source));
    }
  }
  if (job == nullptr || !this->start_job_(job)) {
    this->list_error_ = "Couldn't start the refresh task";
    this->snapshot_dirty_ = true;
    return;
  }
  this->auto_refresh_waiting_ = false;
  this->refreshing_ = true;
  this->set_status_("Refreshing firmware list...");
}

void XmosFirmwareCatalog::apply_refresh_results_(CatalogJob &job) {
  this->refreshing_ = false;
  size_t ok_count = 0;
  std::string first_error;
  for (size_t i = 0; i < job.results.size() && i < this->source_state_.size(); i++) {
    SourceResult &result = job.results[i];
    SourceState &state = this->source_state_[i];
    if (result.ok) {
      ok_count++;
      state.error[0] = '\0';
      if (!result.not_modified) {
        state.entries = std::move(result.entries);
        memcpy(state.etag, result.etag, sizeof(state.etag));
      }
    } else {
      // Keep the last known entries so an outage doesn't empty the list.
      memcpy(state.error, result.error, sizeof(state.error));
      if (first_error.empty())
        first_error = std::string(this->sources_[i].repo) + ": " + result.error;
    }
  }
  this->list_error_ = first_error;
  if (ok_count > 0)
    this->list_loaded_ = true;

  if (this->rebuild_entries_()) {
    if (ok_count > 0)
      this->save_cache_();
    // Let the refresh status reach Home Assistant before it is asked to reconnect.
    this->set_timeout("choices_changed", 500, [this]() { this->announce_choices_changed_(); });
  }
  ESP_LOGI(TAG, "Firmware list refreshed: %u image(s)%s%s", static_cast<unsigned>(this->entries_.size()),
           first_error.empty() ? "" : "; ", first_error.c_str());
  if (this->install_state_ != InstallState::IDLE)
    return;
  if (!first_error.empty()) {
    this->set_status_("Refresh failed: " + first_error);
  } else {
    this->set_status_(this->last_result_.empty() ? "Idle" : this->last_result_);
  }
}

bool XmosFirmwareCatalog::rebuild_entries_() {
  PsramVector<FirmwareEntry> merged;
  for (const auto &state : this->source_state_)
    merged.insert(merged.end(), state.entries.begin(), state.entries.end());
  // Newest upload first; ISO 8601 timestamps sort lexically.
  std::stable_sort(merged.begin(), merged.end(), [](const FirmwareEntry &a, const FirmwareEntry &b) {
    const int order = strcmp(a.date, b.date);
    if (order != 0)
      return order > 0;
    return memcmp(a.version_bytes, b.version_bytes, 5) > 0;
  });

  PsramVector<FirmwareEntry> unique;
  for (const auto &entry : merged) {
    if (unique.size() >= MAX_ENTRIES)
      break;
    if (same_version(entry.version_bytes, this->builtin_bytes_))
      continue;
    bool duplicate = false;
    for (const auto &kept : unique)
      duplicate = duplicate || same_version(kept.version_bytes, entry.version_bytes);
    if (!duplicate)
      unique.push_back(entry);
  }

  bool changed = unique.size() != this->entries_.size();
  for (size_t i = 0; !changed && i < unique.size(); i++)
    changed = !same_entry(unique[i], this->entries_[i]);
  this->entries_ = std::move(unique);
  this->update_select_options_();
  this->publish_catalog_text_();
  this->snapshot_dirty_ = true;
  return changed;
}

void XmosFirmwareCatalog::announce_choices_changed_() {
  if (this->has_choices_automation_) {
    this->choices_changed_callback_.call();
    return;
  }
  // Home Assistant never sees an internal select, so it has no stale options to re-read.
  if (this->select_ == nullptr || this->select_->is_internal())
    return;
#ifdef USE_API
  if (api::global_api_server == nullptr)
    return;
  bool any = false;
  for (const auto &conn : api::global_api_server->active_clients()) {
    conn->on_fatal_error();
    any = true;
  }
  if (any)
    ESP_LOGI(TAG, "Firmware choices changed; dropping the API connection so Home Assistant re-reads them");
#endif
}

void XmosFirmwareCatalog::update_select_options_() {
  if (this->select_ == nullptr)
    return;
  // The select keeps these pointers, so entries_ must not change without calling this again.
  FixedVector<const char *> options;
  options.init(this->entries_.size() + 1);
  options.push_back(this->builtin_label_.c_str());
  size_t index = 0;
  for (size_t i = 0; i < this->entries_.size(); i++) {
    options.push_back(this->entries_[i].version);
    if (this->selected_id_ == this->entries_[i].version)
      index = i + 1;
  }
  this->select_->traits.set_options(options);
  this->selected_id_ = index == 0 ? BUILTIN_ID : this->entries_[index - 1].version;
  this->select_->publish_state(index);
}

// ---------------------------------------------------------------------------
// Background task

bool XmosFirmwareCatalog::start_job_(CatalogJob *job) {
  this->job_ = job;
  this->job_done_.store(false, std::memory_order_release);
#ifdef USE_PSRAM
  const UBaseType_t stack_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
#else
  const UBaseType_t stack_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
#endif
  if (xTaskCreateWithCaps(task_entry_, "xmos_catalog", TASK_STACK_SIZE, this, 1, nullptr, stack_caps) != pdPASS) {
    ESP_LOGE(TAG, "Couldn't create the catalog task");
    this->job_ = nullptr;
    delete_job(job);
    return false;
  }
  return true;
}

void XmosFirmwareCatalog::task_entry_(void *arg) {
  auto *self = static_cast<XmosFirmwareCatalog *>(arg);
  CatalogJob &job = *self->job_;
  if (job.type == JobType::REFRESH) {
    run_refresh_job(job);
  } else {
    run_stage_job(job);
  }
  self->job_done_.store(true, std::memory_order_release);
  // Pairs with xTaskCreateWithCaps; a plain vTaskDelete would leak the stack.
  vTaskDeleteWithCaps(nullptr);
}

void XmosFirmwareCatalog::finish_job_() {
  this->job_done_.store(false, std::memory_order_release);
  CatalogJob *job = this->job_;
  this->job_ = nullptr;

  if (job->type == JobType::REFRESH) {
    this->apply_refresh_results_(*job);
  } else if (this->install_state_ != InstallState::DOWNLOADING) {
    free_image(job->data, job->capacity);
  } else if (!job->ok) {
    free_image(job->data, job->capacity);
    this->fail_install_(job->error);
  } else {
    this->staged_data_ = job->data;
    this->staged_capacity_ = job->capacity;
    this->staged_length_ = job->length;
    this->staged_md5_ = job->md5;
    ESP_LOGI(TAG, "Downloaded %s: %u bytes, MD5 %s", this->install_target_.c_str(),
             static_cast<unsigned>(this->staged_length_), this->staged_md5_.c_str());
    // Set first: the flasher reports its first state change synchronously.
    this->install_state_ = InstallState::FLASHING;
    this->install_state_since_ = millis();
    this->install_saw_erase_ = false;
    this->last_reported_progress_ = 255;
    if (this->flasher_->flash_staged_image(this->staged_data_, this->staged_length_, this->staged_md5_)) {
      this->set_status_("Flashing " + this->install_target_ + " (0%)");
    } else {
      this->fail_install_("The XMOS flasher is busy");
    }
  }
  delete_job(job);
}

// ---------------------------------------------------------------------------
// Install

bool XmosFirmwareCatalog::busy() const {
  return this->install_state_ != InstallState::IDLE || (this->job_ != nullptr && this->job_->type == JobType::STAGE) ||
         this->flasher_->in_progress() || this->flasher_->boot_flash_pending();
}

void XmosFirmwareCatalog::handle_install_request_(const std::string &version) {
  if (this->busy()) {
    ESP_LOGW(TAG, "Ignoring install request for %s: another XMOS operation is running", version.c_str());
    return;
  }
  std::string target;
  if (this->is_builtin_request_(version)) {
    target = BUILTIN_ID;
  } else if (const FirmwareEntry *entry = this->find_entry_(version)) {
    const FirmwareSource &source = this->sources_[entry->source];
    if (source.is_private && this->token_for_source_(entry->source).empty()) {
      this->finish_install_(std::string("Failed: ") + source.repo + " needs a GitHub token", false);
      return;
    }
    target = entry->version;
  } else {
    this->finish_install_("Failed: " + version + " is not in the firmware list", false);
    return;
  }

  this->install_target_ = target;
  this->install_state_ = InstallState::REQUESTED;
  this->install_state_since_ = millis();
  ESP_LOGI(TAG, "Installing XMOS firmware %s", this->target_label_().c_str());
  this->set_status_("Stopping audio to install " + this->target_label_() + "...");
  this->install_request_callback_.call(target);
}

bool XmosFirmwareCatalog::begin_install() {
  if (this->install_state_ != InstallState::REQUESTED) {
    ESP_LOGW(TAG, "No XMOS firmware install is waiting to start");
    return false;
  }
  if (this->flasher_->in_progress() || this->flasher_->boot_flash_pending()) {
    this->fail_install_("The XMOS flasher is busy");
    return false;
  }
  this->install_state_since_ = millis();
  this->install_saw_erase_ = false;
  this->last_reported_progress_ = 255;

  if (this->install_target_ == BUILTIN_ID) {
    // Set first: the flasher reports its first state change synchronously.
    this->install_state_ = InstallState::FLASHING;
    this->set_status_("Flashing " + this->target_label_() + " (0%)");
    this->flasher_->flash_embedded_image();
    return true;
  }

  const FirmwareEntry *entry = this->find_entry_(this->install_target_);
  if (entry == nullptr) {
    this->fail_install_(this->install_target_ + " is no longer in the firmware list");
    return false;
  }
  if (this->job_ != nullptr) {
    this->fail_install_("The firmware list is still refreshing; try again");
    return false;
  }
  if (!internal_ram_available()) {
    ESP_LOGW(TAG, "Install refused: only %u B internal RAM free", static_cast<unsigned>(internal_free()));
    this->fail_install_("not enough free memory to download; try again in a minute");
    return false;
  }

  this->release_staged_image_();
  CatalogJob *job = new_job();
  if (job != nullptr) {
    const FirmwareSource &config = this->sources_[entry->source];
    JobSource source{};
    source.repo = config.repo;
    source.token = this->token_for_source_(entry->source);
    source.include_prereleases = config.include_prereleases;
    source.is_private = config.is_private;
    job->type = JobType::STAGE;
    job->entry = *entry;
    job->sources.push_back(std::move(source));
  }
  if (job == nullptr || !this->start_job_(job)) {
    this->fail_install_("Couldn't start the download task");
    return false;
  }
  this->install_state_ = InstallState::DOWNLOADING;
  this->set_status_("Downloading " + this->install_target_ + " (0%)");
  return true;
}

void XmosFirmwareCatalog::abort_install(const std::string &reason) {
  if (this->install_state_ == InstallState::REQUESTED || this->install_state_ == InstallState::DOWNLOADING)
    this->fail_install_(reason);
}

void XmosFirmwareCatalog::fail_install_(const std::string &reason) {
  this->release_staged_image_();
  this->finish_install_("Failed: " + reason, false);
}

void XmosFirmwareCatalog::finish_install_(const std::string &result, bool ok) {
  this->install_state_ = InstallState::IDLE;
  this->install_state_since_ = millis();
  this->recovery_pending_ = false;
  this->last_result_ = result;
  this->last_result_ok_ = ok;
  if (ok) {
    ESP_LOGI(TAG, "%s", result.c_str());
  } else {
    ESP_LOGW(TAG, "%s", result.c_str());
  }
  this->set_status_(result);
}

void XmosFirmwareCatalog::start_recovery_(const std::string &reason) {
  ESP_LOGW(TAG, "%s; restoring the built-in image", reason.c_str());
  this->recovery_reason_ = reason;
  this->install_state_ = InstallState::RECOVERING;
  this->install_state_since_ = millis();
  this->recovery_pending_ = true;
  this->last_reported_progress_ = 255;
  this->set_status_(std::string("Restoring built-in ") + this->builtin_version_ + "...");
}

void XmosFirmwareCatalog::release_staged_image_() {
  free_image(this->staged_data_, this->staged_capacity_);
  this->staged_data_ = nullptr;
  this->staged_capacity_ = 0;
  this->staged_length_ = 0;
  this->staged_md5_.clear();
}

void XmosFirmwareCatalog::on_flasher_state_() {
  const auto state = this->flasher_->state;
  const bool ours = this->install_state_ == InstallState::FLASHING || this->install_state_ == InstallState::RECOVERING;
  if (state == FLASHER_ERASING || state == FLASHER_FLASHING) {
    if (ours)
      this->install_saw_erase_ = true;
    return;
  }
  if (state != FLASHER_SUCCESS_STATE && state != FLASHER_ERROR_STATE)
    return;
  const auto action = this->flasher_->requested_action;
  if (action == memory_flasher::ACTION_VERIFY_RECORD)
    return;
  const bool catalog_flash = this->install_state_ == InstallState::FLASHING && this->install_target_ != BUILTIN_ID;
  const auto error = this->flasher_->error_code;

  if (state == FLASHER_SUCCESS_STATE) {
    if (catalog_flash) {
      // The flasher has released the buffer. Pin before the XMOS restarts so a reboot now keeps the new image.
      this->release_staged_image_();
      uint8_t expected[5]{};
      if (parse_version(this->install_target_.c_str(), expected))
        this->set_pin_(expected, this->install_target_);
      this->install_state_ = InstallState::STARTING;
      this->install_state_since_ = millis();
      this->set_status_("Starting " + this->install_target_ + "...");
      return;
    }
    // Any other image replaces whatever was pinned.
    this->clear_pin_();
    if (this->install_state_ == InstallState::FLASHING) {
      this->finish_install_(std::string("Installed built-in ") + this->builtin_version_, true);
    } else if (this->install_state_ == InstallState::RECOVERING) {
      this->finish_install_("Failed: " + this->recovery_reason_ + "; restored built-in " + this->builtin_version_,
                            false);
    }
    return;
  }

  const std::string code = " (error " + std::to_string(static_cast<int>(error)) + ")";
  if (catalog_flash) {
    this->release_staged_image_();
    const bool erased = this->install_saw_erase_ || error == memory_flasher::WRITE_TO_FLASH_ERROR ||
                        error == memory_flasher::MD5_MISMATCH_ERROR || error == memory_flasher::CONNECTION_ERROR;
    if (erased) {
      this->start_recovery_("Flashing " + this->install_target_ + " failed" + code);
    } else {
      this->finish_install_("Failed: couldn't start flashing " + this->install_target_ + code, false);
    }
  } else if (this->install_state_ == InstallState::FLASHING) {
    this->finish_install_("Failed: flashing the built-in image failed" + code, false);
  } else if (this->install_state_ == InstallState::RECOVERING) {
    this->finish_install_("Failed: restoring the built-in image failed" + code + "; reboot to retry", false);
  }
}

void XmosFirmwareCatalog::update_install_progress_() {
  const uint32_t now = millis();
  const uint32_t elapsed = now - this->install_state_since_;
  uint8_t progress = 255;
  std::string prefix;

  switch (this->install_state_) {
    case InstallState::REQUESTED:
      if (elapsed > REQUEST_TIMEOUT_MS)
        this->fail_install_("the install did not start");
      return;
    case InstallState::STARTING: {
      uint8_t running[5]{};
      if (this->xmos_running_version_(running)) {
        this->set_pin_(running, this->install_target_);
        std::string result = "Installed " + this->install_target_;
        uint8_t expected[5]{};
        if (parse_version(this->install_target_.c_str(), expected) && !same_version(running, expected))
          result += " (XMOS reports " + format_version(running) + ")";
        this->finish_install_(result, true);
      } else if (elapsed > START_TIMEOUT_MS) {
        this->start_recovery_(this->install_target_ + " did not start");
      }
      return;
    }
    case InstallState::DOWNLOADING:
      progress = this->job_ != nullptr ? this->job_->progress.load() : 0;
      prefix = "Downloading " + this->install_target_;
      break;
    case InstallState::FLASHING:
      if (this->flasher_->state != FLASHER_ERASING && this->flasher_->state != FLASHER_FLASHING)
        return;
      progress = this->flasher_->flashing_progress;
      prefix = "Flashing " + this->target_label_();
      break;
    case InstallState::RECOVERING:
      if (this->flasher_->state != FLASHER_ERASING && this->flasher_->state != FLASHER_FLASHING)
        return;
      progress = this->flasher_->flashing_progress;
      prefix = std::string("Restoring built-in ") + this->builtin_version_;
      break;
    default:
      return;
  }

  if (progress == this->last_reported_progress_ ||
      (now - this->last_progress_ms_ < PROGRESS_PUBLISH_INTERVAL_MS && progress != 100))
    return;
  this->last_reported_progress_ = progress;
  this->last_progress_ms_ = now;
  this->set_status_(prefix + " (" + std::to_string(progress) + "%)");
}

// ---------------------------------------------------------------------------
// Lookups

const FirmwareEntry *XmosFirmwareCatalog::find_entry_(const std::string &version) const {
  uint8_t bytes[5]{};
  const bool parsed = parse_version(version.c_str(), bytes);
  for (const auto &entry : this->entries_) {
    if (version == entry.version || (parsed && same_version(entry.version_bytes, bytes)))
      return &entry;
  }
  return nullptr;
}

bool XmosFirmwareCatalog::is_builtin_request_(const std::string &version) const {
  if (strcasecmp(version.c_str(), BUILTIN_ID) == 0 || strcmp(version.c_str(), this->builtin_label_.c_str()) == 0)
    return true;
  uint8_t bytes[5]{};
  return parse_version(version.c_str(), bytes) && same_version(bytes, this->builtin_bytes_);
}

std::string XmosFirmwareCatalog::token_for_source_(size_t source_index) const {
  const FirmwareSource &source = this->sources_[source_index];
  if (source.token[0] != '\0')
    return source.token;
  return std::string(this->runtime_token_.data(), this->runtime_token_.size());
}

bool XmosFirmwareCatalog::xmos_running_version_(uint8_t version[5]) const {
  satellite1::Satellite1 *satellite = this->flasher_->get_parent();
  if (satellite == nullptr || !satellite->is_xmos_connected())
    return false;
  memcpy(version, satellite->xmos_fw_version, 5);
  return true;
}

std::string XmosFirmwareCatalog::target_label_() const {
  if (this->install_target_ == BUILTIN_ID)
    return std::string("built-in ") + this->builtin_version_;
  return this->install_target_;
}

// ---------------------------------------------------------------------------
// Persistence

void XmosFirmwareCatalog::load_pin_() {
  this->pin_pref_ = global_preferences->make_preference<PinRecord>(fnv1_hash("sat1.xmos.catalog_pin"));
  PinRecord record{};
  if (!this->pin_pref_.load(&record) || record.magic != PIN_MAGIC)
    return;
  record.label[sizeof(record.label) - 1] = '\0';
  this->pin_set_ = true;
  memcpy(this->pin_bytes_, record.version, sizeof(this->pin_bytes_));
  this->pin_label_ = record.label;
  this->flasher_->set_custom_image_pin(this->pin_bytes_);
}

void XmosFirmwareCatalog::set_pin_(const uint8_t version[5], const std::string &label) {
  this->flasher_->set_custom_image_pin(version);
  if (this->pin_set_ && same_version(this->pin_bytes_, version) && this->pin_label_ == label)
    return;
  PinRecord record{};
  record.magic = PIN_MAGIC;
  memcpy(record.version, version, sizeof(record.version));
  set_truncated(record.label, label.c_str());
  this->pin_pref_.save(&record);
  global_preferences->sync();
  this->pin_set_ = true;
  memcpy(this->pin_bytes_, version, sizeof(this->pin_bytes_));
  this->pin_label_ = label;
  this->snapshot_dirty_ = true;
  ESP_LOGI(TAG, "Boot auto-flash will keep XMOS firmware %s", format_version(version).c_str());
}

void XmosFirmwareCatalog::clear_pin_() {
  this->flasher_->clear_custom_image_pin();
  if (!this->pin_set_)
    return;
  PinRecord record{};
  this->pin_pref_.save(&record);
  global_preferences->sync();
  this->pin_set_ = false;
  this->pin_label_.clear();
  this->snapshot_dirty_ = true;
  ESP_LOGI(TAG, "Boot auto-flash follows the built-in XMOS firmware again");
}

uint32_t XmosFirmwareCatalog::sources_hash_() const {
  std::string key;
  for (const auto &source : this->sources_) {
    key += source.repo;
    key += source.is_private ? "+p" : "";
    key += source.include_prereleases ? "+pre;" : ";";
  }
  return fnv1_hash(key);
}

void XmosFirmwareCatalog::load_cache_() {
  this->cache_pref_ = global_preferences->make_preference<CatalogCache>(fnv1_hash("sat1.xmos.catalog_cache"));
  CatalogCache *cache = RAMAllocator<CatalogCache>().allocate(1);
  if (cache == nullptr)
    return;
  if (this->cache_pref_.load(cache) && cache->magic == CACHE_MAGIC && cache->sources_hash == this->sources_hash_() &&
      cache->count <= MAX_ENTRIES) {
    for (uint32_t i = 0; i < cache->count; i++) {
      FirmwareEntry &entry = cache->entries[i];
      entry.tag[sizeof(entry.tag) - 1] = '\0';
      entry.stem[sizeof(entry.stem) - 1] = '\0';
      entry.date[sizeof(entry.date) - 1] = '\0';
      if (entry.source >= this->sources_.size() || !is_url_safe(entry.tag) || !is_url_safe(entry.stem) ||
          !init_entry_version(entry, strcmp(entry.stem, GENERIC_STEM) == 0 ? entry.tag : entry.stem))
        continue;
      this->source_state_[entry.source].entries.push_back(entry);
    }
    this->rebuild_entries_();
    ESP_LOGD(TAG, "Loaded %u cached firmware entries", static_cast<unsigned>(this->entries_.size()));
  }
  RAMAllocator<CatalogCache>().deallocate(cache, 1);
}

void XmosFirmwareCatalog::save_cache_() {
  CatalogCache *cache = RAMAllocator<CatalogCache>().allocate(1);
  if (cache == nullptr)
    return;
  memset(cache, 0, sizeof(CatalogCache));
  cache->magic = CACHE_MAGIC;
  cache->sources_hash = this->sources_hash_();
  for (const auto &entry : this->entries_)
    cache->entries[cache->count++] = entry;
  this->cache_pref_.save(cache);
  global_preferences->sync();
  RAMAllocator<CatalogCache>().deallocate(cache, 1);
}

void XmosFirmwareCatalog::load_token_() {
  this->token_pref_ = global_preferences->make_preference<TokenRecord>(fnv1_hash("sat1.xmos.catalog_token"));
  TokenRecord record{};
  if (this->token_pref_.load(&record) && record.magic == TOKEN_MAGIC) {
    record.token[sizeof(record.token) - 1] = '\0';
    this->runtime_token_ = record.token;
  }
  memset(&record, 0, sizeof(record));
}

void XmosFirmwareCatalog::save_token_(const std::string &token) {
  TokenRecord record{};
  if (!token.empty()) {
    record.magic = TOKEN_MAGIC;
    set_truncated(record.token, token.c_str());
  }
  this->token_pref_.save(&record);
  global_preferences->sync();
  memset(&record, 0, sizeof(record));
}

// ---------------------------------------------------------------------------
// Reporting

void XmosFirmwareCatalog::set_status_(const std::string &status) {
  if (status == this->status_)
    return;
  this->status_ = status;
  if (this->status_sensor_ != nullptr)
    this->status_sensor_->publish_state(status);
  this->snapshot_dirty_ = true;
}

void XmosFirmwareCatalog::publish_catalog_text_() {
  if (this->catalog_sensor_ == nullptr)
    return;
  std::string text;
  if (this->entries_.empty()) {
    if (!this->list_error_.empty()) {
      text = "Unavailable: " + this->list_error_;
    } else {
      text = this->list_loaded_ ? "No firmware published" : "Not loaded yet";
    }
  } else {
    for (const auto &entry : this->entries_) {
      const char *separator = text.empty() ? "" : ", ";
      if (text.size() + strlen(separator) + strlen(entry.version) > CATALOG_TEXT_MAX - 5) {
        text += ", ...";
        break;
      }
      text += separator;
      text += entry.version;
    }
  }
  if (text.size() > CATALOG_TEXT_MAX)
    text.resize(CATALOG_TEXT_MAX);
  if (this->catalog_sensor_->has_state() && this->catalog_sensor_->get_state() == text)
    return;
  this->catalog_sensor_->publish_state(text);
}

static const char *install_state_name(InstallState state) {
  switch (state) {
    case InstallState::REQUESTED:
      return "requested";
    case InstallState::DOWNLOADING:
      return "downloading";
    case InstallState::FLASHING:
      return "flashing";
    case InstallState::STARTING:
      return "starting";
    case InstallState::RECOVERING:
      return "recovering";
    case InstallState::IDLE:
    default:
      return "idle";
  }
}

void XmosFirmwareCatalog::rebuild_snapshot_() {
  this->snapshot_dirty_ = false;
  this->snapshot_built_ = millis();

  uint8_t running[5]{};
  const bool connected = this->xmos_running_version_(running);

#ifdef USE_PSRAM
  json::SpiRamAllocator allocator;
  JsonDocument doc(&allocator);
#else
  JsonDocument doc;
#endif
  JsonObject root = doc.to<JsonObject>();
  root["state"] = install_state_name(this->install_state_);
  root["busy"] = this->busy();
  root["status"] = this->status_;
  root["target"] = this->install_target_;
  root["progress"] = this->last_reported_progress_ == 255 ? 0 : this->last_reported_progress_;
  root["refreshing"] = this->refreshing_;
  root["list_loaded"] = this->list_loaded_;
  if (!this->list_error_.empty())
    root["list_error"] = this->list_error_;
  root["builtin"] = this->builtin_version_;
  if (connected) {
    root["running"] = format_version(running);
    root["running_builtin"] = same_version(running, this->builtin_bytes_);
  }
  if (this->pin_set_)
    root["pinned"] = this->pin_label_;
  root["selected"] = this->selected_id_;
  root["token_set"] = !this->runtime_token_.empty();
  if (!this->last_result_.empty()) {
    root["last_result"] = this->last_result_;
    root["last_result_ok"] = this->last_result_ok_;
  }

  JsonArray entries = root["entries"].to<JsonArray>();
  for (const auto &entry : this->entries_) {
    JsonObject item = entries.add<JsonObject>();
    item["version"] = entry.version;
    item["tag"] = entry.tag;
    item["date"] = entry.date;
    item["prerelease"] = entry.prerelease;
    item["source"] = this->sources_[entry.source].repo;
    item["running"] = connected && same_version(running, entry.version_bytes);
  }

  JsonArray sources = root["sources"].to<JsonArray>();
  for (size_t i = 0; i < this->sources_.size(); i++) {
    JsonObject item = sources.add<JsonObject>();
    item["repo"] = this->sources_[i].repo;
    item["private"] = this->sources_[i].is_private;
    item["has_token"] = this->sources_[i].token[0] != '\0' || !this->runtime_token_.empty();
    if (this->source_state_[i].error[0] != '\0')
      item["error"] = this->source_state_[i].error;
  }

  PsramString json;
  json.resize(measureJson(doc));
  serializeJson(doc, &json[0], json.size() + 1);
  LockGuard lock(this->mutex_);
  this->snapshot_.swap(json);
}

}  // namespace xmos_firmware_catalog
}  // namespace esphome

#endif  // USE_ESP32
