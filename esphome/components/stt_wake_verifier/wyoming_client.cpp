// SPDX-License-Identifier: GPL-3.0-only
#include "wyoming_client.h"

#include <ArduinoJson.h>
#include <algorithm>
#include <cerrno>
#include <cctype>
#include <fcntl.h>
#include <memory>
#include <unistd.h>
#include <utility>
#ifdef ESP_PLATFORM
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#else
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#endif

namespace esphome::stt_wake_verifier {

bool parse_wyoming_endpoint(const std::string &value, WyomingEndpoint &endpoint) {
  endpoint = {};
  const size_t first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
    return false;
  std::string address = value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
  for (const char *prefix : {"tcp://", "wyoming://"}) {
    const std::string scheme(prefix);
    if (address.rfind(scheme, 0) == 0) {
      address.erase(0, scheme.size());
      break;
    }
  }
  const size_t colon = address.rfind(':');
  if (colon == std::string::npos || colon == 0 || colon + 1 == address.size())
    return false;
  std::string host = address.substr(0, colon);
  bool ipv6 = host.front() == '[';
  if (ipv6) {
    if (host.size() < 4 || host.back() != ']')
      return false;
    host = host.substr(1, host.size() - 2);
    if (host.find(':') == std::string::npos)
      return false;
  }
  if (host.empty() || host.size() > 253)
    return false;
  for (unsigned char c : host) {
    if (!(std::isalnum(c) || c == '.' || c == '-' || c == '_' || (ipv6 && (c == ':' || c == '%'))))
      return false;
  }
  unsigned port = 0;
  for (size_t i = colon + 1; i < address.size(); ++i) {
    unsigned char c = address[i];
    if (c < '0' || c > '9')
      return false;
    port = port * 10 + c - '0';
    if (port > 65535)
      return false;
  }
  if (!port)
    return false;
  endpoint = {std::move(host), static_cast<uint16_t>(port)};
  return true;
}

namespace {

class Exchange {
 public:
  Exchange(uint32_t started, uint32_t timeout, uint32_t (*clock)())
      : started_(started), timeout_(timeout), clock_(clock) {}
  ~Exchange() {
    if (fd_ >= 0)
      ::close(fd_);
  }
  bool expired() const { return static_cast<uint32_t>(clock_() - started_) >= timeout_; }

  bool connect(const WyomingEndpoint &endpoint) {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    addrinfo *addresses = nullptr;
    const auto port = std::to_string(endpoint.port);
    // DNS may outlast the deadline, but runs only on the worker. The main audio
    // loop independently allows the wake on timeout and ignores stale results.
    if (::getaddrinfo(endpoint.host.c_str(), port.c_str(), &hints, &addresses) != 0)
      return false;
    // lwIP exposes freeaddrinfo as a function-like macro, so invoke it through
    // a deleter rather than taking its address.
    auto release = [](addrinfo *value) { ::freeaddrinfo(value); };
    std::unique_ptr<addrinfo, decltype(release)> owner(addresses, release);
    for (auto *address = addresses; address && !expired(); address = address->ai_next) {
      fd_ = ::socket(address->ai_family, address->ai_socktype, address->ai_protocol);
      if (fd_ < 0)
        continue;
      if (fd_ >= FD_SETSIZE) {
        ::close(fd_);
        fd_ = -1;
        continue;
      }
      int flags = ::fcntl(fd_, F_GETFL, 0);
      if (flags >= 0 && ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) == 0) {
        const int yes = 1;
        ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
#ifdef SO_NOSIGPIPE
        ::setsockopt(fd_, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof(yes));
#endif
        int status = ::connect(fd_, address->ai_addr, address->ai_addrlen);
        if (status == 0)
          return true;
        if (errno == EINPROGRESS && wait_(true)) {
          int error = 0;
          socklen_t length = sizeof(error);
          if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &error, &length) == 0 && error == 0)
            return true;
        }
      }
      ::close(fd_);
      fd_ = -1;
    }
    return false;
  }

  bool write(const void *bytes, size_t size) {
    const auto *data = static_cast<const uint8_t *>(bytes);
    while (size && !expired()) {
      int flags = 0;
#ifdef MSG_NOSIGNAL
      flags = MSG_NOSIGNAL;
#endif
      const auto count = ::send(fd_, data, std::min<size_t>(size, 4096), flags);
      if (count > 0) {
        data += count;
        size -= count;
      } else if (count < 0 && errno == EINTR) {
        continue;
      } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) && wait_(true)) {
        continue;
      } else {
        return false;
      }
    }
    return size == 0;
  }
  bool write(const std::string &text) { return write(text.data(), text.size()); }

  bool read(void *bytes, size_t size) {
    auto *out = static_cast<char *>(bytes);
    while (size && !expired()) {
      if (begin_ == end_ && !fill_())
        return false;
      const size_t count = std::min(size, end_ - begin_);
      std::copy_n(buffer_ + begin_, count, out);
      begin_ += count;
      out += count;
      size -= count;
    }
    return size == 0;
  }
  bool line(std::string &text) {
    text.clear();
    char c;
    while (text.size() < 1024 && read(&c, 1)) {
      if (c == '\n')
        return true;
      text += c;
    }
    return false;
  }
  bool discard(size_t size) {
    char buffer[256];
    while (size) {
      const size_t count = std::min(size, sizeof(buffer));
      if (!read(buffer, count))
        return false;
      size -= count;
    }
    return true;
  }

 private:
  bool wait_(bool writing) {
    for (;;) {
      const uint32_t elapsed = clock_() - started_;
      if (elapsed >= timeout_)
        return false;
      fd_set ready;
      FD_ZERO(&ready);
      FD_SET(fd_, &ready);
      const uint32_t left = timeout_ - elapsed;
      timeval delay{};
      delay.tv_sec = left / 1000;
      delay.tv_usec = (left % 1000) * 1000;
      const int count = ::select(fd_ + 1, writing ? nullptr : &ready, writing ? &ready : nullptr, nullptr, &delay);
      if (count > 0)
        return true;
      if (count == 0 || errno != EINTR)
        return false;
    }
  }
  bool fill_() {
    while (wait_(false)) {
      const auto count = ::recv(fd_, buffer_, sizeof(buffer_), 0);
      if (count > 0) {
        begin_ = 0;
        end_ = count;
        return true;
      }
      if (count == 0 || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK))
        return false;
    }
    return false;
  }
  int fd_{-1};
  uint32_t started_, timeout_;
  uint32_t (*clock_)();
  char buffer_[512];
  size_t begin_{0}, end_{0};
};

bool length_field(JsonVariantConst field, size_t limit, size_t &length) {
  length = 0;
  if (field.isNull())
    return true;
  if (!field.is<size_t>())
    return false;
  length = field.as<size_t>();
  return length <= limit;
}

}  // namespace

WyomingTranscript transcribe_wyoming(const WyomingEndpoint &endpoint, const uint8_t *pcm, size_t pcm_size,
                                     uint32_t started, uint32_t timeout, uint32_t (*clock)()) {
  Exchange stream(started, timeout, clock);
  if (!stream.connect(endpoint))
    return {"", stream.expired() ? "Wyoming STT timeout" : "Wyoming STT connection failed"};
  if (!stream.write("{\"type\":\"transcribe\"}\n") ||
      !stream.write("{\"type\":\"audio-start\",\"data\":{\"rate\":16000,\"width\":2,\"channels\":1}}\n"))
    return {"", "Wyoming STT send failed"};
  for (size_t offset = 0; offset < pcm_size; offset += 4096) {
    const size_t count = std::min<size_t>(4096, pcm_size - offset);
    const std::string header = "{\"type\":\"audio-chunk\",\"data\":{\"rate\":16000,\"width\":2,\"channels\":1},"
                               "\"payload_length\":" +
                               std::to_string(count) + "}\n";
    if (!stream.write(header) || !stream.write(pcm + offset, count))
      return {"", "Wyoming STT send failed"};
  }
  if (!stream.write("{\"type\":\"audio-stop\"}\n"))
    return {"", "Wyoming STT send failed"};

  std::string partial;
  // Follow the Wyoming event framing: newline JSON header, optional JSON data
  // bytes, then optional raw payload. TCP reads may split anywhere.
  for (unsigned events = 0; events < 32 && !stream.expired(); ++events) {
    std::string line;
    if (!stream.line(line))
      break;
    JsonDocument header;
    if (deserializeJson(header, line) || !header.is<JsonObject>() || !header["type"].is<const char *>())
      return {"", "Wyoming STT invalid response"};
    size_t data_length, payload_length;
    if (!length_field(header["data_length"], 4096, data_length) ||
        !length_field(header["payload_length"], 8192, payload_length))
      return {"", "Wyoming STT response too large or invalid"};
    JsonDocument extra;
    if (data_length) {
      std::string data(data_length, '\0');
      if (!stream.read(data.data(), data_length) || deserializeJson(extra, data) || !extra.is<JsonObject>())
        return {"", "Wyoming STT invalid response data"};
    }
    if (!stream.discard(payload_length))
      break;
    const std::string type = header["type"].as<std::string>();
    if (type == "error")
      return {"", "Wyoming STT provider error"};
    if (type == "transcript-stop")
      return {partial, ""};
    if (type != "transcript" && type != "transcript-chunk")
      continue;
    const JsonVariantConst additional_text = extra["text"];
    const JsonVariantConst inline_text = header["data"]["text"];
    const JsonVariantConst text = additional_text.isUnbound() ? inline_text : additional_text;
    if (!text.is<const char *>() || text.as<std::string>().size() > 255)
      return {"", "Wyoming STT invalid transcript"};
    partial = text.as<std::string>();  // Streaming chunks replace, rather than append, the current text.
    if (type == "transcript")
      return {partial, ""};
  }
  return {"", stream.expired() ? "Wyoming STT timeout" : "Wyoming STT response incomplete"};
}

}  // namespace esphome::stt_wake_verifier
