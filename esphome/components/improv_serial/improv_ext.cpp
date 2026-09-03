#include "improv_ext.h"

#include <cstring>

namespace manufacturer_improv_ext {

ImprovCommand parse_improv_data(const uint8_t *data, size_t length, bool check_checksum) {
  ImprovCommand improv_command;
  const size_t checksum_length = check_checksum ? 1 : 0;
  if (data == nullptr || length < 2 + checksum_length) {
    improv_command.command = UNKNOWN;
    return improv_command;
  }

  const Command command = static_cast<Command>(data[0]);
  const uint8_t data_length = data[1];
  const size_t payload_end = length - checksum_length;
  if (data_length != payload_end - 2) {
    improv_command.command = UNKNOWN;
    return improv_command;
  }

  if (check_checksum) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length - 1; i++)
      checksum += data[i];
    if (checksum != data[length - 1]) {
      improv_command.command = BAD_CHECKSUM;
      return improv_command;
    }
  }

  if (command == WIFI_SETTINGS || command == TRIGGER_ACTION) {
    if (payload_end < 3) {
      improv_command.command = UNKNOWN;
      return improv_command;
    }
    const size_t first_start = 3;
    const size_t first_end = first_start + data[2];
    if (first_end > payload_end || (command == WIFI_SETTINGS && first_end >= payload_end)) {
      improv_command.command = UNKNOWN;
      return improv_command;
    }
    if (command == TRIGGER_ACTION && first_end == payload_end) {
      improv_command.command = command;
      improv_command.ssid.assign(reinterpret_cast<const char *>(data + first_start), first_end - first_start);
      return improv_command;
    }
    const size_t second_start = first_end + 1;
    const size_t second_end = second_start + data[first_end];
    if (second_end != payload_end) {
      improv_command.command = UNKNOWN;
      return improv_command;
    }
    improv_command.command = command;
    improv_command.ssid.assign(reinterpret_cast<const char *>(data + first_start), first_end - first_start);
    improv_command.password.assign(reinterpret_cast<const char *>(data + second_start), second_end - second_start);
    return improv_command;
  }

  improv_command.command = command;
  return improv_command;
}

bool parse_improv_serial_byte(size_t position, uint8_t byte, const uint8_t *buffer,
                              std::function<bool(ImprovCommand)> &&callback, std::function<void(Error)> &&on_error) {
  static const char header[] = "IMPROV";
  if (position < 6)
    return byte == header[position];
  if (position == 6)
    return byte == IMPROV_SERIAL_VERSION;
  if (position <= 8 + buffer[8])
    return true;
  if (position == 9 + buffer[8]) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < position; i++)
      checksum += buffer[i];
    if (checksum != byte) {
      on_error(ERROR_INVALID_RPC);
      return false;
    }
    if (buffer[7] == TYPE_RPC)
      return callback(parse_improv_data(&buffer[9], buffer[8], false));
  }
  return false;
}

std::vector<uint8_t> build_rpc_response(Command command, const std::vector<std::string> &datum, bool add_checksum) {
  size_t data_length = 0;
  for (const auto &item : datum) {
    if (item.length() > UINT8_MAX - 4 || data_length > UINT8_MAX - 4 - item.length())
      return {};
    data_length += 1 + item.length();
  }
  std::vector<uint8_t> out(3 + data_length, 0);
  out[0] = command;
  out[1] = static_cast<uint8_t>(data_length);
  size_t pos = 2;
  for (const auto &item : datum) {
    out[pos++] = static_cast<uint8_t>(item.length());
    std::memcpy(out.data() + pos, item.data(), item.length());
    pos += item.length();
  }
  if (add_checksum) {
    uint8_t checksum = 0;
    for (uint8_t byte : out)
      checksum += byte;
    out.back() = checksum;
  }
  return out;
}

}  // namespace manufacturer_improv_ext
