
/*
 * This file is part of Snapcast integration for ESPHome.
 *
 * Copyright (C) 2025 Mischa Siekmann <FutureProofHomes Inc.>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

#include <string>
#include <optional>

namespace esphome {
namespace snapcast {

struct SnapcastUrl {
    std::string server_ip;
    std::optional<int> stream_port;
    std::optional<std::string> stream_name;
    std::optional<int> rpc_port;

    // Converts the SnapcastUrl back into a Snapcast-formatted URL string
    std::string to_str() const;
};

/**
 * Parses a Snapcast-style URL in the following format:
 *
 *   snapcast://SERVER_IP[:STREAM_PORT][/STREAM_NAME][?RPC_PORT=1705]
 *
 * Components:
 *   - SERVER_IP     : Required, IP address or hostname of the server
 *   - STREAM_PORT   : Optional, port number for the stream
 *   - STREAM_NAME   : Optional, name of the stream
 *   - RPC_PORT      : Optional, specified as a query parameter (e.g., ?RPC_PORT=1705)
 *
 * Example valid URLs:
 *   snapcast://192.168.1.100
 *   snapcast://192.168.1.100:4953
 *   snapcast://192.168.1.100/music
 *   snapcast://192.168.1.100:4953/music
 *   snapcast://192.168.1.100:4953/music?RPC_PORT=1705
 *
 * Returns:
 *   std::optional<SnapcastUrl> – parsed result if valid, std::nullopt otherwise
 */
std::optional<SnapcastUrl> parseSnapcastUrl(const std::string& url);

}
}