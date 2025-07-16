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
#include "snapcast_schema.h"
#include <sstream>
#include <string>
#include <cctype>
#include <iomanip>

namespace esphome {
namespace snapcast {

std::string uri_encode(const std::string &str) {
    std::ostringstream escaped;
    escaped.fill('0');
    escaped << std::hex;

    for (char c : str) {
        // Keep alphanumerics and a few safe characters as-is
        if (isalnum(static_cast<unsigned char>(c)) ||
            c == '-' || c == '_' || c == '.' || c == '~') {
            escaped << c;
        } else {
            // Percent-encode all others
            escaped << '%' << std::uppercase << std::setw(2)
                    << int(static_cast<unsigned char>(c));
        }
    }

    return escaped.str();
}


std::string SnapcastUrl::to_str() const {
        std::ostringstream oss;
        oss << "snapcast://" << server_ip;

        if (stream_port) {
            oss << ":" << *stream_port;
        }

        if (stream_name) {
            oss << "/" << uri_encode(*stream_name);
        }

        if (rpc_port) {
            oss << "?RPC_PORT=" << *rpc_port;
        }
        return oss.str();
}


std::optional<SnapcastUrl> parseSnapcastUrl(const std::string& url) {
    const std::string prefix = "snapcast://";
    if (url.substr(0, prefix.size()) != prefix) {
        return std::nullopt;
    }

    SnapcastUrl result;
    size_t pos = prefix.size();

    // Find position of '/', '?', and ':' after prefix
    size_t slash_pos = url.find('/', pos);
    size_t question_pos = url.find('?', pos);
    size_t colon_pos = url.find(':', pos);

    size_t end_pos = std::min(
        slash_pos != std::string::npos ? slash_pos : url.size(),
        colon_pos != std::string::npos ? colon_pos : url.size()
    );

    // If no colon or slash before query, whole part is server_ip
    result.server_ip = url.substr(pos, end_pos - pos);
    pos = end_pos;

    // Optional stream_port
    if (colon_pos != std::string::npos && colon_pos < slash_pos) {
        size_t port_end = (slash_pos != std::string::npos) ? slash_pos : question_pos;
        result.stream_port = std::stoi(url.substr(colon_pos + 1, port_end - colon_pos - 1));
        pos = colon_pos + 1;
        pos = (slash_pos != std::string::npos) ? slash_pos : port_end;
    }

    // Optional stream_name
    if (slash_pos != std::string::npos && slash_pos < question_pos) {
        size_t name_end = (question_pos != std::string::npos) ? question_pos : url.size();
        result.stream_name = url.substr(slash_pos + 1, name_end - slash_pos - 1);
        pos = name_end;
    }

    // Optional RPC_PORT
    if (question_pos != std::string::npos) {
        std::string query = url.substr(question_pos + 1);
        const std::string key = "RPC_PORT=";
        size_t key_pos = query.find(key);
        if (key_pos != std::string::npos) {
            result.rpc_port = std::stoi(query.substr(key_pos + key.size()));
        }
    }

    return result;
}


}
}