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
#include "snapcast_client.h"
#include "messages.h"

#include "esphome/core/log.h"

#include "esp_transport.h"
#include "esp_transport_tcp.h"

#include "esp_mac.h"
#include "mdns.h"
#include <cstring> 

namespace esphome {
namespace snapcast {

static const char *const TAG = "snapcast_client";


void SnapcastClient::setup(){
    this->network_initialized_ = false;
}

void SnapcastClient::loop(){
  if( !this->enabled_){
    return;
  }
  if (!this->network_initialized_ && network::is_connected()) {
    // Perform network setup once connected to wifi
    this->on_network_ready_();
    this->network_initialized_ = true;
  } 
  else if (this->network_initialized_ && this->stream_.is_destroyed() && this->cfg_server_ip_.empty())
  {
    if( millis() > this->mdns_last_scan_ + this->mdns_scan_interval_ms_ ){
        this->mdns_scan_connect_();
    }
  }
}

void SnapcastClient::on_network_ready_(){
    this->client_id_ = get_mac_address_pretty();
    this->cntrl_session_.client_id_ = this->client_id_;
    
    if(this->cfg_server_ip_.empty()){
        this->mdns_scan_connect_();
    } else {
        //use provided server ip instead
        this->connect_to_server( this->cfg_server_ip_ );    
    }
    
    //callback on status changes, received from the snapcast (binary) stream
    this->stream_.set_on_status_update_callback([this](StreamState state, uint8_t volume, bool muted){
        this->defer([this, state, volume, muted](){
           this->on_stream_state_update(state, volume, muted);
        });
        
    });

    //callback on status changes, received from snapcast control (RPC) stream 
    this->cntrl_session_.set_on_stream_update([this](const StreamInfo &info) {
        this->defer([this, info](){ this->on_stream_update_msg(info); });
    });
}


error_t SnapcastClient::connect_to_server(std::string url, uint32_t stream_port, uint32_t rpc_port){
    // establish a binary stream connection to the snapcast server, MA only shows connected clients as players
    this->stream_.connect(url, stream_port);
    // register for snapcast control events, used to control the media player component
    this->cntrl_session_.connect(url, rpc_port);
    
    this->curr_server_url_.server_ip = url;
    this->curr_server_url_.stream_port = stream_port;
    this->curr_server_url_.rpc_port = rpc_port;
    return ESP_OK;
}

void SnapcastClient::report_volume(float volume, bool muted){
    if( this->stream_.is_connected()){
        uint8_t volume_percent = int(volume * 100. + 0.5);
        volume_percent = volume_percent > 100 ? 100 : volume_percent;
        this->stream_.report_volume(volume_percent, muted);
    }
}


void SnapcastClient::on_stream_state_update(StreamState state, uint8_t volume, bool muted){
    ESP_LOGD( TAG, "Stream component changed to state %d.", state );
    if( state == StreamState::ERROR || state == StreamState::RECONNECTING ){
        ESP_LOGE(TAG, "stream: %s", this->stream_.error_msg_.c_str() );
        if( state == StreamState::RECONNECTING ){
            ESP_LOGI(TAG, "Reconnecting after error...");
        } else {
            this->stream_.disconnect();
            this->cntrl_session_.disconnect();
        }
        return;
    } 
    if (this->media_player_ != nullptr && volume >= 0 && volume <= 100) {
        this->media_player_->make_call()
            .set_volume( volume / 100.)
            .set_command( muted ? media_player::MediaPlayerCommand::MEDIA_PLAYER_COMMAND_MUTE : media_player::MediaPlayerCommand::MEDIA_PLAYER_COMMAND_UNMUTE)
            .perform();
    }
}

void SnapcastClient::on_stream_update_msg(const StreamInfo &info){
  ESP_LOGI(TAG, "Snapcast-stream updated: status=%s", info.status.c_str());
  static std::string last_state = "unknown";  
  if (this->media_player_ != nullptr) {
    if (info.status == "playing" && last_state != "playing") {
        ESP_LOGI(TAG, "Playing stream: %s\n", info.id.c_str());
        this->curr_server_url_.stream_name = info.id;
        this->media_player_->make_call()
            .set_media_url( this->curr_server_url_.to_str() )
            .perform();
        last_state = "playing";
    }
    else if ( info.status != "playing" && last_state == "playing" ) {
      this->media_player_->make_call().set_command(media_player::MediaPlayerCommand::MEDIA_PLAYER_COMMAND_STOP).perform();
      ESP_LOGI(TAG, "Stopping stream: %s\n", info.id.c_str());
      last_state = info.status;
    } 
  }
}




static const char * if_str[] = {"STA", "AP", "ETH", "MAX"};
static const char * ip_protocol_str[] = {"V4", "V6", "MAX"};

//https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/api-reference/protocols/mdns.html
static void mdns_print_results(mdns_result_t *results)
{
    mdns_result_t *r = results;
    mdns_ip_addr_t *a = nullptr;
    int i = 1, t;
    while (r) {
        if (r->esp_netif) {
            printf("%d: Interface: %s, Type: %s, TTL: %" PRIu32 "\n", i++, esp_netif_get_ifkey(r->esp_netif),
                   ip_protocol_str[r->ip_protocol], r->ttl);
        }
        if (r->instance_name) {
            printf("  PTR : %s.%s.%s\n", r->instance_name, r->service_type, r->proto);
        }
        if (r->hostname) {
            printf("  SRV : %s.local:%u\n", r->hostname, r->port);
        }
        if (r->txt_count) {
            printf("  TXT : [%zu] ", r->txt_count);
            for (t = 0; t < r->txt_count; t++) {
                printf("%s=%s(%d); ", r->txt[t].key, r->txt[t].value ? r->txt[t].value : "NULL", r->txt_value_len[t]);
            }
            printf("\n");
        }
        a = r->addr;
        while (a) {
            if (a->addr.type == ESP_IPADDR_TYPE_V6) {
                printf("  AAAA: " IPV6STR "\n", IPV62STR(a->addr.u_addr.ip6));
            } else {
                printf("  A   : " IPSTR "\n", IP2STR(&(a->addr.u_addr.ip4)));
            }
            a = a->next;
        }
        r = r->next;
    }
}


static bool test_tcp_connect(const esp_ip4_addr_t &ip, uint16_t port, uint32_t timeout_ms)
{
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        return false;
    }

    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in dest = {};
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(port);
    dest.sin_addr.s_addr = ip.addr;   // esp_ip4_addr_t has .addr

    bool ok = (connect(sock, (struct sockaddr*)&dest, sizeof(dest)) == 0);
    close(sock);
    return ok;
}


std::string resolve_mdns_host(const char * host_name)
{
    esp_ip4_addr_t addr;
    addr.addr = 0;

    esp_err_t err = mdns_query_a(host_name, 2000,  &addr);
    if(err){
        if(err == ESP_ERR_NOT_FOUND){
            return "";
        }
        return "";
    }
    char buffer[16];  // Enough for "255.255.255.255\0"
    snprintf(buffer, sizeof(buffer), IPSTR, IP2STR(&addr));
    return std::string(buffer);
}

error_t SnapcastClient::mdns_task_()
{
    this->mdns_last_scan_ = millis();
    mdns_result_t *results = nullptr;

    // PTR query: _snapcast._tcp.local
    esp_err_t err = mdns_query_ptr("_snapcast", "_tcp", 6000, 20, &results);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mDNS query failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    if (!results) {
        ESP_LOGW(TAG, "No Snapcast server found via mDNS!");
        return ESP_FAIL;
    }

#if SNAPCAST_DEBUG    
    mdns_print_results(results);
#endif    

    std::string chosen_hostname;
    std::string chosen_ip;
    uint32_t chosen_port = 0;
    bool found_mass = false;

    for (mdns_result_t *r = results; r != nullptr; r = r->next) {

        // If we already found a reachable MA server, we can break early
        if (found_mass) {
            break;
        }

        // Snapcast advertised port
        uint16_t port = r->port;
    
        for (mdns_ip_addr_t *a = r->addr; a != nullptr; a = a->next) {
            if (a->addr.type != ESP_IPADDR_TYPE_V4) {
                continue;
            }

            esp_ip4_addr_t ip4 = a->addr.u_addr.ip4;

            if (test_tcp_connect(ip4, port, 1000)) {
                char buf[16];
                snprintf(buf, sizeof(buf), IPSTR, IP2STR(&ip4));
                chosen_ip       = buf;
                chosen_hostname = std::string(r->hostname) + ".local";
                chosen_port     = port;

                // TXT handling stays the same
                if (r->txt_count) {
                    for (int t = 0; t < r->txt_count; t++) {
                        if (strcmp(r->txt[t].key, "is_mass") == 0) {
                            found_mass = true;
                            break;
                        }
                    }
                }
                break; // found a reachable IP for this result
            }
        }
    }

    mdns_query_results_free(results);

    if (!chosen_ip.empty()) {
        ESP_LOGI(TAG, "Snapcast server found: %s:%d", chosen_hostname.c_str(), chosen_port);
        ESP_LOGI(TAG, "resolved reachable IP: %s:%d", chosen_ip.c_str(), chosen_port);
        this->connect_to_server(chosen_ip, chosen_port, 1705);
        return ESP_OK;
    }

    ESP_LOGD(TAG, "Couldn't find any reachable Snapcast server via mDNS.");
    return ESP_FAIL;
}

error_t SnapcastClient::mdns_scan_connect_(){
    //start mDNS task and search for the MA Snapcast server
    if (this->mdns_task_handle_ == nullptr) {
        xTaskCreate([](void *param) {
        auto *client = static_cast<SnapcastClient *>(param);
        client->mdns_task_();
        client->mdns_task_handle_ = nullptr;
        vTaskDelete(nullptr);
        }, "snap_mdns_task", 4096, this, 5, &this->mdns_task_handle_);
    }
    return ESP_OK;
}


}
}