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
    this->client_id_ = get_mac_address_pretty();
    this->cntrl_session_.client_id_ = this->client_id_;
    
    if(this->server_ip_.empty()){
        //start mDNS task and search for the MA Snapcast server
        if (this->mdns_task_handle_ == nullptr) {
            xTaskCreate([](void *param) {
            auto *client = static_cast<SnapcastClient *>(param);
            client->connect_via_mdns();
            vTaskDelete(nullptr);
            }, "snap_mdns_task", 4096, this, 5, &this->mdns_task_handle_);
        }
    } else {
        //use provided server ip instead
        this->connect_to_server( this->server_ip_ );    
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
    if( state == StreamState::ERROR || state == StreamState::DISCONNECTED ){
        ESP_LOGE(TAG, "stream: %s", this->stream_.error_msg_.c_str() );
        if( this->stream_.reconnect_on_error_ ){
            ESP_LOGI(TAG, "Reconnecting after error...");
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


error_t SnapcastClient::connect_via_mdns(){
    // search for the snapcast server that has `is_mass` in its .txt entries 
    mdns_result_t * results = nullptr;
    esp_err_t err = mdns_query_ptr( "_snapcast", "_tcp", 6000, 20,  &results);
    if(err){
        ESP_LOGE(TAG, "mDNS query Failed");
        return ESP_OK;
    }
    if(!results){
        ESP_LOGW(TAG, "No Snapcast server found via mDNS!");
        return ESP_OK;
    }

#if SNAPCAST_DEBUG    
    mdns_print_results(results);
#endif    
    
    std::string ma_snapcast_hostname;
    std::string ma_snapcast_ip;
    uint32_t port = 0;
    mdns_result_t *r = results;
    while(r){
        std::string check_ip;
        if( r->addr && r->addr->addr.type == ESP_IPADDR_TYPE_V4 ){
            mdns_ip_addr_t *a = r->addr;
            char buffer[16];  // Enough for "255.255.255.255\0"
            snprintf(buffer, sizeof(buffer), IPSTR, IP2STR(&(a->addr.u_addr.ip4)));
            check_ip = std::string(buffer);   
        } else {
            check_ip = resolve_mdns_host( r->hostname);
        }
        if(!check_ip.empty()){
            ma_snapcast_ip = check_ip;
            ma_snapcast_hostname = std::string(r->hostname) + ".local";
            port = r->port;
        }
        if (r->txt_count) {
            for (int t = 0; t < r->txt_count; t++) {
                if( strcmp(r->txt[t].key, "is_mass") == 0){
                    // if music assistant snapcast server is among found servers, take it
                    if(!check_ip.empty()) break;
                }
            }
        }
        r = r->next;
    }
    mdns_query_results_free(results);

    if( !ma_snapcast_ip.empty() ){
        ESP_LOGI(TAG, "MA-Snapcast server found: %s:%d", ma_snapcast_hostname.c_str(), port );
        ESP_LOGI(TAG, "resolved: %s:%d\n", ma_snapcast_ip.c_str(), port );
        
        this->connect_to_server( ma_snapcast_ip, port, 1705);
        return ESP_OK;
    }
    ESP_LOGW(TAG, "Couldn't find MA-Snapcast server.");
    return ESP_FAIL;
}


}
}