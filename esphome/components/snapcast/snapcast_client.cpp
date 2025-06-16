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
    if(this->server_ip_.empty()){
        //this->connect_via_mdns();
        if (this->mdns_task_handle_ == nullptr) {
            xTaskCreate([](void *param) {
            auto *client = static_cast<SnapcastClient *>(param);
            client->connect_via_mdns();  // private task function
            vTaskDelete(nullptr);
            }, "snap_mdns_task", 4096, this, 5, &this->mdns_task_handle_);
        }
    } else {
        this->stream_.connect( this->server_ip_, 1704 );
        this->cntrl_session_.connect( this->server_ip_, 1705);
    }
    
    this->stream_.set_on_status_update_callback([this](StreamState state, uint8_t volume, bool muted){
        this->on_stream_state_update(state, volume, muted);
    });

    this->cntrl_session_.set_on_stream_update([this](const StreamInfo &info) {
        this->on_stream_update_msg(info);
    });

}


error_t SnapcastClient::connect_to_server(std::string url, uint32_t port){
    return ESP_OK;
}

void SnapcastClient::report_volume(float volume, bool muted){
    if( this->stream_.is_connected()){
        uint8_t volume_percent = int(volume * 100. + 0.5);
        volume_percent = volume_percent > 100 ? 100 : volume_percent;
        this->stream_.report_volume(volume_percent, muted);
    }
}


void SnapcastClient::on_stream_update_msg(const StreamInfo &info){
  ESP_LOGI(TAG, "Stream updated: status=%s", info.status.c_str());

  if (this->media_player_ != nullptr) {
    if (info.status == "playing" && !this->stream_.is_running()) {
      this->media_player_->play_snapcast_stream("bla");
      printf("Playing stream: %s\n", info.id.c_str());
    }
    else if ( info.status != "playing" && this->stream_.is_running() ) {
      this->media_player_->make_call().set_command(media_player::MediaPlayerCommand::MEDIA_PLAYER_COMMAND_STOP).perform();
      this->stream_.stop_streaming();
      printf("Stopping steram\n");
    } 
/*    
    else if (info.status == "paused") {
      this->media_player_->pause();
    } else if (info.status == "idle"){
       this->media_player_->stop(); 
    }
*/
  }
}

void SnapcastClient::on_stream_state_update(StreamState state, uint8_t volume, bool muted){
    if (this->media_player_ != nullptr) {
        this->media_player_->make_call()
            .set_volume( volume / 100.)
            .set_command( muted ? media_player::MediaPlayerCommand::MEDIA_PLAYER_COMMAND_MUTE : media_player::MediaPlayerCommand::MEDIA_PLAYER_COMMAND_UNMUTE)
            .perform();
    }
}



static const char * if_str[] = {"STA", "AP", "ETH", "MAX"};
static const char * ip_protocol_str[] = {"V4", "V6", "MAX"};

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
    printf("Query A: %s.local\n", host_name);

    esp_ip4_addr_t addr;
    addr.addr = 0;

    esp_err_t err = mdns_query_a(host_name, 2000,  &addr);
    if(err){
        if(err == ESP_ERR_NOT_FOUND){
            printf("Host was not found!");
            return "";
        }
        printf("Query Failed");
        return "";
    }
    char buffer[16];  // Enough for "255.255.255.255\0"
    snprintf(buffer, sizeof(buffer), IPSTR, IP2STR(&addr));
    return std::string(buffer);
}


error_t SnapcastClient::connect_via_mdns(){
    
    mdns_result_t * results = nullptr;
    esp_err_t err = mdns_query_ptr( "_snapcast", "_tcp", 6000, 20,  &results);
    if(err){
        ESP_LOGE(TAG, "Query Failed");
        return ESP_OK;
    }
    if(!results){
        ESP_LOGW(TAG, "No results found!");
        return ESP_OK;
    }
    
    mdns_print_results(results);
    
    std::string ma_snapcast_hostname;
    std::string ma_snapcast_ip;
    uint32_t port = 0;
    mdns_result_t *r = results;
    while(r){
        if (r->txt_count) {
            for (int t = 0; t < r->txt_count; t++) {
                if( strcmp(r->txt[t].key, "is_mass") == 0){
                    ma_snapcast_hostname = std::string(r->hostname) + ".local";
                    port = r->port;
                    if( r->addr && r->addr->addr.type == ESP_IPADDR_TYPE_V4 ){
                         mdns_ip_addr_t *a = r->addr;
                         char buffer[16];  // Enough for "255.255.255.255\0"
                         snprintf(buffer, sizeof(buffer), IPSTR, IP2STR(&(a->addr.u_addr.ip4)));
                         ma_snapcast_ip = std::string(buffer);   
                    } else {
                        ma_snapcast_ip = resolve_mdns_host( r->hostname);
                    }
                    ESP_LOGI(TAG, "MA-Snapcast server found: %s:%d", ma_snapcast_hostname.c_str(), port );
                    ESP_LOGI(TAG, "resolved: %s:%d\n", ma_snapcast_ip.c_str(), port );
                }
            }
        }
        r = r->next;
    }
    mdns_query_results_free(results);

    if( !ma_snapcast_ip.empty() ){
        this->stream_.connect( ma_snapcast_ip, port );
        this->cntrl_session_.connect( ma_snapcast_ip, 1705);
        return ESP_OK;
    }
    ESP_LOGW(TAG, "Couldn't find MA-Snapcast server.");
    return ESP_FAIL;
}





}
}