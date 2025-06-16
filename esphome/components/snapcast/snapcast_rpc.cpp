#include "snapcast_rpc.h"
#include "esp_transport_tcp.h"

#include "esphome/core/log.h"
#include "esphome/components/network/util.h"


namespace esphome {
namespace snapcast {

static const char *const TAG = "snapcast_rpc";

esp_err_t SnapcastControlSession::connect(std::string server, uint32_t port){
    this->server_ = server;
    this->port_ = port;
    
    // Start the notification handling task
    this->notification_task_should_run_ = true;
    xTaskCreate([](void *param) {
        auto *session = static_cast<SnapcastControlSession *>(param);
        session->notification_loop();
        vTaskDelete(nullptr);
    }, "snapcast_notify", 4096, this, 5, &this->notification_task_handle_);

    return ESP_OK;
}

esp_err_t SnapcastControlSession::disconnect(){
    if( this->transport_ != nullptr ){
        esp_transport_close(this->transport_);
        esp_transport_destroy(this->transport_);
        this->transport_ = nullptr;
        this->notification_task_should_run_ = false;
        vTaskDelay(pdMS_TO_TICKS(50));

        // Then delete the task if still running
        if (this->notification_task_handle_ != nullptr) {
            vTaskDelete(this->notification_task_handle_);
            this->notification_task_handle_ = nullptr;
        }
    }
    return ESP_OK;
}


void SnapcastControlSession::update_from_server_obj_(const JsonObject &server_obj){
    ClientState &state = this->client_state_;
    if( state.from_groups_json( server_obj["groups"], get_mac_address_pretty()) ){
        printf( "group_id: %s stream_id: %s\n", state.group_id.c_str(), state.stream_id.c_str() );
    }
    StreamInfo sInfo;
    if( sInfo.from_streams_json( server_obj["streams"], state.stream_id )){
        printf( "stream: %s state: %s\n", sInfo.id.c_str(), sInfo.status.c_str() );
        if (this->on_stream_update_) {
            printf( "Calling on_stream_update callback\n" );
            this->on_stream_update_(sInfo);
        }
    }
}

void SnapcastControlSession::notification_loop() {
  
  // Initialize transport
  if( this->transport_ != nullptr ){
    esp_transport_close(this->transport_);
    esp_transport_destroy(this->transport_);
  }
  this->transport_ = esp_transport_tcp_init();
  
  if (this->transport_ == nullptr) {
    ESP_LOGE(TAG, "Failed to initialize transport");
    return;
  }
  esp_transport_keep_alive_t keep_alive_config = {
        .keep_alive_enable = true,
        .keep_alive_idle = 10000, // 10 seconds
        .keep_alive_interval = 5000, // 5 seconds
        .keep_alive_count = 5, // Number of keep-alive probes to send before considering the connection dead
    };
  esp_transport_tcp_set_keep_alive(this->transport_, &keep_alive_config);
  // Try to connect
  error_t err = esp_transport_connect(this->transport_, this->server_.c_str(), this->port_, -1);
  if (err != 0) {
    ESP_LOGE(TAG, "Connection failed with error: %d", errno);
    return;
  }

//   // Initialize transport
//   if( this->transport_ != nullptr ){
//     esp_transport_close(this->transport_);
//     esp_transport_destroy(this->transport_);
//   }

  vTaskDelay(pdMS_TO_TICKS(1000));
  //return;

#if 0
  while(true){
    // char chunk[128];  // small read buffer
    // int len = esp_transport_read(this->transport_, chunk, sizeof(chunk), 100);
    // if( len < 0 ){
    //     printf( "Error code while reading from transport stream: %d", len );
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     continue;
    // }
    vTaskDelay(pdMS_TO_TICKS(10000));  
  }

  // Initialize transport
  if( this->transport_ != nullptr ){
    esp_transport_close(this->transport_);
    esp_transport_destroy(this->transport_);
  }

  vTaskDelay(pdMS_TO_TICKS(1000));
  return;
#endif
  // Send initial request after connecting
  this->send_rpc_request_("Server.GetStatus",
    [](JsonObject params) {
      // no params
    },
    static_cast<uint32_t>(RequestId::GetServerStatus)
  );  

  while (this->notification_task_should_run_) {
    char chunk[128];  // small read buffer
    int len = esp_transport_read(this->transport_, chunk, sizeof(chunk), 100);
    if( len < 0 ){
        printf( "Error code while reading from transport stream: %d", len );
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;   
    }
    if (len > 0) {
        this->recv_buffer_.append(chunk, len);
        size_t pos;
        while ((pos = this->recv_buffer_.find('\n')) != std::string::npos) {
            std::string json_line = this->recv_buffer_.substr(0, pos);
            printf( "JSON: %s\n", json_line.c_str() );
            this->recv_buffer_.erase(0, pos + 1);

            json::parse_json(json_line, [this](JsonObject root) -> bool {
                if(root.containsKey("result")){
                   uint32_t id = root["id"]; 
                   switch (static_cast<RequestId>(id)) {
                        case RequestId::GetServerStatus:
                            {
                                ClientState &state = this->client_state_;
                                if( state.from_groups_json( root["result"]["server"]["groups"], get_mac_address_pretty()) ){
                                    printf( "group_id: %s stream_id: %s\n", state.group_id.c_str(), state.stream_id.c_str() );
                                }
                                StreamInfo sInfo;
                                if( sInfo.from_streams_json( root["result"]["server"]["streams"], state.stream_id )){
                                    printf( "stream: %s state: %s\n", sInfo.id.c_str(), sInfo.status.c_str() );
                                }
                                this->update_from_server_obj_(root["result"]["server"].as<JsonObject>());
                            }
                            break;
                        default:
                            ESP_LOGW(TAG, "Unknown request ID: %u", id);
                    }           

                } else if(root.containsKey("method")) {
                    std::string method = root["method"].as<std::string>();    
                    if (method == "Server.OnUpdate" ){
                        this->update_from_server_obj_(root["params"]["server"].as<JsonObject>());
                    } else if( method == "Stream.OnUpdate"){
                       
                       JsonObject params = root["params"];
                       if(params["id"].as<std::string>() == this->client_state_.stream_id){
                            StreamInfo sInfo;
                            if( sInfo.from_json( params["stream"])){
                                printf( "stream: %s state: %s\n", sInfo.id.c_str(), sInfo.status.c_str() );
                            }
                            if (this->on_stream_update_) {
                                printf( "Calling on_stream_update callback from OnUpdate message\n" ); 
                                this->on_stream_update_(sInfo);
                            }
                       } else {
                        printf( "got id: %s, requested: %s\n", params["id"].as<std::string>().c_str(), this->client_state_.stream_id);
                       }
                    } else if (method == "Stream.OnProperties"){
                       JsonObject params = root["params"];
                       if(params["id"].as<std::string>() == this->client_state_.stream_id){
                            StreamInfo sInfo;
                            if( sInfo.from_stream_properties( params["properties"])){
                                printf( "stream: %s state: %s\n", sInfo.id.c_str(), sInfo.status.c_str() );
                            }
                            if (this->on_stream_update_) {
                                printf( "Calling on_stream_update callback from OnProperties message\n" ); 
                                 this->on_stream_update_(sInfo);
                            }
                        }
                    } else if (method == "Group.OnStreamChanged"){
                       JsonObject params = root["params"]; 
                       if(params["id"].as<std::string>() == this->client_state_.group_id){
                            if(this->client_state_.stream_id != params["stream_id"].as<std::string>()){
                                this->send_rpc_request_("Server.GetStatus",
                                    [](JsonObject params) {
                                    // no params
                                    },
                                    static_cast<uint32_t>(RequestId::GetServerStatus)
                                );  
                         }
                       }
                    }   
                }
                return true;
            });
        }
    }
  }
}





void SnapcastControlSession::send_rpc_request_(const std::string &method, std::function<void(JsonObject)> fill_params, uint32_t id) {
  StaticJsonDocument<512> doc;
  doc["jsonrpc"] = "2.0";
  doc["id"] = id;
  doc["method"] = method;
  JsonObject params = doc.createNestedObject("params");
  fill_params(params);

  char json_buf[512];
  size_t len = serializeJson(doc, json_buf, sizeof(json_buf));
  json_buf[len++] = '\n';
  esp_transport_write(this->transport_, json_buf, len, 1000);
}


}
}


  