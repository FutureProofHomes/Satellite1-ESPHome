#pragma once

#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/components/json/json_util.h"
#include "esphome/components/network/util.h"
#include "esphome/core/hal.h"

#include <cstdint>
#include <cstring>
#include <string>

namespace esphome {
namespace snapcast {

enum class message_type : uint8_t
{
    kBase = 0,
    kCodecHeader = 1,
    kWireChunk = 2,
    kServerSettings = 3,
    kTime = 4,
    kHello = 5,
    kStreamTags = 6,
    kClientInfo = 7,

    kFirst = kBase,
    kLast = kClientInfo
};

#pragma pack(push, 1)  // Prevent padding
/// Time value
struct tv
{
    /// seconds
    int32_t sec;
    /// micro seconds
    int32_t usec;
    
    /// c'tor
    tv() : sec(0), usec(0) {}
    /// C'tor, construct from timeval @p tv
    explicit tv(timeval tv) : sec(tv.tv_sec), usec(tv.tv_usec){};
    /// C'tor, construct from @p _sec and @p _usec
    tv(int32_t _sec, int32_t _usec) : sec(_sec), usec(_usec){};
    
    static tv now()
    {
        tv result;
        uint32_t usec_now = micros();
        result.sec = usec_now / 1000000;
        result.usec = usec_now % 1000000;
        return result;
    }
    
    /// add another tv
    tv operator+(const tv& other) const
    {
        tv result(*this);
        result.sec += other.sec;
        result.usec += other.usec;
        if (result.usec > 1000000)
        {
            result.sec += result.usec / 1000000;
            result.usec %= 1000000;
        }
        return result;
    }

    /// subtract another tv
    tv operator-(const tv& other) const
    {
        tv result(*this);
        result.sec -= other.sec;
        result.usec -= other.usec;
        while (result.usec < 0)
        {
            result.sec -= 1;
            result.usec += 1000000;
        }
        return result;
    }
    tv operator/(int32_t div) const
    {
        tv result(*this);
        result.sec /= div;
        result.usec /= div;
        return result;
    }
};


/*
| Field                 | Type   | Description                                                                                       |
|-----------------------|--------|---------------------------------------------------------------------------------------------------|
| type                  | uint16 | Should be one of the typed message IDs                                                            |
| id                    | uint16 | Used in requests to identify the message (not always used)                                        |
| refersTo              | uint16 | Used in responses to identify which request message ID this is responding to                      |
| sent.sec              | int32  | The second value of the timestamp when this message was sent. Filled in by the sender.            |
| sent.usec             | int32  | The microsecond value of the timestamp when this message was sent. Filled in by the sender.       |
| received.sec          | int32  | The second value of the timestamp when this message was received. Filled in by the receiver.      |
| received.usec         | int32  | The microsecond value of the timestamp when this message was received. Filled in by the receiver. |
| size                  | uint32 | Total number of bytes of the following typed message                                              |
*/
struct MessageHeader {
    uint16_t type;
    uint16_t id;
    uint16_t refersTo;
    tv sent;
    tv received;
    uint32_t typed_message_size;  // size of the following message (payload)
    
    static constexpr size_t byteSize() {
        return sizeof(MessageHeader);
    }

    static bool fromBytes(MessageHeader& headerOut, const uint8_t* data, size_t size) {
        if (size < sizeof(MessageHeader)) return false;
        std::memcpy(&headerOut, data, sizeof(MessageHeader));
        return true;
    }
    size_t getMessageSize() const {
        return sizeof(MessageHeader) + this->typed_message_size;
    }
    message_type getMessageType() const {
        return static_cast<message_type>(this->type);
    }
    void toBytes(uint8_t* dest) const {
        std::memcpy(dest, this, sizeof(MessageHeader));
    }
    void print() const {
        printf("Snapcast: BaseMessageHeader: type: %d, id: %d, refersTo: %d, sent.sec: %d, sent.usec: %d, received.sec: %d, received.usec: %d, size: %d\n",
                 type, id, refersTo, sent.sec, sent.usec, received.sec, received.usec, typed_message_size);
    }
   
};
#pragma pack(pop)


/*
| Field      | Type    | Description                                                 |
|------------|---------|-------------------------------------------------------------|
| codec_size | unint32 | Length of the codec string (not including a null character) |
| codec      | char[]  | String describing the codec (not null terminated)           |
| size       | uint32  | Size of the following payload                               |
| payload    | char[]  | Buffer of data containing the codec header                  |
*/
struct CodecHeaderPayloadView {
    uint32_t codec_str_size = 0;
    std::string codec;
    uint32_t payload_size = 0;
    const uint8_t* payload_ptr = nullptr;

    // Bind this view to an external buffer (no copies)
    bool bind(const uint8_t* buffer, size_t size) {
        if (!buffer || size < sizeof(uint32_t)) return false;

        const uint8_t* src = buffer;
        size_t remaining = size;

        // 1. Read codec string size
        std::memcpy(&codec_str_size, src, sizeof(codec_str_size));
        src += sizeof(codec_str_size);
        remaining -= sizeof(codec_str_size);

        if (remaining < codec_str_size) return false;

        // 2. Bind codec string (no null terminator, no copy)
        codec = std::string(reinterpret_cast<const char*>(src), codec_str_size);
        src += codec_str_size;
        remaining -= codec_str_size;

        // 3. Read payload size
        if (remaining < sizeof(payload_size)) return false;

        std::memcpy(&payload_size, src, sizeof(payload_size));
        src += sizeof(payload_size);
        remaining -= sizeof(payload_size);

        if (remaining < payload_size) return false;

        // 4. Point to the payload (view only)
        payload_ptr = src;
        return true;
    }

    // Copy out the payload if needed
    bool copyPayloadTo(uint8_t* dest, size_t destSize) const {
        if (!payload_ptr || destSize < payload_size) return false;
        std::memcpy(dest, payload_ptr, payload_size);
        return true;
    }
    void print() const {
        printf("Snapcast: CodecHeaderPayloadView: codec_str_size: %d, codec: %s, payload_size: %d\n",
               codec_str_size, codec.data(), payload_size);
    }
    std::string get_codec() const {
        return std::string(codec);
    }
};


struct WireChunkMessageView {
    int32_t timestamp_sec = 0;
    int32_t timestamp_usec = 0;
    uint32_t payload_size = 0;
    const uint8_t* payload_ptr = nullptr;

    // Bind the view to a raw buffer
    bool bind(const uint8_t* buffer, size_t size) {
        if (!buffer || size < sizeof(timestamp_sec) + sizeof(timestamp_usec) + sizeof(payload_size)) {
            return false;
        }

        const uint8_t* src = buffer;
        size_t remaining = size;

        // 1. Read timestamp seconds
        std::memcpy(&timestamp_sec, src, sizeof(timestamp_sec));
        src += sizeof(timestamp_sec);
        remaining -= sizeof(timestamp_sec);

        // 2. Read timestamp microseconds
        std::memcpy(&timestamp_usec, src, sizeof(timestamp_usec));
        src += sizeof(timestamp_usec);
        remaining -= sizeof(timestamp_usec);

        // 3. Read payload size
        std::memcpy(&payload_size, src, sizeof(payload_size));
        src += sizeof(payload_size);
        remaining -= sizeof(payload_size);

        // 4. Point to the payload (view only)
        if (remaining < payload_size) return false;

        payload_ptr = src;
        return true;
    }

    // Optionally copy the payload to a caller-provided buffer
    bool copyPayloadTo(uint8_t* dest, size_t destSize) const {
        if (!payload_ptr || destSize < payload_size) return false;
        std::memcpy(dest, payload_ptr, payload_size);
        return true;
    }
};

class SnapcastMessage {
public:
    SnapcastMessage(message_type msgType) {
        this->header_.type = static_cast<uint16_t>(msgType);
        this->header_.typed_message_size = 0;
    }
    SnapcastMessage(MessageHeader header) : header_(header){}
    virtual ~SnapcastMessage() = default;

    size_t getMessageSize() const {
        return this->header_.getMessageSize();
    }
    message_type getMessageType() const {
        return this->header_.getMessageType();
    }
    
    virtual size_t toBytes(uint8_t* dest) const {
        uint8_t* pos = dest;
        std::memcpy(pos, &this->header_, sizeof(MessageHeader));
        pos += sizeof(MessageHeader);
        return pos - dest;
    }
    virtual void print() const {
        this->header_.print();
    }
    void set_send_time() {
        this->header_.sent.sec = static_cast<int32_t>(millis() / 1000);
        this->header_.sent.usec = static_cast<int32_t>(micros() % 1000000);
    }
protected:
    struct MessageHeader header_;    
};

class TimeMessage : public SnapcastMessage {
public:
    TimeMessage() : SnapcastMessage(message_type::kTime) {
        this->header_.typed_message_size = sizeof(tv);
        this->set_send_time();
    }
};


class JsonMessage : public SnapcastMessage {
public:    
    JsonMessage(message_type msgType) : SnapcastMessage(msgType) { 
        this->header_.typed_message_size = sizeof(uint32_t) + this->json_.size();
    }
    JsonMessage(MessageHeader header) : SnapcastMessage(header) {
        assert( static_cast<message_type>(header.type) == message_type::kServerSettings );
    }
    ~JsonMessage() = default;
    
    void set_json(const std::string& json) {
        this->json_ = json;
        this->body_size_ = sizeof(uint32_t) + this->json_.size();
        this->header_.typed_message_size = this->body_size_;
    }

    size_t toBytes(uint8_t* dest) const override {
        uint8_t* pos = dest;
        pos += SnapcastMessage::toBytes(pos);
        std::memcpy(pos, &this->body_size_, sizeof(uint32_t));
        pos += sizeof(uint32_t);
        std::memcpy(pos, this->json_.c_str(), this->json_.size());
        pos += this->json_.size();
        return pos - dest;
    }

protected:
    void serializeBody(uint8_t *data)
    {
        uint32_t size = this->json_.size();
        memcpy(data, &size, sizeof(uint32_t));
        if( this->json_.size() ){
            memcpy(data + sizeof(uint32_t), this->json_.c_str(), this->json_.size());
        }
    }
    
    std::string build_json() const {
        return json::build_json([this](JsonObject root) {
            this->construct_json_(root);
        });
    }
    virtual void construct_json_(JsonObject root) const {}
    
    uint32_t body_size_{0};
    std::string json_;    
};


/*
| Field   | Type   | Description                                              |
|---------|--------|----------------------------------------------------------|
| size    | uint32 | Size of the following JSON string                        |
| payload | char[] | JSON string containing the message (not null terminated) |
*/
class HelloMessage : public JsonMessage {
public:
    HelloMessage() : JsonMessage(message_type::kHello) {
        this->set_json(
            this->build_json() 
        );
    }
    void construct_json_(JsonObject root) const override {
        root["Arch"] = "xtensa";
        root["ClientName"] = "Satellite1";
        root["HostName"] = network::get_use_address();
        root["ID"] = "E4:B0:63:92:A5:64";
        root["Instance"] = 1;
        root["MAC"] = "E4:B0:63:92:A5:64";
        root["OS"] = "ESPHome";
        root["SnapStreamProtocolVersion"] = 2;
        root["Version"] = "0.17.1";
    }
};


/*
| Field   | Type   | Description                                              |
|---------|--------|----------------------------------------------------------|
| size    | uint32 | Size of the following JSON string                        |
| payload | char[] | JSON string containing the message (not null terminated) 
*/
class ServerSettingsMessage : public JsonMessage {
public:
    ServerSettingsMessage(const MessageHeader &header, uint8_t* buffer, size_t len) : JsonMessage(header) {
        assert( static_cast<message_type>(header.type) == message_type::kServerSettings );
        this->deserializeBody_(buffer, len);
    }
    void print() const override {
        this->header_.print();
        printf("ServerSettingsMessage: buffer_ms: %d, latency: %d, volume: %d, muted: %d\n", 
                this->buffer_ms_, this->latency_, this->volume_, this->muted_);
    }
protected:        
    bool deserializeBody_(const uint8_t *data, size_t size) {
        std::string jsonString(reinterpret_cast<const char*>(data+sizeof(uint32_t)), size - sizeof(uint32_t));    
        this->json_ = jsonString;
        this->body_size_ = size;
        
        bool valid = json::parse_json(jsonString, [this](JsonObject root) -> bool {
            if (!root.containsKey("bufferMs") || !root.containsKey("latency") || !root.containsKey("muted")
                || !root.containsKey("volume")) {
                return false;
            }
            this->buffer_ms_ = root["bufferMs"].as<int32_t>();
            this->latency_ = root["latency"].as<int32_t>();
            this->volume_ = root["volume"].as<uint16_t>();
            this->muted_ = root["muted"].as<bool>();
            return true;
       });
       return valid;
    }
public:
    int32_t buffer_ms_;
    int32_t latency_;
    uint16_t volume_;
    bool muted_;
};




}
}

