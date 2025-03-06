#include "protocol.h"

#include <esp_log.h>

#define TAG "Protocol"

void Protocol::OnIncomingJson(std::function<void(const cJSON* root)> callback) {
    on_incoming_json_ = callback;
}

void Protocol::OnIncomingAudio(std::function<void(std::vector<uint8_t>&& data)> callback) {
    on_incoming_audio_ = callback;
}

void Protocol::OnAudioChannelOpened(std::function<void()> callback) {
    on_audio_channel_opened_ = callback;
}

void Protocol::OnAudioChannelClosed(std::function<void()> callback) {
    on_audio_channel_closed_ = callback;
}

void Protocol::OnNetworkError(std::function<void(const std::string& message)> callback) {
    on_network_error_ = callback;
}

void Protocol::SendAbortSpeaking(AbortReason reason) {
    std::string message = "{\"session_id\":\"" + session_id_ + "\",\"type\":\"abort\"";
    if (reason == kAbortReasonWakeWordDetected) {
        message += ",\"reason\":\"wake_word_detected\"";
    }
    message += "}";
    SendText(message);
}

void Protocol::SendWakeWordDetected(const std::string& wake_word) {
    std::string json = "{\"session_id\":\"" + session_id_ + 
                      "\",\"type\":\"listen\",\"state\":\"detect\",\"text\":\"" + wake_word + "\"}";
    SendText(json);
}

void Protocol::SendStartListening(ListeningMode mode) {
    std::string message = "{\"session_id\":\"" + session_id_ + "\"";
    message += ",\"type\":\"listen\",\"state\":\"start\"";
    if (mode == kListeningModeAlwaysOn) {
        message += ",\"mode\":\"realtime\"";
    } else if (mode == kListeningModeAutoStop) {
        message += ",\"mode\":\"auto\"";
    } else {
        message += ",\"mode\":\"manual\"";
    }
    message += "}";
    SendText(message);
}

void Protocol::SendStopListening() {
    std::string message = "{\"session_id\":\"" + session_id_ + "\",\"type\":\"listen\",\"state\":\"stop\"}";
    SendText(message);
}

void Protocol::SendIotDescriptors(const std::string& descriptors) {
    std::string message = "{\"session_id\":\"" + session_id_ + "\",\"type\":\"iot\",\"descriptors\":" + descriptors + "}";
    SendText(message);
}

void Protocol::SendIotStates(const std::string& states) {
    std::string message = "{\"session_id\":\"" + session_id_ + "\",\"type\":\"iot\",\"states\":" + states + "}";
    SendText(message);
}

void Protocol::SendIotContent(const std::string& name, const std::string& type, 
                              const std::string& property, const std::string& value) {
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        ESP_LOGE("Protocol", "Failed to create JSON object");
        return;
    }

    // 添加 JSON 键值对
    cJSON_AddStringToObject(root, "session_id", session_id_.c_str());
    cJSON_AddStringToObject(root, "name", name.c_str());
    cJSON_AddStringToObject(root, "type", type.c_str());
    cJSON_AddStringToObject(root, "property", property.c_str());
    cJSON_AddStringToObject(root, "value", value.c_str());

    // 生成 JSON 字符串
    char *message = cJSON_PrintUnformatted(root);
    if (message) {
        ESP_LOGI("Protocol", "Sending JSON: %s", message);
        SendText(message);
        free(message);  // 释放内存
    }

    // 释放 JSON 对象
    cJSON_Delete(root);
}

void Protocol::SendGreetContent(const std::string& text) {
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        ESP_LOGE("Protocol", "Failed to create JSON object");
        return;
    }

    // 添加 JSON 键值对
    // cJSON_AddStringToObject(root, "session_id", session_id_.c_str());
    // cJSON_AddStringToObject(root, "name", name.c_str());
    cJSON_AddStringToObject(root, "type", "listen");
    cJSON_AddStringToObject(root, "state", "greet");
    cJSON_AddStringToObject(root, "text", text.c_str());

    // 生成 JSON 字符串
    char *message = cJSON_PrintUnformatted(root);
    if (message) {
        ESP_LOGI("Protocol", "Sending JSON: %s", message);
        SendText(message);
        free(message);  // 释放内存
    }

    // 释放 JSON 对象
    cJSON_Delete(root);
}