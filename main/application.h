#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include <string>
#include <mutex>
#include <list>

#include <opus_encoder.h>
#include <opus_decoder.h>
#include <opus_resampler.h>

#include "websocket_protocol.h"
#include "ota.h"
#include "background_task.h"

#include "uart_comm.h"
#include "adcButton.h"
#include "button.h"
#if CONFIG_IDF_TARGET_ESP32S3
#include "wake_word_detect.h"
#include "audio_processor.h"
#endif

#define SCHEDULE_EVENT (1 << 0)
#define AUDIO_INPUT_READY_EVENT (1 << 1)
#define AUDIO_OUTPUT_READY_EVENT (1 << 2)
#define EXTERNAL_VOICE_WAKE_UP_GPIO GPIO_NUM_5
// #define DEVICE_ID "b23b844e-6b44-4c7d-8930-b9cc5690c063"//test1
#define DEVICE_ID "04ea9a7a-9a24-4a66-9fed-115635b2864b"//印测
enum ChatState {
    kChatStateUnknown,
    kChatStateIdle,
    kChatStateConnecting,
    kChatStateListening,
    kChatStateSpeaking,
    kChatStateUpgrading
};

#define OPUS_FRAME_DURATION_MS 60

class Application {
public:
    static Application& GetInstance() {
        static Application instance;
        return instance;
    }
    // 删除拷贝构造函数和赋值运算符
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    void displayTest();
    void KaijiGifStart();
    void Start();
    ChatState GetChatState() const { return chat_state_; }
    void Schedule(std::function<void()> callback);
    void SetChatState(ChatState state);
    void Alert(const std::string& title, const std::string& message);
    void AbortSpeaking(AbortReason reason);
    void ToggleChatState();
    void StartListening();
    void StopListening();
    void UpdateIotStates();
    void UpdateIotContent();
    void sendCjsonToSerial(const char *name, const char *type, const char *property, const char *value, const char *session_id);
    void sendCjsonToCameraSerial(const char *name, const char *type, const char *property, const char *value, const char *session_id);
    void ProcessReceivedJson(cJSON* root);
    void CameraProcessReceivedJson(cJSON* root);


private:
    Application();
    ~Application();

#if CONFIG_IDF_TARGET_ESP32S3
    WakeWordDetect wake_word_detect_;
    AudioProcessor audio_processor_;
#endif
    bool test_yb = false;
    bool flame_warning = false;
    bool flameWarning = false;
    UartComm* uc_uart;
    UartComm* camera_uart;
    std::string camera_string;
    std::string uc_string;
    ADCButtonNetwork* adc_button;
    Button* External_voice_wake_up;
    Ota ota_;
    std::mutex mutex_;
    std::list<std::function<void()>> main_tasks_;
    std::unique_ptr<WebsocketProtocol> protocol_;
    EventGroupHandle_t event_group_;
    volatile ChatState chat_state_ = kChatStateUnknown;
    bool keep_listening_ = false;
    bool IsDisconnect_ = false;
    bool aborted_ = false;
    std::string last_iot_states_;

    // Audio encode / decode
    BackgroundTask background_task_;
    std::chrono::steady_clock::time_point last_output_time_;
    std::list<std::vector<uint8_t>> audio_decode_queue_;

    std::unique_ptr<OpusEncoderWrapper> opus_encoder_;
    std::unique_ptr<OpusDecoderWrapper> opus_decoder_;

    int opus_decode_sample_rate_ = -1;
    OpusResampler input_resampler_;
    OpusResampler reference_resampler_;
    OpusResampler output_resampler_;

    void MainLoop();
    void InputAudio();
    void OutputAudio();
    void ResetDecoder();
    void SetDecodeSampleRate(int sample_rate);
    void CheckNewVersion();

    void PlayLocalFile(const char* data, size_t size);
};

#endif // _APPLICATION_H_
