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
#define EXTERNAL_VOICE_WAKE_UP_GPIO GPIO_NUM_17
// #define DEVICE_ID "b23b844e-6b44-4c7d-8930-b9cc5690c063"//test1
#define DEVICE_ID "28d48d6c-6857-4f6c-bd8a-4b66920ee4c8"//小一
// #define DEVICE_ID "04ea9a7a-9a24-4a66-9fed-115635b2864b"//印测
// #define DEVICE_ID "d4e46e7d-37eb-4209-aaec-bb580a16c229"//yu测
// f6da006c-46ed-4652-a03e-d55e18b6a287
// #define DEVICE_ID "df9b06ed-49ee-4389-aa5d-31e45eb84f4a"//A02
// #define DEVICE_ID "25d115c5-a510-4bad-b9f0-10eb0c8d4174"//A03
// #define DEVICE_ID "06105863-b453-456a-924d-5745c25a724f"//A04
// #define DEVICE_ID "74dc62f6-306c-4c7a-b346-430a7ebc42d0"//A05
// #define DEVICE_ID "292e6bf7-a8b9-4d86-a1ea-1f3e90479820"//A06
// #define DEVICE_ID "b53b886b-f2bc-484f-8b75-c54296b048dd"//A07
// #define DEVICE_ID "fc15d879-370f-4ed8-bd6e-ac277a4d697b"//A08
// #define DEVICE_ID "61cb8cc0-dbd0-444f-8713-4e50519eaa0d"//A09
// #define DEVICE_ID "1b2b09ce-ebd4-401d-9557-c4771c216d49"//A10
// #define DEVICE_ID "de9ef3fa-3191-4ac8-afa1-86e3a08d86e7"//A11
// #define DEVICE_ID "d9b6b89b-74b6-436c-b1cc-5839e2e228e0"//A12
// #define DEVICE_ID "1ab72ad4-33e9-481d-a445-d186338640e0"//A13
// #define DEVICE_ID "2bdbf27e-ca40-492d-ae05-8f36ab819c36"//A14
// #define DEVICE_ID "e2010fb8-f76c-45f3-bb5c-39af2981deef"//A15
// #define DEVICE_ID "38aa0fa8-93f6-4bd9-92a2-f5dcde570129"//A16
// #define DEVICE_ID "9c0d9d0a-16e1-4b63-8f35-dc44c6a29c31"//A17
// #define DEVICE_ID "6d3b3d4a-51d3-407f-81a4-6bc854a896e1"//A18
// #define DEVICE_ID "a4078ae8-dc1e-4433-8c8e-02d667399f33"//A19
// #define DEVICE_ID "9e763c6f-f579-4bb6-9d1b-c527bfa47b5f"//A20
// #define DEVICE_ID "c7652e3c-b778-4ccd-8c30-9498c123a109"//A21

enum ChatState {
    kChatStateUnknown,
    kChatStateIdle,
    kChatStateConnecting,
    kChatStateListening,
    kChatStateSpeaking,
    kChatStateUpgrading
};
enum PlayxiaogeState {

    StateIdle,
    StatePlay,//需要播放
    StatePlaying,
   
    Stateplayend, //需要关闭

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
    void playxiaoge();
    void Start();
    ChatState GetChatState() const { return chat_state_; }
    PlayxiaogeState GetPlayxiaogeState() const { return playxiaoge_; }
    bool protocol_IsConnected() const { 
        if (protocol_ == nullptr) {
            return false;
        }
        if( protocol_->websocket_ == nullptr)
        {
            return false;
        }
        return protocol_->websocket_->IsConnected(); 
    }
    void ClosePlayxiaoge() {
        playxiaoge_ = Stateplayend;
        ResetDecoder();
        // #ifdef CONFIG_IDF_TARGET_ESP32S3
        //     audio_processor_.Stop();
        // #endif
        background_task_.WaitForCompletion();
    }
    void PlayxiaogeIdle() {
        playxiaoge_ = StateIdle;
        
    }
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
    void SetWakeSound(bool wake_sound){
        wake_sound_ = wake_sound;
    }
    bool GetWakeSound(){
        return wake_sound_;
    }
    
private:
    Application();
    ~Application();

#if CONFIG_IDF_TARGET_ESP32S3
    WakeWordDetect wake_word_detect_;
    AudioProcessor audio_processor_;
#endif
    bool test_yb = false;
    volatile bool flame_warning = false;
    volatile bool flameWarning = false;
    volatile PlayxiaogeState playxiaoge_ = StateIdle;
    volatile bool wake_sound_ = false;
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
    void PlayLocalFile_zuse(const char* data, size_t size);
};

#endif // _APPLICATION_H_
