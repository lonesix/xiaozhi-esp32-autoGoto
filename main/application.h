#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include <string>
#include <mutex>
#include <list>
#include <vector>
#include <condition_variable>
#include <memory>

#include <opus_encoder.h>
#include <opus_decoder.h>
#include <opus_resampler.h>

#include "protocol.h"
#include "ota.h"
#include "background_task.h"
#include "audio_processor.h"

#if CONFIG_USE_WAKE_WORD_DETECT
#include "wake_word_detect.h"
#endif

#define SCHEDULE_EVENT (1 << 0)
#define AUDIO_INPUT_READY_EVENT (1 << 1)
#define AUDIO_OUTPUT_READY_EVENT (1 << 2)
#define CHECK_NEW_VERSION_DONE_EVENT (1 << 3)

// #define DEVICE_ID "b23b844e-6b44-4c7d-8930-b9cc5690c063"//test1
// #define DEVICE_ID "28d48d6c-6857-4f6c-bd8a-4b66920ee4c8"//小一
#define DEVICE_ID "04ea9a7a-9a24-4a66-9fed-115635b2864b"//印测
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
enum DeviceState {
    kDeviceStateUnknown,
    kDeviceStateStarting,
    kDeviceStateWifiConfiguring,
    kDeviceStateIdle,
    kDeviceStateConnecting,
    kDeviceStateListening,
    kDeviceStateSpeaking,
    kDeviceStateUpgrading,
    kDeviceStateActivating,
    kDeviceStateFatalError
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

    void Start();
    DeviceState GetDeviceState() const { return device_state_; }
    bool IsVoiceDetected() const { return voice_detected_; }
    void Schedule(std::function<void()> callback);
    void SetDeviceState(DeviceState state);
    void Alert(const char* status, const char* message, const char* emotion = "", const std::string_view& sound = "");
    void DismissAlert();
    void AbortSpeaking(AbortReason reason);
    void ToggleChatState();
    void StartListening();
    void StopListening();
    void UpdateIotStates();
    void Reboot();
    void WakeWordInvoke(const std::string& wake_word);
    void PlaySound(const std::string_view& sound);
    #if CONFIG_BOARD_TYPE_AI_MAGIC_BOX_V3_SPOT 
    EventGroupHandle_t sdEvent_group_;
    void WaitSoundToFinish();
    void StopSpeaking();
    void PlaySoundFromFile(const std::string &file_path_name);
    void TiShiYin_V2();
    void PlaySoundFromFile(int file_number);
    void SetSdEventHandle(EventGroupHandle_t event_group){sdEvent_group_ = event_group;}
    const int SDPLAYERMONITOR_IDLE_BIT = BIT0;
    const int SDPLAYERMONITOR_SPEAKINGTOSTOP_BIT = BIT1;
    const int SDPLAYERMONITOR_SPEAKINGTOIOT_BIT = BIT2;
    const int SDPLAYERMONITOR_SPEAKINGTOIOT_STR_BIT = BIT3;
    const int SDPLAYERMONITOR_SPEAKINGTOIOT_NUM_BIT = BIT4;
    void SetSdEventStop(){
        if (sdEvent_group_ != NULL)
        {
            xEventGroupSetBits(sdEvent_group_, SDPLAYERMONITOR_SPEAKINGTOSTOP_BIT);
            printf("SetSdEventStop\n");
        }
    }
    #endif
    bool CanEnterSleepMode();
  

private:
    Application();
    ~Application();

#if defined(CONFIG_BOARD_TYPE_AI_MAGIC_BOX_V2_SPOT) || defined(CONFIG_BOARD_TYPE_AI_MAGIC_BOX_V3_SPOT) 
    bool IsCloseConnect_ =false; //speaking的stop到来是否需要关闭连接
#endif

#if CONFIG_USE_WAKE_WORD_DETECT
    WakeWordDetect wake_word_detect_;
#endif
    std::unique_ptr<AudioProcessor> audio_processor_;
    Ota ota_;
    std::mutex mutex_;
    std::list<std::function<void()>> main_tasks_;
    std::unique_ptr<Protocol> protocol_;
    EventGroupHandle_t event_group_ = nullptr;
    esp_timer_handle_t clock_timer_handle_ = nullptr;
    volatile DeviceState device_state_ = kDeviceStateUnknown;
    ListeningMode listening_mode_ = kListeningModeAutoStop;
#if CONFIG_USE_DEVICE_AEC || CONFIG_USE_SERVER_AEC
    bool realtime_chat_enabled_ = true;
#else
    bool realtime_chat_enabled_ = false;
#endif
    bool IsDisconnect_ = false;
    bool aborted_ = false;
    bool voice_detected_ = false;
    bool busy_decoding_audio_ = false;
    int clock_ticks_ = 0;
    TaskHandle_t check_new_version_task_handle_ = nullptr;

    // Audio encode / decode
    TaskHandle_t audio_loop_task_handle_ = nullptr;
    BackgroundTask* background_task_ = nullptr;
    std::chrono::steady_clock::time_point last_output_time_;
    std::list<AudioStreamPacket> audio_decode_queue_;
    std::condition_variable audio_decode_cv_;

    // 新增：用于维护音频包的timestamp队列
    std::list<uint32_t> timestamp_queue_;
    std::mutex timestamp_mutex_;
    std::atomic<uint32_t> last_output_timestamp_ = 0;

    std::unique_ptr<OpusEncoderWrapper> opus_encoder_;
    std::unique_ptr<OpusDecoderWrapper> opus_decoder_;

    OpusResampler input_resampler_;
    OpusResampler reference_resampler_;
    OpusResampler output_resampler_;

    void MainEventLoop();
    void OnAudioInput();
    void OnAudioOutput();
    void ReadAudio(std::vector<int16_t>& data, int sample_rate, int samples);
    void ResetDecoder();
    void SetDecodeSampleRate(int sample_rate, int frame_duration);
    void CheckNewVersion();
    void ShowActivationCode();
    void OnClockTimer();
    void SetListeningMode(ListeningMode mode);
    void AudioLoop();
};

#endif // _APPLICATION_H_