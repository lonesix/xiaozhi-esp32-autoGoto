#include "application.h"
#include "board.h"
#include "display.h"
#include "system_info.h"
#include "ml307_ssl_transport.h"
#include "audio_codec.h"
#include "mqtt_protocol.h"
#include "websocket_protocol.h"
#include "font_awesome_symbols.h"
#include "iot/thing_manager.h"

#include <cstring>
#include <esp_log.h>
#include <cJSON.h>
#include <driver/gpio.h>
#include <arpa/inet.h>
#include "adcButton.h"
#include "button.h"
// #define TAG("Application")
const char *TAG = "Application";
const char* ADCButtonNetwork::TAG1 = "adc_button_network";

extern const char p3_err_reg_start[] asm("_binary_err_reg_p3_start");
extern const char p3_err_reg_end[] asm("_binary_err_reg_p3_end");
extern const char p3_err_pin_start[] asm("_binary_err_pin_p3_start");
extern const char p3_err_pin_end[] asm("_binary_err_pin_p3_end");
extern const char p3_err_wificonfig_start[] asm("_binary_err_wificonfig_p3_start");
extern const char p3_err_wificonfig_end[] asm("_binary_err_wificonfig_p3_end");

static const char* const STATE_STRINGS[] = {
    "unknown",
    "idle",
    "connecting",
    "listening",
    "speaking",
    "upgrading",
    "invalid_state"
};

Application::Application() : background_task_(4096 * 8) {
    event_group_ = xEventGroupCreate();

    uc_uart = new UartComm(UART_NUM, TX_PIN, RX_PIN, BAUD_RATE, BUF_SIZE);
    uc_uart->init();
    
    camera_uart = new UartComm(CAMERA_UART_NUM, CAMERA_TX_PIN, CAMERA_RX_PIN, CAMERA_BAUD_RATE, CAMERA_BUF_SIZE);
    camera_uart->init();
    




    ota_.SetCheckVersionUrl(CONFIG_OTA_VERSION_URL);
    ota_.SetHeader("Device-Id", SystemInfo::GetMacAddress().c_str());
}

Application::~Application() {
    vEventGroupDelete(event_group_);
}

void Application::CheckNewVersion() {
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    // Check if there is a new firmware version available
    ota_.SetPostData(board.GetJson());

    while (true) {
        if (ota_.CheckVersion()) {
            if (ota_.HasNewVersion()) {
                // Wait for the chat state to be idle
                do {
                    vTaskDelay(pdMS_TO_TICKS(3000));
                } while (GetChatState() != kChatStateIdle);

                SetChatState(kChatStateUpgrading);
                
                display->SetIcon(FONT_AWESOME_DOWNLOAD);
                display->SetStatus("新版本 " + ota_.GetFirmwareVersion());

                // 预先关闭音频输出，避免升级过程有音频操作
                board.GetAudioCodec()->EnableOutput(false);

                ota_.StartUpgrade([display](int progress, size_t speed) {
                    char buffer[64];
                    snprintf(buffer, sizeof(buffer), "%d%% %zuKB/s", progress, speed / 1024);
                    display->SetStatus(buffer);
                });

                // If upgrade success, the device will reboot and never reach here
                ESP_LOGI(TAG, "Firmware upgrade failed...");
                SetChatState(kChatStateIdle);
            } else {
                ota_.MarkCurrentVersionValid();
                display->ShowNotification("版本 " + ota_.GetCurrentVersion());
            }
            return;
        }

        // Check again in 60 seconds
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

void Application::Alert(const std::string& title, const std::string& message) {
    ESP_LOGW(TAG, "Alert: %s, %s", title.c_str(), message.c_str());
    auto display = Board::GetInstance().GetDisplay();
    display->ShowNotification(message);

    if (message == "PIN is not ready") {
        PlayLocalFile(p3_err_pin_start, p3_err_pin_end - p3_err_pin_start);
    } else if (message == "Configuring WiFi") {
        PlayLocalFile(p3_err_wificonfig_start, p3_err_wificonfig_end - p3_err_wificonfig_start);
    } else if (message == "Registration denied") {
        PlayLocalFile(p3_err_reg_start, p3_err_reg_end - p3_err_reg_start);
    }
}

void Application::PlayLocalFile(const char* data, size_t size) {
    ESP_LOGI(TAG, "PlayLocalFile: %zu bytes", size);
    SetDecodeSampleRate(16000);
    for (const char* p = data; p < data + size; ) {
        auto p3 = (BinaryProtocol3*)p;
        p += sizeof(BinaryProtocol3);

        auto payload_size = ntohs(p3->payload_size);
        std::vector<uint8_t> opus;
        opus.resize(payload_size);
        memcpy(opus.data(), p3->payload, payload_size);
        p += payload_size;

        std::lock_guard<std::mutex> lock(mutex_);
        audio_decode_queue_.emplace_back(std::move(opus));
    }
}

void Application::ToggleChatState() {
    Schedule([this]() {
        if (!protocol_) {
            ESP_LOGE(TAG, "Protocol not initialized");
            return;
        }

        if (chat_state_ == kChatStateIdle) {
            SetChatState(kChatStateConnecting);
            if (!protocol_->OpenAudioChannel()) {
                Alert("Error", "Failed to open audio channel");
                SetChatState(kChatStateIdle);
                return;
            }

            keep_listening_ = true;
            protocol_->SendStartListening(kListeningModeAutoStop);
            SetChatState(kChatStateListening);
        } else if (chat_state_ == kChatStateSpeaking) {
            AbortSpeaking(kAbortReasonNone);
        } else if (chat_state_ == kChatStateListening) {
            protocol_->CloseAudioChannel();
        }
    });
}

void Application::StartListening() {
    Schedule([this]() {
        if (!protocol_) {
            ESP_LOGE(TAG, "Protocol not initialized");
            return;
        }
        if(wake_word_detect_.buttonFlag ){
            return;
        }
        keep_listening_ = false;
        if (chat_state_ == kChatStateIdle) {
            if (!protocol_->IsAudioChannelOpened()) {
                SetChatState(kChatStateConnecting);
                if (!protocol_->OpenAudioChannel()) {
                    SetChatState(kChatStateIdle);
                    Alert("Error", "Failed to open audio channel");
                    return;
                }
            }
            protocol_->SendStartListening(kListeningModeManualStop);
            SetChatState(kChatStateListening);
        } else if (chat_state_ == kChatStateSpeaking) {
            AbortSpeaking(kAbortReasonNone);
            protocol_->SendStartListening(kListeningModeManualStop);
            // FIXME: Wait for the speaker to empty the buffer
            vTaskDelay(pdMS_TO_TICKS(120));
            SetChatState(kChatStateListening);
        }
    });
}

void Application::StopListening() {
    Schedule([this]() {
        if (chat_state_ == kChatStateListening) {
            protocol_->SendStopListening();
            SetChatState(kChatStateIdle);
        }
    });
}
// #include "ui.h"
// #define ALL_WIFI_ICON ""
// #define WIFI_ICON_H ""
// #define WIFI_ICON_M ""
// #define WIFI_ICON_L ""
// #define WIFI_ICON_N ""
// #define ALL_VOLUMN_ICON ""
// #define VOLUMN_ICON_Y ""
// #define VOLUMN_ICON_N ""

// const char *wifiIcon[] = {WIFI_ICON_H,WIFI_ICON_M,WIFI_ICON_L,WIFI_ICON_N};
// const char *volumnIcon[] = {VOLUMN_ICON_Y,VOLUMN_ICON_N};
void Application::displayTest() {
        /* Setup the display */
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    // board.get_touchxy();
    // /*unicode设置网络标志特殊字体测试*/
    // lv_label_set_text(ui_netLabel, wifiIcon[3]);
    // lv_label_set_text(ui_volLabel2, volumnIcon[1]);
    // /*设置字体，网络和音量标志已内置*/
    // // lv_obj_set_style_text_font(ui_AITextArea, &font_alipuhui20, LV_PART_MAIN | LV_STATE_DEFAULT);
    // // lv_obj_set_style_text_font(ui_userTextArea, &font_alipuhui20, LV_PART_MAIN | LV_STATE_DEFAULT);
    // /*设置textarea_text*/
    // // lv_textarea_set_text(ui_AITextArea, "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    // /*设置textarea_text*/
    // lv_bar_set_value(ui_changeBar, 12, LV_ANIM_OFF);

}

void Application::KaijiGifStart()
{
    #if IsGifSwitch
    this->displayTest();
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    while (display->kaijiFinishFlag() == false)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    #endif
    this->Start();
  


}


void Application::Start()
{
    auto& board = Board::GetInstance();
    auto builtin_led = board.GetBuiltinLed();
    builtin_led->SetBlue();
    builtin_led->StartContinuousBlink(100);

    /* Setup the display */
    auto display = board.GetDisplay();

    /* Setup the audio codec */
    auto codec = board.GetAudioCodec();
    opus_decode_sample_rate_ = codec->output_sample_rate();
    opus_decoder_ = std::make_unique<OpusDecoderWrapper>(opus_decode_sample_rate_, 1);
    opus_encoder_ = std::make_unique<OpusEncoderWrapper>(16000, 1, OPUS_FRAME_DURATION_MS);
    if (codec->input_sample_rate() != 16000) {
        input_resampler_.Configure(codec->input_sample_rate(), 16000);
        reference_resampler_.Configure(codec->input_sample_rate(), 16000);
    }
    codec->OnInputReady([this, codec]() {
        BaseType_t higher_priority_task_woken = pdFALSE;
        xEventGroupSetBitsFromISR(event_group_, AUDIO_INPUT_READY_EVENT, &higher_priority_task_woken);
        return higher_priority_task_woken == pdTRUE;
    });
    codec->OnOutputReady([this]() {
        BaseType_t higher_priority_task_woken = pdFALSE;
        xEventGroupSetBitsFromISR(event_group_, AUDIO_OUTPUT_READY_EVENT, &higher_priority_task_woken);
        return higher_priority_task_woken == pdTRUE;
    });
    codec->Start();

    /* Start the main loop */
    xTaskCreate([](void* arg) {
        Application* app = (Application*)arg;
        app->MainLoop();
        vTaskDelete(NULL);
    }, "main_loop", 4096 * 2, this, 2, nullptr);

    /* Wait for the network to be ready */
    board.StartNetwork();

    // Check for new firmware version or get the MQTT broker address
    // xTaskCreate([](void* arg) {
    //     Application* app = (Application*)arg;
    //     app->CheckNewVersion();
    //     vTaskDelete(NULL);
    // }, "check_new_version", 4096 * 2, this, 1, nullptr);

    // 启动串口接收任务
    xTaskCreate([](void *arg)
    {
        Application* app = (Application*)arg;
        while (true) {
            app->uc_uart->receiveDataCjson();
            vTaskDelay(pdMS_TO_TICKS(10));  // 每 ms 检查一次接收的数据
    } }, "uart_receive_task", 4096, this, 1, nullptr);

    // 启动Camera串口接收任务
    xTaskCreate([](void *arg)
    {
        Application* app = (Application*)arg;
        while (true) {
            app->camera_uart->receiveCameraDataCjson();
            vTaskDelay(pdMS_TO_TICKS(10));  // 每 ms 检查一次接收的数据
    } }, "camera_uart_receive_task", 4096, this, 1, nullptr);
    // 自定义语音唤醒
    External_voice_wake_up = new Button(EXTERNAL_VOICE_WAKE_UP_GPIO, 1);

    External_voice_wake_up->OnPressDown([this]() {
    ESP_LOGI(TAG, "VoiceButton released");
    wake_word_detect_.buttonFlag = true;
    Application::GetInstance().StartListening();
    
});
    // ADC按键
    const uint16_t thresholds[] = {
    // (uint16_t)((float)0.38 / 3.3 * 4096-150), // 按键1的阈值
    // (uint16_t)((float)0.82 / 3.3 * 4096-150), // 按键2的阈值
    // (uint16_t)((float)1.18 / 3.3 * 4096-180), // 按键3的阈值
    // (uint16_t)((float)1.57 / 3.3 * 4096-220), // 按键4的阈值
    // (uint16_t)((float)1.98 / 3.3 * 4096-250), // 按键5的阈值
    // (uint16_t)((float)2.38 / 3.3 * 4096-250), // 按键6的阈值
    380, // 按键1的阈值
    820, // 按键2的阈值
    1180, // 按键3的阈值
    1570, // 按键4的阈值
    1980, // 按键5的阈值
    2380, // 按键6的阈值
};

    size_t num_buttons = sizeof(thresholds) / sizeof(thresholds[0]);

    // 创建ADCButtonNetwork对象，并自动启动任务,io4
    adc_button = new ADCButtonNetwork("ADCButtonTask", ADC_UNIT_1, ADC_CHANNEL_3, thresholds, num_buttons);
            // 注册回调函数
    adc_button->registerCallback(0, []() { 
        ESP_LOGI(ADCButtonNetwork::TAG1, "Button 1 Callback Executed");
        // 在这里添加按钮1被按下时的处理逻辑
        Application::GetInstance().StartListening();
        vTaskDelay(pdMS_TO_TICKS(120));
        Application::GetInstance().StopListening();
        });
    adc_button->registerCallback(1, []() {
        ESP_LOGI(ADCButtonNetwork::TAG1, "Button 2 Callback Executed");
        // 在这里添加按钮2被按下时的处理逻辑
    }
    );
    adc_button->registerCallback(2, []() {
        ESP_LOGI(ADCButtonNetwork::TAG1, "Button 3 Callback Executed");
        // 在这里添加按钮3被按下时的处理逻辑
    }
    );
    adc_button->registerCallback(3, []() {
        ESP_LOGI(ADCButtonNetwork::TAG1, "Button 4 Callback Executed");
        // 在这里添加按钮4被按下时的处理逻辑
    }
    );
    adc_button->registerCallback(4, []() {
        ESP_LOGI(ADCButtonNetwork::TAG1, "Button 5 Callback Executed");
        // 在这里添加按钮5被按下时的处理逻辑
    }
    );
    adc_button->registerCallback(5, []() {
        ESP_LOGI(ADCButtonNetwork::TAG1, "Button 6 Callback Executed");
        // 在这里添加按钮6被按下时的处理逻辑
    }
    );
    // xTaskCreate([](void *arg)
    // {
    //     Application* app = (Application*)arg;
    //     while (true) {
    //         if(app->test_yb)
    //             app->UpdateIotContent();
    //         vTaskDelay(pdMS_TO_TICKS(6000));  // 每 ms 检查一次接收的数据
    // } }, "testyb_task", 4096, this, 1, nullptr);

#if CONFIG_IDF_TARGET_ESP32S3
    audio_processor_.Initialize(codec->input_channels(), codec->input_reference());
    audio_processor_.OnOutput([this](std::vector<int16_t>&& data) {
        background_task_.Schedule([this, data = std::move(data)]() mutable {
            opus_encoder_->Encode(std::move(data), [this](std::vector<uint8_t>&& opus) {
                Schedule([this, opus = std::move(opus)]() {
                    protocol_->SendAudio(opus);
                });
            });
        });
    });

    wake_word_detect_.Initialize(codec->input_channels(), codec->input_reference());
    wake_word_detect_.OnVadStateChange([this](bool speaking) {
        Schedule([this, speaking]() {
            auto builtin_led = Board::GetInstance().GetBuiltinLed();
            if (chat_state_ == kChatStateListening) {
                if (speaking) {
                    builtin_led->SetRed(HIGH_BRIGHTNESS);
                } else {
                    builtin_led->SetRed(LOW_BRIGHTNESS);
                }
                builtin_led->TurnOn();
            }
        });
    });

    wake_word_detect_.OnWakeWordDetected([this](const std::string& wake_word) {
        Schedule([this, &wake_word]() {
            if (chat_state_ == kChatStateIdle) {
                SetChatState(kChatStateConnecting);
                wake_word_detect_.EncodeWakeWordData();

                if (!protocol_->OpenAudioChannel()) {
                    ESP_LOGE(TAG, "Failed to open audio channel");
                    SetChatState(kChatStateIdle);
                    wake_word_detect_.StartDetection();
                    return;
                }
                
                std::vector<uint8_t> opus;
                // Encode and send the wake word data to the server
                while (wake_word_detect_.GetWakeWordOpus(opus)) {
                    protocol_->SendAudio(opus);
                }
                // Set the chat state to wake word detected
                protocol_->SendWakeWordDetected(wake_word);
                ESP_LOGI(TAG, "Wake word detected: %s", wake_word.c_str());
                keep_listening_ = true;
                SetChatState(kChatStateListening);
            } else if (chat_state_ == kChatStateSpeaking) {
                AbortSpeaking(kAbortReasonWakeWordDetected);
            }

            // Resume detection
            wake_word_detect_.StartDetection();
        });
    });
    wake_word_detect_.StartDetection();
#endif

    // Initialize the protocol
    display->SetStatus("初始化协议");
#ifdef CONFIG_CONNECTION_TYPE_WEBSOCKET
    protocol_ = std::make_unique<WebsocketProtocol>();
#else
    protocol_ = std::make_unique<MqttProtocol>();
#endif
    protocol_->OnNetworkError([this](const std::string& message) {
        Alert("Error", std::move(message));
    });
    protocol_->OnIncomingAudio([this](std::vector<uint8_t>&& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (chat_state_ == kChatStateSpeaking) {
            audio_decode_queue_.emplace_back(std::move(data));
        }
    });
    protocol_->OnAudioChannelOpened([this, codec, &board]() {
        board.SetPowerSaveMode(false);
        if (protocol_->server_sample_rate() != codec->output_sample_rate()) {
            ESP_LOGW(TAG, "服务器的音频采样率 %d 与设备输出的采样率 %d 不一致，重采样后可能会失真",
                protocol_->server_sample_rate(), codec->output_sample_rate());
        }
        SetDecodeSampleRate(protocol_->server_sample_rate());
        // 物联网设备描述符
        last_iot_states_.clear();
        auto& thing_manager = iot::ThingManager::GetInstance();
        protocol_->SendIotDescriptors(thing_manager.GetDescriptorsJson());
        test_yb = true;
    });
    protocol_->OnAudioChannelClosed([this, &board]() {
        board.SetPowerSaveMode(true);
        Schedule([this]() {
            SetChatState(kChatStateIdle);
        });
    });
    protocol_->OnIncomingJson([this, display](const cJSON* root) {
        // Parse JSON data
        auto type = cJSON_GetObjectItem(root, "type");
        if (strcmp(type->valuestring, "tts") == 0) {
            auto state = cJSON_GetObjectItem(root, "state");
            if (strcmp(state->valuestring, "start") == 0) {
                Schedule([this]() {
                    aborted_ = false;
                    if (chat_state_ == kChatStateIdle || chat_state_ == kChatStateListening) {
                        SetChatState(kChatStateSpeaking);
                    }
                });
            } else if (strcmp(state->valuestring, "stop") == 0) {
                Schedule([this]() {
                    if (chat_state_ == kChatStateSpeaking) {
                        background_task_.WaitForCompletion();
                        if (keep_listening_) {
                            protocol_->SendStartListening(kListeningModeAutoStop);
                            SetChatState(kChatStateListening);
                        } else {
                            SetChatState(kChatStateIdle);
                            keep_listening_ = true;
                        }
                        if(IsDisconnect_ == true)
                        {
                            ESP_LOGI(TAG, "wss断开连接");
                            protocol_->websocket_->transport_->Disconnect();
                            IsDisconnect_ = false;
                        }
                        
                    }
                });
            } else if (strcmp(state->valuestring, "sentence_start") == 0) {
                auto text = cJSON_GetObjectItem(root, "text");
                if (text != NULL) {
                    ESP_LOGI(TAG, "<< %s", text->valuestring);
                    display->SetChatMessage("assistant", text->valuestring);
                    uc_string = text->valuestring;
                    Schedule([this]() {
                        // this->sendCjsonToSerial("tts",uc_string.c_str());
                    });
                }
            }
        } else if (strcmp(type->valuestring, "stt") == 0) {
            auto text = cJSON_GetObjectItem(root, "text");
            if (text != NULL) {
                ESP_LOGI(TAG, ">> %s", text->valuestring);
                display->SetChatMessage("user", text->valuestring);
                uc_string = text->valuestring;
                Schedule([this]() {
                    // this->sendCjsonToSerial("stt",uc_string.c_str());
                });
            }
        } else if (strcmp(type->valuestring, "llm") == 0) {
            auto emotion = cJSON_GetObjectItem(root, "emotion");
            if (emotion != NULL) {
                display->SetEmotion(emotion->valuestring);
                uc_string = emotion->valuestring;
                Schedule([this]() {
                    // this->sendCjsonToSerial("emotion",uc_string.c_str());
                });
            }
        }else if(strcmp(type->valuestring, "command") == 0){
            auto Json_name = cJSON_GetObjectItem(root, "name");
            if ((Json_name != NULL) && (strcmp(Json_name->valuestring, "LLM") == 0)) {
                auto Json_value = cJSON_GetObjectItem(root, "value");
                if (Json_value != NULL) {
                    if (strcmp(Json_value->valuestring, "byebye") == 0 || strcmp(Json_value->valuestring, "leave") == 0) {
                        Schedule([this](){
                            keep_listening_ = false;
                            IsDisconnect_ = true;
                            
                        });
                        
                    }
                }
                // auto Json_property = cJSON_GetObjectItem(root, "property");
                // if ((Json_property != NULL) && (strcmp(Json_property->valuestring, "action") == 0)) {

                //     }
                // }



            }




        } else if (strcmp(type->valuestring, "iot") == 0) {
            // auto commands = cJSON_GetObjectItem(root, "commands");
            // if (commands != NULL) {
            //     auto& thing_manager = iot::ThingManager::GetInstance();
            //     for (int i = 0; i < cJSON_GetArraySize(commands); ++i) {
            //         auto command = cJSON_GetArrayItem(commands, i);
            //         thing_manager.Invoke(command);
            //     }
            // }
            auto iot_content = cJSON_GetObjectItem(root, "content");
            if (iot_content != NULL) {
                auto iot_name = cJSON_GetObjectItem(iot_content, "name");
                auto iot_type = cJSON_GetObjectItem(iot_content, "type");
                auto return_type = "";
                if (strcmp(iot_type->valuestring, "read") == 0){
                    return_type = "read";
                }
                else if (strcmp(iot_type->valuestring, "write") == 0){
                    return_type = "write";
                }
                auto iot_property = cJSON_GetObjectItem(iot_content, "property");
                auto iot_value = cJSON_GetObjectItem(iot_content, "value");
                auto iot_id = cJSON_GetObjectItem(root, "session_id");
                //根据云服务器值解析后 通过串口 下发命令给 协处理器
                //主对协例子：
                //{"session_id":"4a429e61","name":"SG90","type":"write","property":"angle","value":"15"}
                //Application::sendCjsonToSerial(const char *type, const char *text)

                if (strcmp(iot_name->valuestring, "Camera") == 0){
                    //发给Camera
                    this->sendCjsonToCameraSerial( iot_name->valuestring, return_type, iot_property->valuestring, iot_value->valuestring, iot_id->valuestring);
                }else
                {
                    //发给协处理器
                    this->sendCjsonToSerial( iot_name->valuestring, return_type, iot_property->valuestring, iot_value->valuestring, iot_id->valuestring);
                }
                

                

                


                // protocol_->SendIotContent(iot_name->valuestring, return_type, iot_property->valuestring, iot_value->valuestring);
            }
        }
    });

    // Blink the LED to indicate the device is running
    display->SetStatus("待命");
    builtin_led->SetGreen();
    builtin_led->BlinkOnce();

    SetChatState(kChatStateIdle);
}

void Application::Schedule(std::function<void()> callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    main_tasks_.push_back(std::move(callback));
    xEventGroupSetBits(event_group_, SCHEDULE_EVENT);
}

// The Main Loop controls the chat state and websocket connection
// If other tasks need to access the websocket or chat state,
// they should use Schedule to call this function
void Application::MainLoop() {
    while (true) {
        auto bits = xEventGroupWaitBits(event_group_,
            SCHEDULE_EVENT | AUDIO_INPUT_READY_EVENT | AUDIO_OUTPUT_READY_EVENT,
            pdTRUE, pdFALSE, portMAX_DELAY);

        if (bits & AUDIO_INPUT_READY_EVENT) {
            InputAudio();
        }
        if (bits & AUDIO_OUTPUT_READY_EVENT) {
            OutputAudio();
        }
        if (bits & SCHEDULE_EVENT) {
            mutex_.lock();
            std::list<std::function<void()>> tasks = std::move(main_tasks_);
            mutex_.unlock();
            for (auto& task : tasks) {
                task();
            }
        }
    }
}

void Application::ResetDecoder() {
    std::lock_guard<std::mutex> lock(mutex_);
    opus_decoder_->ResetState();
    audio_decode_queue_.clear();
    last_output_time_ = std::chrono::steady_clock::now();
    Board::GetInstance().GetAudioCodec()->EnableOutput(true);
}

void Application::OutputAudio() {
    auto now = std::chrono::steady_clock::now();
    auto codec = Board::GetInstance().GetAudioCodec();
    const int max_silence_seconds = 10;

    std::unique_lock<std::mutex> lock(mutex_);
    if (audio_decode_queue_.empty()) {
        // Disable the output if there is no audio data for a long time
        if (chat_state_ == kChatStateIdle) {
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_output_time_).count();
            if (duration > max_silence_seconds) {
                codec->EnableOutput(false);
            }
        }
        return;
    }

    if (chat_state_ == kChatStateListening) {
        audio_decode_queue_.clear();
        return;
    }

    last_output_time_ = now;
    auto opus = std::move(audio_decode_queue_.front());
    audio_decode_queue_.pop_front();
    lock.unlock();

    background_task_.Schedule([this, codec, opus = std::move(opus)]() mutable {
        if (aborted_) {
            return;
        }

        std::vector<int16_t> pcm;
        if (!opus_decoder_->Decode(std::move(opus), pcm)) {
            return;
        }

        // Resample if the sample rate is different
        if (opus_decode_sample_rate_ != codec->output_sample_rate()) {
            int target_size = output_resampler_.GetOutputSamples(pcm.size());
            std::vector<int16_t> resampled(target_size);
            output_resampler_.Process(pcm.data(), pcm.size(), resampled.data());
            pcm = std::move(resampled);
        }
        
        codec->OutputData(pcm);
    });
}

void Application::InputAudio() {
    auto codec = Board::GetInstance().GetAudioCodec();
    std::vector<int16_t> data;
    if (!codec->InputData(data)) {
        return;
    }

    if (codec->input_sample_rate() != 16000) {
        if (codec->input_channels() == 2) {
            auto mic_channel = std::vector<int16_t>(data.size() / 2);
            auto reference_channel = std::vector<int16_t>(data.size() / 2);
            for (size_t i = 0, j = 0; i < mic_channel.size(); ++i, j += 2) {
                mic_channel[i] = data[j];
                reference_channel[i] = data[j + 1];
            }
            auto resampled_mic = std::vector<int16_t>(input_resampler_.GetOutputSamples(mic_channel.size()));
            auto resampled_reference = std::vector<int16_t>(reference_resampler_.GetOutputSamples(reference_channel.size()));
            input_resampler_.Process(mic_channel.data(), mic_channel.size(), resampled_mic.data());
            reference_resampler_.Process(reference_channel.data(), reference_channel.size(), resampled_reference.data());
            data.resize(resampled_mic.size() + resampled_reference.size());
            for (size_t i = 0, j = 0; i < resampled_mic.size(); ++i, j += 2) {
                data[j] = resampled_mic[i];
                data[j + 1] = resampled_reference[i];
            }
        } else {
            auto resampled = std::vector<int16_t>(input_resampler_.GetOutputSamples(data.size()));
            input_resampler_.Process(data.data(), data.size(), resampled.data());
            data = std::move(resampled);
        }
    }
    
#if CONFIG_IDF_TARGET_ESP32S3
    if (audio_processor_.IsRunning()) {
        audio_processor_.Input(data);
    }
    if (wake_word_detect_.IsDetectionRunning()) {
        wake_word_detect_.Feed(data);
    }
#else
    if (chat_state_ == kChatStateListening) {
        background_task_.Schedule([this, data = std::move(data)]() mutable {
            opus_encoder_->Encode(std::move(data), [this](std::vector<uint8_t>&& opus) {
                Schedule([this, opus = std::move(opus)]() {
                    protocol_->SendAudio(opus);
                });
            });
        });
    }
#endif
}

void Application::AbortSpeaking(AbortReason reason) {
    ESP_LOGI(TAG, "Abort speaking");
    aborted_ = true;
    protocol_->SendAbortSpeaking(reason);
}

void Application::SetChatState(ChatState state) {
    if (chat_state_ == state) {
        return;
    }
    
    chat_state_ = state;
    ESP_LOGI(TAG, "STATE: %s", STATE_STRINGS[chat_state_]);
    // The state is changed, wait for all background tasks to finish
    background_task_.WaitForCompletion();

    auto display = Board::GetInstance().GetDisplay();
    auto builtin_led = Board::GetInstance().GetBuiltinLed();
    switch (state) {
        case kChatStateUnknown:
        case kChatStateIdle:
            builtin_led->TurnOff();
            display->SetStatus("千机赋能");
            display->SetChatMessage("user", "请问有什么可以帮您吗?");
            // display->SetEmotion("neutral");
            Schedule([this](){ /*this->sendCjsonToSerial("status", "Idle");*/ });
#ifdef CONFIG_IDF_TARGET_ESP32S3
            audio_processor_.Stop();
#endif
            break;
        case kChatStateConnecting:
            builtin_led->SetBlue();
            builtin_led->TurnOn();
            display->SetStatus("连接中...");
            Schedule([this](){ /*this->sendCjsonToSerial("status", "Connecting");*/ });
            break;
        case kChatStateListening:
            builtin_led->SetRed();
            builtin_led->TurnOn();
            display->SetStatus("聆听中...");
            // display->SetEmotion("neutral");
            Schedule([this](){ /*this->sendCjsonToSerial("status", "Listening");*/ });
            ResetDecoder();
            opus_encoder_->ResetState();
#if CONFIG_IDF_TARGET_ESP32S3
            audio_processor_.Start();
#endif
            UpdateIotStates();
            break;
        case kChatStateSpeaking:
            builtin_led->SetGreen();
            builtin_led->TurnOn();
            display->SetStatus("说话中...");
            Schedule([this](){ /*this->sendCjsonToSerial("status", "Speaking");*/ });
            ResetDecoder();
#if CONFIG_IDF_TARGET_ESP32S3
            audio_processor_.Stop();
#endif
            break;
        case kChatStateUpgrading:
            builtin_led->SetGreen();
            builtin_led->StartContinuousBlink(100);
            break;
        default:
            ESP_LOGE(TAG, "Invalid chat state: %d", chat_state_);
            return;
    }
}

void Application::SetDecodeSampleRate(int sample_rate) {
    if (opus_decode_sample_rate_ == sample_rate) {
        return;
    }

    opus_decode_sample_rate_ = sample_rate;
    opus_decoder_ = std::make_unique<OpusDecoderWrapper>(opus_decode_sample_rate_, 1);

    auto codec = Board::GetInstance().GetAudioCodec();
    if (opus_decode_sample_rate_ != codec->output_sample_rate()) {
        ESP_LOGI(TAG, "Resampling audio from %d to %d", opus_decode_sample_rate_, codec->output_sample_rate());
        output_resampler_.Configure(opus_decode_sample_rate_, codec->output_sample_rate());
    }
}

void Application::UpdateIotStates() {
    auto& thing_manager = iot::ThingManager::GetInstance();
    auto states = thing_manager.GetStatesJson();
    if (states != last_iot_states_) {
        last_iot_states_ = states;
        protocol_->SendIotStates(states);
    }
}

void Application::UpdateIotContent() {

    auto name = "DHT11";
    auto type = "sensor";       
    auto property = "hum";     
    auto value = "25.5";
    Schedule([this,name, type, property, value](){ 
        protocol_->SendIotContent(name, type, property, value); 
    });
    
}

//主对协例子：
//{"session_id":"4a429e61","name":"SG90","type":"write","property":"angle","value":"15"}
//name传感器名字，目前有："MPU6050","RC522","DHT11","Body","Hall","Light","Soil","Flame","WaterLevel","WS2812","SG90" 。
//const std::string& name, const std::string& type, const std::string& property, const std::string& value
void Application::sendCjsonToSerial(const char *name, const char *type, const char *property, const char *value, const char *session_id)
{
    // 创建一个 cJSON 对象
    cJSON *uc_json = cJSON_CreateObject();
    cJSON_AddStringToObject(uc_json, "session_id", session_id);
    cJSON_AddStringToObject(uc_json, "name", name);
    cJSON_AddStringToObject(uc_json, "type", type);
    cJSON_AddStringToObject(uc_json, "property", property);
    cJSON_AddStringToObject(uc_json, "value", value);

    // 发送该 cJSON 对象
    uc_uart->sendData(uc_json);

    // 将 cJSON 对象转换为字符串
    // char *json_str = cJSON_PrintUnformatted(uc_json); // 不格式化 JSON，节省空间

    // if (json_str != NULL)
    // {
    //     // 打印日志
    //     ESP_LOGI(TAG, "Sent JSON: %s", json_str);
    //     // printf("\n%s\n", json_str);

    //     // 释放动态分配的内存
    //     free(json_str);
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "Failed to convert cJSON to string");
    // }

    // 清理 cJSON 对象
    cJSON_Delete(uc_json);
}

//主对Camera例子：
//{"session_id":"4a429e61","name":"Camera","type":"read","property":"color","value":""}
//const std::string& name, const std::string& type, const std::string& property, const std::string& value
void Application::sendCjsonToCameraSerial(const char *name, const char *type, const char *property, const char *value, const char *session_id)
{
    // 创建一个 cJSON 对象
    cJSON *Camera_json = cJSON_CreateObject();
    cJSON_AddStringToObject(Camera_json, "session_id", session_id);
    cJSON_AddStringToObject(Camera_json, "name", name);
    cJSON_AddStringToObject(Camera_json, "type", type);
    cJSON_AddStringToObject(Camera_json, "property", property);
    cJSON_AddStringToObject(Camera_json, "value", value);

    // 发送该 cJSON 对象
    camera_uart->sendData(Camera_json);


    // 清理 cJSON 对象
    cJSON_Delete(Camera_json);
}

#include "ui_events.h"
extern "C" void ChangeVolumn_cc(int volum)
{
	// Your code here
	auto &board = Board::GetInstance();
	board.GetAudioCodec()->SetOutputVolume(volum);
	// (int)lv_slider_get_value(ui_volumn);
}
extern "C" void send_data_cc()
{
        // 创建根节点
    cJSON *root = cJSON_CreateObject();

    // 添加session_id节点
    cJSON_AddStringToObject(root, "session_id", "4a429e61");

    // 添加name节点
    cJSON_AddStringToObject(root, "name", "SG90");

    // 添加type节点
    cJSON_AddStringToObject(root, "type", "command");

    // 添加property节点
    cJSON_AddStringToObject(root, "property", "angle");

    // 添加value节点
    cJSON_AddStringToObject(root, "value", "15");
        // 生成JSON字符串
    char *json_str = cJSON_Print(root);

    // 打印JSON字符串
    printf("%s\n", json_str);
    Application::GetInstance().ProcessReceivedJson(root);
    cJSON_Delete(root);  // 在任务执行完后删除 JSON 对象
    free(json_str);
    
	// (int)lv_slider_get_value(ui_volumn);
}

//协对主解析
void Application::ProcessReceivedJson(cJSON *root)
{   
    static bool Isinit_xie = false;
    // // 协处理器是否初始化成功
    // if (Isinit_xie == false)
    // {
    //     cJSON *init_item = cJSON_GetObjectItem(root, "Init");
    //     if (init_item != nullptr )
    //     {
    //         Isinit_xie = true;  
    //     }
    //     return;
    // }
    
    if (!test_yb)
    {
        return;
    }
    
    
    // 错误判断
    cJSON *error_item = cJSON_GetObjectItem(root, "error");
    if (error_item == nullptr)
    {
        ESP_LOGE(TAG, "ProcessReceivedJson Not error");
        
    }else
    {
        /* 具体错误解析 */
        ESP_LOGE(TAG, "CMD error: %s", error_item->valuestring );
        return;
    }
    
    // 获取命令返回的
    auto xie_name = cJSON_GetObjectItem(root, "name");
    auto xie_type = cJSON_GetObjectItem(root, "type");
    auto xie_property = cJSON_GetObjectItem(root, "property");
    auto xie_value = cJSON_GetObjectItem(root, "value");
    auto xie_id = cJSON_GetObjectItem(root, "session_id");
    // 会话id匹配

    //
    // // 获取 JSON 中的 type 字段
    // cJSON *type_item = cJSON_GetObjectItem(root, "type");
    // if (type_item == nullptr)
    // {
    //     ESP_LOGE(TAG, "Missing 'type' in received JSON");
    //     return;
    // }

    // // 根据 type 字段的值进行不同的处理
    // const char *type = type_item->valuestring;
    // if (strcmp(type, "cmd") == 0)
    // {
    //     cJSON *text_item = cJSON_GetObjectItem(root, "text");
    //     if (text_item != nullptr)
    //     {
    //         // 判断 text 是否为字符串类型
    //         if (cJSON_IsString(text_item))
    //         {
    //             const char *text = text_item->valuestring;
    //             ESP_LOGI(TAG, "Text: %s", text);
    //             // 判断 text 是否为 "open"
    //             if (strcmp(text, "open") == 0)
    //             {
    //                 // Application::GetInstance().StartListening();
    //                 Schedule([this]()
    //                          {
    //                             keep_listening_ = true;
    //                             if (chat_state_ == kChatStateIdle) {
    //                                 if (!protocol_->IsAudioChannelOpened()) {
    //                                     SetChatState(kChatStateConnecting);
    //                                     if (!protocol_->OpenAudioChannel()) {
    //                                         SetChatState(kChatStateIdle);
    //                                         ESP_LOGE(TAG, "Failed to open audio channel");
    //                                         return;
    //                                     }
    //                                 }
    //                                 protocol_->SendStartListening(kListeningModeAutoStop);
    //                                 SetChatState(kChatStateListening);
    //                         } });
    //             }
    //             else if (strcmp(text, "abort") == 0)
    //             {
    //                 Application::AbortSpeaking(kAbortReasonNone);
    //             }
    //             else if (strcmp(text, "close") == 0)
    //             {
    //                 // Application::GetInstance().StopListening();
    //                 protocol_->SendStopListening();
    //                 SetChatState(kChatStateIdle);
    //             }
    //             else
    //             {
    //                 ESP_LOGW(TAG, "Unrecognized command: %s", text);
    //             }
    //         }
    //         // 判断 text 是否为数值类型（整数）
    //         else if (cJSON_IsNumber(text_item))
    //         {
    //         }
    //         else
    //         {
    //             ESP_LOGE(TAG, "Unsupported type for 'text'");
    //         }
    //     }
    //     else
    //     {
    //         ESP_LOGE(TAG, "Missing 'text' in message");
    //     }
    // }
    // else if (strcmp(type, "volume") == 0)
    // {
    //     cJSON *text_item = cJSON_GetObjectItem(root, "text");
    //     if (text_item != nullptr)
    //     {
    //         // 判断 text 是否为字符串类型
    //         if (cJSON_IsString(text_item))
    //         {
    //         }
    //         // 判断 text 是否为数值类型（整数）
    //         else if (cJSON_IsNumber(text_item))
    //         {
    //             int num = text_item->valueint;
    //             auto &board = Board::GetInstance();
    //             board.GetAudioCodec()->SetOutputVolume(num);
    //             ESP_LOGI(TAG, "Text (Number): %d", num);
    //         }
    //         else
    //         {
    //             ESP_LOGE(TAG, "Unsupported type for 'text'");
    //         }
    //     }
    //     else
    //     {
    //         ESP_LOGE(TAG, "Missing 'text' in message");
    //     }
    // }
    // else
    // {
    //     ESP_LOGW(TAG, "Unknown JSON type: %s", type);
    // }

    protocol_->SendIotContent(xie_name->valuestring, xie_type->valuestring, xie_property->valuestring, xie_value->valuestring);
}
//Camera对主
void Application::CameraProcessReceivedJson(cJSON *root)
{
    static bool Isinit_camera = false;
    // // 协处理器是否初始化成功
    // if (Isinit_xie == false)
    // {
    //     cJSON *init_item = cJSON_GetObjectItem(root, "Init");
    //     if (init_item != nullptr )
    //     {
    //         Isinit_xie = true;  
    //     }
    //     return;
    // }
    
    if (!test_yb)
    {
        return;
    }
    
    
    // 错误判断
    cJSON *error_item = cJSON_GetObjectItem(root, "error");
    if (error_item == nullptr)
    {
        ESP_LOGE(TAG, "CameraProcessReceivedJson Not error");
        
    }else
    {
        /* 具体错误解析 */
        ESP_LOGE(TAG, "CMD error: %s", error_item->valuestring );
        return;
    }
    
    // 获取命令返回的
    auto camera_name = cJSON_GetObjectItem(root, "name");
    auto camera_type = cJSON_GetObjectItem(root, "type");
    auto camera_property = cJSON_GetObjectItem(root, "property");
    auto camera_value = cJSON_GetObjectItem(root, "value");
    auto camera_id = cJSON_GetObjectItem(root, "session_id");
    // 会话id匹配

    protocol_->SendIotContent(camera_name->valuestring, camera_type->valuestring, camera_property->valuestring, camera_value->valuestring);
}
