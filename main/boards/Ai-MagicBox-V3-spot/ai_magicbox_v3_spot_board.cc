// #include "wifi_board.h"
#include "dual_network_board.h"
#include "audio_codecs/es8311_audio_codec.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "iot/thing_manager.h"
#include "sdkconfig.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <math.h>

#include <driver/gpio.h>
#include "esp_timer.h"
#include "led/circular_strip.h"
#include "power_manager.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "imu_bmi270.h"
#include "adcButton.h"
#define TAG "Ai-MagicBox-V3-spot"
const char* ADCButtonNetwork::TAG1 = "adc_button_network";
bool button_released_ = false;
bool shutdown_ready_ = false;
esp_timer_handle_t shutdown_timer;

class AiMagicBoxV2SpotBoard  : public DualNetworkBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    Button key_button_;
    Button External_voice_wake_up_;
    Button wai_key_button_;
    Button Volume_add_button_;
    Button Volume_sub_button_;
    // Button volume_key_button_;


    // adc_oneshot_unit_handle_t volume_adc_button_handle;
    // AdcButton volume_key_button_;
    ADCButtonNetwork* adc_button;
    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t adc1_cali_handle;
    SemaphoreHandle_t adc_cali_mutex = nullptr;

    bool do_calibration = false;
    bool key_long_pressed = false;
    int64_t last_key_press_time = 0;
    static const int64_t LONG_PRESS_TIMEOUT_US = 5 * 1000000ULL;

    void setupPowerManagement() {
        if (!SLEEP_MODE_IS_EXIST) {
                    return;
        }
        auto& power = PowerManager::getInstance();
        
        // 基本配置
        PowerManager::Config config{
            // .wakeup_gpio = GPIO_NUM_0,

            .light_sleep_delay_ms = 30*1000,   // 10秒
            .deep_sleep_delay_ms = 60*1000,    // 1分钟
            .auto_sleep_enable = true
        };

        if (power.init(config) != PowerManager::Error::OK) {
            ESP_LOGE(TAG, "Power management init failed");
            return;
        }
        
        // 设置回调
        power.setPreSleepCallback([this](PowerManager::PowerMode mode) {
            onPreSleep(mode);
        });
        
        power.setPostWakeupCallback([this](PowerManager::PowerMode mode) {
            onPostWakeup(mode);
        });

        // 7. 在系统活动时重置计时器
        // 例如：在按键事件、传感器数据更新等事件中调用
        power.resetInactiveTimer();
        //注册imu回调
        app_imu_register_callback([]() {
            PowerManager::getInstance().resetInactiveTimer();
        });
        // 8. 如果需要禁用自动睡眠
        // power.enableAutoSleep(false);

        // 9. 手动控制睡眠（如果需要）
        // if (some_condition) {
        //     power.enterLightSleep();
        // }

        // 10. 在特定条件下进入深度睡眠
        // if (battery_very_low) {
        //     power.enterDeepSleep();  // 注意：此函数不会返回
        // }
    }

    void onPreSleep(PowerManager::PowerMode mode) {
        // 保存状态
        ESP_LOGI(TAG, "Preparing to enter %s mode",
            mode == PowerManager::PowerMode::LIGHT_SLEEP ? "light sleep" :
            mode == PowerManager::PowerMode::DEEP_SLEEP ? "deep sleep" : "normal");
        if (mode == PowerManager::PowerMode::DEEP_SLEEP)
        {
            // // 注销keyButton，注册RTC唤醒源
            // key_button_.Destroy();
            // rtc_gpio_pullup_dis(KEY_BUTTON_GPIO);
            // rtc_gpio_pulldown_en(KEY_BUTTON_GPIO);
            // 注册外围按键，注册RTC唤醒源
            wai_key_button_.Destroy();
            rtc_gpio_pullup_en(WAI_KEY_GPIO);
            rtc_gpio_pulldown_dis(WAI_KEY_GPIO);
            
            // 配置 EXT0 唤醒
            esp_sleep_enable_ext0_wakeup(WAI_KEY_GPIO, 0); // GPIO12 高电平触发唤醒
            // 配置 EXT1 唤醒
            // uint64_t mask = imu_interrupt_wake_Init();
            // mask |= 1ULL << WAI_KEY_GPIO;
            // esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_HIGH);  // 任意引脚高电平触发唤醒
            
            gpio_hold_dis(PERP_VCC_CTL);
            gpio_set_level(PERP_VCC_CTL, 0);
            gpio_hold_en(PERP_VCC_CTL);
        }
        
            
        // 停止定时器
        // 关闭外设

    }

    void onPostWakeup(PowerManager::PowerMode mode) {
        // 恢复状态
        ESP_LOGI(TAG, "Waking up to %s mode",
            mode == PowerManager::PowerMode::NORMAL ? "normal" :
            mode == PowerManager::PowerMode::LIGHT_SLEEP ? "light sleep" : "deep sleep");

        // 仅在light sleep模式下检查唤醒原因
        if (mode == PowerManager::PowerMode::LIGHT_SLEEP) {
            esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
            switch (wakeup_cause) {
                case ESP_SLEEP_WAKEUP_TIMER:
                    ESP_LOGI(TAG, "Wakeup caused by timer");
                    break;
                case ESP_SLEEP_WAKEUP_GPIO:
                    ESP_LOGI(TAG, "Wakeup caused by GPIO");
                    break;
                default:
                    ESP_LOGI(TAG, "Wakeup caused by other reason: %d", wakeup_cause);
                    break;
            }
        }else if (mode == PowerManager::PowerMode::DEEP_SLEEP)
        {
            // rtc_gpio_deinit(KEY_BUTTON_GPIO) ;
            // key_button_.Reset(true,KEY_BUTTON_GPIO, true);
            // InitializeButtons();
            // 重启定时器
            // auto& power = PowerManager::getInstance();
        }
        
        POWER_MANAGER.resetInactiveTimer();
    }

    void onEvent() {
        // 在任何用户活动或重要事件发生时
        POWER_MANAGER.resetInactiveTimer();
    }
    void InitializeI2c() {

        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
#if IMU_BMI270_IS_EXIST
        i2c_config_t i2c_bus_conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = 400000
        };
        i2c_bus_handle_t i2c_bus_handle_ = i2c_bus_create(I2C_NUM_0, &i2c_bus_conf);

        app_imu_init(i2c_bus_handle_,IMU_BMI270_INT_PIN);

#endif
    }

    void InitializeADC() {
        adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

        adc_oneshot_chan_cfg_t chan_config = {
            .atten = ADC_ATTEN,
            .bitwidth = ADC_WIDTH,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, VBAT_ADC_CHANNEL, &chan_config));

        adc_cali_handle_t handle = NULL;
        esp_err_t ret = ESP_FAIL;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN,
            .bitwidth = ADC_WIDTH,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            do_calibration = true;
            adc1_cali_handle = handle;
            ESP_LOGI(TAG, "ADC Curve Fitting calibration succeeded");
        }
#endif // ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
#ifdef VOLUME_BUTTON_CHANNEL

    // ADC按键
    const uint16_t thresholds[] = {

        810, // 按键1的阈值
        1570, // 按键2的阈值
        2400, // 按键3的阈值

    };
    
        size_t num_buttons = sizeof(thresholds) / sizeof(thresholds[0]);
    
        // 创建ADCButtonNetwork对象，并自动启动任务,
        adc_button = new ADCButtonNetwork(adc1_handle,"ADCButtonTask",ADC_UNIT_1, VOLUME_BUTTON_CHANNEL, thresholds, num_buttons);
        adc_button->registerCallback(0, [this]() { 
            ESP_LOGI(ADCButtonNetwork::TAG1, "Button 1 Callback Executed");
            // 在这里添加按钮1被按下时的处理逻辑
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);

            });
        adc_button->registerCallback(1, []() {
            ESP_LOGI(ADCButtonNetwork::TAG1, "Button 2 Callback Executed");
            // 在这里添加按钮2被按下时的处理逻辑
    
        }
        );
        adc_button->registerCallback(2, [this]() {
            ESP_LOGI(ADCButtonNetwork::TAG1, "Button 3 Callback Executed");
            
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
        }
        );
        adc_cali_mutex = adc_button->get_adc_cali_mutex();
        adc_button->set_adc_cali_handle(adc1_cali_handle);
#endif
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (GetNetworkType() == NetworkType::WIFI) {
                if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                    // cast to WifiBoard
                    auto& wifi_board = static_cast<WifiBoard&>(GetCurrentBoard());
                    wifi_board.ResetWifiConfiguration();
                }
            }
            app.ToggleChatState();
        });
        Volume_add_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            printf("volume = %d\n", volume);
        });
        Volume_sub_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            printf("volume = %d\n", volume);
        });

        External_voice_wake_up_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (GetNetworkType() == NetworkType::WIFI) {
                if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                    // cast to WifiBoard
                    auto& wifi_board = static_cast<WifiBoard&>(GetCurrentBoard());
                    wifi_board.ResetWifiConfiguration();
                }
            }
            app.ToggleChatState();
        });
        wai_key_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (GetNetworkType() == NetworkType::WIFI) {
                if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                    // cast to WifiBoard
                    auto& wifi_board = static_cast<WifiBoard&>(GetCurrentBoard());
                    wifi_board.ResetWifiConfiguration();
                }
            }
            app.ToggleChatState(); 
        });
        key_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            app.ToggleChatState();
            key_long_pressed = false;
        });

        key_button_.OnLongPress([this]() {
            int64_t now = esp_timer_get_time();
            auto* led = static_cast<CircularStrip*>(this->GetLed());

            if (key_long_pressed) {
                if ((now - last_key_press_time) < LONG_PRESS_TIMEOUT_US) {
                    ESP_LOGW(TAG, "Key button long pressed the second time within 5s, shutting down...");
                    led->SetSingleColor(0, {0, 0, 0});
                    if (MCU_VCC_CTL != GPIO_NUM_NC ){
                        gpio_hold_dis(MCU_VCC_CTL);
                        gpio_set_level(MCU_VCC_CTL, 0);
                    }


                } else {
                    last_key_press_time = now;
                    BlinkGreenFor5s();
                }
                key_long_pressed = true;
            } else {
                ESP_LOGW(TAG, "Key button first long press! Waiting second within 5s to shutdown...");
                last_key_press_time = now;
                key_long_pressed = true;

                BlinkGreenFor5s();
            }
        });
        wai_key_button_.OnLongPress([this]() {
            int64_t now = esp_timer_get_time();
            auto* led = static_cast<CircularStrip*>(this->GetLed());

            if (key_long_pressed) {
                if ((now - last_key_press_time) < LONG_PRESS_TIMEOUT_US) {
                    ESP_LOGW(TAG, "Key button long pressed the second time within 5s, shutting down...");
                    led->SetSingleColor(0, {0, 0, 0});
                    if (MCU_VCC_CTL != GPIO_NUM_NC ){
                        gpio_hold_dis(MCU_VCC_CTL);
                        gpio_set_level(MCU_VCC_CTL, 0);
                    }


                } else {
                    last_key_press_time = now;
                    BlinkGreenFor5s();
                }
                key_long_pressed = true;
            } else {
                ESP_LOGW(TAG, "Key button first long press! Waiting second within 5s to shutdown...");
                last_key_press_time = now;
                key_long_pressed = true;

                BlinkGreenFor5s();
            }
        });
    }

    void InitializePowerCtl() {
        rtc_gpio_deinit(KEY_BUTTON_GPIO) ;
        rtc_gpio_deinit(WAI_KEY_GPIO) ;
        InitializeGPIO();
        if (MCU_VCC_CTL != GPIO_NUM_NC ){
            gpio_set_level(MCU_VCC_CTL, 1);
            gpio_hold_en(MCU_VCC_CTL); 
        }


        gpio_hold_dis(PERP_VCC_CTL);
        gpio_set_level(PERP_VCC_CTL, 1);
        gpio_hold_en(PERP_VCC_CTL);
    }

    void InitializeGPIO() {
        gpio_config_t io_pa = {
            .pin_bit_mask = (1ULL << AUDIO_CODEC_PA_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_pa);
        gpio_set_level(AUDIO_CODEC_PA_PIN, 0);

        if (MCU_VCC_CTL != GPIO_NUM_NC )
        {
            gpio_config_t io_conf_1 = {
                .pin_bit_mask = (1ULL << MCU_VCC_CTL),
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE
            };
            gpio_config(&io_conf_1);
        }
        


        gpio_config_t io_conf_2 = {
            .pin_bit_mask = (1ULL << PERP_VCC_CTL),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf_2);
    }

    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Battery"));
        #ifdef SD_IS_EXIST
        #if SD_IS_EXIST == 1
        thing_manager.AddThing(iot::CreateThing("SdPlayer"));
        #endif
        #endif
    }


    void BlinkGreenFor5s() {
        auto* led = static_cast<CircularStrip*>(GetLed());
        if (!led) {
            return;
        }

        led->Blink({50, 25, 0}, 100);

        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                auto* self = static_cast<AiMagicBoxV2SpotBoard*>(arg);
                auto* led = static_cast<CircularStrip*>(self->GetLed());
                if (led) {
                    led->SetSingleColor(0, {0, 0, 0});
                }
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "blinkGreenFor5s_timer"
        };

        esp_timer_handle_t blink_timer = nullptr;
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &blink_timer));
        ESP_ERROR_CHECK(esp_timer_start_once(blink_timer, LONG_PRESS_TIMEOUT_US));
    }

public:
    //NET_IS_WIFI_OR_ML307 在config.h中定义
    AiMagicBoxV2SpotBoard() : DualNetworkBoard(ML307_TX_PIN, ML307_RX_PIN, 4096,NET_IS_WIFI_OR_ML307),
                              boot_button_(false,BOOT_BUTTON_GPIO,false), 
                              key_button_(true,KEY_BUTTON_GPIO, true),
                              External_voice_wake_up_(true,EXTERNAL_VOICE_WAKE_UP_GPIO,false),
                              wai_key_button_(false,WAI_KEY_GPIO, false),
                              Volume_add_button_(false,VOLUME_ADD_BUTTON_GPIO, false),
                              Volume_sub_button_(false,VOLUME_SUB_BUTTON_GPIO, false){
        InitializePowerCtl();
        InitializeADC();
        InitializeI2c();
        InitializeButtons();
        setupPowerManagement();
        InitializeIot();
    }
    // EspSpotS3Bot() : boot_button_(BOOT_BUTTON_GPIO), key_button_(KEY_BUTTON_GPIO, true) {
    //     InitializePowerCtl();
    //     InitializeADC();
    //     InitializeI2c();
    //     InitializeButtons();
    //     InitializeIot();
    // }

    virtual Led* GetLed() override {
        static CircularStrip led(LED_PIN, 1);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
         static Es8311AudioCodec audio_codec(i2c_bus_, I2C_NUM_0,
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE, AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_PA_PIN,
            AUDIO_CODEC_ES8311_ADDR, false);
        return &audio_codec;
    }

    virtual bool GetBatteryLevel(int &level, bool &charging, bool &discharging) {
        if (!adc1_handle) {
            InitializeADC();
        }
        esp_err_t ret ;
        int raw_value = 0;
        int voltage = 0;

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, VBAT_ADC_CHANNEL, &raw_value));

        if (do_calibration) {

            if (adc_cali_mutex != nullptr) {
                // 获取互斥锁，超时时间100ms
                if (xSemaphoreTake(adc_cali_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    // 调用原始的ADC校准函数
                    ret = adc_cali_raw_to_voltage(adc1_cali_handle, raw_value, &voltage);
                    
                    // 释放互斥锁
                    xSemaphoreGive(adc_cali_mutex);
                    
                    if (ret != ESP_OK) {
                        ESP_LOGW(ADC_CALI_TAG, "ADC calibration failed: %s", esp_err_to_name(ret));
                    }
                } else {
                    ESP_LOGE(ADC_CALI_TAG, "Failed to acquire ADC calibration mutex within timeout");
                    ret = ESP_ERR_TIMEOUT;
                }
            }else {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, raw_value, &voltage));
            }

            
            voltage = voltage * 3 / 2; // compensate for voltage divider
            ESP_LOGI(TAG, "Calibrated voltage: %d mV", voltage);
        } else {
            ESP_LOGI(TAG, "Raw ADC value: %d", raw_value);
            voltage = raw_value;
        }

        voltage = voltage < EMPTY_BATTERY_VOLTAGE ? EMPTY_BATTERY_VOLTAGE : voltage;
        voltage = voltage > FULL_BATTERY_VOLTAGE ? FULL_BATTERY_VOLTAGE : voltage;

        // 计算电量百分比
        level = (voltage - EMPTY_BATTERY_VOLTAGE) * 100 / (FULL_BATTERY_VOLTAGE - EMPTY_BATTERY_VOLTAGE);

        // charging = gpio_get_level(MCU_VCC_CTL);
        ESP_LOGI(TAG, "Battery Level: %d%%, Charging: %s", level, charging ? "Yes" : "No");
        return true;
    }
};

DECLARE_BOARD(AiMagicBoxV2SpotBoard);