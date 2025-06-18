
#include "power_manager.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "application.h"
#include "config.h"
static const char* TAG = "PowerManager";
static const uint32_t MONITOR_STACK_SIZE = 4096;
static const uint32_t MONITOR_PRIORITY = 1;
static const uint32_t MONITOR_INTERVAL_MS = 1000;

PowerManager::Error PowerManager::init(const Config& config) {
    if (initialized_) {
        ESP_LOGW(TAG, "Already initialized");
        return Error::OK;
    }

    // 验证配置
    if (
        // config.wakeup_gpio >= GPIO_NUM_MAX ||
        config.light_sleep_delay_ms == 0 ||
        config.deep_sleep_delay_ms <= config.light_sleep_delay_ms) {
        ESP_LOGE(TAG, "Invalid configuration");
        return Error::INVALID_PARAMS;
    }

    // 保存配置
    config_ = config;
    lastActiveTime_ = esp_timer_get_time() / 1000;

    // // 保存GPIO配置
    // if (!saveGPIOConfig()) {
    //     ESP_LOGE(TAG, "Failed to save GPIO config");
    //     return Error::GPIO_ERROR;
    // }

    // 创建监控任务
    BaseType_t ret = xTaskCreate(
        monitorTaskWrapper,
        "power_monitor",
        MONITOR_STACK_SIZE,
        this,
        MONITOR_PRIORITY,
        // &monitorTaskHandle_
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
        return Error::NOT_INITIALIZED;
    }

    initialized_ = true;
    currentMode_ = PowerMode::NORMAL;
    ESP_LOGW(TAG, "Initialized with light_sleep=%ums, deep_sleep=%ums",
        (unsigned int)config_.light_sleep_delay_ms,(unsigned int) config_.deep_sleep_delay_ms);

    return Error::OK;
}

void PowerManager::resetInactiveTimer() {
    if (!initialized_) return;
    
    lastActiveTime_ = esp_timer_get_time() / 1000;
    
    if (currentMode_ != PowerMode::NORMAL) {
        currentMode_ = PowerMode::NORMAL;
        // restoreGPIOConfig();
        if (postWakeupCb_) {
            postWakeupCb_(PowerMode::NORMAL);
        }
    }
}

PowerManager::Error PowerManager::enterLightSleep() {
    if (!initialized_) {
        return Error::NOT_INITIALIZED;
    }

    if (!canEnterSleep(PowerMode::LIGHT_SLEEP)) {
        return Error::SLEEP_ERROR;
    }

    // 执行睡眠前回调
    if (!executePreSleepCallback(PowerMode::LIGHT_SLEEP)) {
        return Error::SLEEP_ERROR;
    }

    // // 配置唤醒源
    // esp_err_t err = esp_sleep_enable_ext0_wakeup(config_.wakeup_gpio, 1);
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to configure wakeup source: %d", err);
    //     return Error::SLEEP_ERROR;
    // }

    // 配置定时器唤醒（使用deep_sleep超时作为最大睡眠时间）
    uint64_t sleep_duration = (config_.deep_sleep_delay_ms - config_.light_sleep_delay_ms) * 1000ULL;
    esp_err_t err = esp_sleep_enable_timer_wakeup(sleep_duration);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure timer wakeup: %d", err);
        return Error::SLEEP_ERROR;
    }

    // 进入浅睡眠
    currentMode_ = PowerMode::LIGHT_SLEEP;
    ESP_LOGI(TAG, "Entering light sleep");
    
    err = esp_light_sleep_start();
    
    // 睡眠返回后恢复
    currentMode_ = PowerMode::NORMAL;
    // restoreGPIOConfig();
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Light sleep failed: %d", err);
        return Error::SLEEP_ERROR;
    }

    // 执行唤醒回调
    executePostWakeupCallback(PowerMode::NORMAL);
    resetInactiveTimer();

    return Error::OK;
}

void PowerManager::enterDeepSleep() {
    if (!initialized_) return;

    if (!canEnterSleep(PowerMode::DEEP_SLEEP)) {
        return;
    }

    // 执行睡眠前回调
    if (!executePreSleepCallback(PowerMode::DEEP_SLEEP)) {
        return;
    }

    // // 配置唤醒源
    // esp_err_t err = esp_sleep_enable_ext0_wakeup(config_.wakeup_gpio, 1);
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to configure deep sleep wakeup: %d", err);
    //     return;
    // }

    currentMode_ = PowerMode::DEEP_SLEEP;
    ESP_LOGI(TAG, "Entering deep sleep");
    
    // 进入深度睡眠（此函数不会返回）
    esp_deep_sleep_start();
}

// bool PowerManager::saveGPIOConfig() {
//     if (gpioBackup_.saved) {
//         return true;
//     }

//     // 获取当前GPIO配置
//     esp_err_t err = gpio_get_config(config_.wakeup_gpio, &gpioBackup_.config);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to get GPIO config: %d", err);
//         return false;
//     }

//     gpioBackup_.saved = true;
//     ESP_LOGD(TAG, "GPIO%d configuration saved", config_.wakeup_gpio);
//     return true;
// }

// bool PowerManager::restoreGPIOConfig() {
//     if (!gpioBackup_.saved) {
//         ESP_LOGW(TAG, "No GPIO configuration to restore");
//         return false;
//     }

//     esp_err_t err = gpio_config(&gpioBackup_.config);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to restore GPIO config: %d", err);
//         return false;
//     }

//     ESP_LOGD(TAG, "GPIO%d configuration restored", config_.wakeup_gpio);
//     return true;
// }

void PowerManager::monitorTaskWrapper(void* arg) {
    static_cast<PowerManager*>(arg)->monitorTask();
}

void PowerManager::monitorTask() {
    auto& app = Application::GetInstance();
    auto codec = Board::GetInstance().GetAudioCodec();

    while (true) {
        if (app.GetDeviceState() != kDeviceStateIdle) {
            lastActiveTime_ = esp_timer_get_time() / 1000;  
        }
#if SD_IS_EXIST == 1
        if (app.GetDeviceState() == kDeviceStateIdle && app.GetSdEvent_power()) {
            lastActiveTime_ = esp_timer_get_time() / 1000;
            if (codec->output_enabled() == false)
            {
                codec->EnableOutput(true);
            }
            
        }
#endif

        if (initialized_ && config_.auto_sleep_enable) {
            int64_t current_time = esp_timer_get_time() / 1000;
            int64_t inactive_time = current_time - lastActiveTime_;

            // 检查是否需要进入深度睡眠
            if ((inactive_time >= config_.deep_sleep_delay_ms) && IS_DEEP_SLEEP) {
                ESP_LOGI(TAG, "Inactive time reached deep sleep threshold");
                enterDeepSleep();
            }
            // 检查是否需要进入浅睡眠
            // else if (inactive_time >= config_.light_sleep_delay_ms && 
            //          currentMode_ == PowerMode::NORMAL) {
            //     ESP_LOGI(TAG, "Inactive time reached light sleep threshold");
            //     enterLightSleep();
            // }
        }
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}

bool PowerManager::executePreSleepCallback(PowerMode mode) {
    if (!preSleepCb_) {
        return true;
    }

    try {
        preSleepCb_(mode);
        return true;
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Pre-sleep callback exception: %s", e.what());
        return false;
    } catch (...) {
        ESP_LOGE(TAG, "Pre-sleep callback unknown exception");
        return false;
    }
}

void PowerManager::executePostWakeupCallback(PowerMode mode) {
    if (!postWakeupCb_) {
        return;
    }

    try {
        postWakeupCb_(mode);
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Post-wakeup callback exception: %s", e.what());
    } catch (...) {
        ESP_LOGE(TAG, "Post-wakeup callback unknown exception");
    }
}

bool PowerManager::canEnterSleep(PowerMode mode) {
    if (!canSleepCb_) {
        return true;
    }

    try {
        return canSleepCb_(mode);
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Can-sleep callback exception: %s", e.what());
        return false;
    } catch (...) {
        ESP_LOGE(TAG, "Can-sleep callback unknown exception");
        return false;
    }
}
