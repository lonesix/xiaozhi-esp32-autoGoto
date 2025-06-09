
#pragma once

#include <driver/gpio.h>
#include <esp_err.h>
#include <functional>
#include <memory>

#define IS_DEEP_SLEEP 1 // 1:exist 0:not exist
class PowerManager {
public:
    // 电源模式枚举
    enum class PowerMode {
        NORMAL,
        LIGHT_SLEEP,
        DEEP_SLEEP
    };

    // 错误码定义
    enum class Error {
        OK = 0,
        NOT_INITIALIZED,
        INVALID_PARAMS,
        GPIO_ERROR,
        SLEEP_ERROR
    };

    // 配置结构
    struct Config {
        // gpio_num_t wakeup_gpio;           // 唤醒GPIO
        // gpio_config_t wakeup_gpio_config;            // 当前GPIO配置
        uint32_t light_sleep_delay_ms;    // 进入浅睡眠的不活动时间
        uint32_t deep_sleep_delay_ms;     // 进入深睡眠的不活动时间
        bool auto_sleep_enable{true};     // 是否启用自动睡眠
    };

    // 回调函数类型定义
    using PreSleepCallback = std::function<void(PowerMode)>;
    using PostWakeupCallback = std::function<void(PowerMode)>;
    using CanSleepCallback = std::function<bool(PowerMode)>;

    // 获取单例
    static PowerManager& getInstance() {
        static PowerManager instance;
        return instance;
    }

    // 删除拷贝构造和赋值操作
    PowerManager(const PowerManager&) = delete;
    PowerManager& operator=(const PowerManager&) = delete;

    // 初始化
    Error init(const Config& config);

    // 电源管理函数
    void resetInactiveTimer();
    Error enterLightSleep();
    void enterDeepSleep(); // 注意：此函数不会返回

    // 回调注册
    void setPreSleepCallback(PreSleepCallback cb) { preSleepCb_ = cb; }
    void setPostWakeupCallback(PostWakeupCallback cb) { postWakeupCb_ = cb; }
    void setCanSleepCallback(CanSleepCallback cb) { canSleepCb_ = cb; }

    // 状态查询
    PowerMode getCurrentMode() const { return currentMode_; }
    bool isInitialized() const { return initialized_; }
    int64_t getLastActiveTime() const { return lastActiveTime_; }

    // 配置相关
    void enableAutoSleep(bool enable) { config_.auto_sleep_enable = enable; }
    bool isAutoSleepEnabled() const { return config_.auto_sleep_enable; }

protected:
    // 构造函数保护
    PowerManager() : initialized_(false), currentMode_(PowerMode::NORMAL) {}

private:
    // 内部函数
    // bool saveGPIOConfig();
    // bool restoreGPIOConfig();
    static void monitorTaskWrapper(void* arg);
    void monitorTask();
    bool executePreSleepCallback(PowerMode mode);
    void executePostWakeupCallback(PowerMode mode);
    bool canEnterSleep(PowerMode mode);

    // 内部变量
    Config config_;
    bool initialized_{false};
    PowerMode currentMode_{PowerMode::NORMAL};
    int64_t lastActiveTime_{0};
    
    // GPIO配置备份
    struct {
        gpio_config_t config;
        bool saved{false};
    } gpioBackup_;

    // 任务句柄
    // TaskHandle_t monitorTaskHandle_{nullptr};

    // 回调函数
    PreSleepCallback preSleepCb_;
    PostWakeupCallback postWakeupCb_;
    CanSleepCallback canSleepCb_;

    // 后续扩展用的保留字段
    struct {
        uint32_t reserved1;
        uint32_t reserved2;
        void* reserved_ptr;
    } extension_;
};

// 全局访问宏
#define POWER_MANAGER PowerManager::getInstance()
