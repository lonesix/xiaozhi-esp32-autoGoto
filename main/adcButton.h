#ifndef _ADCBUTTON_H_
#define _ADCBUTTON_H_
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include <functional> // 为了使用std::function
#include <vector>     // 为了使用std::vector
#include <string.h>   // 为了使用memcpy

class ADCButtonNetwork {
public:
    // using CallbackType = std::function<void()>;
    static const char* TAG1; // 静态成员变量，用于日志
    adc_oneshot_unit_handle_t adc1_handle; // 单次ADC转换句柄
    adc_cali_handle_t adc1_cali_chan_handle;
        // 注册回调函数
    void registerCallback(size_t button_index, void (*callback)()) {
        if (button_index < num_buttons_) {
            callbacks_[button_index] = callback;
        } else {
            ESP_LOGE(TAG1, "Button index out of range");
        }
    }
    // 示例回调函数
    static void button1Callback() {
        ESP_LOGI(ADCButtonNetwork::TAG1, "Button 1 Callback Executed");
        // 在这里添加按钮1被按下时的处理逻辑
    }

    static void button3Callback() {
        ESP_LOGI(ADCButtonNetwork::TAG1, "Button 3 Callback Executed");
        // 在这里添加按钮3被按下时的处理逻辑
    }
    static void button4Callback() {
        ESP_LOGI(ADCButtonNetwork::TAG1, "Button 4 Callback Executed");
        // 在这里添加按钮4被按下时的处理逻辑

    }
    static void button5Callback() {
        ESP_LOGI(ADCButtonNetwork::TAG1, "Button 5 Callback Executed");
        // 在这里添加按钮5被按下时的处理逻辑
    }
    static void button6Callback() {
        ESP_LOGI(ADCButtonNetwork::TAG1, "Button 6 Callback Executed");
        // 在这里添加按钮6被按下时的处理逻辑
    }

    static void button2Callback() {
        ESP_LOGI(ADCButtonNetwork::TAG1, "Button 2 Callback Executed");
        // 在这里添加按钮2被按下时的处理逻辑
    }

    ADCButtonNetwork(const char* task_name, adc_unit_t unit, adc_channel_t channel, const uint16_t* thresholds, size_t num_buttons, uint32_t stack_depth = 2048*2, UBaseType_t priority = tskIDLE_PRIORITY + 1)
        : task_handle_(nullptr), adc_unit_(unit), adc_channel_(channel), num_buttons_(num_buttons) {
        // 复制阈值数组到内部存储
        thresholds_.resize(num_buttons);
        memcpy(thresholds_.data(), thresholds, num_buttons * sizeof(uint16_t));
        


        // adc_digi_pattern_config_t adc1_digi_pattern_config; // 定义 ADC1 的模式配置结构体
        // adc_digi_configuration_t adc1_init_config; // 定义 ADC1 的初始化配置结构体

        // /* 配置 ADC1 */
        // adc1_digi_pattern_config.atten = ADC_ATTEN_DB_11; // 设置衰减为 11dB
        // adc1_digi_pattern_config.channel = adc_channel_; // 设置通道为 ADC_ADCX_CHY
        // adc1_digi_pattern_config.unit = adc_unit_; // 设置单元为 ADC_UNIT_1
        // adc1_digi_pattern_config.bit_width = ADC_BITWIDTH_12; // 设置位宽为 12 位
        // adc1_init_config.adc_pattern = &adc1_digi_pattern_config; // 将 ADC1 的模式配置结构体赋值给初始化配置结构体的 adc_pattern 成员
        // adc_digi_controller_configure(&adc1_init_config); // 调用 adc_digi_controller_configure 函数配置 ADC 数字控制器
        //-------------ADC1 Init---------------//
        
        adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = adc_unit_,
        };
        adc_oneshot_new_unit(&init_config1, &adc1_handle);

        //-------------ADC1 Config---------------//
        adc_oneshot_chan_cfg_t config = {
            .atten = ADC_ATTEN_DB_11,
            .bitwidth = ADC_BITWIDTH_12,
        };

        adc_oneshot_config_channel(adc1_handle, adc_channel_, &config);


        // //-------------ADC1 Calibration Init---------------//
        // adc1_cali_chan_handle = NULL;

        // bool do_calibration1_chan = example_adc_calibration_init(adc_unit_, adc_channel_, ADC_ATTEN_DB_11, &adc1_cali_chan_handle);
    
        // 注册回调函数
        this->registerCallback(0, button1Callback);
        this->registerCallback(1, button2Callback);
        this->registerCallback(2, button3Callback);
        this->registerCallback(3, button4Callback);
        this->registerCallback(4, button5Callback);
        this->registerCallback(5, button6Callback);
        // // 初始化ADC配置
        // adc1_config_width(ADC_WIDTH_BIT_12);
        // adc1_config_channel_atten(adc_channel_, ADC_ATTEN_DB_11);

        // 创建FreeRTOS任务
        BaseType_t ret = xTaskCreate(
            [](void* pvParameters) -> void {
                static_cast<ADCButtonNetwork*>(pvParameters)->run();
            },
            task_name,
            stack_depth,
            this,
            priority,
            &task_handle_
        );

        if (ret != pdPASS) {
            ESP_LOGE(TAG1, "Failed to create ADCButtonNetwork task");
        }
    }

    ~ADCButtonNetwork() {
        // 删除FreeRTOS任务（注意：这应该在对象不再需要且任务可以安全删除时调用）
        if (task_handle_ != nullptr) {
            vTaskDelete(task_handle_);
        }
    }

    // 禁用复制构造函数和赋值运算符，因为任务句柄是唯一的
    ADCButtonNetwork(const ADCButtonNetwork&) = delete;
    ADCButtonNetwork& operator=(const ADCButtonNetwork&) = delete;


private:
    
    TaskHandle_t task_handle_; // FreeRTOS任务句柄
    adc_unit_t adc_unit_;
    adc_channel_t adc_channel_;
    std::vector<uint16_t> thresholds_; // 阈值数组
    void (*callbacks_[6])(void); // 回调函数数组
    size_t num_buttons_; // 按钮数量

    void run() {
        while (true) {
            int adc_value;
            adc_oneshot_read(adc1_handle, adc_channel_, &adc_value);
            // ESP_LOGI(TAG1, "ADC Value: %d", adc_value);

            for (size_t i = 0; i < num_buttons_; i++) {
                if (adc_value > thresholds_[i] &&
                    (i == num_buttons_ - 1 || adc_value <= thresholds_[i + 1]) &&
                    adc_value < 4090) {
                    ESP_LOGI(TAG1, "ADC Value: %d", adc_value);
                    ESP_LOGI(TAG1, "thresholds_ADC Value: %d", thresholds_[i]);
                    ESP_LOGI(TAG1, "Button %zu Pressed", i + 1);
                    if (callbacks_[i]) {
                        callbacks_[i](); // 调用回调函数
                    }
                    break; // 假设一次只有一个按键被按下，找到后退出循环
                }
            }

            vTaskDelay(pdMS_TO_TICKS(200)); // 延时以避免过于频繁的日志输出
        }
    }
};


#endif // _APPLICATION_H_

// 定义静态成员变量TAG
// const char* ADCButtonNetwork::TAG = "adc_button_network";

// // 示例回调函数
// void button1Callback() {
//     ESP_LOGI(ADCButtonNetwork::TAG, "Button 1 Callback Executed");
//     // 在这里添加按钮1被按下时的处理逻辑
// }

// void button2Callback() {
//     ESP_LOGI(ADCButtonNetwork::TAG, "Button 2 Callback Executed");
//     // 在这里添加按钮2被按下时的处理逻辑
// }

// // 在app_main中使用ADCButtonNetwork类
// void app_main(void) {
//     // 假设的按键阈值数组
//     const uint16_t thresholds[] = {
//         1000, // 按键1的阈值
//         2000, // 按键2的阈值
//         // ... 可以添加更多按键的阈值
//     };
//     size_t num_buttons = sizeof(thresholds) / sizeof(thresholds[0]);

//     // 创建ADCButtonNetwork对象，并自动启动任务
//     ADCButtonNetwork adc_button_network("ADCButtonTask", ADC1, ADC1_CHANNEL_0, thresholds, num_buttons);

//     // 注册回调函数
//     adc_button_network.registerCallback(0, button1Callback);
//     adc_button_network.registerCallback(1, button2Callback);
//     // 可以为更多按钮注册回调函数...

//     // 注意：这里不需要显式调用run()，因为任务已经在构造函数中启动了。
//     // 相反，我们应该让app_main函数尽快返回，以便FreeRTOS可以接管调度。
// }   `