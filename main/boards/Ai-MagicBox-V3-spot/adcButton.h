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
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

// 全局ADC校准互斥锁

static const char* ADC_CALI_TAG = "ADC_CALI";

class ADCButtonNetwork {
public:
    // using CallbackType = std::function<void()>;
    static const char* TAG1; // 静态成员变量，用于日志
    
    adc_cali_handle_t adc1_cali_chan_handle = NULL;
    SemaphoreHandle_t adc_cali_mutex = nullptr; // 全局ADC校准互斥锁
        // 注册回调函数
    void registerCallback(size_t button_index,std::function<void()> callback) {
        if (button_index < num_buttons_) {
            callbacks_[button_index] = callback;
        } else {
            ESP_LOGE(TAG1, "Button index out of range");
        }
    }

    esp_err_t adc_cali_mutex_init(void) {
        if (adc_cali_mutex == nullptr) {
            adc_cali_mutex = xSemaphoreCreateMutex();
            if (adc_cali_mutex == nullptr) {
                ESP_LOGE(ADC_CALI_TAG, "Failed to create ADC calibration mutex");
                return ESP_ERR_NO_MEM;
            }
            ESP_LOGI(ADC_CALI_TAG, "ADC calibration mutex initialized");
        }
        return ESP_OK;
    }
    void adc_cali_mutex_deinit(void) {
        if (adc_cali_mutex != nullptr) {
            vSemaphoreDelete(adc_cali_mutex);
            adc_cali_mutex = nullptr;
            ESP_LOGI(ADC_CALI_TAG, "ADC calibration mutex destroyed");
        }
    }
    
    esp_err_t adc_cali_raw_to_voltage_safe(adc_cali_handle_t handle, int raw, int *voltage) {
        // 检查互斥锁是否已初始化
        if (adc_cali_mutex == nullptr) {
            ESP_LOGE(ADC_CALI_TAG, "ADC calibration mutex not initialized");
            return ESP_ERR_INVALID_STATE;
        }
        
        // 参数验证
        if (handle == nullptr || voltage == nullptr) {
            ESP_LOGE(ADC_CALI_TAG, "Invalid parameters");
            return ESP_ERR_INVALID_ARG;
        }
        
        esp_err_t ret = ESP_FAIL;
        
        // 获取互斥锁，超时时间max_wait_time
        if (xSemaphoreTake(adc_cali_mutex, portMAX_DELAY) == pdTRUE) {
            // 调用原始的ADC校准函数
            ret = adc_cali_raw_to_voltage(handle, raw, voltage);
            
            // 释放互斥锁
            xSemaphoreGive(adc_cali_mutex);
            
            if (ret != ESP_OK) {
                ESP_LOGW(ADC_CALI_TAG, "ADC calibration failed: %s", esp_err_to_name(ret));
            }
        } else {
            ESP_LOGE(ADC_CALI_TAG, "Failed to acquire ADC calibration mutex within timeout");
            ret = ESP_ERR_TIMEOUT;
        }
        
        return ret;
    }
    
    SemaphoreHandle_t get_adc_cali_mutex(void) {
        return adc_cali_mutex; // 返回全局ADC校准互斥锁句柄
    }
    void set_adc_cali_handle(adc_cali_handle_t handle) {
        adc1_cali_chan_handle = handle;
    }

    ADCButtonNetwork(adc_oneshot_unit_handle_t adc1_handle,const char* task_name,adc_unit_t uuid,  adc_channel_t channel, const uint16_t* thresholds, size_t num_buttons, uint32_t stack_depth = 2048*2, UBaseType_t priority = tskIDLE_PRIORITY + 1)
        :adc1_handle_(adc1_handle), task_handle_(nullptr),adc_unit_(uuid) , adc_channel_(channel), num_buttons_(num_buttons) {
        // 复制阈值数组到内部存储
        thresholds_.resize(num_buttons);
        memcpy(thresholds_.data(), thresholds, num_buttons * sizeof(uint16_t));
        


        //-------------ADC1 Init---------------//
        
        // adc_oneshot_unit_init_cfg_t init_config1 = {
        //     .unit_id = adc_unit_,
        // };
        // adc_oneshot_new_unit(&init_config1, &adc1_handle);

        //-------------ADC1 Config---------------//
        adc_oneshot_chan_cfg_t config = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };

        adc_oneshot_config_channel(adc1_handle_, adc_channel_, &config);

        adc_cali_mutex_init();
    // // 初始化校准方案
    // adc1_cali_chan_handle = NULL;
    // adc_cali_curve_fitting_config_t cali_config = {
    //     .unit_id = adc_unit_,
    //     .atten = ADC_ATTEN_DB_12,
    //     .bitwidth = ADC_BITWIDTH_12,
    // };
    // esp_err_t rett = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_chan_handle);


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
        adc_cali_mutex_deinit();
    }

    // 禁用复制构造函数和赋值运算符，因为任务句柄是唯一的
    ADCButtonNetwork(const ADCButtonNetwork&) = delete;
    ADCButtonNetwork& operator=(const ADCButtonNetwork&) = delete;


private:
    adc_oneshot_unit_handle_t adc1_handle_; // 单次ADC转换句柄
    TaskHandle_t task_handle_; // FreeRTOS任务句柄
    adc_unit_t adc_unit_;
    adc_channel_t adc_channel_;
    std::vector<uint16_t> thresholds_; // 阈值数组
    std::function<void()> callbacks_[6]; // 回调函数数组
    size_t num_buttons_; // 按钮数量

    void run() {
        while (true) {

            int adc_value;
            esp_err_t ret = adc_oneshot_read(adc1_handle_, adc_channel_, &adc_value);
            // ESP_LOGI(TAG1, "ADC Value: %d", adc_value);
            // 将原始值转换为电压
            int voltage = 0;
            if (adc1_cali_chan_handle != NULL)
            {
                /* code */
                // 新的受保护调用
                ret = adc_cali_raw_to_voltage_safe(adc1_cali_chan_handle, adc_value, &voltage);
                if (ret != ESP_OK) {
                    ESP_LOGE("ADC_BUTTON", "ADC calibration failed: %s", esp_err_to_name(ret));
                    // 错误处理逻辑
                    
                }
            }
            if (ret == ESP_OK)
            {
                const int Voltage_offset = 100;

                for (size_t i = 0; i < num_buttons_; i++) {
                    if (voltage > (thresholds_[i]-Voltage_offset) &&
                        (i == num_buttons_ - 1 || voltage <= (thresholds_[i + 1]-Voltage_offset)) &&
                        voltage < 2700) {
                        ESP_LOGI(TAG1, "ADC Value: %d", adc_value);
                        ESP_LOGI(TAG1, "ADC Voltage: %dmV", voltage);
                        ESP_LOGI(TAG1, "thresholds_ADC Voltage: %dmV", thresholds_[i]-Voltage_offset);
                        ESP_LOGI(TAG1, "Button %zu Pressed", i + 1);
                        if (callbacks_[i]) {
                            callbacks_[i](); // 调用回调函数
                        }
                        break; // 假设一次只有一个按键被按下，找到后退出循环
                    }
                }
            }
            



            vTaskDelay(pdMS_TO_TICKS(300)); // 延时以避免过于频繁的日志输出
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