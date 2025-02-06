#include "uart_comm.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/task.h"
#include "string.h"

#include "application.h"  // 添加这个头文件，确保 Application 类可以被识别


const char* UartComm::TAG = "UART_Comm";

// 构造函数
UartComm::UartComm(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate, int buf_size)
    : uart_num(uart_num), tx_pin(tx_pin), rx_pin(rx_pin), baud_rate(baud_rate), buf_size(buf_size)
{
    
    rx_buffer = (uint8_t*) malloc(buf_size);
    
}

// 析构函数
UartComm::~UartComm()
{
    free(rx_buffer);
}

// 初始化串口
void UartComm::init()
{
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num, buf_size * 2, buf_size * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART%d initialized with baud rate %d", uart_num, baud_rate);
}

// 发送数据
void UartComm::sendData(const char* data)
{
    uart_write_bytes(uart_num, data, strlen(data));
    ESP_LOGI(TAG, "Sent data: %s", data);
}

// 发送数据
void UartComm::sendData(cJSON* json)
{
   // 将 cJSON 对象转换为字符串
    char* json_str = cJSON_PrintUnformatted(json);  // 不格式化 JSON，节省空间
    
    if (json_str != NULL)
    {
        // 发送 JSON 字符串到串口
        uart_write_bytes(uart_num, json_str, strlen(json_str));
        // uart_write_bytes(uart_num, "++", strlen("++"));
        // 打印日志
        ESP_LOGI(TAG, "Sent JSON: %s", json_str);
        
        // 释放动态分配的内存
        free(json_str);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to convert cJSON to string");
    }
}

// 接收数据
void UartComm::receiveData()
{
    int length = uart_read_bytes(uart_num, rx_buffer, buf_size, pdMS_TO_TICKS(1000));  // 超时1秒
    if (length > 0) {
        rx_buffer[length] = '\0';  // 确保字符串结尾
        ESP_LOGI(TAG, "Received data: %s", rx_buffer);
    }
}

void UartComm::receiveDataCjson() {
    // 清空接收缓冲区以避免读取旧数据
    memset(rx_buffer, 0, buf_size);
    int length = uart_read_bytes(uart_num, rx_buffer, buf_size, pdMS_TO_TICKS(10));  // 超时
    if (length > 0) {
        rx_buffer[length] = '\0';  // 确保字符串结尾
        ESP_LOGI(TAG, "Received data: %s", rx_buffer);

        // 尝试解析收到的数据为 cJSON 对象
        cJSON* root = cJSON_Parse((const char*)rx_buffer);
        if (root == nullptr) {
            ESP_LOGE(TAG, "Failed to parse received data as JSON");
            return;
        }

        // 调用外部接口处理解析后的 JSON
        Application::GetInstance().Schedule([root]() {
            Application::GetInstance().ProcessReceivedJson(root);
            cJSON_Delete(root);  // 在任务执行完后删除 JSON 对象
        });
    }
}
