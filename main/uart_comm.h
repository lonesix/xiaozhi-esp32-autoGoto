#ifndef UART_COMM_H
#define UART_COMM_H

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <cJSON.h>

// 串口配置
#define UART_NUM    UART_NUM_1        // 使用UART1
#define TX_PIN      11                // TX引脚
#define RX_PIN      10                // RX引脚
#define BUF_SIZE    1024              // 缓冲区大小
#define BAUD_RATE   115200            // 波特率

#define CAMERA_UART_NUM    UART_NUM_2        // 使用UART1
#define CAMERA_TX_PIN      17                // TX引脚
#define CAMERA_RX_PIN      18                // RX引脚
#define CAMERA_BUF_SIZE    1024              // 缓冲区大小
#define CAMERA_BAUD_RATE   9600            // 波特率

// 串口通信类
class UartComm {
public:
    UartComm(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate = 115200, int buf_size = 1024);
    ~UartComm();
    
    void init();                             // 初始化串口
    void sendData(const char* data);         // 发送数据
    void sendData(cJSON* json);
    void receiveData();                      // 接收数据
    void receiveDataCjson();
    void receiveCameraDataCjson();

private:
    uart_port_t uart_num;                    // UART端口
    int tx_pin, rx_pin;                      // TX、RX引脚
    int baud_rate;                           // 波特率
    int buf_size;                            // 缓冲区大小
    uint8_t* rx_buffer;                      // 接收缓冲区
    static const char* TAG;                  // 日志标签
};

#endif // UART_COMM_H
