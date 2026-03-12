#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_vfs.h"
#include <fstream>

#define IS_V4  1
#ifdef IS_V4
#define CHARGE_GPIO              GPIO_NUM_41
#define VOLUME_ADD_BUTTON_GPIO   GPIO_NUM_11
#define VOLUME_SUB_BUTTON_GPIO   GPIO_NUM_10
#else
#define CHARGE_GPIO              GPIO_NUM_38
#define VOLUME_ADD_BUTTON_GPIO   GPIO_NUM_10
#define VOLUME_SUB_BUTTON_GPIO   GPIO_NUM_42
#endif

#define NET_IS_WIFI_OR_ML307 NET_IS_WIFI
#define NET_IS_WIFI 0
#define NET_IS_ML307 1
// #if NET_IS_WIFI_OR_ML307 == NET_IS_ML307
#define ML307_RX_PIN GPIO_NUM_47
#define ML307_TX_PIN GPIO_NUM_48
// #endif
#define IMU_BMI270_IS_EXIST 0 // 1:exist 0:not exist
#define TISHIYIN_IS_EXIST 1 // 1:exist 0:not exist
#define SLEEP_MODE_IS_EXIST 0 // 1:exist 0:not exist
#define SD_IS_EXIST 0 // 1:exist 0:not exist
#define CHARGE_QUWEI_IS_EXIST 1 // 1:exist 0:not exist
#define FASTBEE_MCP_IS_EXIST 1 // 1:exist 0:not exist


enum SERVERCONNECTIONMETHOD{
    PROTOCOL_NONE = 0,
    PROTOCOL_MQTT,
    PROTOCOL_WEBSOCKET_XIAOZHI,
    PROTOCOL_WEBSOCKET_QJG,
};
#define ServerConnectionMethod  PROTOCOL_MQTT //选择服务器连接方式
#define QJG_WS_SERVER_OPTIONS  (PROTOCOL_WEBSOCKET_QJG == ServerConnectionMethod)? 1:0
#define CHANGE_WS_SERVER_URL  (CONFIG_BOARD_TYPE_AI_MAGIC_BOX_V3_SPOT && (ServerConnectionMethod == PROTOCOL_WEBSOCKET_QJG))? 1:0

#define QJG_WS_SERVER_URL "wss://aihub.nankai.edu.cn/terminal-server"

#define XIAOZHI_WS_SERVER_URL "wss://api.tenclass.net/xiaozhi/v1/"


#define AUDIO_INPUT_SAMPLE_RATE  24000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

#define AUDIO_INPUT_REFERENCE    false

#define AUDIO_I2S_GPIO_MCLK      GPIO_NUM_NC
#define AUDIO_I2S_GPIO_WS        GPIO_NUM_6
#define AUDIO_I2S_GPIO_BCLK      GPIO_NUM_4
#define AUDIO_I2S_GPIO_DIN       GPIO_NUM_5
#define AUDIO_I2S_GPIO_DOUT      GPIO_NUM_7

#define AUDIO_CODEC_PA_PIN       GPIO_NUM_8
#define AUDIO_CODEC_I2C_SDA_PIN  GPIO_NUM_2
#define AUDIO_CODEC_I2C_SCL_PIN  GPIO_NUM_1
#define AUDIO_CODEC_ES8311_ADDR  ES8311_CODEC_DEFAULT_ADDR

#define EXTERNAL_VOICE_WAKE_UP_GPIO GPIO_NUM_40
#define BOOT_BUTTON_GPIO         GPIO_NUM_0
#define KEY_BUTTON_GPIO          GPIO_NUM_11 //
#define LED_PIN                  GPIO_NUM_3
#define WAI_KEY_GPIO             GPIO_NUM_14
#define WAI4_KEY_GPIO             GPIO_NUM_21

// #define VOLUME_BUTTON_GPIO       GPIO_NUM_10
// #define VOLUME_BUTTON_CHANNEL      ADC_CHANNEL_9
#define IMU_BMI270_INT_PIN       GPIO_NUM_13
#define VCC_4G_EXTERNAL_VOICE_WAKE_UP_EN GPIO_NUM_12


#define VBAT_ADC_CHANNEL         ADC_CHANNEL_8  // S3: IO9
#define MCU_VCC_CTL              GPIO_NUM_NC     // set 1 to power on MCU
#define PERP_VCC_CTL             GPIO_NUM_15     // set 1 to power on peripherals

#define ADC_ATTEN                ADC_ATTEN_DB_12
#define ADC_WIDTH                ADC_BITWIDTH_12
#define FULL_BATTERY_VOLTAGE     4100
#define EMPTY_BATTERY_VOLTAGE    3200

//SD卡
#define BSP_SD_CLK          (GPIO_NUM_17)
#define BSP_SD_CMD          (GPIO_NUM_18)
#define BSP_SD_D0           (GPIO_NUM_16)
#define MOUNT_POINT              "/sdcard"
#define EXAMPLE_MAX_CHAR_SIZE    64
#define AUDIO_FILE_EXTENSION    ".p3"
#endif // _BOARD_CONFIG_H_
