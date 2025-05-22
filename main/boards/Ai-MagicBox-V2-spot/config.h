#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define NET_IS_WIFI_OR_ML307 NET_IS_ML307
#define NET_IS_WIFI 0
#define NET_IS_ML307 1
#if NET_IS_WIFI_OR_ML307 == NET_IS_ML307
#define ML307_RX_PIN GPIO_NUM_47
#define ML307_TX_PIN GPIO_NUM_48
#endif

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 16000

#define AUDIO_INPUT_REFERENCE    false

#define AUDIO_I2S_GPIO_MCLK      GPIO_NUM_8
#define AUDIO_I2S_GPIO_WS        GPIO_NUM_17
#define AUDIO_I2S_GPIO_BCLK      GPIO_NUM_16
#define AUDIO_I2S_GPIO_DIN       GPIO_NUM_15
#define AUDIO_I2S_GPIO_DOUT      GPIO_NUM_18

#define AUDIO_CODEC_PA_PIN       GPIO_NUM_40
#define AUDIO_CODEC_I2C_SDA_PIN  GPIO_NUM_2
#define AUDIO_CODEC_I2C_SCL_PIN  GPIO_NUM_1
#define AUDIO_CODEC_ES8311_ADDR  ES8311_CODEC_DEFAULT_ADDR

#define BOOT_BUTTON_GPIO         GPIO_NUM_0
#define KEY_BUTTON_GPIO          GPIO_NUM_12
#define LED_PIN                  GPIO_NUM_11

#define VBAT_ADC_CHANNEL         ADC_CHANNEL_9  // S3: IO10
#define MCU_VCC_CTL              GPIO_NUM_4     // set 1 to power on MCU
#define PERP_VCC_CTL             GPIO_NUM_6     // set 1 to power on peripherals

#define ADC_ATTEN                ADC_ATTEN_DB_12
#define ADC_WIDTH                ADC_BITWIDTH_DEFAULT
#define FULL_BATTERY_VOLTAGE     4100
#define EMPTY_BATTERY_VOLTAGE    3200

#endif // _BOARD_CONFIG_H_
