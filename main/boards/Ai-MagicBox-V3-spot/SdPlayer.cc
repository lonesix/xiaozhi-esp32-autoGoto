#include "iot/thing.h"
#include "board.h"
#include "audio_codec.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "config.h"
#include <esp_log.h>
#include "application.h"
#define TAG "SD_PLAYER"


namespace iot {

// 这里仅定义 SD_PLAYER 的属性和方法，不包含具体的实现
class SdPlayer : public Thing {
public:
SdPlayer() : Thing("sd_player", "可以播放sd卡音乐的机器人") {
        // // 定义设备的属性
        // properties_.AddNumberProperty("volume", "Current audio volume value", [this]() -> int {
        //     auto codec = Board::GetInstance().GetAudioCodec();
        //     return codec->output_volume();
        // });

        // 初始化sd卡   
        // Board::GetInstance().InitSDCard();
        // 定义设备可以被远程执行的指令
        bsp_sdcard_mount();
        methods_.AddMethod("play", "播放音乐", ParameterList({
            Parameter("file_name", "音乐名称", kValueTypeString, true)
        }), [this](const ParameterList& parameters) {
            // auto codec = Board::GetInstance().GetAudioCodec();
            // codec->SetOutputVolume(static_cast<uint8_t>(parameters["volume"].number()));
            playMusic(static_cast<std::string>(parameters["file_name"].string()));
        });
    }

    esp_err_t bsp_sdcard_mount(void) {
        esp_err_t ret;

        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = false,   // 如果挂载不成功是否需要格式化SD卡
            .max_files = 5, // 允许打开的最大文件数
            .allocation_unit_size = 16 * 1024  // 分配单元大小
        };
        
        sdmmc_card_t *card;
        const char mount_point[] = MOUNT_POINT;
        ESP_LOGI(TAG, "Initializing SD card");
        ESP_LOGI(TAG, "Using SDMMC peripheral");
    
        sdmmc_host_t host = SDMMC_HOST_DEFAULT(); // SDMMC主机接口配置
        sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT(); // SDMMC插槽配置
        slot_config.width = 1;  // 设置为1线SD模式
        slot_config.clk = BSP_SD_CLK; 
        slot_config.cmd = BSP_SD_CMD;
        slot_config.d0 = BSP_SD_D0;
        slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP; // 打开内部上拉电阻
    
        ESP_LOGI(TAG, "Mounting filesystem");
        ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card); // 挂载SD卡
    
        if (ret != ESP_OK) {  // 如果没有挂载成功
            if (ret == ESP_FAIL) { // 如果挂载失败
                ESP_LOGE(TAG, "Failed to mount filesystem. ");
            } else { // 如果是其它错误 打印错误名称
                ESP_LOGE(TAG, "Failed to initialize the card (%s). ", esp_err_to_name(ret));
            }
            return ret;
        }
        ESP_LOGI(TAG, "Filesystem mounted"); // 提示挂载成功
        sdmmc_card_print_info(stdout, card); // 终端打印SD卡的一些信息
        return ret;
    }
    void playMusic(std::string fileName) {
        // bsp_sdcard_mount();
        auto &app = Application::GetInstance();
        if (fileName == "未知歌曲")
        {
            app.PlaySoundFromFile(2);
        }else
        {
            char file_path[256];
            snprintf(file_path,sizeof(file_path), "%s/%s.p3", MOUNT_POINT, fileName.c_str());
            ESP_LOGW(TAG, "Playing file: %s", file_path);
            
            app.PlaySoundFromFile(file_path);
        }
        
        

    }
};

} // namespace iot

DECLARE_THING(SdPlayer);
