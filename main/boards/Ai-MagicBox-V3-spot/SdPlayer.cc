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
    bool sdIsInited_ = false;
    bool sdIsInited() const { return sdIsInited_; }//返回sd卡是否初始化成功
    unsigned int musicNumber_ = 0;
    unsigned int GetmusicNumber(){
        DIR *dir = opendir(MOUNT_POINT);
        if (dir == NULL)
        {
        ESP_LOGE(TAG, "Failed to open directory: %s", MOUNT_POINT);
        musicNumber_ = 0;
        return musicNumber_;
        }
        struct dirent *entry;
        std::vector<std::string> audio_files; // 用来存储符合条件的音频文件
        // 遍历目录中的文件
        while ((entry = readdir(dir)) != NULL)
        {
            printf("file name: %s\n", entry->d_name);
        // 只处理扩展名为 .p3 的文件
        if (strstr(entry->d_name, AUDIO_FILE_EXTENSION))
        {
            audio_files.push_back(entry->d_name); // 将符合条件的文件存入容器
        }
        }
        ESP_LOGE(TAG, " file number: %d", audio_files.size());
        closedir(dir);
        if (audio_files.empty())
        {
        ESP_LOGE(TAG, "No valid audio file found.");
        musicNumber_ = 0;
        return musicNumber_;
        }
        musicNumber_ = audio_files.size();
        return musicNumber_;
    }
    sdmmc_card_t *card; // SD卡句柄
    bool check_sd_card_status() {
        if (card == nullptr) {
            ESP_LOGW(TAG, "SD card handle is null");
            return false;
        }
        
        // 检查卡是否就绪
        esp_err_t ret = sdmmc_get_status(card);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "SD card status check failed: %s", esp_err_to_name(ret));
            return false;
        }

        return true;
    }
    EventGroupHandle_t sdPlayerMonitor_EventGroup_;//SD卡播放监控事件组
    std::string currentPlayingMusic_;//当前播放的音乐self->sdIsInited_ = self->check_sd_card_status();
    int currentPlayingMusicNumber_ = 0;//当前播放的音乐编号
    // static SdPlayer& GetInstance() {
    //     static SdPlayer instance;
    //     return instance;
    // }
SdPlayer() : Thing("sd_player", "可以播放sd卡音乐的机器人") {

        esp_err_t ret = bsp_sdcard_mount();
        if (ret == ESP_OK)    
        {
            sdIsInited_ = true;
            ESP_LOGI(TAG, "SD card mounted successfully");
            GetmusicNumber();
            
        }
        Create_sdPlayerMonitor_Task();

        // 定义设备的属性
        properties_.AddBooleanProperty("sdIsInited", "sd卡初始化状态，true为初始化成功。false为初始化失败，请提示用户可能因为未插入sd卡。", [this]() -> bool {
            return sdIsInited();
        });
        
        // 定义设备的属性
        properties_.AddNumberProperty("musicNumber", "sd卡中的可播放的音乐数量", [this]() -> int {
            return musicNumber_;
        });
        methods_.AddMethod("sdInit", "初始化sd卡,sdIsInited为false才能调用,调用前需要经过用户确认，调用后仅尝试初始化,sd卡状态未知", ParameterList(), [this](const ParameterList& parameters) {
            if (!sdIsInited_)
            {
                /* code */            
                esp_err_t ret = bsp_sdcard_mount();
                if (ret == ESP_OK)    
                {
                    sdIsInited_ = true;
                    ESP_LOGI(TAG, "SD card mounted successfully");
                    GetmusicNumber();
                    
                }
                else
                {
                    ESP_LOGE(TAG, "SD card mount failed: %s", esp_err_to_name(ret));
                }

                
            }
            

        });
        methods_.AddMethod("play_str", "播放音乐(按歌曲名称),sdIsInited为true才能调用。", ParameterList({
            Parameter("file_name", "音乐名称，若不知道歌名请设置为：未知歌曲", kValueTypeString, true)
        }), [this](const ParameterList& parameters) {
            // auto codec = Board::GetInstance().GetAudioCodec();
            // codec->SetOutputVolume(static_cast<uint8_t>(parameters["volume"].number()));
            currentPlayingMusic_ = static_cast<std::string>(parameters["file_name"].string());
            setSdEventBit_str();
            
        });

        methods_.AddMethod("play_num", "播放音乐(按歌曲编号),sdIsInited为true才能调用。", ParameterList({
            Parameter("file_number", "歌曲编号，歌曲编号不能超过音乐数量", kValueTypeNumber, true)
        }), [this](const ParameterList& parameters) {
            // auto codec = Board::GetInstance().GetAudioCodec();
            // codec->SetOutputVolume(static_cast<uint8_t>(parameters["volume"].number()));
            currentPlayingMusicNumber_ = static_cast<int>(parameters["file_number"].number());
            // playMusic_num(static_cast<int>(parameters["file_number"].number()));
            setSdEventBit_num();
        });
    }

    esp_err_t bsp_sdcard_mount(void) {
        esp_err_t ret;

        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = false,   // 如果挂载不成功是否需要格式化SD卡
            .max_files = 5, // 允许打开的最大文件数
            .allocation_unit_size = 16 * 1024  // 分配单元大小
        };
        
        // sdmmc_card_t *card;
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
    void playMusic_str(std::string fileName) {
        sdIsInited_ = check_sd_card_status();
        if (sdIsInited_ == false)
        {
            ESP_LOGE(TAG, "playMusic:SD card is not inited, please init SD card first.");
            return;
        }
        
        auto &app = Application::GetInstance();
        if (fileName == "未知歌曲")
        {
            app.PlaySoundFromFile(0);
        }else
        {
            char file_path[256];
            snprintf(file_path,sizeof(file_path), "%s/%s.p3", MOUNT_POINT, fileName.c_str());
            ESP_LOGW(TAG, "Playing file: %s", file_path);
            
            app.PlaySoundFromFile(file_path);
        }
    }

    void playMusic_num(int fileNumber) {
        sdIsInited_ = check_sd_card_status();
        if (sdIsInited_ == false)
        {
            ESP_LOGE(TAG, "playMusic:SD card is not inited, please init SD card first.");
            return;
        }
        auto &app = Application::GetInstance();

        app.PlaySoundFromFile(fileNumber-1);

    } 
    void SetSdEventHandle(EventGroupHandle_t eventGroup) {
        sdPlayerMonitor_EventGroup_ = eventGroup;
    }
    void SetSdEventBit(int bit){
        if (sdPlayerMonitor_EventGroup_ != NULL)
        {
            xEventGroupSetBits(sdPlayerMonitor_EventGroup_, bit);
        }    
    }
    const int SDPLAYERMONITOR_IDLE_BIT = BIT0;
    const int SDPLAYERMONITOR_SPEAKINGTOSTOP_BIT = BIT1;
    const int SDPLAYERMONITOR_SPEAKINGTOIOT_BIT = BIT2;
    const int SDPLAYERMONITOR_SPEAKINGTOIOT_STR_BIT = BIT3;
    const int SDPLAYERMONITOR_SPEAKINGTOIOT_NUM_BIT = BIT4;
    const int SDPLAYE_PLAYING_BIT = BIT5;
    void setSdEventBit_str(){
        if (sdPlayerMonitor_EventGroup_ != NULL)
        {
            xEventGroupSetBits(sdPlayerMonitor_EventGroup_, SDPLAYERMONITOR_SPEAKINGTOIOT_BIT);
            xEventGroupSetBits(sdPlayerMonitor_EventGroup_, SDPLAYERMONITOR_SPEAKINGTOIOT_STR_BIT);
        }
    }
    void setSdEventBit_num(){
        if (sdPlayerMonitor_EventGroup_ != NULL)
        {
            xEventGroupSetBits(sdPlayerMonitor_EventGroup_, SDPLAYERMONITOR_SPEAKINGTOIOT_BIT);
            xEventGroupSetBits(sdPlayerMonitor_EventGroup_, SDPLAYERMONITOR_SPEAKINGTOIOT_NUM_BIT);
        }
    }
    void Create_sdPlayerMonitor_Task() {


        xTaskCreate([](void* arg) {
             // 将pvParameters转换回this指针
            SdPlayer* self = static_cast<SdPlayer*>(arg);
            //定义事件组句柄
            EventGroupHandle_t sdPlayerMonitor_EventGroup;
            //创建事件组
            sdPlayerMonitor_EventGroup = xEventGroupCreate();
            self->SetSdEventHandle(sdPlayerMonitor_EventGroup);
            //定义事件组位
            const int SDPLAYERMONITOR_IDLE_BIT = BIT0;
            const int SDPLAYERMONITOR_SPEAKINGTOSTOP_BIT = BIT1;
            const int SDPLAYERMONITOR_SPEAKINGTOIOT_BIT = BIT2;
            const int SDPLAYERMONITOR_SPEAKINGTOIOT_STR_BIT = BIT3;
            const int SDPLAYERMONITOR_SPEAKINGTOIOT_NUM_BIT = BIT4;
            const int SDPLAYE_PLAYING_BIT = BIT5;
            auto &app = Application::GetInstance();
            app.SetSdEventHandle(sdPlayerMonitor_EventGroup);
            while (1) {
                
                //等待事件组位

                EventBits_t receivedBits = xEventGroupWaitBits(
                    sdPlayerMonitor_EventGroup,
                    SDPLAYERMONITOR_SPEAKINGTOSTOP_BIT | SDPLAYERMONITOR_SPEAKINGTOIOT_BIT , 
                    pdTRUE,// pdTRUE, // pdTRUE表示等待位被置位后清除该位，pdFALSE表示不清除
                    pdFALSE,// pdFALSE, // pdTRUE表示逻辑与，pdFALSE表示逻辑或
                    portMAX_DELAY);


                if (receivedBits & SDPLAYERMONITOR_SPEAKINGTOIOT_BIT) {
                    ESP_LOGI(TAG, "SDPlayerMonitor: speaking to iot");
                    //等待事件组位
                    if (receivedBits & SDPLAYERMONITOR_SPEAKINGTOSTOP_BIT) {
                        ESP_LOGI(TAG, "SDPlayerMonitor: speaking to iot and speaking to stop");
                        
                        if (app.GetDeviceState() == DeviceState::kDeviceStateSpeaking)
                        {
                            //等待播放队列为空
                            app.WaitSoundToFinish();
                            xEventGroupSetBits(sdPlayerMonitor_EventGroup, SDPLAYE_PLAYING_BIT);
                        }
                    }else
                    {
                        receivedBits = xEventGroupWaitBits(
                            sdPlayerMonitor_EventGroup,
                            SDPLAYERMONITOR_SPEAKINGTOSTOP_BIT  , 
                            pdTRUE,// pdTRUE, // pdTRUE表示等待位被置位后清除该位，pdFALSE表示不清除
                            pdFALSE,// pdFALSE, // pdTRUE表示逻辑与，pdFALSE表示逻辑或
                            portMAX_DELAY);
                    }
  
                }else if (receivedBits & SDPLAYERMONITOR_SPEAKINGTOSTOP_BIT) {
                    ESP_LOGI(TAG, "SDPlayerMonitor: speaking to stop");
                    if (app.GetDeviceState() == DeviceState::kDeviceStateSpeaking)
                    {
                        //等待播放队列为空
                        app.WaitSoundToFinish();
                    }
                    
                }
                //等待事件组位
                receivedBits = xEventGroupWaitBits(
                    sdPlayerMonitor_EventGroup,
                    SDPLAYERMONITOR_SPEAKINGTOIOT_STR_BIT | SDPLAYERMONITOR_SPEAKINGTOIOT_NUM_BIT, 
                    pdTRUE,// pdTRUE, // pdTRUE表示等待位被置位后清除该位，pdFALSE表示不清除
                    pdFALSE,// pdFALSE, // pdTRUE表示逻辑与，pdFALSE表示逻辑或
                    pdMS_TO_TICKS(50));
                if (receivedBits & SDPLAYERMONITOR_SPEAKINGTOIOT_STR_BIT) {
                    if (app.GetDeviceState() == DeviceState::kDeviceStateSpeaking)
                    {
                        self->playMusic_str(self->currentPlayingMusic_);
                    }
                }
                if (receivedBits & SDPLAYERMONITOR_SPEAKINGTOIOT_NUM_BIT) {
                    if (app.GetDeviceState() == DeviceState::kDeviceStateSpeaking)
                    {
                        self->playMusic_num(self->currentPlayingMusicNumber_);
                    }
                }
                //等待播放队列为空
                app.WaitSoundToFinish();
                xEventGroupClearBits(sdPlayerMonitor_EventGroup, SDPLAYE_PLAYING_BIT);
                //检查sd卡是否插上
                // self->sdIsInited_ = self->check_sd_card_status();
                //播放提示音
                #if TISHIYIN_IS_EXIST
                if (app.GetDeviceState() == DeviceState::kDeviceStateSpeaking)
                {
                    /* code */
                    app.TiShiYin_V2();
                    //等待播放队列为空
                    app.WaitSoundToFinish();
                }
                #endif

                //停止speaking状态
                if (app.GetDeviceState() == DeviceState::kDeviceStateSpeaking)
                {
                    app.StopSpeaking();
                }
                //清除事件组位  
                // xEventGroupClearBits(sdPlayerMonitor_EventGroup, SDPLAYERMONITOR_SPEAKINGTOSTOP_BIT);
                //播放音乐
            }
        }, "sdPlayerMonitor", 4096, this, 2, NULL);
    }
    // //包含事件组头文件
    // #include "freertos/event_groups.h"
    // //定义事件组句柄
    // EventGroupHandle_t sdPlayerMonitor_EventGroup;
    // //创建事件组
    // sdPlayerMonitor_EventGroup = xEventGroupCreate();
    // //定义事件组位
    // const int SDPLAYERMONITOR_IDLE_BIT = BIT0;
    // const int SDPLAYERMONITOR_SPEAKINGTOSTOP_BIT = BIT1;
    
    // //传递事件组句柄
    // xEventGroupSetBits(sdPlayerMonitor_EventGroup, SDPLAYERMONITOR_IDLE_BIT);
    // //等待事件组位
    // xEventGroupWaitBits(sdPlayerMonitor_EventGroup, SDPLAYERMONITOR_SPEAKINGTOSTOP_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
    // //清除事件组位  
    // xEventGroupClearBits(sdPlayerMonitor_EventGroup, SDPLAYERMONITOR_SPEAKINGTOSTOP_BIT);

    // //将sdPlayerMonitor_EventGroup通过函数
    // //创建一个参数为EventGroupHandle_t的函数，名字为setSdPlayerMonitorEventGroupHandle
    // void setSdPlayerMonitorEventGroupHandle(EventGroupHandle_t eventGroupHandle) {
    //     sdPlayerMonitor_EventGroup = eventGroupHandle;
    // }







};

} // namespace iot

DECLARE_THING(SdPlayer);
