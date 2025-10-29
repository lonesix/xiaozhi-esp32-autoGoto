#include "st7789_display.h"
#include "font_awesome_symbols.h"

#include <esp_log.h>
#include <esp_err.h>
#include <driver/ledc.h>
#include <vector>
#include <algorithm>
static const char* TAG ="St7789Display";
#define LCD_LEDC_CH LEDC_CHANNEL_0

#define ST7789_LVGL_TICK_PERIOD_MS 2
#define ST7789_LVGL_TASK_MAX_DELAY_MS 20
#define ST7789_LVGL_TASK_MIN_DELAY_MS 1
#define ST7789_LVGL_TASK_STACK_SIZE (9 * 1024)
#define ST7789_LVGL_TASK_PRIORITY 10

LV_FONT_DECLARE(font_puhui_14_1);
LV_FONT_DECLARE(font_awesome_30_4);
// LV_FONT_DECLARE(font_awesome_14_1);
LV_FONT_DECLARE(font_puhui_16_4);

static lv_disp_drv_t disp_drv;
static lv_indev_drv_t indev_drv;
static void st7789_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(&disp_drv);
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
static void st7789_lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;

    switch (drv->rotated)
    {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
#if CONFIG_ST7789_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
#endif
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
#if CONFIG_ST7789_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
#endif
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
#if CONFIG_ST7789_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
#endif
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
#if CONFIG_ST7789_LCD_TOUCH_ENABLED
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
#endif
        break;
    }
}

void St7789Display::LvglTask() {
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = ST7789_LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (Lock())
        {
            task_delay_ms = lv_timer_handler();
            Unlock();
        }else ESP_LOGI(TAG,"Locking");
        if (task_delay_ms > ST7789_LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = ST7789_LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < ST7789_LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = ST7789_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

#include "esp_lvgl_port.h"
#include "esp_lcd_touch_ft5x06.h"
void touch_driver_read(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
//     uint16_t touch_x[1];
//     uint16_t touch_y[1];
//     uint16_t touch_strength[1];
//     uint8_t touch_cnt = 0;
//     esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)indev_drv->user_data;
    // ESP_LOGI(TAG, "X=%d Y=%d", data->point.x, data->point.y);
//     bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, touch_x, touch_y, touch_strength, &touch_cnt, 1);
//       if(touchpad_pressed) {
//     data->point.x = touch_x[0];
//     data->point.y = touch_y[0];
//     data->state = LV_INDEV_STATE_PRESSED;
//     ESP_LOGI(TAG, "X=%d Y=%d", data->point.x, data->point.y);
//   } else {
//     data->state = LV_INDEV_STATE_RELEASED;
//   }

    assert(indev_drv);
    // lvgl_port_touch_ctx_t *touch_ctx = (lvgl_port_touch_ctx_t *)indev_drv->user_data;
    esp_lcd_touch_handle_t touch_ctx = (esp_lcd_touch_handle_t)indev_drv->user_data;
    assert(touch_ctx);

    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;
    uint16_t touch_strength[1];
    /* Read data from touch controller into memory */
    esp_lcd_touch_read_data(touch_ctx);

    /* Read data from touch controller */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(touch_ctx, touchpad_x, touchpad_y, touch_strength, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
        ESP_LOGI(TAG, "X=%d Y=%d", data->point.x, data->point.y);
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
    
    data->continue_reading = false;
    
}

St7789Display::St7789Display(esp_lcd_touch_handle_t tp,esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                           gpio_num_t backlight_pin, bool backlight_output_invert,
                           int width, int height, int offset_x, int offset_y, bool mirror_x, bool mirror_y, bool swap_xy)
    : tp_(tp) ,panel_io_(panel_io), panel_(panel), backlight_pin_(backlight_pin), backlight_output_invert_(backlight_output_invert),
      mirror_x_(mirror_x), mirror_y_(mirror_y), swap_xy_(swap_xy) {
    width_ = width;
    height_ = height;
    offset_x_ = offset_x;
    offset_y_ = offset_y;

    
    InitializeBacklight(backlight_pin);

    // draw white
    std::vector<uint16_t> buffer(width_, 0x0000);
    for (int y = 0; y < height_; y++) {
        esp_lcd_panel_draw_bitmap(panel_, 0, y, width_, y + 1, buffer.data());
    }

    // Set the display to on
    ESP_LOGI(TAG, "Turning display on");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(width_ * 10 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(width_ * 10 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, width_ * 10);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = width_;
    disp_drv.ver_res = height_;
    disp_drv.offset_x = offset_x_;
    disp_drv.offset_y = offset_y_;
    disp_drv.flush_cb = st7789_lvgl_flush_cb;
    disp_drv.drv_update_cb = st7789_lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_;
    disp_drv.sw_rotate = 1;
    disp_drv.rotated = LV_DISP_ROT_270;
    lv_disp_drv_register(&disp_drv);

    	/*触摸屏输入接口配置*/
            /* 添加LVGL接口 */
    // const lvgl_port_touch_cfg_t touch_cfg = {
    //     .disp = disp,
    //     .handle = tp,
    // };

    // lvgl_port_add_touch(&touch_cfg);
    ESP_LOGI(TAG, "Register touch driver to LVGL");
	// lv_indev_drv_t indev_drv;
	lv_indev_drv_init(&indev_drv);
	indev_drv.read_cb = touch_driver_read;
	indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.user_data = tp;
	// lv_indev_drv_register(&indev_drv);
    if (lv_indev_drv_register(&indev_drv) ==NULL )
    {
        printf("lv_indev_drv_register failed");
    }
    
    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = [](void* arg) {
            lv_tick_inc(ST7789_LVGL_TICK_PERIOD_MS);
        },
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "LVGL Tick Timer",
        .skip_unhandled_events = false
    };
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer_));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer_, ST7789_LVGL_TICK_PERIOD_MS * 1000));

    lvgl_mutex_ = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mutex_ != nullptr);
    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate([](void *arg) {
        static_cast<St7789Display*>(arg)->LvglTask();
        vTaskDelete(NULL);
    }, "LVGL", ST7789_LVGL_TASK_STACK_SIZE, this, ST7789_LVGL_TASK_PRIORITY, NULL);

    SetBacklight(100);

    SetupUI();
}

St7789Display::~St7789Display() {
    ESP_ERROR_CHECK(esp_timer_stop(lvgl_tick_timer_));
    ESP_ERROR_CHECK(esp_timer_delete(lvgl_tick_timer_));

    if (content_ != nullptr) {
        lv_obj_del(content_);
    }
    if (status_bar_ != nullptr) {
        lv_obj_del(status_bar_);
    }
    if (side_bar_ != nullptr) {
        lv_obj_del(side_bar_);
    }
    if (container_ != nullptr) {
        lv_obj_del(container_);
    }

    if (panel_ != nullptr) {
        esp_lcd_panel_del(panel_);
    }
    if (panel_io_ != nullptr) {
        esp_lcd_panel_io_del(panel_io_);
    }
    vSemaphoreDelete(lvgl_mutex_);
}

void St7789Display::InitializeBacklight(gpio_num_t backlight_pin) {
    if (backlight_pin == GPIO_NUM_NC) {
        return;
    }

    // Setup LEDC peripheral for PWM backlight control
    const ledc_channel_config_t backlight_channel = {
        .gpio_num = backlight_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {
            .output_invert = backlight_output_invert_,
        }
    };
    const ledc_timer_config_t backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };

    ESP_ERROR_CHECK(ledc_timer_config(&backlight_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&backlight_channel));
}

void St7789Display::SetBacklight(uint8_t brightness) {
    if (backlight_pin_ == GPIO_NUM_NC) {
        return;
    }

    if (brightness > 100) {
        brightness = 100;
    }

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness);
    // LEDC resolution set to 10bits, thus: 100% = 1023
    uint32_t duty_cycle = (1023 * brightness) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH));
}

bool St7789Display::Lock(int timeout_ms) {
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to 0, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mutex_, timeout_ticks) == pdTRUE;
}

void St7789Display::Unlock() {
    xSemaphoreGiveRecursive(lvgl_mutex_);
}
bool St7789Display::kaijiFinishFlag()
{
    return kaijiGif_finish_flag;
}
#include "ui.h"
void St7789Display::SetStatus(const std::string &status)
{
    if (ui_notificationLabel == nullptr) {
        return;
    }
    DisplayLockGuard lock(this);
    lv_label_set_text(ui_notificationLabel, status.c_str());
}
void St7789Display::ShowNotification(const std::string &notification, int duration_ms)
{
    if (ui_notificationLabel == nullptr) {
        return;
    }
    DisplayLockGuard lock(this);
    lv_label_set_text(ui_notificationLabel, notification.c_str());
}
void St7789Display::SetEmotion(const std::string &emotion)
{
        struct Emotion {
        const char* icon;
        const char* text;
    };
/*
#表情列表
"开心": "<|emotion:happy|>",,
"生气": "<|emotion:angry|>",,
"惊讶": "<|emotion:surprise|>",
"厌恶": "<|emotion:disgust|>",1222
"害怕": "<|emotion:fear|>",12222
"伤心": "<|emotion:sad|>",
"害羞": "<|emotion:shy|>",1222
"尴尬": "<|emotion:embarrassed|>",
"紧张": "<|emotion:anxious|>",1222
"懊恼": "<|emotion:frustrated|>",1222
"兴奋": "<|emotion:excited|>",
"好奇": "<|emotion:curious|>"
*/
    static const std::vector<Emotion> emotions = {
        {FONT_AWESOME_EMOJI_NEUTRAL, "neutral"},
        {FONT_AWESOME_EMOJI_NEUTRAL, "disgust"},
        {FONT_AWESOME_EMOJI_HAPPY, "happy"},
        {FONT_AWESOME_EMOJI_LAUGHING, "excited"},
        {FONT_AWESOME_EMOJI_FUNNY, "funny"},
        {FONT_AWESOME_EMOJI_SAD, "sad"},
        {FONT_AWESOME_EMOJI_ANGRY, "angry"},
        {FONT_AWESOME_EMOJI_LOVING, "shy"},
        {FONT_AWESOME_EMOJI_EMBARRASSED, "embarrassed"},
        {FONT_AWESOME_EMOJI_SURPRISED, "surprise"},
        {FONT_AWESOME_EMOJI_SHOCKED, "fear"},
        {FONT_AWESOME_EMOJI_THINKING, "curious"},
        {FONT_AWESOME_EMOJI_WINKING, "winking"},
        {FONT_AWESOME_EMOJI_COOL, "cool"},
        {FONT_AWESOME_EMOJI_RELAXED, "relaxed"},
        {FONT_AWESOME_EMOJI_DELICIOUS, "delicious"},
        {FONT_AWESOME_EMOJI_KISSY, "kissy"},
        {FONT_AWESOME_EMOJI_CONFIDENT, "confident"},
        {FONT_AWESOME_EMOJI_SLEEPY, "frustrated"},
        {FONT_AWESOME_EMOJI_SILLY, "silly"},
        {FONT_AWESOME_EMOJI_CONFUSED, "confused"}
    };
    
    // 查找匹配的表情
    auto it = std::find_if(emotions.begin(), emotions.end(),
        [&emotion](const Emotion& e) { return e.text == emotion; });

    DisplayLockGuard lock(this);
    // ESP_LOGI("St7789Display", "12312");
    if (ui_emotionlabel == nullptr) {
        return;
    }

    // 如果找到匹配的表情就显示对应图标，否则显示默认的neutral表情
    // lv_obj_set_style_text_font(ui_emotionlabel, fonts_.emoji_font, 0);
    if (it != emotions.end()) {
        lv_label_set_text(ui_emotionlabel, it->icon);
        // ESP_LOGI("St7789Display", "1");
    } else {
        lv_label_set_text(ui_emotionlabel, FONT_AWESOME_EMOJI_NEUTRAL);
        // ESP_LOGI("St7789Display", "2");
    }
}
void St7789Display::SetChatMessage(const std::string &role, const std::string &content)
{
    // std::string user= "user";
    DisplayLockGuard lock(this);
    if (strcmp("user", role.c_str()) == 0) {
        //user
        lv_textarea_set_text(ui_userTextArea, content.c_str());
    } else {
        //assistant
        // lv_textarea_set_text(ui_AITextArea, content.c_str());
        lv_textarea_set_text(ui_userTextArea, content.c_str());
    }
}
void St7789Display::SetIcon(const char *icon)
{
    // if (emotion_label_ == nullptr) {
    //     return;
    // }
    // DisplayLockGuard lock(this);
    // lv_label_set_text(emotion_label_, icon);
}
// #define ALL_WIFI_ICON ""
// #define WIFI_ICON_H ""
// #define WIFI_ICON_M ""
// #define WIFI_ICON_L ""
// #define WIFI_ICON_N ""
// #define ALL_VOLUMN_ICON ""
// #define VOLUMN_ICON_Y ""
// #define VOLUMN_ICON_N ""

const char *wifiIcon[] = {WIFI_ICON_H,WIFI_ICON_M,WIFI_ICON_L,WIFI_ICON_N};
const char *volumnIcon[] = {VOLUMN_ICON_Y,VOLUMN_ICON_N};
#include "board.h"
#include "application.h"
#include "audio_codec.h"
void St7789Display::Update()
{
    if (ui_volLabel2 == nullptr) {
        return;
    }

    auto& board = Board::GetInstance();
    auto codec = board.GetAudioCodec();

    DisplayLockGuard lock(this);
    // 如果静音状态改变，则更新图标
    if (codec->output_volume() == 0 && !muted_) {
        muted_ = true;
        lv_label_set_text(ui_volLabel2, volumnIcon[1]);
    } else if (codec->output_volume() > 0 && muted_) {
        muted_ = false;
        lv_label_set_text(ui_volLabel2, volumnIcon[0]);
    }

    // 更新电池图标
    int battery_level;
    bool charging;
    const char* icon = nullptr;
    if (board.GetBatteryLevel(battery_level, charging)) {
        if (charging) {
            icon = FONT_AWESOME_BATTERY_CHARGING;
        } else {
            const char* levels[] = {
                FONT_AWESOME_BATTERY_EMPTY, // 0-19%
                FONT_AWESOME_BATTERY_1,    // 20-39%
                FONT_AWESOME_BATTERY_2,    // 40-59%
                FONT_AWESOME_BATTERY_3,    // 60-79%
                FONT_AWESOME_BATTERY_FULL, // 80-99%
                FONT_AWESOME_BATTERY_FULL, // 100%
            };
            icon = levels[battery_level / 20];
        }
        if (battery_icon_ != icon) {
            battery_icon_ = icon;
            // lv_label_set_text(battery_label_, battery_icon_);
            lv_bar_set_value(ui_changeBar, battery_level, LV_ANIM_OFF);
        }
    }

    // 仅在聊天状态为空闲时，读取网络状态（避免升级时占用 UART 资源）
    auto chat_state = Application::GetInstance().GetChatState();
    if (chat_state == kChatStateIdle || chat_state == kChatStateUnknown) {
        icon = board.GetNetworkStateIcon();
        if (network_icon_ != icon) {
            network_icon_ = icon;
        int wifi_num = 0;
        if (strcmp(network_icon_, FONT_AWESOME_WIFI) == 0) {
            wifi_num = 0;
            
        } else if (strcmp(network_icon_, FONT_AWESOME_WIFI_FAIR) == 0) {
            wifi_num++;
            
        } else if (strcmp(network_icon_, FONT_AWESOME_WIFI_WEAK) == 0) {
            wifi_num++;
            
        } else if (strcmp(network_icon_, FONT_AWESOME_WIFI_OFF) == 0) {
            wifi_num++;
            
        }

            lv_label_set_text(ui_netLabel, wifiIcon[wifi_num]);
        }
    }
}
void St7789Display::GotoQRcodePage()
{
    _ui_screen_change(&ui_QRcode, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_QRcode_screen_init);
}   
void St7789Display::GotoMainPage()
{
    _ui_screen_change(&ui_menu, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_menu_screen_init);
}
static bool *flag;
void next_frame_task_cb(lv_event_t *event)
{
    lv_event_code_t code = lv_event_get_code(event);
    switch (code)
    {
    case LV_EVENT_READY:
    {
        printf("----gif play finsh----\n");

        ui_init();
        /*unicode设置网络标志特殊字体测试*/
        lv_label_set_text(ui_netLabel, wifiIcon[3]);
        lv_label_set_text(ui_volLabel2, volumnIcon[1]);
        lv_label_set_text(ui_emotionlabel, FONT_AWESOME_AI_CHIP);
        lv_bar_set_value(ui_changeBar, 100, LV_ANIM_OFF);
        /*设置字体，网络和音量标志已内置*/
        lv_obj_set_style_text_font(ui_AITextArea, &font_puhui_16_4, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(ui_userTextArea, &font_puhui_16_4, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(ui_notificationLabel, &font_puhui_14_1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(ui_QRcodeLabel, &font_puhui_14_1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(ui_emotionlabel, &font_awesome_30_4, LV_PART_MAIN | LV_STATE_DEFAULT);
        *flag = true;
        break;
    }
    default:
        break;
    }
}

void St7789Display::SetupUI()
{
    DisplayLockGuard lock(this);
    #if IsGifSwitch
    flag = &kaijiGif_finish_flag;
    // GifSwitch = true;
    static lv_obj_t *gif_anim = NULL;    
    //kaiji_gif
    LV_IMG_DECLARE(kaiji_gif);
    // lv_obj_t *img;
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
    gif_anim = lv_gif_create(lv_scr_act());

        /* add event to gif_anim */
    lv_obj_add_event_cb(gif_anim, next_frame_task_cb, LV_EVENT_ALL, NULL);

    lv_gif_set_src(gif_anim, &kaiji_gif);
    lv_obj_set_pos(gif_anim, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    if (gif_anim == NULL) {
    // 处理错误，例如打印错误消息或退出程序
    printf( "gif_anim is NULL\n");
    
    }
    ((lv_gif_t *)gif_anim)->gif->loop_count = 1;
    #else
        ui_init();
        /*unicode设置网络标志特殊字体测试*/
        lv_label_set_text(ui_netLabel, wifiIcon[3]);
        lv_label_set_text(ui_volLabel2, volumnIcon[1]);
        lv_label_set_text(ui_emotionlabel, FONT_AWESOME_AI_CHIP);
        lv_bar_set_value(ui_changeBar, 100, LV_ANIM_OFF);
        /*设置字体，网络和音量标志已内置*/
        lv_obj_set_style_text_font(ui_AITextArea, &font_puhui_16_4, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(ui_userTextArea, &font_puhui_16_4, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(ui_notificationLabel, &font_puhui_14_1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(ui_QRcodeLabel, &font_puhui_14_1, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(ui_emotionlabel, &font_awesome_30_4, LV_PART_MAIN | LV_STATE_DEFAULT);
    #endif
        // auto screen = lv_screen_active();

        preview_image_ = lv_img_create(NULL);
        lv_obj_set_size(preview_image_, width_ * 0.5, height_ * 0.5);
        lv_obj_align(preview_image_, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_flag(preview_image_, LV_OBJ_FLAG_HIDDEN);
        /*设置textarea_text*/
        // lv_textarea_set_text(ui_AITextArea, "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
        /*设置textarea_text*/
        // lv_bar_set_value(ui_changeBar, 12, LV_ANIM_OFF);
    // _ui_screen_change(&ui_menu, LV_SCR_LOAD_ANIM_FADE_ON, 100, 0, &ui_menu_screen_init);
    // auto screen = lv_disp_get_scr_act(lv_disp_get_default());

    // // 设置屏幕背景颜色为黑色
    // lv_obj_set_style_bg_color(screen, lv_color_black(), 0);

    // lv_obj_set_style_text_font(screen, &font_alipuhui20, 0);
    // lv_obj_set_style_text_color(screen, lv_color_black(), 0);

    // /* Container */
    // container_ = lv_obj_create(screen);
    // lv_obj_set_size(container_, LV_HOR_RES, LV_VER_RES);
    // lv_obj_set_flex_flow(container_, LV_FLEX_FLOW_COLUMN);
    // lv_obj_set_style_pad_all(container_, 0, 0);
    // lv_obj_set_style_border_width(container_, 0, 0);
    // lv_obj_set_style_pad_row(container_, 0, 0);

    // /* Status bar */
    // status_bar_ = lv_obj_create(container_);
    // lv_obj_set_size(status_bar_, LV_HOR_RES, 18 * 2);
    // lv_obj_set_style_radius(status_bar_, 0, 0);

    // lv_obj_set_style_bg_color(status_bar_, lv_color_black(), 0);
    // lv_obj_set_style_line_color(status_bar_, lv_color_white(), 0);
    // lv_obj_set_style_outline_color(status_bar_, lv_color_white(), 0);
    // lv_obj_set_style_border_color(status_bar_, lv_color_white(), 0);

    // /*
    // 设置一组点，用来提供给line组件画线
    // 这个数组应该是静态、全局或动态分配的，不能是函数中的局部变量
    // 因为lv_line_set_points保存的只是该数组的指针
    // */
    // static lv_point_t line_points[] = {{0, 0}, {320 - 0, 0}};

    // lv_obj_t *line = lv_line_create(container_);

    // lv_line_set_points(line, line_points, 2); // 设置点数组。line将连接这些点，按顺序画出直线
    // /*创建一个共享样式*/
    // static lv_style_t style_line;
    // lv_style_init(&style_line);
    // // 下面3个样式是 line 的专有样式接口，类似于arc
    // lv_style_set_line_width(&style_line, 1);
    // lv_style_set_line_color(&style_line, lv_palette_main(LV_PALETTE_BLUE));
    // // 使用这个样式能让画出来的线条看起来更平滑
    // lv_style_set_line_rounded(&style_line, true);
    // lv_obj_add_style(line, &style_line, 0);
    
    // /* Content */
    // content_ = lv_obj_create(container_);
    // lv_obj_set_scrollbar_mode(content_, LV_SCROLLBAR_MODE_OFF);
    // lv_obj_set_style_radius(content_, 0, 0);
    // lv_obj_set_width(content_, LV_HOR_RES);
    // lv_obj_set_flex_grow(content_, 1);

    // lv_obj_set_style_bg_color(content_, lv_color_black(), 0);
    // lv_obj_set_style_border_color(content_, lv_color_black(), 0);
    // lv_obj_set_style_outline_color(content_, lv_color_black(), 0);
    // lv_obj_set_style_line_color(content_, lv_color_black(), 0);

    // emotion_label_ = lv_label_create(content_);
    // lv_obj_set_style_text_font(emotion_label_, &font_alipuhui20, 0); // 使用合适的字体
    // lv_label_set_text(emotion_label_, "");                           // 设置初始文本
    // lv_obj_set_width(emotion_label_, LV_HOR_RES - 20);               // 设置标签宽度，留出边距
    // lv_label_set_long_mode(emotion_label_, LV_LABEL_LONG_WRAP);      // 设置为自动换行模式
    // lv_obj_align(emotion_label_, LV_ALIGN_CENTER, 0, 0);             // 居中对齐

    // /* Status bar */
    // lv_obj_set_flex_flow(status_bar_, LV_FLEX_FLOW_ROW);
    // lv_obj_set_style_pad_all(status_bar_, 0, 0);
    // lv_obj_set_style_border_width(status_bar_, 0, 0);
    // lv_obj_set_style_pad_column(status_bar_, 0, 0);

    // network_label_ = lv_label_create(status_bar_);
    // lv_label_set_text(network_label_, "");
    // lv_obj_set_style_text_font(network_label_, &font_awesome_14_1, 0);

    // notification_label_ = lv_label_create(status_bar_);
    // lv_obj_set_flex_grow(notification_label_, 1);
    // lv_obj_set_style_text_align(notification_label_, LV_TEXT_ALIGN_CENTER, 0);
    // lv_label_set_text(notification_label_, "通知");
    // lv_obj_add_flag(notification_label_, LV_OBJ_FLAG_HIDDEN);

    // status_label_ = lv_label_create(status_bar_);
    // lv_obj_set_flex_grow(status_label_, 1);
    // lv_label_set_text(status_label_, "正在初始化");
    // lv_obj_set_style_text_align(status_label_, LV_TEXT_ALIGN_CENTER, 0);

    // mute_label_ = lv_label_create(status_bar_);
    // lv_label_set_text(mute_label_, "");
    // lv_obj_set_style_text_font(mute_label_, &font_awesome_14_1, 0);

    // battery_label_ = lv_label_create(status_bar_);
    // lv_label_set_text(battery_label_, "");
    // lv_obj_set_style_text_font(battery_label_, &font_awesome_14_1, 0);

    // // 设置屏幕文本颜色为白色（如果需要的话，但通常屏幕对象本身不显示文本）
    // // 这里主要为了示例，实际上应该为具体的文本对象设置文本颜色
    // lv_obj_set_style_bg_color(emotion_label_, lv_color_black(), 0);
    // lv_obj_set_style_bg_color(network_label_, lv_color_black(), 0);
    // lv_obj_set_style_bg_color(notification_label_, lv_color_black(), 0);
    // lv_obj_set_style_bg_color(status_label_, lv_color_black(), 0);
    // lv_obj_set_style_bg_color(mute_label_, lv_color_black(), 0);
    // lv_obj_set_style_bg_color(battery_label_, lv_color_black(), 0);
    // // 设置所有标签的字体颜色为白色
    // lv_obj_set_style_text_color(emotion_label_, lv_color_make(0xff, 0x00, 0x00), 0);
    // lv_obj_set_style_text_color(network_label_, lv_color_white(), 0);
    // lv_obj_set_style_text_color(notification_label_, lv_color_make(0x99, 0xff, 0x33), 0);
    // lv_obj_set_style_text_color(status_label_, lv_color_make(0x99, 0xff, 0x33), 0);
    // lv_obj_set_style_text_color(mute_label_, lv_color_white(), 0);
    // lv_obj_set_style_text_color(battery_label_, lv_color_white(), 0);
}

void St7789Display::SetPreviewImage(const lv_img_dsc_t* img_dsc) {
    DisplayLockGuard lock(this);
    if (preview_image_ == nullptr) {
        return;
    }
    
    if (img_dsc != nullptr) {
        // lv_img_set_angle
        // zoom factor 0.5
        lv_img_set_zoom(preview_image_, 128 * width_ / img_dsc->header.w);
        // 设置图片源并显示预览图片
        lv_img_set_src(preview_image_, img_dsc);
        lv_obj_clear_flag(preview_image_, LV_OBJ_FLAG_HIDDEN);
        // 隐藏emotion_label_
        if (emotion_label_ != nullptr) {
            lv_obj_add_flag(emotion_label_, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        // 隐藏预览图片并显示emotion_label_
        lv_obj_add_flag(preview_image_, LV_OBJ_FLAG_HIDDEN);
        if (emotion_label_ != nullptr) {
            lv_obj_clear_flag(emotion_label_, LV_OBJ_FLAG_HIDDEN);
        }
    }
}
