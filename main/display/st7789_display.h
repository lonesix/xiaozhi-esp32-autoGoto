#ifndef ST7789_DISPLAY_H
#define ST7789_DISPLAY_H

#include "display.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_timer.h>
#include "esp_lcd_touch_ft5x06.h"

class St7789Display : public Display {
private:
    // bool GifSwitch = false;
    esp_lcd_touch_handle_t tp_ = nullptr;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    gpio_num_t backlight_pin_ = GPIO_NUM_NC;
    bool backlight_output_invert_ = false;
    bool mirror_x_ = false;
    bool mirror_y_ = false;
    bool swap_xy_ = false;
    int offset_x_ = 0;
    int offset_y_ = 0;
    SemaphoreHandle_t lvgl_mutex_ = nullptr;
    esp_timer_handle_t lvgl_tick_timer_ = nullptr;
    
    lv_obj_t* status_bar_ = nullptr;
    lv_obj_t* content_ = nullptr;
    lv_obj_t* container_ = nullptr;
    lv_obj_t* side_bar_ = nullptr;

    void InitializeBacklight(gpio_num_t backlight_pin);
    virtual void SetBacklight(uint8_t brightness);
    void SetupUI();
    void LvglTask();

    virtual void SetStatus(const std::string &status);
    virtual void ShowNotification(const std::string &notification, int duration_ms = 3000);
    virtual void SetEmotion(const std::string &emotion);
    virtual void SetChatMessage(const std::string &role, const std::string &content);
    virtual void SetIcon(const char* icon);
    virtual void Update();

    virtual void GotoQRcodePage();
    virtual void GotoMainPage();

    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;

    
public:
    St7789Display(esp_lcd_touch_handle_t tp,esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                  gpio_num_t backlight_pin, bool backlight_output_invert,
                  int width, int height,  int offset_x, int offset_y, bool mirror_x, bool mirror_y, bool swap_xy);
    ~St7789Display();
    bool kaijiGif_finish_flag = false;
    
    virtual bool kaijiFinishFlag();
};

#endif // ST7789_DISPLAY_H
