#ifndef AUDIO_PROCESSOR_H
#define AUDIO_PROCESSOR_H

#include <esp_afe_sr_models.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <string>
#include <vector>
#include <functional>

class AudioProcessor {
public:
    AudioProcessor();
    ~AudioProcessor();

    void Initialize(int channels, bool reference);
    void Input(std::vector<int16_t>& data);
    void Start();
    void Stop();
    bool IsRunning();
    void OnOutput(std::function<void(std::vector<int16_t>&& data)> callback);
    void OnVadStateChange(std::function<void(bool speaking)> callback);
    void OnVadIsOpen(std::function<bool()> callback);
    void OnVadWsSendFlag(std::function<void(bool vad_flag)> callback);
private:
    EventGroupHandle_t event_group_ = nullptr;
    esp_afe_sr_data_t* afe_communication_data_ = nullptr;
    std::vector<int16_t> input_buffer_;
    std::function<void(std::vector<int16_t>&& data)> output_callback_;
    std::function<void(bool speaking)> vad_state_callback_;
    std::function<bool()> vad_isopen_callback_;
    std::function<void(bool vad_flag)> vad_wssendflag_callback_;
    int channels_;
    bool reference_;

    void AudioProcessorTask();
};

#endif
