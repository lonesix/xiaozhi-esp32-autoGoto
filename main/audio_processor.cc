#include "audio_processor.h"
#include <esp_log.h>

#define PROCESSOR_RUNNING 0x01

static const char* TAG = "AudioProcessor";

AudioProcessor::AudioProcessor()
    : afe_communication_data_(nullptr) {
    event_group_ = xEventGroupCreate();
}

void AudioProcessor::Initialize(int channels, bool reference) {
    channels_ = channels;
    reference_ = reference;
    int ref_num = reference_ ? 1 : 0;

    afe_config_t afe_config = {
        .aec_init = false,
        .se_init = true,
        .vad_init = true,
        .wakenet_init = false,
        .voice_communication_init = true,
        .voice_communication_agc_init = true,
        .voice_communication_agc_gain = 10,
        .vad_mode = VAD_MODE_0,
        .wakenet_model_name = NULL,
        .wakenet_model_name_2 = NULL,
        .wakenet_mode = DET_MODE_90,
        .afe_mode = SR_MODE_LOW_COST,
        .afe_perferred_core = 1,
        .afe_perferred_priority = 1,
        .afe_ringbuf_size = 50,
        .memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM,
        .afe_linear_gain = 1.0,
        .agc_mode = AFE_MN_PEAK_AGC_MODE_2,
        .pcm_config = {
            .total_ch_num = channels_,
            .mic_num = channels_ - ref_num,
            .ref_num = ref_num,
            .sample_rate = 16000,
        },
        .debug_init = false,
        .debug_hook = {{ AFE_DEBUG_HOOK_MASE_TASK_IN, NULL }, { AFE_DEBUG_HOOK_FETCH_TASK_IN, NULL }},
        .afe_ns_mode = NS_MODE_SSP,
        .afe_ns_model_name = NULL,
        .fixed_first_channel = true,
    };

    afe_communication_data_ = esp_afe_vc_v1.create_from_config(&afe_config);
    
    xTaskCreate([](void* arg) {
        auto this_ = (AudioProcessor*)arg;
        this_->AudioProcessorTask();
        vTaskDelete(NULL);
    }, "audio_communication", 4096 * 2, this, 1, NULL);
}

AudioProcessor::~AudioProcessor() {
    if (afe_communication_data_ != nullptr) {
        esp_afe_vc_v1.destroy(afe_communication_data_);
    }
    vEventGroupDelete(event_group_);
}

void AudioProcessor::Input(std::vector<int16_t>& data) {
    input_buffer_.insert(input_buffer_.end(), data.begin(), data.end());

    auto chunk_size = esp_afe_vc_v1.get_feed_chunksize(afe_communication_data_) * channels_;
    while (input_buffer_.size() >= chunk_size) {
        auto chunk = input_buffer_.data();
        esp_afe_vc_v1.feed(afe_communication_data_, chunk);
        input_buffer_.erase(input_buffer_.begin(), input_buffer_.begin() + chunk_size);
    }
}

void AudioProcessor::Start() {
    xEventGroupSetBits(event_group_, PROCESSOR_RUNNING);
}

void AudioProcessor::Stop() {
    xEventGroupClearBits(event_group_, PROCESSOR_RUNNING);
}

bool AudioProcessor::IsRunning() {
    return xEventGroupGetBits(event_group_) & PROCESSOR_RUNNING;
}

void AudioProcessor::OnOutput(std::function<void(std::vector<int16_t>&& data)> callback) {
    output_callback_ = callback;
}
void AudioProcessor::OnVadStateChange(std::function<void(bool speaking)> callback) {
    vad_state_callback_ = callback;
}
void AudioProcessor::OnVadIsOpen(std::function<bool()> callback){
    vad_isopen_callback_ = callback;
}
void AudioProcessor::OnVadWsSendFlag(std::function<void(bool vad_flag)> callback){
    vad_wssendflag_callback_ = callback;
}
#define CHUNK_DURATION_MS 25 // 每块数据的时长（毫秒） 
#define SPEECH_THRESHOLD_MS 160  
#define SILENCE_THRESHOLD_MS 500  
void AudioProcessor::AudioProcessorTask() {
    int chunk_size = esp_afe_vc_v1.get_fetch_chunksize(afe_communication_data_);
    ESP_LOGI(TAG, "Audio communication task started, chunk size: %d", chunk_size);
    static bool current_state = false; // 当前状态：false = 静音，true = 人声  
    static uint64_t speech_start_time = 0;  
    static uint64_t silence_start_time = 0; 
    static uint64_t silence_duration_ms =0;
    while (true) {
        xEventGroupWaitBits(event_group_, PROCESSOR_RUNNING, pdFALSE, pdTRUE, portMAX_DELAY);

        auto res = esp_afe_vc_v1.fetch(afe_communication_data_);
        if ((xEventGroupGetBits(event_group_) & PROCESSOR_RUNNING) == 0) {
            continue;
        }
        if (res == nullptr || res->ret_value == ESP_FAIL) {
            if (res != nullptr) {
                ESP_LOGI(TAG, "Error code: %d", res->ret_value);
            }
            continue;
        }
        // VAD state change
        // 状态机逻辑  
  
        if (vad_isopen_callback_()) {
            bool is_current_speech = false;
            uint64_t current_time = esp_log_timestamp();
            if (res->vad_state == AFE_VAD_SPEECH ) {
                is_current_speech = true;
            } else if (res->vad_state == AFE_VAD_SILENCE ) {
                is_current_speech = false;
            }
            if (is_current_speech)  
            {  
                if (!current_state)  
                {  
                    // 从静音切换到人声  
                    speech_start_time = current_time;  
                    current_state = true;  
                    ESP_LOGI(TAG, "Speech detected at %llu ms", speech_start_time);  
                    if (vad_wssendflag_callback_)
                    {
                        vad_wssendflag_callback_(true);
                    }
                    
                }  
                if (current_state)
                {
                    silence_duration_ms = 0;
                }
                
            }  
            else  
            {  
            if (current_state)  
            {  
                // 从人声切换到可能的静音  
                silence_duration_ms += CHUNK_DURATION_MS;  
        
                // 检查是否达到了静音阈值  
                if (silence_duration_ms >= SILENCE_THRESHOLD_MS)  
                {  
                    // 静音阈值已达到，记录静音结束时间  
                    uint64_t silence_end_time = current_time ;  
                    // 注意：由于我们是按块处理的，所以silence_end_time可能不是完全准确的，  
                    // 但它应该足够接近实际的静音结束时间，用于大多数应用。  
                    // 如果你需要更精确的时间，你可能需要实现更复杂的音频处理算法。  
                    ESP_LOGI(TAG, "Speech ended and silence started at approximately %llu ms (end at %llu ms)",  
                            speech_start_time, silence_end_time);  
                    if (vad_wssendflag_callback_)
                    {
                        vad_wssendflag_callback_(false);
                         ESP_LOGI(TAG,"vad_wssendflag_callback_(false)");
                    }
                    
                    // 重置状态变量  
                    current_state = false;  
                    speech_start_time = 0;  
                    silence_duration_ms = 0;  
                }  
            }  
            else  
            {  
                // 如果当前已经是静音状态，则继续累积静音持续时间  
                silence_duration_ms += CHUNK_DURATION_MS;  
                // 但在这里我们不需要做额外的处理，因为我们已经处于静音状态了。  
            }  
        }

        }
        if (output_callback_) {
            output_callback_(std::vector<int16_t>(res->data, res->data + res->data_size / sizeof(int16_t)));
        }
    }
}
