#include "config.h"
#include "mcp_server.h"
#include "iot/thing.h"
#include "board.h"
#include <cJSON.h>
static const char* TAG = "fastbee_mcp";

#include <cctype>
#include <sstream>
#include <iomanip>

// 辅助函数：数字字符转中文
std::string DigitToChinese(char c) {
    switch (c) {
        case '0': return "零";
        case '1': return "一";
        case '2': return "二";
        case '3': return "三";
        case '4': return "四";
        case '5': return "五";
        case '6': return "六";
        case '7': return "七";
        case '8': return "八";
        case '9': return "九";
        default: return "";
    }
}

// 核心转换函数
std::string DecimalToChineseText(const std::string& decimal_str) {
    if (decimal_str.empty()) return "";

    std::string result;
    size_t dot_pos = decimal_str.find('.');
    std::string int_part = decimal_str.substr(0, dot_pos);
    std::string dec_part = (dot_pos != std::string::npos) ? decimal_str.substr(dot_pos + 1) : "";

    // --- 处理整数部分 ---
    if (!int_part.empty()) {
        // 处理负号
        if (int_part[0] == '-') {
            result += "负";
            int_part = int_part.substr(1);
        }

        int int_val = std::stoi(int_part);
        
        // 特殊情况：0
        if (int_val == 0) {
            result += "零";
        } else {
            // 处理 10-19 (例如 11 -> 十一)
            if (int_val >= 10 && int_val < 20) {
                result += "十";
                if (int_val % 10 != 0) {
                    result += DigitToChinese(int_part.back());
                }
            } 
            // 处理 20-99 (例如 30 -> 三十)
            else if (int_val < 100) {
                result += DigitToChinese(int_part[0]);
                result += "十";
                if (int_val % 10 != 0) {
                    result += DigitToChinese(int_part.back());
                }
            }
            // 处理更大的数字 (简单实现，仅支持百位)
            // 如果需要支持 1234 (一千二百三十四)，需要更复杂的逻辑
            else if (int_val < 1000) {
                result += DigitToChinese(int_part[0]);
                result += "百";
                // 处理中间的零，例如 101 -> 一百零一
                if (int_part[1] == '0') {
                    if (int_part[2] != '0') result += "零"; 
                } else {
                    result += DigitToChinese(int_part[1]);
                    result += "十";
                }
                
                if (int_part[2] != '0') {
                    result += DigitToChinese(int_part[2]);
                }
            }
            // 更大的数字暂不支持，直接逐位读
            else {
                for (char c : int_part) {
                    result += DigitToChinese(c);
                }
            }
        }
    }

    // --- 处理小数部分 ---
    if (!dec_part.empty()) {
        result += "点";
        for (char c : dec_part) {
            result += DigitToChinese(c);
        }
    }

    return result;
}

std::string UrlEncodes(const std::string& value) {
    std::ostringstream escaped;
    escaped.fill('0');
    escaped << std::hex;

    for (std::string::const_iterator i = value.begin(), n = value.end(); i != n; ++i) {
        std::string::value_type c = (*i);

        // 保留字母数字字符
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            escaped << c;
        } else {
            // 任何其他字符都进行百分号编码
            escaped << std::uppercase;
            escaped << '%' << std::setw(2) << int((unsigned char) c);
            escaped << std::nouppercase;
        }
    }

    return escaped.str();
}
class FASTBEEMCP {

private:
   std::string Authorization_token;
public:
    FASTBEEMCP() {
        RegisterMcpTools();
    }
    ~FASTBEEMCP() {
        
    }

void setToken(const std::string& token) {
    Authorization_token = token;
}

std::string parse_value(const std::string& result) {
    std::string data;
    cJSON *root = cJSON_Parse(result.c_str());

    if (root == NULL) {
        printf("JSON parse error\n");
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            printf("Error before: %s\n", error_ptr);
        }
        return "result parse error";
    }

    // 2. 获取 "rows" 数组
    cJSON *rows = cJSON_GetObjectItemCaseSensitive(root, "rows");
    if (!cJSON_IsArray(rows)) {
        printf("Error: rows is not an array\n");
        cJSON_Delete(root);
        return "result parse error";
    }

    // 3. 获取数组的第 1 个元素 (索引 0)
    cJSON *first_item = cJSON_GetArrayItem(rows, 0);
    if (first_item == NULL) {
        printf("Error: rows array is empty\n");
        cJSON_Delete(root);
        return "result parse error";
    }

    // 4. 获取该元素中的 "value" 字段
    cJSON *value_item = cJSON_GetObjectItemCaseSensitive(first_item, "value");
    
    // 5. 检查并提取值
    if (cJSON_IsString(value_item) && (value_item->valuestring != NULL)) {
        printf("Value (String): %s\n", value_item->valuestring);
        
        // 如果你需要将其作为浮点数使用:
        double value_double = atof(value_item->valuestring);
        printf("Value (Double): %f\n", value_double);
        data = value_item->valuestring;
        
        
    } else {
        printf("Error: value field not found or not a string\n");
    }


    // 6. 删除根对象，释放内存
    cJSON_Delete(root);
    return data;
}
std::string getSingleData(std::string modelName) {
    if (Authorization_token.empty()) {
        return "{\"state\": false, \"message\": \"Authorization_token is not set\"}";
    }
    std::string urlEnc = UrlEncodes(modelName);
    std::string url = "http://36.140.249.159:60080/prod-api/iot/device/listThingsModel?pageNum=1&pageSize=1&deviceId=267&modelName=" + urlEnc;
    auto http = Board::GetInstance().CreateHttp();
    http->SetHeader("Authorization",   Authorization_token);
    if (!http->Open("GET", url)) {
        return "{\"state\": false, \"message\": \"Failed to connect to URL\"}";
    }

    if (http->GetStatusCode() != 200) {
        ESP_LOGE(TAG, "Failed to upload photo, status code: %d", http->GetStatusCode());
        return "{\"state\": false, \"message\": \"Failed to Get\"}";
    }


    std::vector<char> buffer(4096/2); 

    int read_len = 0;
    std::string result;
    while (true) {
        // 调用底层 Read 函数读取数据
        read_len = http->Read(buffer.data(), buffer.size());

        if (read_len > 0) {
            // 成功读取到数据，追加到结果字符串中
            result.append(buffer.data(), read_len);
        } 
        else if (read_len == 0) {
            // read_len 为 0 表示服务器关闭了连接（Connection: close）
            // 或者读取到了 Chunked 编码的结束符
            ESP_LOGI(TAG, "Connection closed or end of stream. Total bytes read: %d", result.length());
            break;
        } 
        else {
            // read_len < 0 表示读取过程中发生错误
            ESP_LOGE(TAG, "Error occurred while reading data: %d", read_len);
            break;
        }
    }
    printf("result: %s\n", result.c_str());


     //= http->GetBody();
    http->Close();
    delete http;
    std::string data ;
    if (!result.empty()) 
    {
        data = parse_value(result);
        data = "\""+DecimalToChineseText(data)+"\"";
        printf("data: %s\n", data.c_str());
    }else{
        return "{\"state\": false, \"message\": \"http Get result is empty\"}";
    }
    

    
    return data;
    
}


void RegisterMcpTools() {
    auto& mcp_server = McpServer::GetInstance();
    mcp_server.AddTool("self.camera.getSingleData",
        "获取fastbee云平台设备单个数据\n"
        "`token`:鉴权信息，`modlename`:数据名称\n"
        "返回值:失败:失败原因json,成功:数据\n",
        PropertyList({
            Property("token", kPropertyTypeString),
            Property("modlename", kPropertyTypeString)
        }),
        [this](const PropertyList& properties) -> ReturnValue {
            
            auto token = properties["token"].value<std::string>();
            auto modlename = properties["modlename"].value<std::string>();
            setToken(token);
            return getSingleData(modlename);
        });


}

};

static FASTBEEMCP* FastbeeMcp = nullptr;

void InitializeFastbeeMcp() {
    if (FastbeeMcp == nullptr) {
        FastbeeMcp = new FASTBEEMCP();
        ESP_LOGI(TAG, "FastbeeMcp已初始化并注册MCP工具");
    }
}

namespace iot {
    std::string UrlEncode(const std::string& value) {
        std::ostringstream escaped;
        escaped.fill('0');
        escaped << std::hex;
    
        for (std::string::const_iterator i = value.begin(), n = value.end(); i != n; ++i) {
            std::string::value_type c = (*i);
    
            // 保留字母数字字符
            if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
                escaped << c;
            } else {
                // 任何其他字符都进行百分号编码
                escaped << std::uppercase;
                escaped << '%' << std::setw(2) << int((unsigned char) c);
                escaped << std::nouppercase;
            }
        }
    
        return escaped.str();
    }
    class FastbeeIot : public Thing {
    private:
        std::string Authorization_token;
        std::string modleName;
    public:
        

    void setToken(const std::string& token) {
        Authorization_token = token;
    }
    void setModleName(const std::string& name) {
        modleName = name;
    }
    

    std::string parse_value(const std::string& result) {
        std::string data;
        cJSON *root = cJSON_Parse(result.c_str());

        if (root == NULL) {
            printf("JSON parse error\n");
            const char *error_ptr = cJSON_GetErrorPtr();
            if (error_ptr != NULL) {
                printf("Error before: %s\n", error_ptr);
            }
            return "result parse error";
        }

        // 2. 获取 "rows" 数组
        cJSON *rows = cJSON_GetObjectItemCaseSensitive(root, "rows");
        if (!cJSON_IsArray(rows)) {
            printf("Error: rows is not an array\n");
            cJSON_Delete(root);
            return "result parse error";
        }

        // 3. 获取数组的第 1 个元素 (索引 0)
        cJSON *first_item = cJSON_GetArrayItem(rows, 0);
        if (first_item == NULL) {
            printf("Error: rows array is empty\n");
            cJSON_Delete(root);
            return "result parse error";
        }

        // 4. 获取该元素中的 "value" 字段
        cJSON *value_item = cJSON_GetObjectItemCaseSensitive(first_item, "value");
        
        // 5. 检查并提取值
        if (cJSON_IsString(value_item) && (value_item->valuestring != NULL)) {
            printf("Value (String): %s\n", value_item->valuestring);
            
            // 如果你需要将其作为浮点数使用:
            double value_double = atof(value_item->valuestring);
            printf("Value (Double): %f\n", value_double);
            data = value_item->valuestring;
            
            
        } else {
            printf("Error: value field not found or not a string\n");
        }

    
        // 6. 删除根对象，释放内存
        cJSON_Delete(root);
        return data;
    }
    std::string getSingleData(std::string modelName) {
        if (Authorization_token.empty()) {
            return "{\"state\": false, \"message\": \"Authorization_token is not set\"}";
        }
        std::string urlEnc = UrlEncode(modelName);
        std::string url = "http://36.140.249.159:60080/prod-api/iot/device/listThingsModel?pageNum=1&pageSize=1&deviceId=267&modelName=" + urlEnc;
        auto http = Board::GetInstance().CreateHttp();
        http->SetHeader("Authorization",   Authorization_token);
        if (!http->Open("GET", url)) {
            return "{\"state\": false, \"message\": \"Failed to connect to URL\"}";
        }

        if (http->GetStatusCode() != 200) {
            ESP_LOGE(TAG, "Failed to upload photo, status code: %d", http->GetStatusCode());
            return "{\"state\": false, \"message\": \"Failed to Get\"}";
        }


        std::vector<char> buffer(4096/2); 
    
        int read_len = 0;
        std::string result;
        while (true) {
            // 调用底层 Read 函数读取数据
            read_len = http->Read(buffer.data(), buffer.size());
    
            if (read_len > 0) {
                // 成功读取到数据，追加到结果字符串中
                result.append(buffer.data(), read_len);
            } 
            else if (read_len == 0) {
                // read_len 为 0 表示服务器关闭了连接（Connection: close）
                // 或者读取到了 Chunked 编码的结束符
                ESP_LOGI(TAG, "Connection closed or end of stream. Total bytes read: %d", result.length());
                break;
            } 
            else {
                // read_len < 0 表示读取过程中发生错误
                ESP_LOGE(TAG, "Error occurred while reading data: %d", read_len);
                break;
            }
        }
        printf("result: %s\n", result.c_str());


         //= http->GetBody();
        http->Close();
        delete http;
        std::string data ;
        if (!result.empty()) 
        {
            data = parse_value(result);
        }else{
            return "{\"state\": false, \"message\": \"http Get result is empty\"}";
        }
        

        
        return data;
        
    }

    FastbeeIot() : Thing("FastbeeIot", "fastbee云平台设备工具") {
        printf("FastbeeMcp已初始化并注册MCP工具\n");
        properties_.AddStringProperty("getSingleData", "可以获取fastbee云平台设备单个数据,使用前必须调用setToken方法设置健全信息和数据名称。", [this]() -> std::string {
            std::string data = getSingleData(modleName);
            printf("getSingleData: %s\n", data.c_str());
            return data;
            
        });
        methods_.AddMethod("setToken", "设置鉴权信息和数据名称", ParameterList({
            Parameter("token", "鉴权信息", kValueTypeString, true),
            Parameter("modlename", "数据名称", kValueTypeString, true)
        }), [this](const ParameterList& parameters) {
            // auto codec = Board::GetInstance().GetAudioCodec();
            // codec->SetOutputVolume(static_cast<uint8_t>(parameters["volume"].number()));
            auto token = static_cast<std::string>(parameters["token"].string());
            auto modlename = static_cast<std::string>(parameters["modlename"].string());
            setToken(token);
            setModleName(modlename);
            
            
        });

    }
    // void RegisterMcpTools() {
    //     auto& mcp_server = McpServer::GetInstance();
    //     mcp_server.AddTool("self.camera.getSingleData",
    //         "获取fastbee云平台设备单个数据\n"
    //         "`token`:鉴权信息，`modlename`:数据名称\n"
    //         "返回值:失败:失败原因json,成功:数据\n",
    //         PropertyList({
    //             Property("token", kPropertyTypeString),
    //             Property("modlename", kPropertyTypeString)
    //         }),
    //         [camera](const PropertyList& properties) -> ReturnValue {
                
    //             auto token = properties["token"].value<std::string>();
    //             auto modlename = properties["modlename"].value<std::string>();
    //             setToken(token);
    //             return getSingleData(modlename);
    //         });

    };
}
DECLARE_THING(FastbeeIot);