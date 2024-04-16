//#include "hmi.h"
#include <iostream>
#include <cstring>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>

// 测试用结构体
struct MyStruct {
    int id;
    std::string name;
    float score;
};

// 回调函数，用于处理HTTP响应
size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* response) {
    size_t totalSize = size * nmemb;
    response->append((char*)contents, totalSize);
    return totalSize;
}

int main() {
    // 创建结构体对象
    MyStruct obj;
    obj.id = 1;
    obj.name = "John";
    obj.score = 95.5;

    // 将结构体对象转换为JSON
    Json::Value json;
    json["id"] = obj.id;
    json["name"] = obj.name;
    json["score"] = obj.score;
    std::string jsonString = json.toStyledString();

    // 初始化CURL
    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Failed to initialize CURL" << std::endl;
        return 1;
    }

    // 设置HTTP请求的URL
    std::string url = "http://localhost:8888"; // 假设Vue前端的URL是http://localhost:8888
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

    // 设置HTTP请求的方法为POST
    curl_easy_setopt(curl, CURLOPT_POST, 1L);

    // 设置HTTP请求的数据
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonString.c_str());

    // 设置HTTP请求的回调函数和缓冲区
    std::string response;
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    // 发送HTTP请求
    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "Failed to send HTTP request: " << curl_easy_strerror(res) << std::endl;
        return 1;
    }

    // 打印HTTP响应
    std::cout << "Response: " << response << std::endl;

    // 清理CURL资源
    curl_easy_cleanup(curl);

    return 0;
}