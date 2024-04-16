#include <iostream>
#include <cstring>
#include <curl/curl.h>

// 回调函数，用于接收从URL获取的数据
size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* response)
{
    size_t totalSize = size * nmemb;
    response->append((char*)contents, totalSize);
    return totalSize;
}

int main()
{
    // 初始化CURL
    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "Failed to initialize CURL" << std::endl;
        return 1;
    }

    // 设置URL
    std::string url = "http://localhost:8888";
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

    // 设置回调函数
    std::string response;
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    // 执行HTTP请求
    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "Failed to perform CURL request: " << curl_easy_strerror(res) << std::endl;
        curl_easy_cleanup(curl);
        return 1;
    }

    // 输出获取到的JSON数据
    std::cout << "Response: " << response << std::endl;

    // 清理CURL资源
    curl_easy_cleanup(curl);

    return 0;
}