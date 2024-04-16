#include"http_vue.h"



http_vue::http_vue()
{

}

http_vue::~http_vue()
{
   
}
void http_vue::switch_json(const std_msgs::String msg){

Json::Value json_msg;
  json_msg["message"] = msg.data;
   // 将JSON消息发送到Vue
  // 使用curl库发送HTTP请求
  CURL* curl;
  CURLcode res;
  curl_global_init(CURL_GLOBAL_DEFAULT);
  curl = curl_easy_init();
  if(curl) {
    std::string url = "http://localhost:8080";  // 替换为Vue HTTP端点的URL
    std::string json_data = json_msg.toStyledString();
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());
    res = curl_easy_perform(curl);
    if(res == CURLE_OK)
      ROS_INFO("消息已成功发送到Vue");
    else
      ROS_ERROR("消息发送失败");
    curl_easy_cleanup(curl);
  }
  curl_global_cleanup();



}