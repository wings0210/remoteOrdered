#ifndef HTTP_VUE_H
#define  HTTP_VUE_H
#include <ros/ros.h>
#include <std_msgs/String.h>
#include<string>
#include <jsoncpp/json/json.h> // 需要安装jsoncpp库
#include<curl/curl.h>


class http_vue
{
private:

    
public:
    http_vue();
    ~http_vue();
    void  switch_json(const std_msgs::String msg);
    
    
};

































#endif