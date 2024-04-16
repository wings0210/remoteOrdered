#include<ros/ros.h>
#include<std_msgs/String.h>
#include"http_vue.h"

void callback(const std_msgs::String msg){

          http_vue* hv=new http_vue();
          hv->switch_json(msg);

}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "hmi_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/ros_topic", 1, callback);
  ros::spin();
  return 0;
}