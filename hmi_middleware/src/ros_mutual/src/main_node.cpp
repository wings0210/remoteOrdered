#include<ros/ros.h>
#include<std_msgs/String.h>
#include"http_vue.h"
#include"hmi.h"





int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "main_node");
  ros::NodeHandle n;
  ros::NodeHandle thread;

//声明异步Callback队列
  ros::CallbackQueue my_queue;
  thread.setCallbackQueue(&my_queue);
  //异步启停，开启指定数量的Spinner线程并发执行Callback队列中的可用回调，并指定Callback队列
  ros::AsyncSpinner spinner(3);
  ros::AsyncSpinner new_spinner(10, &my_queue);
  
  Hmi* hmi=new Hmi(n,thread);
  hmi->clear_log();
  hmi->run(spinner,new_spinner);

  ros::waitForShutdown();

  delete hmi;
  
  return 0;
}