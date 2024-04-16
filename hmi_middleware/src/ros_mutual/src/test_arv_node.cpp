#include<ros/ros.h>
#include<iostream>
#include"hmi.h"



void call_back(const plusgo_msgs::basic_info basic_info_msg)
{
  std:: cout<<basic_info_msg.angle<<std::endl;
  std:: cout<<basic_info_msg.vin<<std::endl;
}

void car_call_back(std_msgs::String msg)
{
  std:: cout<<msg.data<<std::endl;
}

void vin_params_call_back(const plusgo_msgs::set_vin_params &msg)
{
  std::cout<<msg.paramChange;
}
void location_call_back(const plusgo_msgs::middleware pose_msg)
{
  std::cout<<pose_msg.stationName<<std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_arv_node");
  ros::NodeHandle nh;

  
  // ros::NodeHandle nh_;


  // middlewares_msgs::relocation pose_msg_test;

  ros::Subscriber sub=nh.subscribe("/basic_info",10,call_back);
   ros::Subscriber sub_test=nh.subscribe("/vin_params",10,vin_params_call_back);
  // ros::Subscriber pose_sub=nh.subscribe("/force_Pose",10,location_call_back);

  // ros::Publisher relocation_pub=nh.advertise<std_msgs::Bool>("/init_match_success",10);
  
  // std_msgs::Bool init_msg;
  // init_msg.data=true;
  // relocation_pub.publish(init_msg);
  // poseReplyPub=thread.advertise<middlewares_msgs::middleware>("/ros_reply_reLocation",10);
  // poseReplySub=thread.subscribe("/init_match_success",1,&Hmi::posereply_Callback,this);


  // ros::Publisher  posePub=nh_.advertise<middlewares_msgs::relocation>("/force_Pose",10);

  // posePub.publish(pose_msg_test);

  // ros::spin();
  // while (ros::ok())
  // {
    // ros::Rate loop_rate(10);
    // std_msgs::Bool init_msg;
    // init_msg.data=true;
    // relocation_pub.publish(init_msg);

    // ros::spinOnce();  // 处理回调函数
    // loop_rate.sleep();  // 控制发布频率
    
  // } const char* JsonString=ClientData.c_str();


}































