#ifndef HMI_H
#define HMI_H

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <ros/console.h>
#include <jsoncpp/json/json.h>
#include <yaml-cpp/yaml.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <codecvt>
#include <locale>
#include <set>
#include <vector>

#include"plusgo_msgs/middleware.h"
#include"plusgo_msgs/ctrl_params.h"
#include"plusgo_msgs/set_vin_params.h"
#include"plusgo_msgs/RadarParam.h"
#include"plusgo_msgs/system_params.h"
#include"plusgo_msgs/basic_info.h"
#include"plusgo_msgs/ErrorCode.h"
#include "plusgo_msgs/ByWireVehicleState.h"
#include "plusgo_msgs/SystemInfoReport.h"
#include "plusgo_msgs/VehicleLocation.h"

#include <dirent.h>
#include <ros/callback_queue.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <thread>




class Hmi
{
public:
    
    Hmi(ros::NodeHandle n,ros::NodeHandle thread);
    ~Hmi();
    void run(ros::AsyncSpinner spinner,ros::AsyncSpinner new_spinner);
    void from_car_log_create(const plusgo_msgs::basic_info& basic_info_msg);
    void from_hmi_log_create(const plusgo_msgs::middleware& hmi_msg);
    std::string load_config();
    std::string create_SendLog_file();
    std::string create_ReceiveLog_file();
    void clear_log();
    std::string decodeUnicode(const std::string& input);
    static int location_flag;
    static int loc_status;

//C++里的private修饰符号，跟Java里不一样，标识为只能在该类中使用，故没有this调用这一说，即只能被该类的成员函数使用，不能被实例化
private:
    int basic_info_hz;

	int PNC_count;

    bool basic_flag;
    
    void execute_function( ros::AsyncSpinner spinner,ros::AsyncSpinner new_spinner);

    void vehicle_state_Callback(const plusgo_msgs::ByWireVehicleState &msg);

    void slam_Callback(const geometry_msgs::PoseStamped &msg);

    void pnc_Callback(const plusgo_msgs::SystemInfoReport &msg);

    void position_Callback(const plusgo_msgs::VehicleLocation &msg);

//来自hmi话题的订阅回调
    //站点
    // void client_Callback(const std_msgs::String &msg);
    void client_Callback(const plusgo_msgs::middleware& msg);

    //灯光控制
    void vehicle_ctrl_Callback(const plusgo_msgs::ctrl_params &msg);
        
    //参数控制
    void set_vin_Callback(const plusgo_msgs::set_vin_params &msg);

    //系统参数控制
    void system_params_Callback(const plusgo_msgs::system_params &msg);

    //重定位
    // void pose_stamp_Callback(const std_msgs::String &msg);
    void pose_stamp_Callback(const plusgo_msgs::middleware &msg);
    // void pose_Callback(const std_msgs::bool &msg);
    void posereply_Callback(const std_msgs::BoolConstPtr& msg );

    //下一站
    void nextStation_Callback(const std_msgs::String &msg );

    //对重定位的数据进行格式转换
    void pose2poseStamped(const double lon, const double lat,const double azi,const double alt);

   void  change_route_Callback(const std_msgs::String &msg);

    //void jungle_location();

    ros::Subscriber vehicle_state_subscriber;
    ros::Subscriber pnc_subscriber;
    ros::Subscriber slam_subscriber;
    ros::Publisher basic_info_publisher;
    ros::Publisher car_response;

    ros::Subscriber veh_location_sub;

    //站点
    ros::Subscriber client_subscriber;
    ros::Publisher assign_publisher;

    //车辆控制参数
    ros::Subscriber vehicle_ctrl_subscriber;
    ros::Publisher vehicle_ctrl_publisher;

    //车辆系统参数
    ros::Publisher system_params_publisher;     
    ros::Subscriber system_params_subscriber;

    //车辆vin等特殊参数
    ros::Publisher set_vin_params_publisher;
    ros::Subscriber  set_vin_params_subscriber;

    //重定位
    ros::Publisher posemsgPub;
    ros::Subscriber posemsgSub;
    //重定位的反馈
    ros::Publisher  poseReplyPub;
    ros::Subscriber poseReplySub;

   //change_route
   ros::Subscriber  change_route_sub;
   ros::Publisher  change_route_pub;
    //下一站
    // ros::Publisher nextStationPub;
    // ros::Subscriber nextStationSub;

    std::string bywire_state_topic;
    std::string basic_info_topic;
    std::string slam_topic;
    std::string pnc_system_Info;


    //自定义缓存订阅车端topic信息的数据结构，并作为发布topic的信息格式
   plusgo_msgs::basic_info  basic_info_msg;
   plusgo_msgs::basic_info  basic_msg_init;

    //接收客户端发来的站点msg信息
    std_msgs::String  hmi_string;
    std_msgs::String pose_string;
    std_msgs::String nextstation_string;

//接收hmi客户端发来的参数状态信息
   plusgo_msgs::ctrl_params params_msg;
    plusgo_msgs::set_vin_params vin_msg;
    plusgo_msgs::system_params sys_msg;

    //待解析的std_msgs::String hmi_string.data
    // std::string ClientData;

    //从hmi接收，灯光控制等信息，向车端发送的msg
    // middlewares_msgs::middleware hmi_msg;
   plusgo_msgs::middleware station_msg;

    //重定位，从hmi接收，发给定位模块的msg
    plusgo_msgs::middleware pose_msg;
    // std::string ClientData_2;
    std_msgs::Bool initLocation_reply;
    // middlewares_msgs::reply_reLocation rosreply_msg;

    //下一站，从hmi接收，发给pnc
    // middlewares_msgs::middleware nextStation_msg;
    // middlewares_msgs::middleware nextStation_string;

    //自定义文件名字符串，存储每次启动生成log文件名
    std::string SendFileName;
    std::string ReceiveFileName;


    std::string  route_dir="";

    
  
};


#endif // HMI_H
