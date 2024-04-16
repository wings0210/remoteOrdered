#include "hmi.h"
#include <unicode/ustream.h>
#include <unicode/unistr.h>
#include <unicode/ucnv.h>

# define M_PI		3.14159265358979323846
const double ang_to_rad_ratio = M_PI / 180.0;
const double deg_to_rad = 3.1415926535 / 180.0;
int Hmi::location_flag=0;
int Hmi::loc_status=0;
//构造函数
Hmi::Hmi(ros::NodeHandle n,ros::NodeHandle thread)
{
    //从参数服务器取值
    n.param<std::string>("bywire_state_topic", bywire_state_topic, std::string("/bywire_vehicle_state")); //从参数服务器中取出"bywire_state_topic"的值给bywire_state_topic，如果服务器中没有赋予默认值"/bywire_vehicle_state"
    n.param<std::string>("basic_info_topic", basic_info_topic, std::string("/basic_info"));
    n.param<std::string>("slam_topic", slam_topic, std::string("/global_pose"));
    n.param<std::string>("pnc_system_Info", pnc_system_Info, std::string("/pnc_system_Info"));
    n.param("basic_info_hz", basic_info_hz, 10);

PNC_count =0;

    //启动时生成log文件，并将文件名赋值给相应的fileName
    SendFileName=create_SendLog_file();
    ReceiveFileName=create_ReceiveLog_file();

    //初始化发布器和进行订阅
    slam_subscriber= n.subscribe(slam_topic, 1, &Hmi::slam_Callback, this);
    vehicle_state_subscriber = n.subscribe(bywire_state_topic, 1, &Hmi::vehicle_state_Callback, this);
    pnc_subscriber = n.subscribe(pnc_system_Info, 1, &Hmi::pnc_Callback, this);
    veh_location_sub=n.subscribe("/vehicleLocalization",1,&Hmi::position_Callback,this);

    basic_info_publisher = n.advertise<plusgo_msgs::basic_info>(basic_info_topic, 10);

    assign_publisher=n.advertise<plusgo_msgs::middleware>("/assign",10);
    vehicle_ctrl_publisher=thread.advertise<plusgo_msgs::ctrl_params>("/params",10);
    set_vin_params_publisher=thread.advertise<plusgo_msgs::set_vin_params>("/vin_params",10); 
    system_params_publisher=thread.advertise<plusgo_msgs::system_params>("/sys_params",10);

   
    //重定位
    posemsgPub=thread.advertise<geometry_msgs::PoseStamped>("/force_Pose",10);
    // poseReplyPub=thread.advertise<middlewares_msgs::reply_reLocation>("/ros_reply_reLocation",10);

    //下一站
    //nextStationPub=thread.advertise<middlewares_msgs::middleware>("/nestStation_topic",10);

    //测试
    car_response=n.advertise<std_msgs::String>("/car_rev",10);

    //订阅来自hmi信息和向车端发布站点信息
    client_subscriber=thread.subscribe("/client_hmi",1,&Hmi::client_Callback,this);   

    //订阅来自hmi的控制参数状态信息
    vehicle_ctrl_subscriber=thread.subscribe("/ctrl_params",1,&Hmi::vehicle_ctrl_Callback,this);
    set_vin_params_subscriber=thread.subscribe("/set_vin_param",1,&Hmi::set_vin_Callback,this);
    system_params_subscriber=thread.subscribe("/ros_set_vehicle_param",1,&Hmi::system_params_Callback,this);

    //hmi重定位
    posemsgSub=thread.subscribe("/ros_reLocation",1,&Hmi::pose_stamp_Callback,this);//订阅hmi下发的重定位命令和坐标
    //change route
    change_route_sub=thread.subscribe("/ros_change_route",1, &Hmi::change_route_Callback,     this);
    //change_route_pub=thread.advertise<std_msgs::String>("/change_route",10);
   
    //poseReplySub=thread.subscribe("/init_match_success",1,&Hmi::posereply_Callback,this);

    //下一站
    // nextStationSub=thread.subscribe("/nextStation_topic_hmi",1,&Hmi::nextStation_Callback,this);

    //未使用属性初始化（非订阅topic取值）
    basic_info_msg.fault_code="";
    basic_info_msg.fault_desc="";
    basic_info_msg.FallAlarmStart=0;
    basic_info_msg.WindscenWipSt=0;    
    basic_info_msg.Highbeem=0;
    basic_info_msg.Windows=0;
    basic_info_msg.Doorlock=0;
    basic_info_msg.FaultSelfDiagnosist=0;
    basic_info_msg.tirePressure=0;
    basic_info_msg.SysPowMode=2;
    basic_info_msg.location.status=1;
    basic_info_msg.location.returnMsg="";
    basic_info_msg.perception.status=1;
    basic_info_msg.perception.returnMsg="";
    basic_info_msg.control.status=1;
    basic_info_msg.control.returnMsg="";
    basic_info_msg.plan.status=1;
    basic_info_msg.plan.returnMsg="";
    basic_info_msg.decision.status=1;
    basic_info_msg.decision.returnMsg="";
    basic_info_msg.chassic.status=1;
    basic_info_msg.chassic.returnMsg="";
    basic_info_msg.vin=load_config();
    basic_info_msg.signal=0;

    basic_msg_init.fault_code="";
    basic_msg_init.fault_code="";
    basic_msg_init.fault_desc="";
    basic_msg_init.FallAlarmStart=0;
    basic_msg_init.WindscenWipSt=0;    
    basic_msg_init.Highbeem=0;
    basic_msg_init.Windows=0;
    basic_msg_init.Doorlock=0;
    basic_msg_init.FaultSelfDiagnosist=0;
    basic_msg_init.tirePressure=0;
    basic_msg_init.SysPowMode=2;
    basic_msg_init.location.status=1;
    basic_msg_init.location.returnMsg="";
    basic_msg_init.perception.status=1;
    basic_msg_init.perception.returnMsg="";
    basic_msg_init.control.status=1;
    basic_msg_init.control.returnMsg="";
    basic_msg_init.plan.status=1;
    basic_msg_init.plan.returnMsg="";
    basic_msg_init.decision.status=1;
    basic_msg_init.decision.returnMsg="";
    basic_msg_init.chassic.status=1;
    basic_msg_init.chassic.returnMsg="";
    basic_msg_init.vin=load_config();
    basic_msg_init.signal=0;

    //测试

    //有意义但暂时不会改变的属性
    basic_info_msg.coor_system=0;
    
    //可以给但还没给值的属性 
    basic_info_msg.bottonB=0;
}
//析构函数
Hmi::~Hmi()
{
}

//前置函数
void Hmi::run(ros::AsyncSpinner spinner,ros::AsyncSpinner new_spinner)
{   
    execute_function(spinner,new_spinner);
}

//响应与发布topic函数
void Hmi::execute_function( ros::AsyncSpinner spinner,ros::AsyncSpinner new_spinner)
{
    ros::Rate  rate(basic_info_hz);

    
    while (ros::ok())
    {       
        //ros::spinOnce();  
        spinner.start();
        new_spinner.start();
        
         
        //发给hmi
        if(basic_flag)
        {
            if(location_flag==1)    location_flag=0;   
            else                                    basic_info_msg.location.status=1;
             basic_info_publisher.publish(basic_info_msg); 
    
		    PNC_count++;
		    if(PNC_count>=10)
		    {
		    basic_flag=false; 
		    }      
        }
        else
        {
            if(location_flag==1)    location_flag=0;   
            else                                    basic_msg_init.location.status=1;
            basic_info_publisher.publish(basic_msg_init);
        }
        const double deg_to_rad = 3.1415926535 / 180.0;

        from_car_log_create(basic_info_msg);                  
        rate.sleep();
    }
}

//生成来自车端的log文件
std::string Hmi::create_SendLog_file()
{

    std::time_t currentTime = std::time(nullptr);
    std::tm* localTime = std::localtime(&currentTime);

    // 格式化时间字符串
    char timeStr[100];
    std::strftime(timeStr, sizeof(timeStr), "%Y-%m%d-%H:%M:%S", localTime);

    //获取ROS软件包绝对路径
    std::string package_path= ros::package::getPath("ros_mutual")+"/record_log/";
 

    //定义产生文件名
    std::string FileName = package_path+std::string(timeStr) + "-send_hmi.txt";
 
    // 创建文件流对象
    std::ofstream file;
    file.open(FileName);

   if(file.is_open())
   {
        std::cout<<"来自车端信息日志生成成功"<<std::endl;
        file.close();
    }
    else
    {
        std::cout<<"来自车端信息日志生成失败"<<std::endl;
    }
    return FileName;

}

//生成来自hmi的log文件
std::string Hmi::create_ReceiveLog_file()
{
    std::time_t currentTime = std::time(nullptr);
    std::tm* localTime = std::localtime(&currentTime);

    // 格式化时间字符串
    char timeStr[100];
    std::strftime(timeStr, sizeof(timeStr), "%Y-%m%d-%H:%M:%S", localTime);

    //获取ROS软件包绝对路径
    std::string package_path= ros::package::getPath("ros_mutual")+"/record_log/";
 

    //定义产生文件名
    std::string FileName = package_path+std::string(timeStr) + "-recieve_hmi.txt";
    // 创建文件流对象
    std::ofstream file;
    file.open(FileName);

   if(file.is_open())
   {
        std::cout<<"来自hmi信息日志生成成功"<<std::endl;
        file.close();
    }
    else
    {
        std::cout<<"来自hmi信息日志生成失败"<<std::endl;
    }
    return FileName;
}

//向前端发送的数据写入log
void Hmi::from_car_log_create(const plusgo_msgs::basic_info& basic_info_msg)
{
    // 获取当前时间
    std::time_t currentTime = std::time(nullptr);
    std::tm* localTime = std::localtime(&currentTime);

    // 格式化时间字符串
    char timeStr[100];
    std::strftime(timeStr, sizeof(timeStr), "%Y-%m%d-%H:%M:%S", localTime);
    Json::Value jsonData;

    //bywire信息
    jsonData["brake"]=basic_info_msg.brake;
    jsonData["speed"]=basic_info_msg.speed;
    jsonData["angle"]=basic_info_msg.angle;
    jsonData["EPB"]=basic_info_msg.EPB;
    jsonData["soc"]=basic_info_msg.soc;
    jsonData["Lowbeem"]=basic_info_msg.Lowbeem;
    jsonData["emergencybutton"]=basic_info_msg.emergencybutton;
    jsonData["bottonA"]=basic_info_msg.bottonA;
    jsonData["bottonB"]=basic_info_msg.bottonB;
    jsonData["bottonC"]=basic_info_msg.bottonC;
    jsonData["EBS"]=basic_info_msg.EBS;
    jsonData["VCU"]=basic_info_msg.VCU;
    jsonData["EPS"]=basic_info_msg.EPS;
    jsonData["BCM"]=basic_info_msg.BCM;
    jsonData["control_mode"]=basic_info_msg.control_mode;
    jsonData["HzrdLtIO"]=basic_info_msg.HzrdLtIO;

    //slam信息
    jsonData["location_veh"]=basic_info_msg.location_veh;
    jsonData["latitude"]=basic_info_msg.latitude;
    jsonData["longitude"]=basic_info_msg.longitude;
    jsonData["azi"]=basic_info_msg.azi;

    //pnc信息
    jsonData["location_stop"]=basic_info_msg.location_stop;
    jsonData["routestop"]=basic_info_msg.routestop;
    jsonData["trip_distance"]=basic_info_msg.trip_distance;
    jsonData["trip_eta"]=basic_info_msg.trip_eta;
    jsonData["trip_left_mileage"]=basic_info_msg.trip_left_mileage;
    jsonData["veh_status"]=basic_info_msg.veh_status;
    jsonData["maxSpeed"]=basic_info_msg.maxSpeed;
    jsonData["signal"]=basic_info_msg.signal;
    jsonData["current_stop"]=basic_info_msg.current_stop;
    jsonData["obsmode"]=basic_info_msg.obsmode;
    jsonData["osVersion"]=basic_info_msg.osVersion;
    jsonData["routesVersion"]=basic_info_msg.routesVersion;
    jsonData["plan.status"]=basic_info_msg.plan.status;
    jsonData["plan.returnMsg"]=basic_info_msg.plan.returnMsg;
    jsonData["decision.status"]=basic_info_msg.decision.status;
    jsonData["decision.returnMsg"]=basic_info_msg.decision.returnMsg;
    jsonData["location.status"]=basic_info_msg.location.status;
    jsonData["location.returnMsg"]=basic_info_msg.location.returnMsg;
    jsonData["control.status"]=basic_info_msg.control.status;
    jsonData["control.returnMsg"]=basic_info_msg.control.returnMsg;
    jsonData["chassic.status"]=basic_info_msg.chassic.status;
    jsonData["chassic.returnMsg"]=basic_info_msg.chassic.returnMsg;
    jsonData["perception.status"]=basic_info_msg.perception.status;
    jsonData["perception.returnMsg"]=basic_info_msg.perception.returnMsg;
    jsonData["vin"]=basic_info_msg.vin;
    jsonData["timestamp"]=basic_info_msg.timestamp;

   // 将时间戳和数据写入日志文件
    std::ofstream logFile(SendFileName, std::ios::app);
    Json::StreamWriterBuilder writer;
    logFile <<std::string(timeStr) << " - " << Json::writeString(writer,jsonData)<< std::endl;
    logFile.close();

}

//向车端发送的数据写入log
void Hmi:: from_hmi_log_create(const plusgo_msgs::middleware& station_msg)
{
    // 获取当前时间
    std::time_t currentTime = std::time(nullptr);
    std::tm* localTime = std::localtime(&currentTime);

    // 格式化时间字符串
    char timeStr[100];
    std::strftime(timeStr, sizeof(timeStr), "%Y-%m%d-%H:%M:%S", localTime);


    Json::Value jsonData;


    //hmi信息
    //jsonData["IsPause"]=station_msg.IsPause;
    jsonData["task"]=station_msg.task;
    jsonData["longitude"]=station_msg.longitude;
    jsonData["latitude"]=station_msg.latitude;
    jsonData["azi"]=station_msg.azi;

    // 将时间戳和数据写入日志文件
    std::ofstream logFile(ReceiveFileName, std::ios::app);
    Json::StreamWriterBuilder writer;
    logFile <<std::string(timeStr) << " - " << Json::writeString(writer,jsonData)<< std::endl;
    logFile.close();

}

//冗余日志清理
void Hmi::clear_log(){

  std::string dirPath= ros::package::getPath("ros_mutual")+"/record_log";
  std::streampos maxSize = 100* 1024 * 1024;
    DIR* dir = opendir(dirPath.c_str());
    if (!dir) {
        std::cerr << "无法打开目录：" << dirPath << std::endl;
        return;
    }

    dirent* entry;
    while ((entry = readdir(dir)) != nullptr) 
    {
        std::string filename = entry->d_name;
        std::string filePath = dirPath + "/" + filename;

        // 检查文件是否是普通文件且扩展名为 .txt
        if (entry->d_type == DT_REG && filename.substr(filename.length() - 4) == ".txt") 
        {
            std::ifstream file(filePath, std::ios::binary | std::ios::ate);
            if (!file) 
            {
                std::cerr << "无法打开文件：" << filePath << std::endl;
                continue;
            }

            std::streampos fileSize = file.tellg();
            if (fileSize > maxSize)
            {
                file.close();

                std::ofstream clearFile(filePath, std::ios::trunc);
                if (!clearFile) 
                {
                    std::cerr << "无法清空文件：" << filePath << std::endl;
                    continue;
                }
                std::cout << "文件 " << filePath << " 已成功清空。" << std::endl;
            }
            else
            {
            }
        }
    }
    closedir(dir);
 }

//读取yaml配置文件信息
std::string Hmi:: load_config(){

    //读取yaml
    std::string package_path= ros::package::getPath("ros_mutual");
    std::ifstream fin(package_path+"/config/hmi.yaml");
    
    // 将文件内容解析为YAML文档
    YAML::Node doc = YAML::Load(fin);

    // 读取vin字符串的值
    std::string vin = doc["vin"].as<std::string>();

    return vin;
}

// "/bywire_vehicle_state"topic订阅回调
void Hmi::vehicle_state_Callback(const plusgo_msgs::ByWireVehicleState &msg)
{
    basic_info_msg.gears=std::to_string(msg.gear);
    basic_info_msg.brake=msg.real_brake;
    basic_info_msg.speed=msg.vehicle_speed/3.6;
    basic_info_msg.angle=std::to_string(msg.steer_angle);
    basic_info_msg.EPB=msg.EPB;
    basic_info_msg.soc=(double)msg.SOC;
    basic_info_msg.Lowbeem=msg.lowlight;
    basic_info_msg.emergencybutton=msg.stop;
    basic_info_msg.bottonA=msg.nextstation;
    basic_info_msg.bottonC=msg.autoflag;
    basic_info_msg.EBS=msg.EBS_mode;
    basic_info_msg.VCU=msg.mode>0?1:0;
    basic_info_msg.EPS=msg.steer_mode_valid==1?msg.steer_mode:0;
    basic_info_msg.BCM=msg.BCM_valid==1?msg.BCM_mode:0;
    basic_info_msg.control_mode=msg.mode;
    basic_info_msg.HzrdLtIO=msg.leftlight&msg.rightlight;
}

// "/global_pose"topic订阅回调
void Hmi::slam_Callback(const geometry_msgs::PoseStamped &msg)
{
    basic_info_msg.location_veh=std::to_string(msg.pose.position.x)+","+std::to_string(msg.pose.position.y);
}

// "/pnc_system_Info"topic订阅回调
void Hmi::pnc_Callback(const plusgo_msgs::SystemInfoReport &msg)
{
    basic_flag=true;
PNC_count=0;
   // basic_info_msg.location_stop=std::to_string(msg.location_stop_lat)+","+std::to_string(msg.location_stop_lon);
    basic_info_msg.routestop=msg.route_stop;
    basic_info_msg.trip_distance=msg.trip_distance;
    basic_info_msg.trip_eta=msg.trip_eta;
    basic_info_msg.trip_left_mileage=msg.trip_left_mileage;
    basic_info_msg.veh_status=msg.veh_status;
    basic_info_msg.maxSpeed=msg.maxSpeed;
    basic_info_msg.signal=msg.signalLevel;
    basic_info_msg.current_stop=msg.current_stop;
    basic_info_msg.obsmode=msg.obsmode;
    basic_info_msg.osVersion=msg.osVersion;
    basic_info_msg.routesVersion=msg.routesVersion;
    basic_info_msg.plan.status=msg.plan.status;
    basic_info_msg.plan.returnMsg=msg.plan.returnMsg;
    basic_info_msg.decision.status=msg.decision.status;
    basic_info_msg.decision.returnMsg=msg.decision.returnMsg;
    basic_info_msg.control.status=msg.decision.status;
    basic_info_msg.chassic.status=msg.decision.status;
    basic_info_msg.perception.status=msg.perception.status;
    basic_info_msg.timestamp=std::to_string(ros::Time::now().toSec());
    basic_info_msg.stationName=msg.target_station_name;
}

//下发站点topic订阅回调
void Hmi::client_Callback(const plusgo_msgs::middleware& msg)
{
    station_msg.cmd_id=1;
    station_msg.azi=msg.azi;
    station_msg.latitude=msg.latitude;
    station_msg.longitude=msg.longitude;
    station_msg.nextStation=msg.nextStation;
    station_msg.stationName=msg.stationName;
    station_msg.task=msg.task;
    station_msg.route_dir="";
    assign_publisher.publish(station_msg);              //“/assign”，站点调度

    from_hmi_log_create(station_msg);
}

//订阅HMI车辆控制参数回调
void Hmi::vehicle_ctrl_Callback(const plusgo_msgs::ctrl_params &msg)
{
    params_msg.doorlock=msg.doorlock;                                                                    //门锁状态
    params_msg.emergencybutton=msg.emergencybutton;                               //急停按钮
    params_msg.fallAlarmStart=msg.fallAlarmStart;                                               //误检启动
    params_msg.faultSelfDiagnosis=msg.faultSelfDiagnosis;                             //故障自检
    params_msg.highbeem=msg.highbeem;                                                              //远光灯
    params_msg.hzrdLtIO=msg.hzrdLtIO;                                                                    //双闪
    params_msg.lowbeem=msg.lowbeem;                                                                 //近光灯
    params_msg.speed=msg.speed;                                                                              //车速
    params_msg.windows=msg.windows;                                                                  //车窗
    params_msg.windscenWipSt=msg.windscenWipSt;                                       //雨刮

    //将消息发布给车端
    vehicle_ctrl_publisher.publish(params_msg);       //“/ctrl_params”，车灯参数等
}

//HMI的vin等特殊参数回调
void Hmi::set_vin_Callback(const plusgo_msgs::set_vin_params &msg)
{
    vin_msg.fastStart=msg.fastStart;
    vin_msg.routeCode=msg.routeCode;
    vin_msg.routeNumber=msg.routeNumber;
    vin_msg.vin=msg.vin;
    vin_msg.paramChange=msg.paramChange;

    //接收雷达参数数据，暂不处理
    std::vector<plusgo_msgs::RadarParam>params_msg=msg.radarParam;

    for (const auto &param : params_msg) 
    {
        ROS_INFO("Parameter Name: %s, Parameter Number: %f", param.name.c_str(), param.number);
        
    }

    //将消息发布给车端
    set_vin_params_publisher.publish(vin_msg);      //“/vin_params”，vin等其他参数
}

//HMI的系统参数回调
void Hmi::system_params_Callback(const plusgo_msgs::system_params &msg)
{
    sys_msg.brakeDistance=msg.brakeDistance;
    sys_msg.decelerationDistance=msg.decelerationDistance;
    sys_msg.materialWide=msg.materialWide;
    sys_msg.testDistance=msg.testDistance;
    sys_msg.testRadius=msg.testRadius;
    sys_msg.vehicleWide=msg.vehicleWide;
    sys_msg.paramChange=msg.paramChange;
    
    //将消息发布给车端
    system_params_publisher.publish(sys_msg);       //“/sys_params”，系统参数

}

//HMI重定位回调
void Hmi::pose_stamp_Callback(const plusgo_msgs::middleware &msg)
{
    pose_msg.latitude=msg.latitude;
    pose_msg.longitude=msg.longitude;
    pose_msg.azi=msg.azi;
    pose_msg.alt=msg.alt;
    ROS_INFO("msg.azi: %f",msg.azi);
    pose2poseStamped(pose_msg.longitude,pose_msg.latitude,pose_msg.azi,pose_msg.alt);
}

//转化重定位的经纬度
void Hmi::pose2poseStamped(const double lon, const double lat, const double azi,const double alt)
{
    // 输入 
    double longitude= lon;
    double latitude = lat;
    double altitude = alt; // 柳州平均海拔95
    double yaw      = azi;
    double roll     = 0;
    double pitch    = 0;

    // 输出
    if( yaw > 180 )
    {
        yaw -= 360;
    }
    yaw = yaw * deg_to_rad;
    // ROS_INFO("yaw: %f",yaw);
    tf::Quaternion orientation;
    orientation = tf::createQuaternionFromRPY(roll,-pitch, -yaw); 

    // 输出
    geometry_msgs::PoseStamped msg; //重新定义

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "force_Pose";
    msg.pose.position.x =latitude;
    msg.pose.position.y =longitude;
    msg.pose.position.z = altitude;
    
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();
    msg.pose.orientation.w = orientation.w();

    // 发布 
    posemsgPub.publish(msg);
}



void Hmi::posereply_Callback(const std_msgs::BoolConstPtr& msg )
{
    // initLocation_reply.data=bool(msg);
    // rosreply_msg.isrelocation=bool(true);
    // poseReplyPub.publish(rosreply_msg);
}

void Hmi::position_Callback(const plusgo_msgs::VehicleLocation &msg)
{
    basic_info_msg.longitude=msg.position.y;
    basic_info_msg.latitude=msg.position.x;

    basic_msg_init.longitude=msg.position.y;
    basic_msg_init.latitude=msg.position.x;

    basic_info_msg.location.status=loc_status;
    basic_msg_init.location.status=loc_status;

    double tmp_azi=msg.pose.z;
    tmp_azi=360.0-(tmp_azi/ang_to_rad_ratio);
    if(tmp_azi >= 360.0)
            tmp_azi -= 360.0;
    if(tmp_azi <0)
           tmp_azi += 360.0;

    basic_info_msg.azi=tmp_azi;
    basic_msg_init.azi=tmp_azi;
    
    location_flag=1;

}


std::string Hmi::decodeUnicode(const std::string& input)
{
    std::string decodedString;
    size_t pos = 0;
    
    while (pos < input.length()) {
        //if (input[pos] == '\\' && pos + 3 <= input.length() && input.substr(pos, 2) == "\\u")
         if ( pos + 3 <= input.length() )
         {
            std::string hexString = input.substr(pos , pos+2);
            int unicodeValue = std::stoi(hexString, 0, 16); // 将16进制字符串转换为整数
            decodedString += static_cast<wchar_t>(unicodeValue); // 将整数转换为宽字符
            pos += 3;
        } else {
            decodedString += input[pos];
            pos++;
        }
    }
    return decodedString;
}


void  Hmi::change_route_Callback(const std_msgs::String &msg){
     route_dir=msg.data;
      //  change_route_pub.publish(route_dir);
     station_msg.cmd_id=2;
     station_msg.azi=0;
     station_msg.latitude=0;
     station_msg.longitude=0;
     station_msg.nextStation=0;
     station_msg.stationName="";
     station_msg.task="";
     station_msg.route_dir=route_dir;
     assign_publisher.publish(station_msg);    
}


 //void Hmi::jungle_location(){
//     std::thread inspect_thread([=](){
//     while (1) {
//          if(location_flag==0)
//                 loc_status=1;
//          location_flag=0;
//     std::this_thread::sleep_for(std::chrono::milliseconds(50));
//     }
//     });
//     inspect_thread.detach();
 //}

















