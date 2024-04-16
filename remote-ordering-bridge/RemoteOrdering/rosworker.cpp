#include "rosworker.h"

int RosWorker::cmdid(0);

RosWorker* RosWorker::instance_ = nullptr;

RosWorker& RosWorker::getInstance() {
    if (!instance_) {
        instance_ = new RosWorker;
    }
    return *instance_;
}

RosWorker::RosWorker(QObject *parent): QObject(parent){
    ros::init(ros::M_string(), "qt_ros_dispatch", ros::init_options::NoSigintHandler);
    nh_ = new ros::NodeHandle;
    io_publisher=nh_->advertise<std_msgs::String>("/io_control",1);
    client_ = nh_->serviceClient<plusgo_msgs::dispatch_cmd>("/dispatch_srv");
    relocation_pub = nh_->advertise<geometry_msgs::PoseStamped>("/force_Pose", 1);
    slam_subscriber= nh_->subscribe("/vehicleLocalization", 1, &RosWorker::slam_Callback, this);
    vehicle_state_subscriber = nh_->subscribe("/bywire_vehicle_state", 1, &RosWorker::vehicle_state_Callback, this);
    pnc_subscriber = nh_->subscribe("/pnc_system_Info", 1, &RosWorker::pnc_Callback, this);

    sub_assign = nh_->subscribe("/assign", 1, &RosWorker::AssignCallback, this);

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this ,&RosWorker::spin);
    timer->start(100); // 每100毫秒处理一次消息回调
}

RosWorker::~RosWorker(){
    ros::shutdown();
}
void RosWorker::callMessage(const struct Station& station)
{
    send_cmd.request.cmdid = cmdid;
    send_cmd.request.cmd_type = 0;
    send_cmd.request.target_station_name =station.stationName.toStdString();
    send_cmd.request.target_point.Lon = station.lon;
    send_cmd.request.target_point.Lat = station.lat;
    send_cmd.request.target_point.yaw = station.azi;

    send_cmd.request.target_station_id=station.stationId;

    qDebug()<<"send station to pnc : "<<station.stationName;

    io_msg.data="Launch";
    io_publisher.publish(io_msg);
    io_msg.data="";

    if(client_.call(send_cmd))
    {
        if (!send_cmd.response.succeed)
        {
            qDebug()<<"命令执行失败";
            return ;
        }
        if ( send_cmd.response.cmdid != cmdid)
        {
            qDebug()<<"命令执行顺序有误";
            return ;
        }
        cmdid += 1;
        //ack_responce(cloud);
    }
    else {
        qDebug()<<"接受反馈失败";
    }
}
void RosWorker::subMessage(const struct Cloud &cloud)
{
    double yaw =cloud.data.relocation->azi;
    if( yaw > 180 )
        yaw -= 360;
    yaw = yaw * deg_to_rad;
    tf::Quaternion orientation;
    orientation = tf::createQuaternionFromRPY(0, 0, -yaw);
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "cloud";
    msg.pose.position.x = cloud.data.relocation->lat;
    msg.pose.position.y = cloud.data.relocation->lon;
    msg.pose.position.z = 95;
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();
    msg.pose.orientation.w = orientation.w();
    relocation_pub.publish(msg);
}
void RosWorker::callMessage(const struct Cloud& cloud){
    send_cmd.request.cmdid = cmdid;
    switch (cloud.cmd)
    {
    case Cmd::stop_ :
        send_cmd.request.cmd_type = 3;
        break;
    case Cmd::start_ :
        emit this->sendstation();
        qDebug()<<"start to rossendstation";
        return;
        break;
    case Cmd::pause_ :
        send_cmd.request.cmd_type = 5;
        break;
    case Cmd::resume_ :
        send_cmd.request.cmd_type = 6;
        break;
    case Cmd::param_:
        send_cmd.request.cmd_type = 7;
        send_cmd.request.Lowbeem = cloud.data.param.Lowbeem;
        send_cmd.request.HzrdLtIO = cloud.data.param.HzrdLtIO;
    default :
        qDebug()<< "error cmd :";
        //printjson();
        return;
    }
    if(client_.call(send_cmd))
    {
        if (!send_cmd.response.succeed)
        {
            qDebug()<<"命令执行失败";
            return ;
        }
        if ( send_cmd.response.cmdid != cmdid)
        {
            qDebug()<<"命令执行顺序有误";
            return ;
        }
        cmdid += 1;
        //ack_responce(cloud);
    }
    else {
        qDebug()<<"接受反馈失败";
    }
}

void RosWorker::spin()
{
    ros::spinOnce();
    if(instation && go_next_station)
    {
        emit this->sendstation();
    }
    else if(go_next_station)
    {
        emit this->sendlasteststation();
    }
}
void RosWorker::subscriberCallback(QByteArray jsonQString){
    QJsonParseError errorjson;
    QJsonDocument doc = QJsonDocument::fromJson(jsonQString, &errorjson);
    QString error="";
    struct Cloud cloud_;
    if(errorjson.error == QJsonParseError::NoError) {
        bool get_cmd=prase_json(doc,error,cloud_);
        if(!get_cmd)
            return ;
        cloud.push_back(cloud_);
    }
    else{
        //        bool force_get_cmd_failed = extract_from_invalid(jsonQString,error);
        //        if(force_get_cmd_failed)
        return;
    }
    if(emit_to_main)
    {
        emit receivedMessage();
        emit_to_main = false;
    }

}

void RosWorker::sendSignal()
{
    emit_to_main = true;
}

void RosWorker::slam_Callback(const plusgo_msgs::VehicleLocation &msg)
{
    Cloud_task::carState.carLongitude=msg.position.y;
    //qDebug()<<Cloud_task::carState.carLongitude;
    Cloud_task::carState.carLatitude=msg.position.x;
   // Cloud_task::carState.carLongitude=120.6473481;
    //Cloud_task::carState.carLatitude=31.4633679;
}
void RosWorker::pnc_Callback(const plusgo_msgs::SystemInfoReport &msg)
{
    Cloud_task::carState.destLatitude=msg.target_point.Lat;
    Cloud_task::carState.destLongitude=msg.target_point.Lon;
    Cloud_task::carState.route_stop=QString::fromStdString(msg.route_stop);
    Cloud_task::carState.trip_distance=msg.trip_distance;
    Cloud_task::carState.trip_eta=msg.trip_eta;
    Cloud_task::carState.trip_left_mileage=msg.trip_left_mileage;
    Cloud_task::carState.veh_status=msg.veh_status;
    Cloud_task::carState.maxSpeed=msg.maxSpeed;
    Cloud_task::carState.signal=msg.signal;
    Cloud_task::carState.current_stop=QString::fromStdString(msg.current_stop);
    //Cloud_task::carState.obsmode=msg.obsmode;
    Cloud_task::carState.osVersion=msg.osVersion;
    Cloud_task::carState.routesVersion=msg.routesVersion;
    Cloud_task::carState.fault_code=+msg.plan.status;
    Cloud_task::carState.fault_desc+=QString::fromStdString(msg.plan.returnMsg);
    Cloud_task::carState.fault_code+=msg.decision.status;
    Cloud_task::carState.fault_desc+=QString::fromStdString(msg.decision.returnMsg);
    //Cloud_task::carState.timestamp=std::to_string(ros::Time::now().toSec());
    if(msg.stop_mode == 2){
        if(instation==0)
            Cloud_task::state.isArriveSite = true;
        instation = 1;
    }
    else {
        instation = 0;
    }
}
void RosWorker::vehicle_state_Callback(const plusgo_msgs::ByWireVehicleState &msg)
{
    Cloud_task::carState.gears=msg.gear;
    //Cloud_task::carState.brake=msg.real_brake;
    Cloud_task::carState.speed=msg.vehicle_speed;
    Cloud_task::carState.angle=msg.steer_angle;
    //Cloud_task::carState.EPB=msg.EPB;
    Cloud_task::carState.soc=(double)msg.SOC;
    Cloud_task::carState.Lowbeem=msg.lowlight;
    Cloud_task::carState.emergencybutton=msg.stop;
    Cloud_task::carState.buttonA=msg.nextstation;
    if(msg.nextstation == 1 && last_button_nextstation_state ==0)
    {
        button_nextstation_cnt = 10;
        go_next_station = true;
    }
    if(button_nextstation_cnt)
    {
        button_nextstation_cnt --;
    }
    else {
        go_next_station = false;
    }
    last_button_nextstation_state = msg.nextstation;
    //Cloud_task::carState.bottonC=msg.autoflag;
    //Cloud_task::carState.EBS=msg.EBS_mode;
    //Cloud_task::carState.VCU=msg.mode>0?1:0;
    //Cloud_task::carState.EPS=msg.steer_mode_valid==1?msg.steer_mode:0;
    //Cloud_task::carState.BCM=msg.BCM_valid==1?msg.BCM_mode:0;
    Cloud_task::carState.control_mode=!msg.mode;
    Cloud_task::carState.HzrdLtIO=msg.leftlight&msg.rightlight;
}

void RosWorker::AssignCallback(const plusgo_msgs::middleware &message)
{
    send_cmd.request.cmdid = cmdid;
    if(message.cmd_id == 1)
    {
        QString task = QString::fromStdString(message.task);
        if(!message.nextStation)
        {//选择下发站点
            if(task == "start")
            {
                send_cmd.request.cmd_type = 0;
                send_cmd.request.target_station_name = message.stationName;
                send_cmd.request.target_point.Lon = message.longitude;
                send_cmd.request.target_point.Lat = message.latitude;
                send_cmd.request.target_point.yaw = message.azi;

                qDebug() << "send station to pnc : " << QString::fromStdString(message.stationName);
            }else if(task == "pause"){
                send_cmd.request.cmd_type = 5;
            }else{
                send_cmd.request.cmd_type = 6;
            }
            if(client_.call(send_cmd))
            {
                if (!send_cmd.response.succeed)
                {
                    qDebug()<<"命令执行失败";
                    return ;
                }
                if ( send_cmd.response.cmdid != cmdid)
                {
                    qDebug()<<"命令执行顺序有误";
                    return ;
                }
                cmdid += 1;
                //ack_responce(cloud);
            }
            else {
                qDebug()<<"接受反馈失败";
            }
        }else{
            //下一站
            emit this->HMINextStation();
        }
    }else if(message.cmd_id == 2)
    {
        QStringList stringList = QString::fromStdString(message.route_dir).split("@");
        QString fileName = "../../../conf/remoteorder-conf/conf.json";
        if (!QFile::exists(fileName)) {
            qDebug() << "File" << fileName << "does not exist";
            return ;
        }
        QFile file(fileName);

        if (!file.open(QIODevice::ReadWrite | QIODevice::Text)) {
            qDebug() << "无法打开文件";
            return ;
        }
        QByteArray jsonData = file.readAll();
        QJsonParseError error;
        QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData, &error);
        if (error.error != QJsonParseError::NoError) {
            qDebug() << "解析 JSON 数据错误：" << error.errorString();
            file.close(); // 关闭文件
            return ;
        }
        QJsonObject jsonObject = jsonDoc.object();

        jsonObject["map"] = stringList[0];
        jsonObject["station"] = stringList[1];
        jsonDoc = QJsonDocument(jsonObject);

        file.resize(0);
        file.write(jsonDoc.toJson());
        file.close();

        send_cmd.request.map_dir = stringList[0].toStdString();
        send_cmd.request.stop_dir = stringList[1].toStdString();
        send_cmd.request.cmd_type = 8;
        if(client_.call(send_cmd))
        {
            if (!send_cmd.response.succeed)
            {
                qDebug()<<"命令执行失败";
                return ;
            }
            if ( send_cmd.response.cmdid != cmdid)
            {
                qDebug()<<"命令执行顺序有误";
                return ;
            }
            cmdid += 1;
            //ack_responce(cloud);
        }
        else {
            qDebug()<<"接受反馈失败";
        }
        emit this->changeRoute();
    }
}
