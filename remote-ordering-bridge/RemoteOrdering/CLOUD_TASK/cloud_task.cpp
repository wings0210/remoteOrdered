#include"CLOUD_TASK/cloud_task.h"
std::mutex mtx;
std::queue<QByteArray> Cloud_task::buffer_queue = std::queue<QByteArray>();
CarState Cloud_task::carState ={"","",0,0,0,0,0,0,0,0,"",0,0,0,0,"","","",0,0,0,0,0,0,0,0,0,0,0,"",0,0,0};
State Cloud_task::state={false,NodeParam::default_n,false,false,false,false,false,false,false,false};
Cloud_task::Cloud_task(){

    YAML::Node config = YAML::LoadFile("../../../conf/remoteorder-conf/conf.yaml");

    // 读取相应参数的值
    string Vin = config["Vin"].as<string>();
    vinCode=QString::fromStdString(Vin);
    PubTimer = new QTimer ();
    PubTimer->setInterval(1000);
    PubTimer->moveToThread(&PubThread);
    QObject::connect(PubTimer,&QTimer::timeout,this,&Cloud_task::PubTimeTick);
    QObject::connect(&PubThread,SIGNAL(started()),PubTimer,SLOT(start()));
    QObject::connect(&PubThread,SIGNAL(finished()),PubTimer,SLOT(stop()));
    PubThread.start();
}
Cloud_task::~Cloud_task(){
    if(PubTimer != nullptr)
        delete PubTimer;
    PubThread.exit();
}
void Cloud_task::sendTextMessage(const QString &cmd, const QJsonObject &data, QString repCmdId, QString cmdId)
{
        QJsonObject rootJson;
        rootJson["cmd"] = cmd;
        rootJson["cmdId"] = cmdId;
        rootJson["repCmdId"] = repCmdId;
        rootJson["timestamp"] = GetTimeStamp();
        /*
#ifdef wlcloud_no1
        rootJson["vin"] = "CCSPT1D2022101869";
#endif
#ifdef wlcloud_no2
        rootJson["vin"] = "XXHPT1D2022101833";
#endif
#ifdef szcloud
        rootJson["vin"] = "XXHPT1D2022101830";
#endif
        */
        rootJson["vin"] = vinCode;
        rootJson["data"] = data;
        QString rootStr = QString(QJsonDocument(rootJson).toJson());
        persist_QByteArray(rootStr.toUtf8());
}

void Cloud_task::response(const QString &cmd, QString repCmdId)
{
    QJsonObject ackDataJson;
    ackDataJson["result"] = 0;
    ackDataJson["errorCode"] = 0;
    ackDataJson["errorMessage"] = "";
    sendTextMessage(cmd,ackDataJson, repCmdId,create_uuid());
}

void Cloud_task::persist_QByteArray(const QByteArray &data){
      //MainWindow::waiting_for_send(data);
//    std::lock_guard<std::mutex> lock(mtx);
       buffer_queue.push(data);



}






void Cloud_task::arriveSite()
{
    if(state.node == NodeParam::station)
    {
        struct StationInfo currentStation = current.stationInfo;
        QJsonObject dataJson;
        dataJson["stationId"] = currentStation.stationId;
        dataJson["stationName"] = currentStation.stationName;
        sendTextMessage("arriveSite",dataJson,"",create_uuid());
        qDebug()<<"arriveSite sending completed! ";
    }
}
void Cloud_task::pubArriveStation_offline(int id, QString name ,double trip)
{
    QJsonObject dataJson;
    dataJson["stationId"] = id;
    dataJson["stationName"] = name;
    dataJson["day_mileage"] = trip;
    //sendTextMessage("arriveSite",dataJson);
    qDebug()<<"arrvied station sending completed! ";
}

//void Cloud_task::pubArriveStation(Station station)
//{
//    QJsonObject dataJson;
//    dataJson["stationId"] = station.id;
//    dataJson["stationName"] = station.name;
//    //sendTextMessage("arriveSite",dataJson);
//    qDebug()<<"arrvied station sending completed! ";
//}


void Cloud_task::PubTimeTick()
{


    if(true || state.isSendCarState){
        QJsonObject dataJson;
        dataJson["vin"] = vinCode;// vin;
        dataJson["timestamp"] = GetTimeStamp();
        dataJson["fault_code"] = carState.fault_code;
        dataJson["fault_desc"] = "";
        dataJson["coor_system"] = carState.coor_system;
        dataJson["location_veh"] = QString::number(carState.carLongitude, 10, 8)+","+QString::number(carState.carLatitude, 10, 8);
        dataJson["location_stop"] = QString::number(carState.destLongitude, 10, 8)+","+QString::number(carState.destLatitude, 10, 8);
        dataJson["speed"] = carState.speed;
        dataJson["angle"] = carState.angle;
        dataJson["soc"] = carState.soc;
        dataJson["route_stop"] = carState.route_stop;
        dataJson["trip_distance"] = carState.trip_distance;
        dataJson["trip_eta"] = carState.trip_eta;
        dataJson["trip_left_mileage"] = carState.trip_left_mileage;
        dataJson["veh_status"] = carState.veh_status;
        dataJson["current_stop"] = carState.current_stop;
        dataJson["gears"] = carState.gears;
        dataJson["SysPowMode"] = carState.SysPowMode;
        dataJson["HzrdLtIO"] = carState.HzrdLtIO;
        dataJson["WindscenWipSt"] = carState.WindscenWipSt;
        dataJson["Highbeem"] = carState.Highbeem;
        dataJson["Lowbeem"] = carState.Lowbeem;
        dataJson["Windows"] = carState.Windows;
        dataJson["Doorlock"] = carState.Doorlock;
        dataJson["emergencybutton"] = carState.emergencybutton;
        dataJson["tirePressure"] = carState.tirePressure;
        dataJson["control_mode"] = carState.control_mode;
        dataJson["maxSpeed"] = carState.maxSpeed;
        dataJson["signal"] = carState.signal;
        dataJson["routeName"] = carState.routeName;
        dataJson["osVersion"] = carState.osVersion;
        dataJson["routesVersion"] = carState.routesVersion;
        dataJson["buttonA"] = carState.buttonA;
        //qDebug()<<dataJson;
        sendTextMessage("status",dataJson,"",create_uuid());
        //qDebug()<<"car state sending completed! ";

    }
    if(state.isArriveSite)
    {
        arriveSite();
        state.isArriveSite =false;
    }
}



void Cloud_task::getNextStopNode()
{
    current.destInfo.state = false;
    current.stationInfo.state = false;
    if(state.node == NodeParam::dest)
    {
        for(multimap<QString,struct DestInfo>::iterator it = destInfos.destMap.begin(); it != destInfos.destMap.end(); it++){
            if(it->second.state)
            {
                current.destInfo = it->second;
                it->second.state = false;
                break;
            }
        }
    }
    else if(state.node == NodeParam::station)
    {
        for(multimap<QString,struct StationInfo>::iterator it = stationInfos.stationMap.begin(); it != stationInfos.stationMap.end(); it++){
            if(it->second.state)
            {
                current.stationInfo = it->second;
                it->second.state = false;
                break;
            }
        }
    }
}
void Cloud_task::updateCarStateNow(CarState cs)
{

    carState = cs;
}





//暂未使用列表
void Cloud_task::PubPauseTick(){
    QJsonObject dataJson;
    QJsonObject dataJson1;
    dataJson1["cmd"] = "resume";
    dataJson["content"]=dataJson1;
    //sendTextMessage("pause",dataJson,zmqSendTopic,"",create_uuid());
    qDebug()<<"cloudtask pause start !!!!!!!!!!!"<<dataJson;
}

void Cloud_task::PubResumeTick(){
    QJsonObject dataJson;
    QJsonObject dataJson1;
    dataJson1["cmd"] = "pause";
    dataJson["content"]=dataJson1;
    //sendTextMessage("resume",dataJson,zmqSendTopic,"",create_uuid());
    qDebug()<<"cloudtask resume start !!!!!!!!!!!"<<dataJson;
}



void Cloud_task::checkOSVersion()
{
    QJsonObject dataJson;
    dataJson["osId"] = 0;
    dataJson["version"] = 0;
    //sendTextMessage("checkOSVersion",dataJson);
}

void Cloud_task::checkRoutesVersion()
{
    QJsonObject dataJson;
    dataJson["version"] = 0;
    //sendTextMessage("checkRoutesVersion",dataJson);
}

void Cloud_task::downloadRouteListFile()
{
    QJsonObject dataJson;
   //sendTextMessage("downloadRouteListFile",dataJson,"car2middle1");
}

void Cloud_task::getRoute()
{
    QJsonObject dataJson;
    dataJson["routeId"] = 0;
    dataJson["version"] = 0;
    //sendTextMessage("getRoute",dataJson);
}

void Cloud_task::getNextStationList(){
    QJsonObject dataJson;
    dataJson["routeId"] = 0;
    //sendTextMessage("getNextStation",dataJson);

}



void Cloud_task::setCloudVin(QString code)
{
    vinCode = code;
}
qint64 GetTimeStamp()
{
    QDateTime time = QDateTime::currentDateTime();
    qint64 timeT = time.toMSecsSinceEpoch();
    return timeT;
}
QString create_uuid()
{
    std::stringstream stream;
    auto random_seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 seed_engine(random_seed);
    std::uniform_int_distribution<std::size_t> random_gen ;
    std::size_t value = random_gen(seed_engine);
    stream << std::hex << value;

    return QString::fromStdString(stream.str());
}

