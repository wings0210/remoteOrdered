#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    cloud=new Cloud_task();
    mythread.start();
    //日志记录连接函数

    mqttConnect();
    rosWorker = &RosWorker::getInstance();
    initConnections();
    load_conf();
    load_local_map();
    load_local_station();
}

MainWindow::~MainWindow()
{
    isMqttRecv = false;
    isMqttRecvNew = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    delete rosWorker;
    if(mqtt!= nullptr)
        delete mqtt;
    if(cloud!=nullptr)
        delete cloud;
    if(rosWorker!=nullptr)
        delete rosWorker;
    mythread.quit();
    mythread.wait();
    delete ui;
}

void MainWindow::initConnections()
{
    connect(rosWorker, &RosWorker::receivedMessage, this, &MainWindow::dispatchCmd);
    connect(rosWorker, &RosWorker::sendstation, this, &MainWindow::sendstation);
    connect(rosWorker, &RosWorker::sendlasteststation, this, &MainWindow::sendlasteststation);
    connect(rosWorker, &RosWorker::HMINextStation, this, &MainWindow::on_send_station_clicked);
    connect(rosWorker, &RosWorker::changeRoute, this, &MainWindow::on_reloadconf_clicked);
    connect(this,&MainWindow::logSignals,this,&MainWindow::logSlots);
    QTimer *loop_timer = new QTimer(this);
    connect(loop_timer, &QTimer::timeout, this ,&MainWindow::loop_func);
    loop_timer->start(100); // 每100毫秒处理一次消息回调
}

void MainWindow::loop_func()
{
    double distance = 1;
    int point_id_tmp;
    int lane_id_tmp;
    update_now();
    bool re_valid = get_point_lane_id(distance,point_id_tmp,lane_id_tmp, now.lon ,now.lat);
    if(distance > 0.00003)
    {
        route_success = false;
        //qDebug()<<"now:"<<now.lat<<"   "<<now.lon<<"   "<< distance;
    }
    else if(distance<= 0.00003&&!route_success){
        route();
    }
}
void MainWindow::dispatchCmd()
{
    std::deque<Cloud> &cloud_ =  rosWorker->cloud;
    while(!cloud_.empty())
    {
        struct Cloud cloud__;
        cloud__ = cloud_.front();

        cloud_.pop_front();
        switch(cloud__.cmd)
        {
        case Cmd::stop_ :
            rosWorker->callMessage(cloud__);
            cloud->response("ackStop", *cloud__.cmdId);
            qDebug()<<"send stop";
            break;
        case Cmd::start_ :
            rosWorker->callMessage(cloud__);
            cloud->response("ackStart", *cloud__.cmdId);
            qDebug()<<"receive start";
            break;
        case Cmd::pause_ :
            rosWorker->callMessage(cloud__);
            cloud->response("ackPause", *cloud__.cmdId);
            qDebug()<<"send pause";
            break;
        case Cmd::resume_ :
            rosWorker->callMessage(cloud__);
            cloud->response("ackResume", *cloud__.cmdId);
            qDebug()<<"send resume";
            break;
        case Cmd::param_ :
            rosWorker->callMessage(cloud__);
            cloud->response("ackParam", *cloud__.cmdId);
            qDebug()<<"send param";
            break;
        case Cmd::relocation_ :
            for(int i=0;locallist.size();i++)
            {
                if(cloud__.data.relocation->lat - locallist[i].station.lat <= 0.00001 && cloud__.data.relocation->lon - locallist[i].station.lon <= 0.00001)
                {
                    cloud__.data.relocation->azi=locallist[i].station.azi ;
                    break;
                }
            }
            rosWorker->subMessage(cloud__);
            cloud->response("ackrelocation", *cloud__.cmdId);
            delete cloud__.data.relocation;
            break;
        case Cmd::stationList_  :
            qDebug()<<"receive stationlist";
            update_stationlist(cloud__);

            cloud->response("ackStationList", *cloud__.cmdId);
            have_new_station = 2;
            bool flag_send1;
            flag_send1= get_new_station();
            delete cloud__.data.stationlist;
            if(flag_send1)
                rosWorker->callMessage(send_station);
            break;
        case Cmd::nextstationList_ :
            qDebug()<<"receive nextstationlist";
            update_nextstationlist(cloud__);
            cloud->response("ackNextStationList", *cloud__.cmdId);
            have_new_station = 1;
            bool flag_send2 ;
            flag_send2= get_new_station();
            delete cloud__.data.nextstationlist;
            if(flag_send2)
                rosWorker->callMessage(send_station);
            break;
        default :
            qDebug()<<"cmd unkonw :"<<cloud__.cmd;
            break;
        }
    }
    rosWorker->sendSignal();
}

void MainWindow::update_stationlist(struct Cloud &cloud__)
{
    if(cloud__.data.stationlist->stationlist.empty())
    {
        qDebug()<<"stationlist 为空";
        return ;
    }
    is_start = cloud__.data.stationlist->isstart;
    if(cloud__.data.stationlist->method == "set"){
        stationlist.clear();
    }
    struct StationProperties station_tmp;
    int size_tmp__size = cloud__.data.stationlist->stationlist.size();
    for(int i =0; i< size_tmp__size; i++)
    {
        station_tmp.station = cloud__.data.stationlist->stationlist[i];
        double destance =1;
        get_point_lane_id(destance,station_tmp.pointid,station_tmp.laneid,station_tmp.station.lon,station_tmp.station.lat);
        for(int i=0;locallist.size();i++)
        {
            if(station_tmp.station.stationId == locallist[i].station.stationId)
            {
                station_tmp=locallist[i];
                break;
            }
        }
        stationlist.push_back(station_tmp);
        qDebug()<<"stationlist station:"<<station_tmp.station.stationName;
    }
}

void MainWindow::update_nextstationlist(struct Cloud &cloud__)
{
    if(cloud__.data.nextstationlist->stationlist.empty())
    {
        qDebug()<<"nextstationlist 为空";
        return ;
    }
    if(cloud__.data.nextstationlist->method == "set"){
        nextstationlist.clear();
    }
    struct StationProperties station_tmp;
    int size_tmp_nextstationlist = cloud__.data.nextstationlist->stationlist.size();
    for(int i =0; i< size_tmp_nextstationlist; i++)
    {
        station_tmp.station = cloud__.data.nextstationlist->stationlist[i];
        double destance=1;
        get_point_lane_id(destance,station_tmp.pointid,station_tmp.laneid,station_tmp.station.lon,station_tmp.station.lat);
        for(int i=0;locallist.size();i++)
        {
            if(station_tmp.station.stationId == locallist[i].station.stationId)
            {
                station_tmp=locallist[i];
                break;
            }
        }
        nextstationlist.push_back(station_tmp);
        qDebug()<<"nextstationlist station:"<<station_tmp.station.stationName;
    }
}

void MainWindow::load_conf()
{
    QString fileName = "../../../conf/remoteorder-conf/conf.json";
    if (!QFile::exists(fileName)) {
        qDebug() << "File" << fileName << "does not exist";
        return ;
    }
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Failed to open file";
        return ;
    }
    QByteArray jsonData = file.readAll();
    file.close();
    QJsonParseError jsonError;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData, &jsonError);
    if (jsonError.error != QJsonParseError::NoError)
    {
        qWarning() << "解析 JSON 数据错误: " << jsonError.errorString();
        return ;
    }
    QJsonObject jsonObj = jsonDoc.object();
    Map_Path = jsonObj["map"].toString();
    Local_Station_Path = jsonObj["station"].toString();
    qDebug()<<"Map_Path:"<<Map_Path;
    qDebug()<<"Local_Station_Path:"<<Local_Station_Path;
}

void MainWindow::load_local_map()
{
    QString fileName = Map_Path;
    QString topofile = Map_Path.replace(fileName.length()-4, 4, "-type.add");
    if (!QFile::exists(fileName) || !QFile::exists(topofile)) {
        qDebug() << "File" << fileName << "does not exist";
        return ;
    }
    QFile filep(fileName);
    QFile filet(topofile);
    if (!filep.open(QIODevice::ReadOnly | QIODevice::Text) || !filet.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Failed to open file";
        return ;
    }
    std::vector<int>tmp_line;
    topo.clear();
    while (!filet.atEnd())
    {
        QByteArray line = filet.readLine().trimmed();
        QList<QByteArray> args = line.split('\t');
        LaneProperties lane_property;
        int ID = args[1].toInt();
        tmp_line.push_back(ID);
        QList<QByteArray> nextlane = args[2].split(',');
        for(QByteArray str : nextlane)
        {
            lane_property.next_lane.push_back(str.toInt());
        }
        topo[ID] = lane_property;
    }
    std::vector<Point> line_point_tmp;
    line_point_tmp.clear();
    int i = 0;
    struct Point tmp_p;
    double max_lon_tmp=0;
    double max_lat_tmp=0;
    double min_lon_tmp=361;
    double min_lat_tmp=361;
    map_.clear();
    while (!filep.atEnd())
    {

        QByteArray line = filep.readLine().trimmed();
        QList<QByteArray> args = line.split('\t');
        if(args.size()<3)
            continue;
        int ID = args[0].toInt();
        tmp_p.lon = args[1].toDouble();
        tmp_p.lat = args[2].toDouble();
        if(tmp_p.lon > max_lon_tmp)
            max_lon_tmp = tmp_p.lon;
        if(tmp_p.lat > max_lat_tmp)
            max_lat_tmp = tmp_p.lat;
        if(tmp_p.lon < min_lon_tmp)
            min_lon_tmp = tmp_p.lon;
        if(tmp_p.lat < min_lat_tmp)
            min_lat_tmp = tmp_p.lat;
        if(ID == 0 && line_point_tmp.size())
        {
            map_[tmp_line[i]] = line_point_tmp;
            topo[tmp_line[i]].sum_point = line_point_tmp.size();
            topo[tmp_line[i]].max_lat = max_lat_tmp;
            topo[tmp_line[i]].max_lon = max_lon_tmp;
            topo[tmp_line[i]].min_lat = min_lat_tmp;
            topo[tmp_line[i]].min_lon = min_lon_tmp;
            line_point_tmp.clear();
            i++;
        }
        line_point_tmp.push_back(tmp_p);
    }
    map_[tmp_line[i]] = line_point_tmp;
}

void MainWindow::load_local_station()
{
    QString fileName = Local_Station_Path;
    if (!QFile::exists(fileName)) {
        qDebug() << "File" << fileName << "does not exist";
        return ;
    }
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Failed to open file";
        return ;
    }
    locallist.clear();
    struct StationProperties station_tmp;
    while(!file.atEnd())
    {
        QByteArray line = file.readLine().trimmed();
        QList<QByteArray> args = line.split('\t');
        station_tmp.station.stationId = args[0].toInt();
        station_tmp.station.stationName = args[1];
        station_tmp.station.lon = args[2].toDouble();
        station_tmp.station.lat = args[3].toDouble();
        station_tmp.station.azi = args[4].toDouble();
        double distance = 1;
        int point_id_tmp;
        int lane_id_tmp;
        bool re_valid = get_point_lane_id(distance,point_id_tmp,lane_id_tmp, station_tmp.station.lon ,station_tmp.station.lat);
        if(!re_valid)
        {
            qDebug()<<" error 3";
            return;
        }
        if(distance < 0.00001)
        {
            station_tmp.laneid = lane_id_tmp;
            station_tmp.pointid = point_id_tmp;
        }
        else
        {
            qDebug()<<" error to read :"<<fileName;
            return;
        }
        locallist.push_back(station_tmp);
    }
    qDebug()<<"load local station:";
    for(int i=0;i<locallist.size();i++)
    {
        QString local_load_str="ID:"+QString::number(locallist[i].station.stationId)+"\tname:"+locallist[i].station.stationName+\
                                "\tlon:"+QString::number(locallist[i].station.lon)+"\tlat:"   \
                                +QString::number(locallist[i].station.lat)+"\tazi:"+QString::number(locallist[i].station.azi);
        qDebug()<<local_load_str;
        QByteArray bytes1=local_load_str.toUtf8();

        Tools::writeJsonDataToFile(bytes1,"../records/load_local_stationlist"+QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
    }
}

bool MainWindow::get_point_lane_id(double &distance, int &point_id_tmp, int &lane_id_tmp, double &lon, double &lat)
{
    bool returnflag = false;
    for(auto it = topo.begin();it != topo.end(); it++)
    {
            int size_tmp_Map = map_[it->first].size();
            for(int i = 0; i < size_tmp_Map; i++) {
                    if(distance > qAbs(lon - map_[it->first][i].lon) + qAbs(lat - map_[it->first][i].lat))
                    {
                        distance = qAbs(lon - map_[it->first][i].lon) + qAbs(lat - map_[it->first][i].lat);
                        point_id_tmp = i;
                        lane_id_tmp = it->first;
                        returnflag = true;
                    }
            }

    }
    if(returnflag)
    {
        //qDebug()<<"distance="<<distance<<"point_id_tmp="<<point_id_tmp<<"lane_id_tmp="<<lane_id_tmp;
    }
    return returnflag;
}

bool MainWindow::get_new_station()
{
    if(stationlist.size() > 0)
    {
        if(have_new_station == 2 && is_start)
        {
            send_station = stationlist.front().station;
            rosWorker->first_start =false;
            have_new_station = 0;
            is_start = false;
            stationlist.pop_front();
            qDebug()<<"isstart,get station from stationlist :"<<send_station.stationName;
            return true;
        }
        if(rosWorker->instation == 1 && rosWorker->go_next_station)
        {
            send_station = stationlist.front().station;
            rosWorker->first_start =false;
            rosWorker->instation = 0;
            rosWorker->go_next_station = false;
            stationlist.pop_front();
            qDebug()<<"instation,get station from stationlist :"<<send_station.stationName;
            return true;
        }
    }
    else
    {
        if(nextstationlist.size() > 0)
        {
            if(have_new_station == 1 && rosWorker->go_next_station)
            {
                bool flag_station;
                flag_station= get_nearest_station();
                if(!flag_station) return false;
                have_new_station = 0;
                rosWorker->go_next_station = false;
                qDebug()<<"first,get station from nextstationlist :"<<send_station.stationName;
                return true;
            }
            if(rosWorker->instation == 1 && rosWorker->go_next_station)
            {
                bool flag_station;
                flag_station= get_nearest_station();
                if(!flag_station) return false;
                rosWorker->instation = 0;
                rosWorker->go_next_station = false;
                qDebug()<<"instation,get station from stationlist :"<<send_station.stationName;
                return true;
            }
        }
        else
        {
            if(rosWorker->go_next_station && rosWorker->first_start)
            {
                bool flag_station;
                flag_station= get_nearest_station();
                if(!flag_station) return false;
                rosWorker->go_next_station = false;
                qDebug()<<"first,get station from locallist :"<<send_station.stationName;
                QString str1="first,get station from locallist :"+send_station.stationName;
                QByteArray bytes2=str1.toUtf8();
                Tools::writeJsonDataToFile(bytes2,"../records/first_get_station_from_locallist_"+QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
                return true;
            }
            if(rosWorker->instation == 1 && rosWorker->go_next_station)
            {
                bool flag_station;
                flag_station= get_nearest_station();
                if(!flag_station) return false;
                rosWorker->instation = 0;
                rosWorker->go_next_station = false;
                qDebug()<<"instation,get station from locallist :"<<send_station.stationName;
                QString str2="instation,get station from locallist :"+send_station.stationName;
                QByteArray bytes3=str2.toUtf8();
                Tools::writeJsonDataToFile(bytes3,"../records/instation_get_station_from_locallist"+QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
                return true;
            }
        }
    }
    return false;
}

bool MainWindow::get_nearest_station()
{
    double distance = 1;
    int point_id_tmp;
    int lane_id_tmp;
    update_now();
    bool re_valid = get_point_lane_id(distance,point_id_tmp,lane_id_tmp, now.lon ,now.lat);
    if(!re_valid)
    {
        qDebug()<<" error 2";
        return false;
    }
    //int index_lane_tmp = 0;
    int offset_a = 0;
    bool find_success_tmp = false;
    int size_tmp_plan_route = plan_route.size();
    for (;offset_a < size_tmp_plan_route; offset_a++)
    {
        if(plan_route[offset_a] == lane_id_tmp) {
            find_success_tmp = true;
            break;
        }
    }
    if(!find_success_tmp)
    {
        qDebug()<<"查找失败，路线上无该站点";
        return false;
    }
    struct StationProperties *it;
    struct StationProperties *it2;
    int length_size_list = 0;
    if(mode == Station_Mode :: nextstation)
    {
        it = &nextstationlist[0];
        it2 = it;
        length_size_list = nextstationlist.size();
    }
    else if(mode == Station_Mode :: local)
    {
        it = &locallist[0];
        it2 = it;
        length_size_list = locallist.size();
    }
    int offset_b = 100000000;
    int offset_c = 100000000;
    int index = 0;
    int length_size_route = size_tmp_plan_route;
    for(int i = 0; i < length_size_list; i++ , it++)
    {
        for( int j = 0; j < length_size_route; j++)
        {
            int test_offset_a = (offset_a + j) % size_tmp_plan_route;
            if (plan_route[test_offset_a] == it->laneid)
            {
                if (j == 0 && it->pointid - point_id_tmp < 10){
                    if (offset_b > size_tmp_plan_route)
                    {
                        offset_b = size_tmp_plan_route;
                        offset_c = it->pointid;
                        index = i;
                    }
                    else if (offset_b ==  size_tmp_plan_route && offset_c > it->pointid)
                    {
                        offset_c = it->pointid;
                        index = i;
                    }
                    break;
                }
                else {
                    if(j < offset_b )
                    {
                        offset_b = j;
                        offset_c = it->pointid;
                        index = i;
                    }
                     else if(j==offset_b && it->pointid < offset_c)
                    {
                        offset_b = 0;
                        offset_c = it->pointid;
                        index = i;
                    }
                    break;
                }
            }
        }
        length_size_route = length_size_route > offset_b ? offset_b : length_size_route;
    }
    if(offset_b <= size_tmp_plan_route)
    {
        send_station = (it2 + index)->station;
        rosWorker->first_start =false;
        return true;

    }
    else {
        qDebug()<<"找不到该站点";
        return false;
    }
    return false;
}

void MainWindow::route()
{
    double distance = 1;
    int point_id_tmp;
    int lane_id_tmp;
    update_now();
    bool re_valid = get_point_lane_id(distance,point_id_tmp,lane_id_tmp, now.lon ,now.lat);
    if(!re_valid)
    {
        qDebug()<<" error 1";
        return;
    }
    plan_route.clear();
    QString plan_str =QString::number(lane_id_tmp);
    plan_route.push_back(lane_id_tmp);
    for(int i= topo[lane_id_tmp].next_lane[0] ; i!=lane_id_tmp ; i=topo[i].next_lane[0])
    {
        plan_route.push_back(i);
        plan_str+="\t"+QString::number(i);
    }
    route_success = true;
}
void MainWindow::update_now()
{
    now.lat = Cloud_task::carState.carLatitude;
    //qDebug()<<Cloud_task::carState.carLatitude;
    now.lon = Cloud_task::carState.carLongitude;
}
void MainWindow::sendstation()
{
    bool flag_send;
    flag_send= get_new_station();
    if(flag_send)
    {
     rosWorker->callMessage(send_station);
     rosWorker->go_next_station =false;
     rosWorker->instation = false;
    }
}
void MainWindow::sendlasteststation()
{
    if(rosWorker->first_start)
    {
        bool flag_send;
        flag_send= get_new_station();
        if(!flag_send) return ;
    }
    rosWorker->callMessage(send_station);
    rosWorker->go_next_station =false;
}
//连接mqtt，并向车端发送信息
void MainWindow::mqttConnect()
{
    // 将文件内容解析为YAML文档
    YAML::Node config = YAML::LoadFile("../../../conf/remoteorder-conf/conf.yaml");

    // 读取相应参数的值
    string DeviceId = config["DeviceId"].as<string>();
    string ClientId = config["ClientId"].as<string>();
    string Username = config["Username"].as<string>();
    string Password = config["Password"].as<string>();
    string SubTopic = config["SubTopic"].as<string>();
    string PubTopic = config["PubTopic"].as<string>();

    ui->lineEdit_mqttHost->setText(mqttHost);
    ui->lineEdit_mqttDeviceId->setText(QString::fromStdString(DeviceId));
    ui->lineEdit_mqttUsername->setText(QString::fromStdString(Username));
    ui->lineEdit_mqttPassword->setText(QString::fromStdString(Password));
    ui->lineEdit_mqttSubTopic->setText(QString::fromStdString(SubTopic));
    ui->lineEdit_mqttPubTopic->setText(QString::fromStdString(PubTopic));

    //mqtt = new MQTT(mqttHost.toStdString(),"XXHPT1D2022101830",1883,Username,Password,ClientId);  //测试
    mqtt = new MQTT(mqttHost.toStdString(),DeviceId,1883,Username,Password,ClientId);

    string ThaHost     = config["ThaHost"].as<string>();
    string ThaDeviceId = config["ThaDeviceId"].as<string>();
    string ThaClientId = config["ThaClientId"].as<string>();
    string ThaSubTopic = config["ThaSubTopic"].as<string>();

    newMqtt =new MQTT(ThaHost,ThaDeviceId,1883,ThaClientId);

    //指针重定向函数列表
    mqtt->onConnectSuccess = new_onConnectSuccess;
    mqtt->onDisconnect = new_onDisconnect;
    mqtt->onPublishFailure = new_onPublishFailure;
    mqtt->onPublishSuccess = new_onPublishSuccess;
    mqtt->connectionLost = new_connectionLost;
    mqtt->messageArrived = new_messageArrived;
    mqtt->onSubcribeSuccess = new_onSubcribeSuccess;

    newMqtt->onConnectSuccess = onConnectSuccess;
    newMqtt->onDisconnect = onDisconnect;
    newMqtt->onPublishFailure = onPublishFailure;
    newMqtt->onPublishSuccess = onPublishSuccess;
    newMqtt->connectionLost = connectionLost;
    newMqtt->messageArrived = messageArrived;
    newMqtt->onSubcribeSuccess = onSubcribeSuccess;




    mqtt->subTopic = SubTopic;
    mqtt->init();
    mqtt->connect();

    newMqtt->subTopic = ThaSubTopic;
    newMqtt->init();
    newMqtt->connect();

    //单开一个线程，用于向车端发送mqtt接受的信息
    isMqttRecv = true;
    const char *ba1="}";
    QByteArray jsondata("");
    std::thread Send_Car_Thread([=](){
        while (isMqttRecv) {
            if(mqtt->mqttRecvQue.size() > 0)
            {

                std::string data = mqtt->mqttRecvQue.front();
                //清洗要被写入文件的json，并写入
                const char* data_ = data.data();
                char* datac;
                datac = const_cast<char*>(data_);
                while(!tool.checkParenthesis(datac)){
                    datac = tool.chopLastChar(datac);

                }
                int len = std::strlen(datac);
                while(datac[len-1] != ba1[0]){
                    len--;
                }
                QByteArray jsondata(datac, len);
                QString new_data=jsondata;
                //std::string new_data(jsondata.constData(),jsondata.length());

                //待修改
                //zmqPublisher->sendData(QString::fromStdString(zmqPubTopic),QString::fromStdString(new_data).toUtf8());
                if(jsondata != "")
                    rosWorker->subscriberCallback(jsondata);
                mqtt->mqttRecvQue.pop();

                //在组件上打印log信息
                QString log = "Sending car's data:"+new_data;
                emit logSignals(log);

                /*** Add cloud platform logging function--no1 start ***/
                tool.writeJsonDataToFile(jsondata, arvfilePath);
                /*** Add cloud platform logging function--no1 end ***/

            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    });
    Send_Car_Thread.detach();

    isMqttRecvNew = true;
    const char *ba2="}";
    QByteArray jsondataNew("");
    std::thread Send_Car_Thread_New([=](){
        while (isMqttRecvNew) {
            if(newMqtt->mqttRecvQue.size() > 0)
            {

                std::string data = newMqtt->mqttRecvQue.front();
                //清洗要被写入文件的json，并写入
                const char* data_ = data.data();
                char* datac;
                datac = const_cast<char*>(data_);
                while(!tool.checkParenthesis(datac)){
                    datac = tool.chopLastChar(datac);

                }
                int len = std::strlen(datac);
                while(datac[len-1] != ba2[0]){
                    len--;
                }
                QByteArray jsondata(datac, len);
                QString new_data=jsondata;
                //std::string new_data(jsondata.constData(),jsondata.length());

                //待修改
                //zmqPublisher->sendData(QString::fromStdString(zmqPubTopic),QString::fromStdString(new_data).toUtf8());
                if(jsondata != "")
                    rosWorker->subscriberCallback(jsondata);
                newMqtt->mqttRecvQue.pop();

                //在组件上打印log信息
                QString log = "Sending car's data:"+new_data;
                emit logSignals(log);

                /*** Add cloud platform logging function--no1 start ***/
                tool.writeJsonDataToFile(jsondata, arvfilePath);
                /*** Add cloud platform logging function--no1 end ***/

            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    });
    Send_Car_Thread_New.detach();
}

//组件打印输出log函数
void MainWindow::logSlots(const QString &message)
/*** Keep the editor on the last line of the cursor ***/
{
    QTextCursor cursor = ui->textEdit_log->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->textEdit_log->setTextCursor(cursor);
    ui->textEdit_log->insertPlainText(message);
    QDateTime currenttime=QDateTime::currentDateTime();
    qDebug() << "log: " <<currenttime.date()<< message;

}

void MainWindow::on_reloadconf_clicked()
{
    load_conf();
    load_local_map();
    load_local_station();
}

void MainWindow::on_send_relocation_clicked()
{
    test_cloud.cmd = Cmd::relocation_;
    test_cloud.cmdId = new QString("");
    test_cloud.data.relocation = new Station;
    test_cloud.data.relocation->azi = ui->write_azi->text().toDouble();
    test_cloud.data.relocation->lat = ui->write_lat->text().toDouble();
    test_cloud.data.relocation->lon = ui->write_lon->text().toDouble();
    rosWorker->subMessage(test_cloud);
    delete test_cloud.cmdId;
    delete test_cloud.data.relocation;
}

void MainWindow::on_send_station_clicked()
{
    get_nearest_station();
    rosWorker->callMessage(send_station);
}
