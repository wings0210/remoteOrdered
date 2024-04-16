#include "prasejson.h"

bool prase_json(QJsonDocument &doc,QString &error,struct Cloud &cloud)
{
    QJsonObject rootJsondata = doc.object();
    QStringList expected = {"content"};
    bool is_not_vaild = check_not_valid(rootJsondata,expected,error);
    if(is_not_vaild)
        return false;
    QJsonObject rootJson = rootJsondata["content"].toObject();
    expected = QStringList{"timestamp","cmd","cmdId"};
    is_not_vaild = check_not_valid(rootJson,expected,error);
    if(is_not_vaild)
        return false;
    qint64 cur_timestamp = QDateTime::currentMSecsSinceEpoch();
    qint64 rcv_timestamp = rootJson["timestamp"].toVariant().toLongLong();
    long long receive_delay = cur_timestamp - rcv_timestamp;
//    if(receive_delay>5000){
//        error += "数据接收延时超过5s，延时为" + QString::number(receive_delay) + "ms";
//        return false;
//    }
    cloud.cmdId = new QString(rootJson["cmdId"].toString());
    QJsonObject dataJson = rootJson["data"].toObject();
    QString cmd_ = rootJson["cmd"].toString();
    if(cmd_ == "stationList" )
        return prase_stationList(dataJson,error,cloud);
    else if(cmd_ == "nextStationList" )
        return prase_nextStationList(dataJson,error,cloud);
    else if(cmd_ == "param")
         return prase_param(dataJson,cloud);
    else if(cmd_ == "relocation")
         return prase_relocation(dataJson,cloud);
    else if(cmd_ == "start")
        return prase_start(cloud);
    else if(cmd_ == "stop")
        return prase_stop(cloud);
    else if(cmd_ == "pause")
        return prase_pause(cloud);
    else if(cmd_ == "resume")
       return prase_resume(cloud);
    else
    {
        error += "cmd命令未知："+rootJson["cmd"].toString();
        return false;
    }

}

bool check_not_valid(QJsonObject &jsonObj,QStringList &expected,QString &error)
{
    foreach(QString field, expected){
        if(!jsonObj.contains(field)){
            error += "接收到消息中未包含" + field;
            return true;
        }
        if(jsonObj[field].isUndefined()){
            error += "接收到消息中未包含" + field + "的键值";
            return true;
        }
    }
    return false;
}

inline bool prase_stationList(QJsonObject &dataJson ,QString &error,struct Cloud &cloud)
{
    QStringList expected = {"method","isStart","stationList"};
    bool is_not_vaild = check_not_valid(dataJson,expected,error);
    if(is_not_vaild)
        return false;
    expected = QStringList{"stationId","pt","yaw","stationName"};
    QJsonArray stlJson= dataJson["stationList"].toArray();
    if(dataJson["method"].toString() != "set" && dataJson["method"].toString() != "add")
    {
        error += "method未知："+dataJson["method"].toString();
        return false;
    }
    cloud.cmd = Cmd::stationList_;
    cloud.data.stationlist = new StationList;
    if(dataJson["isStart"].toBool() == true || dataJson["isStart"].toInt() == 1)
    cloud.data.stationlist->isstart = true;
    else cloud.data.stationlist->isstart = false;
    cloud.data.stationlist->method = dataJson["method"].toString();
    for(auto stationValue : stlJson)
    {
        struct Station station;
        QJsonObject stationJson = stationValue.toObject() ;
        station.stationId = stationJson["stationId"].toInt();
        station.stationName = stationJson["stationName"].toString();
        station.lat = stationJson["pt"].toString().split(',')[1].toDouble();
        station.lon = stationJson["pt"].toString().split(',')[0].toDouble();
        station.azi = stationJson["yaw"].toDouble();
        cloud.data.stationlist->stationlist.push_back(station);
    }
    return true;
}

inline bool prase_nextStationList(QJsonObject &dataJson,QString &error,struct Cloud &cloud)
{
    QStringList expected = {"method","stationList"};
    bool is_not_vaild = check_not_valid(dataJson,expected,error);
    if(is_not_vaild)
        return false;
    expected = QStringList{"stationId","pt","yaw","stationName"};
    QJsonArray stlJson = dataJson["stationList"].toArray();
    if(dataJson["method"].toString() != "set" && dataJson["method"].toString() != "add")
    {
        error += "method未知："+dataJson["method"].toString();
        return false;
    }
    cloud.cmd = Cmd::nextstationList_;
    cloud.data.nextstationlist = new NextStationList;
    cloud.data.nextstationlist->method = dataJson["method"].toString();
    for(auto stationValue : stlJson)
    {
        struct Station station;
        QJsonObject stationJson = stationValue.toObject() ;
        station.stationId = stationJson["stationId"].toInt();
        station.stationName = stationJson["stationName"].toString();
        station.lat = stationJson["pt"].toString().split(',')[1].toDouble();
        station.lon = stationJson["pt"].toString().split(',')[0].toDouble();
        station.azi = stationJson["yaw"].toDouble();
        cloud.data.nextstationlist->stationlist.push_back(station);
    }
    return true;
}

inline bool prase_param(QJsonObject &dataJson,struct Cloud &cloud)
{
    cloud.cmd=Cmd::param_;
    cloud.data.param.speed = dataJson["speed"].toDouble();
    cloud.data.param.HzrdLtIO = dataJson["hzrdLtIO"].toBool();
    cloud.data.param.WindscenWipSt = dataJson["windscenWipSt"].toBool();
    cloud.data.param.Highbeem = dataJson["highbeem"].toBool();
    cloud.data.param.Lowbeem = dataJson["lowbeem"].toBool();
    cloud.data.param.Windows = dataJson["Windows"].toBool();
    cloud.data.param.Doorlock = dataJson["doorlock"].toBool();
    cloud.data.param.emergencybutton = dataJson["emergencybutton"].toBool();

    return true;
}

inline bool prase_relocation(QJsonObject &dataJson,struct Cloud &cloud)
{
    cloud.cmd = Cmd::relocation_;
    cloud.data.relocation = new Relocation;
    cloud.data.relocation->stationId = dataJson["stationId"].toInt();
    cloud.data.relocation->stationName = dataJson["stationId"].toString();
    cloud.data.relocation->lon = dataJson["x"].toDouble();
    cloud.data.relocation->lat = dataJson["y"].toDouble();
    cloud.data.relocation->azi = dataJson["yaw"].toDouble();
    return true;
}

inline bool prase_start(struct Cloud &cloud){
    cloud.cmd = Cmd::start_;
    cloud.data.start = true;
    return true;
}

inline bool prase_stop(struct Cloud &cloud){
    cloud.cmd = Cmd::stop_;
    cloud.data.stop = true;
    return true;
}

inline bool prase_pause(struct Cloud &cloud){
    cloud.cmd = Cmd::pause_;
    cloud.data.pause = true;
    return true;
}

inline bool prase_resume(struct Cloud &cloud){
    cloud.cmd = Cmd::resume_;
    cloud.data.resume = true;
    return true;
}


