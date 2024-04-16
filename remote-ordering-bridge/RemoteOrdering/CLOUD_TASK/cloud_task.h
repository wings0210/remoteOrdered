#ifndef CLOUD_TASK_H
#define CLOUD_TASK_H


#include <QObject>
#include <QString>
#include <QTimer>
#include <QThread>
#include <map>
#include <queue>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <QDateTime>
#include <QJsonValue>
#include <iostream>
#include <QtCore>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <functional>
#include <random>
#include <stack>
#include <mutex>

extern std::mutex mtx;
qint64 GetTimeStamp();
QString create_uuid();

using namespace std;


enum TState{default_t,start,pause_t,resume,stop};
enum NodeParam{default_n,dest,station};
struct CarState
{
    QString fault_code = "";
    QString fault_desc = "";
    int coor_system = 0; //坐标系
    double carLatitude = 30;
    double carLongitude = 120;
    double destLatitude = 0;
    double destLongitude = 0;
    double  speed = 0;
    double angle = 0;
    double soc = 0;
    QString route_stop = "";
    double trip_distance = 0;
    int trip_eta = -1;
    double trip_left_mileage = 0;
    int veh_status = 6;
    QString current_stop = "";
    QString gears = "";
    QString SysPowMode = "";
    int HzrdLtIO = 0;
    int WindscenWipSt = 0;
    int Highbeem = 0;
    int Lowbeem = 0;
    int Windows = 0;
    int Doorlock = 0;
    int emergencybutton = 0;
    double tirePressure = 0;
    int control_mode = 0;
    double maxSpeed = 0;
    int signal = 0;
    QString routeName = "";
    int osVersion = 0;
    int routesVersion = 0;
    int buttonA = 0;

//    bool state = false;
};
struct StationInfo
{
    int stationId = -1;
    QString stationName = "";
    double longitude = 0;
    double latitude = 0;
    double yaw = 0;
    int type = -1;
    int* hookTypes = nullptr;
    bool state = false; //if the station not passed, state = true
};
struct StationInfos
{
    QString CmdId = "";
    qint64 CmdTimestamp = -1;
    int coor_system = -1;
    //    int isStart = -1;
    int routeId = -1;
    QString routeName = "";
    multimap<QString,struct StationInfo> stationMap;
};
struct DestInfo
{
    QString ptName = "";
    double longitude = 0;
    double latitude = 0;
    double yaw = 0;
    int type = -1;
    int speed = -1;
    int stop = -1;
    bool isStation = false;
    StationInfo stationInfo;
    bool state = false; //if the dest not passed, state = true
};
struct DestInfos
{
    int coor_system = -1;
    //    int isStart = -1;
    multimap<QString,struct DestInfo> destMap;
};
struct RelocationInfo
{
    int stationId = -1;
    QString stationName = "";
    double longitude = 0;
    double latitude = 0;
    double yaw = 0;
    int orderByRouteId = -1;
    int locationMathytpe = 0;
    QString call_sationId = "";
};

struct Current
{
    struct DestInfo destInfo;
    struct StationInfo stationInfo;
};
struct RouteInfo
{
    QString routeName = "";
    int routeId = -1;
    QString routeVersion = "";
    bool state = false;
};
struct State
{
//    bool isRecvSite = false;
//    NodeParam node = NodeParam::default_n;
//    bool isRePlan = false;
//    bool isStart = false; //
//    bool isArriveSite = false;
//    bool isGoNextNode = false;
//    bool isTaskState = false;
//    bool isSetParam = false;
//    bool isModeSwitch = false;

//    bool isSendCarState = false;

        bool isRecvSite ;
        NodeParam node ;
        bool isRePlan ;
        bool isStart ;
        bool isArriveSite ;
        bool isGoNextNode ;
        bool isTaskState ;
        bool isSetParam ;
        bool isModeSwitch;

        bool isSendCarState;

};
struct TaskState
{
    QString CmdId = "";
    qint64 CmdTimestamp = -1;
    enum TState tState = TState::default_t;
};

struct SetParam
{
    double speed = 0;
    int HzrdLtIO = -1;
    int WindscenWipSt = -1;
    int Highbeem = -1;
    int Lowbeem = -1;
    int Windows = -1;
    int Doorlock = -1;
    int emergencybutton = -1;
};

struct ModeSwitch_t
{
    int control_mode = -1;
};

struct OSVersionInfo
{
    int osId = -1;
    int version = -1;
    QString versionName = "";
    QString downloadUrl = "";
    bool forceUpdate = false;
    bool state = false;
};

struct RoutesVersionInfo
{
    int version = -1;
    QString versionName = "";
    QString downloadUrl = "";
    bool forceUpdate = false;
    bool state = false;
};

struct AloneRouteUrlInfo
{
    int routeId = -1;
    QString routeCode = "";
    int version = -1;
    QString versionName = "";
    QString downloadUrl = "";
    bool state = false;
};

class Cloud_task:public QObject
{
    Q_OBJECT

public:
    Cloud_task();
    ~Cloud_task();
    void sendTextMessage(const QString &cmd, const QJsonObject &data,QString repCmdId, QString cmdId);
    void response(const QString &cmd, QString repCmdId);
    void persist_QByteArray(const QByteArray &data);

    void arriveSite();
    void getNextStopNode();
    //void pubArriveStation(struct Station station);
    void pubArriveStation_offline(int id, QString name ,double trip);
    void checkOSVersion();
    void checkRoutesVersion();
    void downloadRouteListFile();
    void getRoute();

    void PubPauseTick();
    void PubResumeTick();
    void getNextStationList();
    void updateCarStateNow(CarState cs);

    void setCloudVin(QString code);

    static State state;
    struct DestInfos destInfos;
    struct StationInfos stationInfos;
    struct StationInfos nexstationlist;
    struct Current current;
    struct RouteInfo routeInfo;
    struct TaskState taskState;
    struct SetParam setParam;
    struct ModeSwitch_t modeSwitch;
    static struct CarState carState;
    struct OSVersionInfo osVersionInfo;
    struct RoutesVersionInfo routesVersionInfo;
    struct AloneRouteUrlInfo aloneRouteUrlInfo;
    struct RelocationInfo relocInfo;

public:
    static std::queue<QByteArray> buffer_queue;

private:
    QString vinCode = "";
    QTimer *PubTimer;
    QThread PubThread;

private slots:
    void PubTimeTick();

};

#endif
