#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QApplication>
#include <QMainWindow>
#include <memory>
#include <QThread>
#include <QQueue>
#include "MQTT/mqtt.h"
#include "CLOUD_TASK/cloud_task.h"
#include "CLOUD_TASK/tools.h"
#include <thread>
#include <stack>
#include <cstring>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QFileInfo>
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QUrlQuery>
#include <QByteArray>
#include <QCryptographicHash>
#include <QCoreApplication>
#include <QNetworkConfigurationManager>
#include <yaml-cpp/yaml.h>
#include <qdebug.h>
#include<MQTT/pointer.h>

#include <map>
#include "ui_mainwindow.h"
#include <vector>
#include <deque>
#include "rosworker.h"


using namespace std;

namespace Ui {
class MainWindow;
}
//static MQTT* mqtt=nullptr;

class Mythread:public QThread
{
public:
    //对从车端接收到的信息，进行处理
    void run(){
        QDateTime currentDateTime = QDateTime::currentDateTime();
        QString sndfileName = currentDateTime.toString("yyyy-MM-dd hh:mm:ss") + "_msgSnd" + ".txt";
        QString sndfilePath = "../records/" + sndfileName;
        while(1){
            //std::lock_guard<std::mutex> lock(mtx);
            if(mqtt != nullptr&&Cloud_task::buffer_queue.empty()!=1){

                //qDebug()<<Cloud_task::buffer_queue.size();

             const QByteArray data= Cloud_task::buffer_queue.front();
             Tools::writeJsonDataToFile(data, sndfilePath);

             YAML::Node config = YAML::LoadFile("../../../conf/remoteorder-conf/conf.yaml");

             // 读取相应参数的值
             string PubTopic = config["PubTopic"].as<string>();
             string ThaPubTopic = config["ThaPubTopic"].as<string>();

              mqtt->send(PubTopic,const_cast<char*>(data.data()));
              newMqtt->send(ThaPubTopic,const_cast<char*>(data.data()));
            //emit logSignals(QString(data));

              Cloud_task::buffer_queue.pop();
            }
              msleep(1000);
         }
      }

};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    Mythread mythread;
    Cloud_task* cloud;
    /*** Add cloud platform logging function--no3 start ***/
    QDateTime currentDateTime = QDateTime::currentDateTime();
    QString arvfileName = currentDateTime.toString("yyyy-MM-dd hh:mm:ss") + "_msgArv" + ".txt";
    QString arvfilePath = "../records/" + arvfileName;
    /*** Add cloud platform logging function--no3 start ***/

private slots:
    void logSlots(const QString &message);
    void dispatchCmd();
    void sendstation();
    void sendlasteststation();
    void loop_func();
    void on_reloadconf_clicked();

    void on_send_relocation_clicked();

    void on_send_station_clicked();

signals:
    void logSignals(const QString &message);

private:
    Ui::MainWindow *ui;
    Tools tool;
    bool isMqttRecv = false;
    bool isMqttRecvNew = false;
    QString mqttHost = "36efeb1442.iotda-device.cn-south-4.myhuaweicloud.com";
    RosWorker *rosWorker;
    std::map<QString, QWidget*> uiWidgets;
    Station send_station;
    std::deque <StationProperties> stationlist;
    std::deque <StationProperties> locallist;
    std::vector <StationProperties> nextstationlist;
    QString Map_Path;
    QString Local_Station_Path;
    std::map <int ,LaneProperties> topo;
    std::map <int,std::vector<Point>> map_;
    std::deque <int> plan_route;
    int have_new_station = 0;
    bool is_start = false;
     Cloud test_cloud;
    bool route_success = false;
    Station_Mode mode = Station_Mode::local;
    Point now;
    void update_now();
    void load_conf();
    void load_local_map();
    void load_local_station();
    void mqttConnect();
    void initConnections();
    void update_stationlist(struct Cloud &cloud);
    void update_nextstationlist(struct Cloud &cloud);
    bool get_point_lane_id(double &distance, int &point_id_tmp, int &lane_id_tmp, double &lon, double &lat);
    bool get_new_station();
    bool get_nearest_station();
    void route();

};





#endif // MAINWINDOW_H






