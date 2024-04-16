#ifndef REGIST_H
#define REGIST_H
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
#include <iostream>
#include <fstream>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QJsonObject>
#include <QJsonArray>

using namespace std;


class Regist{

public:
    Regist();
    ~Regist();


private:
    void get_args_from_wulin();
    void get_client_args();
    QString md5Hash(const QByteArray& data);
    void switch_json(QByteArray& jsonData);
    string deviceId;
    string deviceSecret;



};








#endif
