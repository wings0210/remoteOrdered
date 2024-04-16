#ifndef MQTT_H
#define MQTT_H


#include <MQTTAsync.h>
#include <string>
#include <queue>
#include <QDebug>
#include<QDateTime>
#include <chrono>
#include <QDebug>
#include <QString>
#include <functional>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QJsonObject>
#include <QJsonArray>
#include <QTimer>
#include <QProcess>

#include <signal.h>
#include <memory.h>
#include <stdlib.h>
#include <iostream>

#include <unistd.h>
#include <openssl/hmac.h>
#include <openssl/bio.h>

using namespace std;






class MQTT {
public:

    int (*messageArrived)(void *context, char *topicName, int topicLen, MQTTAsync_message *m);
    void (*onConnectFailure)(void *context, MQTTAsync_failureData *response);
    void (*onConnectSuccess)(void *context, MQTTAsync_successData *response);
    void (*onSubcribeSuccess)(void *context, MQTTAsync_successData *response);
    void (*onDisconnect)(void *context, MQTTAsync_successData *response);
    void (*onPublishFailure)(void *context, MQTTAsync_failureData *response);
    void (*onPublishSuccess)(void *context, MQTTAsync_successData *response);
    void (*connectionLost)(void *context, char *cause);

    MQTT();
    MQTT(string host_,string deviceId_,int port_,string username_,string password_,string clientId_ );
    MQTT(string host_, string deviceId_, int port_, string clientId_);
    ~MQTT();
    void init(void);
    int connect(void);
    int disconnect(void);
    int send(std::string topic, char *msg);
    void reconnect_mqtt();


    int connected;
    bool useSSL;
    MQTTAsync client;
    string subTopic = "";
    queue<string> mqttRecvQue;

/*
    struct messageArrivedFunctor {
            MQTT* mqtt;
            int operator()(void *context, char *topicName, int topicLen, MQTTAsync_message *m) {
                // 处理消息到达事件的逻辑
                qDebug()<<"recv message from %s ,body is %s\n"<<topicName<<(char *) m->payload;

                string topic(topicName);
                string str="sys/disconnect";
                if(topic.find(str) != std::string::npos){
                        QProcess::startDetached(QApplication::applicationFilePath());
                        QApplication::quit();
                }
                char* datac = (char *) m->payload;

                if(mqtt != nullptr)
                {
                    mqtt->mqttRecvQue.push(string(datac));
                }

                MQTTAsync_freeMessage(&m);
                MQTTAsync_free(topicName);
                return 1;

            }
        };
    struct onConnectSuccessFunctor {
            MQTT* mqtt;
            void operator()(void *context, MQTTAsync_successData *response) {
                if(mqtt != nullptr)
               {
                       mqtt->connected = true;
                       qDebug()<<"mqtt connect success !";
                       MQTTAsync client = (MQTTAsync) context;
                       MQTTAsync_responseOptions sub_opts = MQTTAsync_responseOptions_initializer;
                       //sub_opts.onSuccess = mqtt->onSubcribeSuccess;

                       MQTTAsync_subscribe(client, mqtt->subTopic.c_str(), 0, &sub_opts);
                       qDebug()<<(QString::fromStdString(mqtt->subTopic));
                       QJsonObject rootJson;
                       rootJson["cmd"] = "mqttLogin";
                       rootJson["data"] = "[mqtt] success to connect and success to subscribe!";
                       mqtt->mqttRecvQue.push(QString(QJsonDocument(rootJson).toJson()).toStdString());
                       qDebug()<<"success to subscribe!";
                }

            }
        };
    struct connectionLostFunctor {
            MQTT* mqtt;
            void operator()(void *context, char *cause) {
                if(mqtt != nullptr)
                    mqtt->connected = false;
                printf("mqtt:client:connection lost\n");
                mqtt->reconnect_mqtt();


            }
        };
    struct onConnectFailureFunctor {
            void operator()(void *context, MQTTAsync_failureData *response) {
                qDebug()<<"mqtt connect fail !";
                MQTTAsync client = (MQTTAsync) context;

            }
        };
    struct onPublishSuccessFunctor {
            void operator()(void *context, MQTTAsync_successData *response) {

                printf("Publish success\n");
            }
        };
    struct onPublishFailureFunctor {
            void operator()(void *context, MQTTAsync_successData *response) {


            }
        };
    struct onSubcribeSuccessFunctor {
            void operator()(void *context, MQTTAsync_successData *response) {


            }
        };
    struct onDisconnectFunctor {
            MQTT* mqtt;
            void operator()(void *context, MQTTAsync_successData *response) {
                if(mqtt != nullptr)
                   mqtt->connected = false;

            }
        };


*/

private:

    string host = "";

    string deviceId = "";

    int port = 1883;

    string username = "";
    string password = "";
    string clientId = "";
/*
    messageArrivedFunctor  messageArrived;
    onConnectSuccessFunctor onConnectSuccess;
    connectionLostFunctor  connectionLost;
    onConnectFailureFunctor onConnectFailure;
    onPublishSuccessFunctor onPublishSuccess;
    onPublishFailureFunctor onPublishFailure;
    onSubcribeSuccessFunctor onSubcribeSuccess;
    onDisconnectFunctor onDisconnect;

*/



};


#endif // MQTT_H
