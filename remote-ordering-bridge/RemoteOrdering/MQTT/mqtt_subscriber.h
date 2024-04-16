#ifndef MQTT_SUBSCRIBER_H
#define MQTT_SUBSCRIBER_H
#include <functional>
#include <map>
#include <queue>
#include "mqtt.h"

using namespace std;

static MQTT *mqttServer = nullptr;

static map<string,queue<string>> recvDatas;

int messageArrivedServer(void *context, char *topicName, int topicLen, MQTTAsync_message *m);
void onConnectFailureServer(void *context, MQTTAsync_failureData *response);
void onSubcribeServer(void *context, MQTTAsync_successData *response);
void onConnectServer(void *context, MQTTAsync_successData *response);
void onDisconnectServer(void *context, MQTTAsync_successData *response);
void onPublishFailureServer(void *context, MQTTAsync_failureData *response);
void connectionLostServer(void *context, char *cause);


class MQTT_Subscriber
{
public:
    MQTT_Subscriber(string host_,string deviceId_,string topic_,int port_ = 1883,string username_ = "",string password_ = "");
    ~MQTT_Subscriber();
     bool getConnected();
     map<string,queue<string>> getRecv();
};

#endif // MQTT_SUBSCRIBER_H
