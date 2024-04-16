#ifndef MQTTMANAGER_H
#define MQTTMANAGER_H
#include "mqtt.h"

//void onConnectFailure(void *context, MQTTAsync_failureData *response);
//void onConnectSuccess(void *context, MQTTAsync_successData *response);
//void onDisconnectClient(void *context, MQTTAsync_successData *response);
//void onFailure(void *context, MQTTAsync_failureData *response);
////void onClient(void *context, MQTTAsync_successData *response);
//void connectionLost(void *context, char *cause);

void onConnectFailure(void *context, MQTTAsync_failureData *response);
void onConnectSuccess(void *context, MQTTAsync_successData *response);

void onDisconnect(void *context, MQTTAsync_successData *response);

void onPublishFailure(void *context, MQTTAsync_failureData *response);
void onPublishSuccess(void *context, MQTTAsync_successData *response);

int messageArrived(void *context, char *topicName, int topicLen, MQTTAsync_message *m);
void connectionLost(void *context, char *cause);

class MQTTManager
{
public:
    MQTTManager(string host_,string deviceId_,int port_ = 1883,string username_ = "",string password_ = "");
    ~MQTTManager();
    void connectServer();
    void disconnectServer();


private:
    MQTT *mqttClient = nullptr;

};

#endif // MQTTMANAGER_H
