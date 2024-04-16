#include "mqttmanager.h"

void onConnectFailure(void *context, MQTTAsync_failureData *response);
void onConnectSuccess(void *context, MQTTAsync_successData *response);

void onDisconnect(void *context, MQTTAsync_successData *response);

void onPublishFailure(void *context, MQTTAsync_failureData *response);
void onPublishSuccess(void *context, MQTTAsync_successData *response);

int messageArrived(void *context, char *topicName, int topicLen, MQTTAsync_message *m);
void connectionLost(void *context, char *cause);


MQTTManager::MQTTManager(string host_,string deviceId_,int port_,string username_,string password_)
{
    if(mqttClient == nullptr)
    {
        mqttClient = new MQTT ();
        mqttClient->host = host_;
        mqttClient->deviceId = deviceId_;
        mqttClient->port = port_;
        mqttClient->accessKey = username_;
        mqttClient->secretKey = password_;

        mqttClient->onConnectFailure = onConnectFailure;
        mqttClient->onConnectClient = onConnectSuccess;
        mqttClient->onDisconnect = onDisconnect;
        mqttClient->onPublishFailure = onPublishFailure;
        mqttClient->onPublish = onPublishSuccess;
        mqttClient->connectionLost = connectionLost;
        mqttClient->messageArrived = messageArrived;
        mqttClient->init();
        mqttClient->connect();
    }
}
