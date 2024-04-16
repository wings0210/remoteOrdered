#include "mqtt_publisher.h"

MQTT_Publisher::MQTT_Publisher(string host_,string deviceId_,int port_,string username_,string password_)
{
    if(mqttClient == nullptr)
    {
//        mqtt_Client = std::make_shared<MQTT>(new MQTT());
        mqttClient = new MQTT ();
        mqttClient->host = host_;
        mqttClient->deviceId = deviceId_;
        mqttClient->port = port_;
        mqttClient->accessKey = username_;
        mqttClient->secretKey = password_;

        mqttClient->onConnectFailure = onConnectFailureClient;
        mqttClient->onConnectClient = onConnectClient;
        mqttClient->onDisconnect = onDisconnectClient;
        mqttClient->onPublishFailure = onPublishFailureClient;
        mqttClient->onPublish = onPublishClient;
        mqttClient->connectionLost = connectionLostClient;
        mqttClient->init();
        mqttClient->publish_connect();
    }
}

MQTT_Publisher::~MQTT_Publisher()
{
    delete mqttClient;
}

void MQTT_Publisher::send(string topic,string message)
{
    if(mqttClient != nullptr && mqttClient->connected==1)
    {
        mqttClient->topic = topic;
        mqttClient->publish_send(const_cast<char*>(message.c_str()));
    }
}

bool MQTT_Publisher::getConnected()
{
    return (mqttClient!=nullptr)&&(mqttClient->connected);
}

void onConnectFailureClient(void *context, MQTTAsync_failureData *response)
{
    if(mqttClient != nullptr)
        mqttClient->connected = 0;
    printf("mqtt:connect failed, rc %d\n", response ? response->code : -1);
    MQTTAsync client = (MQTTAsync) context;
}

void onConnectClient(void *context, MQTTAsync_successData *response)
{
    if(mqttClient != nullptr)
        mqttClient->connected = 1;
    //连接成功的回调，只会在第一次 connect 成功后调用，后续自动重连成功时并不会调用，因此应用需要自行保证每次 connect 成功后重新订阅
    printf("mqtt:client:connect success\n");
}

void onDisconnectClient(void *context, MQTTAsync_successData *response)
{
    if(mqttClient != nullptr)
        mqttClient->connected = 0;
    printf("mqtt:connect disconnect\n");
}

void onPublishFailureClient(void *context, MQTTAsync_failureData *response)
{
    printf("Publish failed, rc %d\n", response ? -1 : response->code);
}

void onPublishClient(void *context, MQTTAsync_successData *response)
{
    printf("mqtt:publish:send success\n");
}

void connectionLostClient(void *context, char *cause)
{
    if(mqttClient != nullptr)
        mqttClient->connected = 0;
    printf("mqtt:client:connection lost\n");
}
