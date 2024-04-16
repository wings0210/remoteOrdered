#ifndef MQTT_PUBLISHER_H
#define MQTT_PUBLISHER_H

#include <memory>
#include "mqtt.h"


using namespace std;


static MQTT *mqttClient = nullptr;

void onConnectFailureClient(void *context, MQTTAsync_failureData *response);
void onConnectClient(void *context, MQTTAsync_successData *response);
void onDisconnectClient(void *context, MQTTAsync_successData *response);
void onPublishFailureClient(void *context, MQTTAsync_failureData *response);
void onPublishClient(void *context, MQTTAsync_successData *response);
void connectionLostClient(void *context, char *cause);



class MQTT_Publisher
{
public:
    MQTT_Publisher(string host_,string deviceId_,int port_ = 1883,string username_ = "",string password_ = "");
    ~MQTT_Publisher();
    void send(string topic,string message);
    bool getConnected();



private:
//    std::shared_ptr<MQTT> mqtt_Client;
};

#endif // MQTT_PUBLISHER_H
