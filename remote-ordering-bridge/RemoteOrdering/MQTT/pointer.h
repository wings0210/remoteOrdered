#ifndef POINTER_H
#define POINTER_H


#include <MQTTAsync.h>
#include <string>
#include <QApplication>
#include <QProcess>
#include <iostream>
#include<MQTT/mqtt.h>

using namespace std;



extern  MQTT *mqtt;
extern  MQTT *newMqtt;

extern int new_messageArrived(void *context, char *topicName, int topicLen, MQTTAsync_message *m);
extern void new_onDisconnect(void *context, MQTTAsync_successData *response);
extern void new_onSubcribeSuccess(void *context, MQTTAsync_successData *response);
extern void new_onPublishFailure(void *context, MQTTAsync_failureData *response);
extern void new_onPublishSuccess(void *context, MQTTAsync_successData *response);
extern void new_onConnectFailure(void *context, MQTTAsync_failureData *response);
extern void new_connectionLost(void *context, char *cause);
extern void new_onConnectSuccess(void *context, MQTTAsync_successData *response);



//泰国自有调度系统MQTT接入接口列表
extern int messageArrived(void *context, char *topicName, int topicLen, MQTTAsync_message *m);
extern void onDisconnect(void *context, MQTTAsync_successData *response);
extern void onSubcribeSuccess(void *context, MQTTAsync_successData *response);
extern void onPublishFailure(void *context, MQTTAsync_failureData *response);
extern void onPublishSuccess(void *context, MQTTAsync_successData *response);
extern void onConnectFailure(void *context, MQTTAsync_failureData *response);
extern void connectionLost(void *context, char *cause);
extern void onConnectSuccess(void *context, MQTTAsync_successData *response);

#endif
