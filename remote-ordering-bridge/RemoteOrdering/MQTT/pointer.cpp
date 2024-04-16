#include<MQTT/pointer.h>

MQTT* mqtt = nullptr;
MQTT* newMqtt = nullptr;
/***relocated functions,do not modify readily***/

//重定位函数列表
//Mqtt订阅回调函数
int new_messageArrived(void *context, char *topicName, int topicLen, MQTTAsync_message *m){
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


void new_onDisconnect(void *context, MQTTAsync_successData *response){
    if(mqtt != nullptr)
        mqtt->connected = false;
}

void new_onSubcribeSuccess(void *context, MQTTAsync_successData *response)
{
   qDebug()<<"success to subscribe,the topic's name is:";
   qDebug()<<(QString::fromStdString(mqtt->subTopic));
}

void new_onPublishFailure(void *context, MQTTAsync_failureData *response){
    printf("Publish failed, rc %d\n", response ? -1 : response->code);
}
void new_onPublishSuccess(void *context, MQTTAsync_successData *response){

}


void new_onConnectFailure(void *context, MQTTAsync_failureData *response)
{
    qDebug()<<"mqtt connect fail !";
    MQTTAsync client = (MQTTAsync) context;

}
void new_connectionLost(void *context, char *cause){
    if(mqtt != nullptr)
        mqtt->connected = false;
    qDebug()<<"mqtt:client:connection lost\n";
    mqtt->reconnect_mqtt();


}

void new_onConnectSuccess(void *context, MQTTAsync_successData *response)
{
    if(mqtt != nullptr)
    {
        mqtt->connected = true;
        qDebug()<<"mqtt connect success !";
        MQTTAsync client = (MQTTAsync) context;
        MQTTAsync_responseOptions sub_opts = MQTTAsync_responseOptions_initializer;
        sub_opts.onSuccess = mqtt->onSubcribeSuccess;

        MQTTAsync_subscribe(client, mqtt->subTopic.c_str(), 0, &sub_opts);

        QJsonObject rootJson;
        rootJson["cmd"] = "mqttLogin";
        rootJson["data"] = "[mqtt] success to connect and success to subscribe!";
        mqtt->mqttRecvQue.push(QString(QJsonDocument(rootJson).toJson()).toStdString());

    }
}


/***selfDispatchSystem, do not modify readily***/


int messageArrived(void *context, char *topicName, int topicLen, MQTTAsync_message *m){
    qDebug()<<"recv message from %s ,body is %s\n"<<topicName<<(char *) m->payload;

    string topic(topicName);
    string str="sys/disconnect";
    if(topic.find(str) != std::string::npos){
            QProcess::startDetached(QApplication::applicationFilePath());
            QApplication::quit();
    }
    char* datac = (char *) m->payload;

    if(newMqtt != nullptr)
    {
        newMqtt->mqttRecvQue.push(string(datac));
    }

    MQTTAsync_freeMessage(&m);
    MQTTAsync_free(topicName);
    return 1;
}


void onDisconnect(void *context, MQTTAsync_successData *response){
    if(newMqtt != nullptr)
        newMqtt->connected = false;
}

void onSubcribeSuccess(void *context, MQTTAsync_successData *response)
{
   qDebug()<<"success to subscribe,the topic's name is:";
   qDebug()<<(QString::fromStdString(newMqtt->subTopic));
}

void onPublishFailure(void *context, MQTTAsync_failureData *response){
    printf("Publish failed, rc %d\n", response ? -1 : response->code);
}
void onPublishSuccess(void *context, MQTTAsync_successData *response){

}


void onConnectFailure(void *context, MQTTAsync_failureData *response)
{
    qDebug()<<"mqtt connect fail !";
    MQTTAsync client = (MQTTAsync) context;

}
void connectionLost(void *context, char *cause){
    if(newMqtt != nullptr)
        newMqtt->connected = false;
    qDebug()<<"mqtt:client:connection lost\n";
    newMqtt->reconnect_mqtt();


}

void onConnectSuccess(void *context, MQTTAsync_successData *response)
{
    if(newMqtt != nullptr)
    {
        newMqtt->connected = true;
        qDebug()<<"mqtt connect success !";
        MQTTAsync client = (MQTTAsync) context;
        MQTTAsync_responseOptions sub_opts = MQTTAsync_responseOptions_initializer;
        sub_opts.onSuccess = newMqtt->onSubcribeSuccess;

        MQTTAsync_subscribe(client, newMqtt->subTopic.c_str(), 0, &sub_opts);

        QJsonObject rootJson;
        rootJson["cmd"] = "mqttLogin";
        rootJson["data"] = "[mqtt] success to connect and success to subscribe!";
        newMqtt->mqttRecvQue.push(QString(QJsonDocument(rootJson).toJson()).toStdString());

    }
}

