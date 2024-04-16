#include "mqtt.h"

MQTT::MQTT()
{

}

MQTT::MQTT(string host_,string deviceId_,int port_,string username_,string password_,string clientId_)
{

    host = host_;// mqtt服务器ip地址
   //mqtt核心登录参数
    deviceId = deviceId_;
    clientId = clientId_;
    username = username_;
    password = password_;
    //默认1833
    port = port_;


    useSSL = false;
    connected=0;

}
MQTT::MQTT(string host_, string deviceId_, int port_, string clientId_)
{

    host = host_;// mqtt服务器ip地址
   //mqtt核心登录参数
    deviceId = deviceId_;
    clientId = clientId_;
    //默认1833
    port = port_;


    useSSL = false;
    connected=0;

}
MQTT::~MQTT()
{}
void MQTT::init(void)
{
    /*
    messageArrived=new_messageArrived;
    onConnectSuccess = new_onConnectSuccess;
    onDisconnect = new_onDisconnect;
    onPublishFailure = new_onPublishFailure;
    onPublish = new_onPublishSuccess;
    connectionLost = new_connectionLost;
    messageArrived = new_messageArrived;
    onSubcribe = new_onSubcribe;
      callback = [this](void *context, char *topicName, int topicLen, MQTTAsync_message *m) {
            return new_messageArrived(context, topicName, topicLen, m);
        };
      callback = [this](void *context, char *topicName, int topicLen, MQTTAsync_message *m) {
          return new_messageArrived(context, topicName, topicLen, m);
      };
      onConnectFailure=[this](void *context, MQTTAsync_failureData *response) {
          return new_onConnectFailure(context,response);
      };
      onSubcribe=[this](void *context, MQTTAsync_successData *response){
          return new_onSubcribe(context,response);
      };
      onConnectSuccess=[this](void *context, MQTTAsync_successData *response){
          return new_onConnectSuccess(context,response);
      };
      onDisconnect=[this](void *context, MQTTAsync_successData *response){
          return new_onDisconnect(context,response);
      };
      onPublishFailure=[this](void *context, MQTTAsync_failureData *response){
          return new_onPublishFailure(context,response);
      };
      onPublish=[this](void *context, MQTTAsync_successData *response){
          return new_onPublishSuccess(context,response);
      };
      connectionLost=[this](void *context, char *cause){
          return new_connectionLost(context,cause);
      };

*/

}

int MQTT::connect(void)
{

//    useSSL = true;
    int cleanSession = 1;    //连接会话质量，1表示临时会话，多用于配合Qos=0；0表示持久会话，多用于配合需要保证服务质量，与Qos1,2配合
    int rc = 0;              //用于接收返回值，用作判断，下同

    //以下为create client，即创建客户端过程
    MQTTAsync_createOptions create_opts = MQTTAsync_createOptions_initializer;
    //create_opts.sendWhileDisconnected = 0;
    create_opts.sendWhileDisconnected = 1;         //断连后是否继续发送信息，此时会将信息发送至本地缓冲池，待重连后发送，1表示yes
    //create_opts.maxBufferedMessages = 10;
    create_opts.maxBufferedMessages = 1;           //本地缓冲池大小

    char url[100];
    if (useSSL) {
        snprintf(url, 100, "ssl://%s:%d", host.c_str(), port);
    } else {
        snprintf(url, 100, "tcp://%s:%d", host.c_str(), port);
    }

    //创建异步客户端函数
    rc = MQTTAsync_createWithOptions(&client, url, const_cast<char*>(clientId.c_str()), MQTTCLIENT_PERSISTENCE_NONE, NULL, &create_opts);
    //设置回调函数,第一个参数是客户端对象，第二个是传递的参数，第三个是断线后启动重连的回调函数，第四个是有消息到达时的订阅回调函数，第五个为消息发布成功后的回调函数
    rc = MQTTAsync_setCallbacks(client, client, connectionLost, messageArrived, NULL);

    //2.connect to server
    //创建异步连接结构体，并进行属性设置
    MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
    conn_opts.MQTTVersion = MQTTVERSION_3_1_1;
    conn_opts.keepAliveInterval = 30;
    conn_opts.cleansession = cleanSession;
    conn_opts.username = deviceId.c_str();
    conn_opts.password = password.c_str();
    conn_opts.onSuccess = onConnectSuccess;
    conn_opts.onFailure = onConnectFailure;
    conn_opts.context = client;
    //如果需要使用 SSL 加密
    if (useSSL) {
        MQTTAsync_SSLOptions ssl =MQTTAsync_SSLOptions_initializer;
        conn_opts.ssl = &ssl;
    } else {
        conn_opts.ssl = NULL;
    }
    conn_opts.automaticReconnect = 1;                     //断线自动重连参数设置
    conn_opts.minRetryInterval=5;                         //最小重试间隔
    conn_opts.maxRetryInterval=365*24*60*60;
    conn_opts.connectTimeout = 60;                        //单次连接超时时间


    //若连接失败，打印输出
    if ((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS) {
        std::cout<<"Failed to start connect, return code %d\n"<< rc<<endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;

}

//主动断连函数
int MQTT::disconnect(void)
{

    int rc = 0;
    MQTTAsync_disconnectOptions disc_opts = MQTTAsync_disconnectOptions_initializer;
    disc_opts.onSuccess = this->onDisconnect;
    if ((rc = MQTTAsync_disconnect(client, &disc_opts)) != MQTTASYNC_SUCCESS) {
        printf("Failed to start disconnect, return code %d\n", rc);
        return EXIT_FAILURE;
    }
    while (connected)
        sleep(1);
    MQTTAsync_destroy(&client);
    return EXIT_SUCCESS;
}

//publish函数
int MQTT::send(std::string topic, char *msg)
{

    int rc = 0;

    //publish msg
    MQTTAsync_responseOptions pub_opts = MQTTAsync_responseOptions_initializer;
    pub_opts.onSuccess = onPublishSuccess;
    pub_opts.onFailure = onPublishFailure;
    rc = MQTTAsync_send(client, topic.c_str(), strlen(msg), msg, 0, 0, &pub_opts);
    if(rc != MQTTASYNC_SUCCESS) {
        //printf("mqtt:publish:failed:%s\n", msg);
        return EXIT_FAILURE;
    } else {
        //qDebug()<<"publishing to cloud is successiful !";
        return EXIT_SUCCESS;
    }
}

//自定义断线自动重连函数
void MQTT::reconnect_mqtt(){

   /*
    while (1) {
          // 执行后续的其他任务

          // 检查 MQTT 连接状态
          if (MQTTAsync_isConnected(this->client) != MQTTASYNC_SUCCESS) {
              printf("MQTT 连接断开，重新连接中...\n");
              disconnect();
              sleep(60);
              connect();
          }
      }
    */
    this->disconnect();
    qDebug()<<"MQTT 连接断开，重新连接中...\n";
    sleep(1);
    this->connect();
}

