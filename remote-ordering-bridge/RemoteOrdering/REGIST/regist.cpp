#include "regist.h"

Regist::Regist(){

  get_args_from_wulin();
  //get_client_args();

}
Regist::~Regist(){

}

//在五菱服务器注册并获取车辆参数过程（后续通过该参数，在华为云网站进行注册获取mqtt登录参数）
void Regist::get_args_from_wulin(){

    QString baseurl = "https://ccscarapi.sgmw.com.cn/vehicle/open/getVehicleByVin";
    QString accessId = "dacheng";
    QString secret = "QJFWyAqYd1zf";

    // 将文件内容解析为YAML文档
    YAML::Node config = YAML::LoadFile("../../../conf/remoteorder-conf/conf.yaml");


    // 读取相应参数的值
    string Vin = config["Vin"].as<string>();
    QString vin =QString::fromStdString(Vin);
    //QString vin = "XXHPT1D2022101869";

    QNetworkAccessManager manager;
    QUrl url(baseurl);                                 //基础url部分
    QUrlQuery urlQuery;//parm without baseurl          //？后的query_url部分

    //（1.accessid + 2.timestamp + 3.vin + 4.secret） ->md5->sign          //query_url组成以及迭代
    //（1.accessid + 2.timestamp + 3.vin + 4.sign） -> urlQuery

    urlQuery.addQueryItem("accessId", accessId);
    urlQuery.addQueryItem("timestamp", QString::number(QDateTime::currentDateTime().toSecsSinceEpoch()));
    urlQuery.addQueryItem("vin", vin);
    urlQuery.addQueryItem("secret", secret);
    QString sign = md5Hash(urlQuery.toString().toUtf8()).toUpper();
    urlQuery.removeAllQueryItems("secret");
    urlQuery.addQueryItem("sign", sign);

    url.setQuery(urlQuery);
    qDebug()<<url.toString();    //查看url

    QNetworkConfigurationManager configManager;
    bool isOnline = configManager.isOnline();
    if (!isOnline) {
        qDebug() << "No network connection available.";
        return ;
    }

    QNetworkRequest request(url);
    QNetworkReply *reply = manager.get(request);

    QEventLoop loop;
    QObject::connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
    loop.exec();

    if (reply->error() == QNetworkReply::NoError) {
        QByteArray responseData = reply->readAll();
        qDebug() << "Response:" << responseData;
        switch_json(responseData);
    }


}

void Regist::get_client_args(){



}

//md5加密函数，返回sign
QString Regist::md5Hash(const QByteArray& data){

    QCryptographicHash hash(QCryptographicHash::Md5);
    hash.addData(data);

    QByteArray result = hash.result();
    QString md5 = result.toHex();

    return md5;


}
//比特数组解析为json
void Regist::switch_json(QByteArray& jsonData){
    QJsonDocument jsonDoc = QJsonDocument::fromJson(jsonData);
      if (!jsonDoc.isNull()) {
          if (jsonDoc.isObject()) {
              QJsonObject jsonObj = jsonDoc.object();
              QJsonObject target_json=jsonObj["data"].toObject();
              //qDebug()<<jsonObj["data"];

              // 从JSON对象中获取数据
              QString car_id = target_json["deviceId"].toString();
              //int age = jsonObj["age"].toInt();
              QString car_secret = target_json["deviceSecret"].toString();

              // 输出解析结果
              qDebug() << "车辆id:" << car_id;
              qDebug() << "车辆密码:" << car_secret;

              std::string id =car_id.toStdString();
              std::string secret =car_secret.toStdString();

              YAML::Node config = YAML::LoadFile("../../../conf/remoteorder-conf/conf.yaml"); // 加载配置文件
              config["DeviceId"] =id ; // 将vin值写入配置文件的键
              config["DeviceSecret"]=secret;

              std::ofstream fout("../../../conf/remoteorder-conf/conf.yaml"); // 打开配置文件以写入更新后的内容
              fout << config; // 将更新后的配置写入文件

          }
      }
/*
      //测试解析字符串（字符数组）为json
      QString jsonStr = "{\"name\":\"John\",\"age\":30,\"address\":[{\"street\":\"123 Main St\",\"city\":\"New York\"},{\"name\":\"hello\",\"password\":\"123456\"}],\"hobbies\":[\"reading\",\"coding\"]}";
      QJsonDocument jsonDoc1 = QJsonDocument::fromJson(jsonStr.toUtf8());
      if (!jsonDoc1.isNull()) {
          if (jsonDoc1.isObject()) {
              // 获取根对象
              QJsonObject jsonObj1 = jsonDoc1.object();

              // 访问根对象的属性
              QString name = jsonObj1["name"].toString();
              int age = jsonObj1["age"].toInt();

              // 访问嵌套数组json对象的属性
              QJsonArray arr=jsonObj1["address"].toArray();
              QJsonObject arr_1=arr[1].toObject();
              QString name1=arr_1["name"].toString();


              // 访问嵌套数组
              QJsonArray hobbiesArray = jsonObj1["hobbies"].toArray();
              QStringList hobbies;
              for (int i = 0; i < hobbiesArray.size(); ++i) {
                  hobbies.append(hobbiesArray[i].toString());
              }

              // 打印解析结果
              qDebug() << "name:" << name1;pushButton_mqttConnect
              qDebug()<<arr_1;
              qDebug()<<hobbies;

          }
      }

*/

}
