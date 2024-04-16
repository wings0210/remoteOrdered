#ifndef PRASEJSON_H
#define PRASEJSON_H
#include "struct.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QList>
#include <QDateTime>
#include <QJsonArray>
bool prase_json(QJsonDocument &doc,QString &error,struct Cloud &cloud);
bool prase_stationList(QJsonObject &rootJson,QString &error,struct Cloud &cloud);
bool check_not_valid(QJsonObject &jsonObj,QStringList &expected,QString &error);
inline bool prase_stationList(QJsonObject &dataJson ,QString &error,struct Cloud &cloud);
inline bool prase_nextStationList(QJsonObject &dataJson,QString &error,struct Cloud &cloud);
inline bool prase_param(QJsonObject &dataJson,struct Cloud &cloud);
inline bool prase_relocation(QJsonObject &dataJson,struct Cloud &cloud);
inline bool prase_start(struct Cloud &cloud);
inline bool prase_stop(struct Cloud &cloud);
inline bool prase_pause(struct Cloud &cloud);
inline bool prase_resume(struct Cloud &cloud);
#endif // PRASEJSON_H
