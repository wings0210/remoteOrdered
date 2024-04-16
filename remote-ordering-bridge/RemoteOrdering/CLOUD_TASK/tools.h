#ifndef TOOLS_H
#define TOOLS_H
#include<QDateTime>
#include <chrono>
#include <QDebug>
#include <QString>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QJsonObject>
#include <QJsonArray>
#include <QTimer>
#include <QProcess>
#include <QFile>
#include <QFileInfo>
#include <stack>

class Tools{

public:
      Tools();
      ~Tools();
      static void writeJsonDataToFile(const QByteArray& jsonData, const QString& filePath);
      static void clearFileIfExceedSize(const QString& filePath, qint64 maxSizeInBytes);
      bool checkParenthesis(const char* datac);
      char* chopLastChar(char* datac);

private:

};

#endif
