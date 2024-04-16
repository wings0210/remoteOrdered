QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RemoteOrdering
TEMPLATE = app

DEFINES += QT_DEPRECATED_WARNINGS


CONFIG += c++14

SOURCES += main.cpp \
        mainwindow.cpp \
        MQTT/mqtt.cpp \
        REGIST/regist.cpp \
        CLOUD_TASK/tools.cpp \
        CLOUD_TASK/cloud_task.cpp \
        MQTT/pointer.cpp \
        prasejson.cpp \
        rosworker.cpp \



HEADERS += mainwindow.h \
        MQTT/mqtt.h \
        REGIST/regist.h \
        CLOUD_TASK/tools.h \
        CLOUD_TASK/cloud_task.h \
        MQTT/pointer.h \
        prasejson.h \
        rosworker.h \
        struct.h

FORMS += mainwindow.ui

TRANSLATIONS += remotedispatch_zh_CN.ts

linux{
    LIBS += -L$$PWD/ -lpaho-mqtt3a
    LIBS += -lpaho-mqttpp3
}
INCLUDEPATH += ../../../messages/devel/include/
LIBS += -lstdc++
LIBS += -L/usr/local/lib -lyaml-cpp

INCLUDEPATH += /opt/ros/melodic/include
DEPENDPATH += /opt/ros/melodic/include
LIBS += -L/opt/ros/melodic/lib -lroscpp -lroslib -lrosconsole
LIBS += /opt/ros/melodic/lib/lib*.so


message("-----------------------------编译环境------------------------------")
ubuntu_version = $$system(lsb_release -rs)
message("ubuntu version : $$ubuntu_version")
contains(QMAKE_HOST.arch, aarch64){
    message("编译目标平台: arm-64")
    message("编译器平台类型: $$QMAKE_HOST.arch")
    LIBS += /usr/lib/aarch64-linux-gnu/libboost_system*.so

    equals(ubuntu_version, "20.04") {
        #orin 20.04
        message(系统类型: $$QMAKE_HOST.os)
        message("系统版本: $$QMAKE_HOST.version")

        DEPENDPATH += /opt/ros/noetic/include
        INCLUDEPATH += /opt/ros/noetic/include
        LIBS += -L/opt/ros/noetic/lib/ -lroscpp -lroslib -lrosconsole
        LIBS += /opt/ros/noetic/lib/lib*.so
    }else:equals(ubuntu_version, "18.04") {
        #xvaier 18.04
        message(系统类型: $$QMAKE_HOST.os)
        message("系统版本: $$QMAKE_HOST.version")
        DEPENDPATH += /opt/ros/melodic/include
        INCLUDEPATH += /opt/ros/melodic/include
        LIBS += -L/opt/ros/melodic/lib/ -lroscpp -lroslib -lrosconsole
        LIBS += /opt/ros/melodic/lib/lib*.so
    }
}else{
    #x86平台
    message("编译目标平台: x86")
    message("编译器平台类型: $$QMAKE_HOST.arch")
    LIBS += /usr/lib/x86_64-linux-gnu/libboost_system*.so

    equals(ubuntu_version, "18.04") {
        #pc 18.04
        message(系统类型: $$QMAKE_HOST.os)
        message("系统版本: $$QMAKE_HOST.version")

        INCLUDEPATH += /opt/ros/melodic/include
        DEPENDPATH += /opt/ros/melodic/include
        LIBS += -L/opt/ros/melodic/lib -lroscpp -lroslib -lrosconsole
        LIBS += /opt/ros/melodic/lib/lib*.so
    }
    else:equals(ubuntu_version, "20.04") {
        message(系统类型: $$QMAKE_HOST.os)
        message("系统版本: $$QMAKE_HOST.version")

        INCLUDEPATH += /opt/ros/noetic/include
        DEPENDPATH += /opt/ros/noetic/include
        LIBS += -L/opt/ros/noetic/lib -lroscpp -lroslib -lrosconsole
        LIBS += /opt/ros/noetic/lib/lib*.so
    }
}
