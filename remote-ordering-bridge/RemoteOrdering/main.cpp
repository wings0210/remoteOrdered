#include "mainwindow.h"
#include "REGIST/regist.h"


int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    Regist regist;

    MainWindow w;
    w.show();



    return a.exec();
}
