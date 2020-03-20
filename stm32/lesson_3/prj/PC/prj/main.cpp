#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowFlags(Qt::Dialog);
    w.setFixedSize(540,100);
    w.show();
    return a.exec();
}
