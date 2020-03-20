#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QTimer>
#include <QMessageBox>

#include "lib/bass.h"
#include "lib/basswasapi.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();
    void slotTimerAlarm();

    void on_pushButton_2_clicked();

private:
    Ui::MainWindow *ui;
    QSerialPort *serial;
    QTimer *timer;

    QMessageBox msgBox;

    int flag_connect = 0;
    int flag_open_audio = 0;

    void initSerialPort();
    int connectSerialPort();
    void closeSerialPort();


};
#endif // MAINWINDOW_H
