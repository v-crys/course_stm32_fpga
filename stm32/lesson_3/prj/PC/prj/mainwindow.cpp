#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtSerialPort/QSerialPort>
#include <minwindef.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    initSerialPort();

    DWORD stream;

    BASS_SetConfig(BASS_CONFIG_UPDATETHREADS, 1);
    int res = BASS_Init(0, 44100, BASS_DEVICE_DEFAULT, 0, NULL);

    if (!res){
        msgBox.setText("Init error");
        msgBox.exec();
        exit(0);
    }
/*
    char filename[] = "C://1.mp3";
    stream = BASS_StreamCreateFile(FALSE, filename, 0, 0, 0);

    if (res == 0){
        msgBox.setText("Init error");
        msgBox.exec();
        exit(0);
    }

    BASS_ChannelPlay(stream,TRUE);*/

    BASS_WASAPI_DEVICEINFO info_dev;
    for (unsigned long device=0;BASS_WASAPI_GetDeviceInfo(device,&info_dev);device++)
    {
        if ((info_dev.flags & BASS_DEVICE_ENABLED ) && (info_dev.flags & BASS_DEVICE_LOOPBACK ))
        {
            ui->comboBox->addItem(QString::number(device) + QString::fromLocal8Bit(" - ") + QString::fromLocal8Bit(info_dev.name));
        }
    }

    timer = new QTimer();
    connect(timer, SIGNAL(timeout()), this, SLOT(slotTimerAlarm()));
    timer->start(80);
}


void MainWindow::initSerialPort(){
    serial = new QSerialPort(this);
}

int MainWindow::connectSerialPort()
{
    serial->setPortName( QString::fromLocal8Bit("COM") + ui->spinBox->text());
    serial->setBaudRate(serial->Baud9600);
    serial->setDataBits( serial->Data8 );
    serial->setParity(serial->NoParity);
    serial->setStopBits(serial->OneStop);
    serial->setFlowControl( serial->NoFlowControl);

    return serial->open(QIODevice::ReadWrite);
}

void MainWindow::closeSerialPort()
{
    if (serial->isOpen())
        serial->close();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::slotTimerAlarm()
{
    QByteArray send_data = "";
    DWORD level = BASS_WASAPI_GetLevel();
    unsigned int left = (unsigned int) (level & 0xffff); // the left level
    unsigned int right =(unsigned int) (level >> 16); // the right level

    ui->progressBar->setValue(left / (32768.0 / 100));
    ui->progressBar_2->setValue(right / (32768.0 / 100));

    if (flag_connect)
    {
        send_data.append("S");
        send_data.append("3");
        send_data.append(left / (32768.0 / 8) + '0');

        send_data.append("S");
        send_data.append("4");
        send_data.append(right / (32768.0 / 8) + '0');

        serial->write( send_data );
    }
}

void MainWindow::on_pushButton_clicked()
{
    if (flag_connect == 0)
    {
        if (!connectSerialPort()) return;

        ui->pushButton->setText("Disconnect");
    } else {
        closeSerialPort();
        ui->pushButton->setText("Connect");
    }
    flag_connect = !flag_connect;
}

// WASAPI callback, required for continuous recording
DWORD CALLBACK Process(void *buffer, DWORD length, void *user)
{
    return length;
}


void MainWindow::on_pushButton_2_clicked()
{
        if (!flag_open_audio)
        {
            QString dev_text = ui->comboBox->currentText();
            int id_dev = ((dev_text.split(' '))[0]).toInt();

            bool result = BASS_WASAPI_Init(id_dev, 0, 0, BASS_WASAPI_BUFFER, 1.0f, 0.05f, Process, NULL);

            if (!result){
                msgBox.setText("Init error");
                msgBox.exec();
                return;
            }

            flag_open_audio = !flag_open_audio;
            ui->pushButton_2->setText("CloseAudio");
            BASS_WASAPI_Start();
        } else {
            BASS_WASAPI_Stop(true);
            ui->pushButton_2->setText("OpenAudio");
            flag_open_audio = !flag_open_audio;
        }
}

