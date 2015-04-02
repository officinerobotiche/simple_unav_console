#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <ParserPacket.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    bool sendMotorParams(quint8 motIdx, double k_vel, double k_ang,
                         qint8 versus, quint8 enable_mode );

    void updateSerialPortList();

    bool connectSerial();
    void disconnectSerial();

    bool stopMotor( quint8 motorIdx );

private slots:
    void on_actionParameters_triggered();
    void on_pushButton_update_serial_list_clicked();
    void on_pushButton_connect_clicked(bool checked);



private:
    Ui::MainWindow *ui;

    ParserPacket* _uNav; ///< uNav communication object
};

#endif // MAINWINDOW_H
