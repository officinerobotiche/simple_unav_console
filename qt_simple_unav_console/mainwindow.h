#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include "interface/unavinterface.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:

    bool connectSerial();
    void disconnectSerial();

    bool sendEnable(int motIdx, bool enable );

    bool sendRobotSpeeds(double fwSpeed, double rotSpeed );

    bool sendMotorSpeeds( int16_t speed0, int16_t speed1 ); ///< Send motor speeds in rad/sec
    bool stopMotor( quint8 motorIdx ); ///< Stops a motor

    bool getMotorSpeeds(); ///< Retrieve the speed of both motors

private:
    void updateSerialPortList();

private slots:
    void on_actionParameters_triggered();
    void on_pushButton_update_serial_list_clicked();
    void on_pushButton_connect_clicked(bool checked);

    void on_verticalSlider_max_fw_speed_valueChanged(int value);
    void on_verticalSlider_max_rot_speed_valueChanged(int value);

    void onNewJoypadValues(float X, float Y);
    void onStatusTimerTimeout();
    void onCommandTimerTimeout();


    void on_verticalSlider_fixed_fw_speed_sliderMoved(int position);

    void on_horizontalSlider_fixed_rot_speed_sliderMoved(int position);

    void on_pushButton_stop_clicked();

private:
    Ui::MainWindow *ui;

    UNavInterface* _uNavComm;

    QTimer _statusUpdateTimer;  ///< Timer to retrieve the status of the motors from uNav
    QTimer _commandSendTimer;   ///< Timer to send speed command to uNav

    double _prevFwSpeed;    ///< Last forward speed to send to uNav
    double _prevRotSpeed;   ///< Last rotation speed to send to uNav
    double _fwSpeed;    ///< forward speed to send to uNav
    double _rotSpeed;   ///< rotation speed to send to uNav

    int _cpr;
    double _ratio;
    double _wheel_rad_mm;
    double _wheel_base_mm;
    qint8 _versus_left;
    qint8 _versus_right;
    quint8 _enable_mode;
    quint8 _enc_pos;
    qint16 _bridge_V;

    QVector <double> _vel_vec_0;
    QVector <double> _vel_vec_1;
    int _vel_idx;
    int _vel_count;
};

#endif // MAINWINDOW_H

