#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

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

protected:

    bool connectSerial();
    void disconnectSerial();

    bool sendRobotSpeeds(double fwSpeed, double rotSpeed );

    bool sendMotorSpeed( quint8 motorIdx, int16_t speed ); ///< Send motor speed in rad/sec
    bool sendMotorSpeeds( int16_t speed0, int16_t speed1 ); ///< Send motor speeds in rad/sec
    bool stopMotor( quint8 motorIdx ); ///< Stops a motor

    bool getMotorStatus(quint8 motIdx); ///< Retrieve the status of the motor

    bool sendMotorParams(quint8 motIdx, double k_vel, double k_ang,
                         qint8 versus, quint8 enable_mode );

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


private:
    Ui::MainWindow *ui;

    ParserPacket* _uNav; ///< uNav communication object

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
    double _k_ang;
    double _k_vel;
    qint8 _versus_left;
    qint8 _versus_right;
    quint8 _enable_mode;

    double _current_value_0;
    double _current_setPoint_0;
    double _current_error_0;
    double _current_control_0;

    double _current_value_1;
    double _current_setPoint_1;
    double _current_error_1;
    double _current_control_1;
};

#endif // MAINWINDOW_H
