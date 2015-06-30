#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "robotparamscalculatedialog.h"

#include <string>

#include <QSerialPortInfo>
#include <QDebug>
#include <QMessageBox>
#include "csettings.h"

#define CMD_SEND_PERIOD 30
#define STATUS_UPDATE_PERIOD 30

#define RAD2DEG 57.29577951308233
#define DEG2RAD 0.017453293

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    _uNav(NULL)
{
    ui->setupUi(this);

    updateSerialPortList();
}

MainWindow::~MainWindow()
{
    if(_uNav)
    {
        delete _uNav;
        _uNav = NULL;
    }

    delete ui;
}

void MainWindow::updateSerialPortList()
{
    ui->comboBox_serial_port->clear();

    QList<QSerialPortInfo> serialList;
    serialList = QSerialPortInfo::availablePorts();

    foreach( QSerialPortInfo info, serialList)
    {
        ui->comboBox_serial_port->addItem( info.systemLocation() );
    }
}

void MainWindow::on_pushButton_update_serial_list_clicked()
{
    updateSerialPortList();
}

void MainWindow::on_actionParameters_triggered()
{
    RobotParamsCalculateDialog dlg;

    if( QDialog::Accepted != dlg.exec() )
        return;

    // Params update
    g_settings->loadMotorParams( _cpr, _ratio, _wheel_rad_mm, _wheel_base_mm, _k_ang, _k_vel,
                                _versus_left, _versus_right, _enable_mode  );

    sendMotorParams( 0, _k_vel, _k_ang, _versus_left, _enable_mode );
    // TODO error handling

    sendMotorParams( 1, _k_vel, _k_ang, _versus_right, _enable_mode );
    // TODO error handling



}

bool MainWindow::sendMotorParams(quint8 motIdx, double k_vel, double k_ang,
                                 qint8 versus, quint8 enable_mode )
{
    if( !_uNav )
        return false;

    //if( !_connected )
    //    return false;

    try
    {
        parameter_motor_t param;
        param.enable_set = enable_mode;
        param.k_ang = k_ang;
        param.k_vel = k_vel;
        param.versus = versus;

        quint8 command;
        if(motIdx==0)
            command = PARAMETER_MOTOR_L;
        else
            command = PARAMETER_MOTOR_R;

        _uNav->parserSendPacket(_uNav->createDataPacket(command, HASHMAP_MOTION, (abstract_message_u*) & param), 3, boost::posix_time::millisec(200));
    }
    catch( parser_exception& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg(e.what());

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch(...)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: Unknown error");

        throw;
        return false;
    }

    sleep(1);

    return true;
}

void MainWindow::on_pushButton_connect_clicked(bool checked)
{
    if( checked )
    {
        try
        {
            if( !connectSerial() )
            {
                ui->pushButton_connect->setChecked(false);
                QMessageBox::warning( this, tr("Error"), tr("Please verify the correctness of the connection to the board") );
                return;
            }
        }
        catch( parser_exception& e)
        {
            ui->pushButton_connect->setChecked(false);
            QMessageBox::warning( this, tr("Connection error"), e.what() );
            return;
        }
        catch( boost::system::system_error& e)
        {
            ui->pushButton_connect->setChecked(false);
            QMessageBox::warning( this, tr("Connection error"), e.what() );
            return;
        }
        catch(...)
        {
            ui->pushButton_connect->setChecked(false);
            QMessageBox::warning( this, tr("Connection error"), tr("Unknown error") );
            return;
        }

        ui->pushButton_connect->setText( tr("Disconnect") );
    }
    else
    {
        try
        {
            disconnectSerial();
        }
        catch( parser_exception& e)
        {
            ui->pushButton_connect->setChecked(false);
            QMessageBox::warning( this, tr("Connection error"), e.what() );
            return;
        }
        catch( boost::system::system_error& e)
        {
            ui->pushButton_connect->setChecked(false);
            QMessageBox::warning( this, tr("Connection error"), e.what() );
            return;
        }
        catch(...)
        {
            ui->pushButton_connect->setChecked(false);
            QMessageBox::warning( this, tr("Connection error"), tr("Unknown error") );
            return;
        }

        ui->pushButton_connect->setText( tr("Connect") );
    }
}

void MainWindow::disconnectSerial()
{
    //if( !_uNav || !_connected )
    //    return;

    try
    {
        // >>>>> Stop timers
        _commandSendTimer.stop();
        _statusUpdateTimer.stop();
        // <<<<< Stop timers

        // >>>>> Qt signals disconnecting
        disconnect( ui->widget, SIGNAL(newJoypadValues(float,float)),
                    this, SLOT(onNewJoypadValues(float,float) ) );
        disconnect( &_commandSendTimer, SIGNAL(timeout()),
                    this, SLOT(onCommandTimerTimeout()) );
        disconnect( &_statusUpdateTimer, SIGNAL(timeout()),
                    this, SLOT(onStatusTimerTimeout()) );
        // <<<<< Qt signals disconnecting

        stopMotor(0);
        stopMotor(1);

        _uNav->close();

        delete _uNav;
        _uNav = NULL;
    }
    catch( parser_exception& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Disconnection error: %1").arg(e.what());

        throw e;
        return;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Disconnection error: %1").arg( e.what() );

        throw e;
        return;
    }
    catch(...)
    {
        qDebug() << Q_FUNC_INFO << tr("Disconnection error: Unknown error");

        throw;
        return;
    }


}

bool MainWindow::connectSerial()
{
    if( _uNav )
        delete _uNav;

    string serialPort = ui->comboBox_serial_port->currentText().toStdString();

    try
    {
        _uNav = new ParserPacket( serialPort, 115200 );

    }
    catch( parser_exception& e)
    {
        qDebug() << tr("Connection error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << tr("Connection error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch(...)
    {
        qDebug() << tr("Connection error: Unknown error");

        throw;
        return false;
    }

    _prevFwSpeed = 0.0;
    _prevRotSpeed = 0.0;
    _fwSpeed = 0.0;
    _rotSpeed = 0.0;

    // >>>>> Qt signals connecting
    connect( ui->widget, SIGNAL(newJoypadValues(float,float)),
             this, SLOT(onNewJoypadValues(float,float) ) );
    connect( &_commandSendTimer, SIGNAL(timeout()),
             this, SLOT(onCommandTimerTimeout()) );
    connect( &_statusUpdateTimer, SIGNAL(timeout()),
             this, SLOT(onStatusTimerTimeout()) );
    // <<<<< Qt signals connecting

    // >>>>> Start timers
    _commandSendTimer.start( CMD_SEND_PERIOD );
    _statusUpdateTimer.start( STATUS_UPDATE_PERIOD );
    // <<<<< Start timers

    // >>>>> Robot params updating
    g_settings->loadMotorParams( _cpr, _ratio, _wheel_rad_mm, _wheel_base_mm, _k_ang, _k_vel,
                                _versus_left, _versus_right, _enable_mode  );

    sendMotorParams( 0, _k_vel, _k_ang, _versus_left, _enable_mode );
    sendMotorParams( 1, _k_vel, _k_ang, _versus_right, _enable_mode );
    // <<<<< Robot params updating

    sendEnable( 0, true );
    sendEnable( 1, true );

    sendPIDGains( 0, 0.05, 0.2, 0.45 );
    sendPIDGains( 1, 0.05, 0.2, 0.45 );

    return true;
}

bool MainWindow::sendEnable(int motIdx, bool enable )
{
    //if( !_connected )
    //    return false;

    try
    {
        motor_control_t enable_val = enable ? STATE_CONTROL_VELOCITY : STATE_CONTROL_DISABLE;

        quint8 command;
        if(motIdx==0)
            command = ENABLE_MOTOR_L;
        else
            command = ENABLE_MOTOR_R;

        _uNav->parserSendPacket( _uNav->createDataPacket( command, HASHMAP_MOTION, (abstract_message_u*)&enable_val ) );
    }
    catch( parser_exception& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg(e.what());

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch(...)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: Unknown error");

        throw;
        return false;
    }

    return true;
}

void MainWindow::on_verticalSlider_max_fw_speed_valueChanged(int value)
{
    ui->lineEdit_max_fw_speed->setText( tr("%1").arg( ((double)value)/1000.0, 3, 'f', 2 ) );
}

void MainWindow::on_verticalSlider_max_rot_speed_valueChanged(int value)
{
    ui->lineEdit_max_rot_speed->setText( tr("%1").arg( ((double)value)/10.0, 3, 'f', 1 ) );
}

void MainWindow::onNewJoypadValues(float X, float Y)
{
    _prevFwSpeed = _fwSpeed;
    _prevRotSpeed = _rotSpeed;

    float maxAxis = ui->widget->getMaxAbsAxisValue();

    float signX = X<0?-1.0:1.0;
    float signY = Y<0?-1.0:1.0;

    float newX = signX*pow(X/maxAxis,2); // Mappatura quadratica
    float newY = signY*pow(Y/maxAxis,2);

    _fwSpeed  = (((float)ui->verticalSlider_max_fw_speed->value())/1000.0f)*newY;
    _rotSpeed = (((float)ui->verticalSlider_max_rot_speed->value())/10.0f)*newX;

    qDebug() << _rotSpeed;
}

void MainWindow::onStatusTimerTimeout()
{
    bool ok0 = getMotorStatus( 0 );
    bool ok1 = getMotorStatus( 1 );

    if( !ok0 || !ok1 )
        return;

    double wheel_rad_m = _wheel_rad_mm/1000.0;
    double L = _wheel_base_mm/1000.0;

    double robot_speed_fw = 0.5*(_current_value_0 + _current_value_1)*wheel_rad_m;
    double robot_speed_rot = (_current_value_0 - _current_value_1)*wheel_rad_m/L;

    ui->lcdNumber_fw_speed->display(robot_speed_fw);
    ui->lcdNumber_rot_speed->display(robot_speed_rot*RAD2DEG);

    double perc_fw = robot_speed_fw/(((double)ui->verticalSlider_max_fw_speed->value())/1000.0) * 100.0;
    ui->progressBar_fw_speed->setValue((int)(perc_fw+0.5));

    double perc_rot = fabs(RAD2DEG*robot_speed_rot)/(((double)ui->verticalSlider_max_rot_speed->value())/10.0) * 100.0;
    if(robot_speed_rot>0)
    {
        ui->progressBar_rot_speed_pos->setValue((int)(perc_rot+0.5));
        ui->progressBar_rot_speed_neg->setValue(0);
    }
    else
    {
        ui->progressBar_rot_speed_neg->setValue((int)(perc_rot+0.5));
        ui->progressBar_rot_speed_pos->setValue(0);
    }
}

void MainWindow::onCommandTimerTimeout()
{
    /*if( (fabs( _prevFwSpeed - _fwSpeed ) < 0.005 ) ||
            (fabs( _prevRotSpeed - _rotSpeed ) < 0.1 ) )
        return;*/

    sendRobotSpeeds( _fwSpeed, _rotSpeed );
}

bool MainWindow::sendRobotSpeeds( double fwSpeed, double rotSpeed )
{
    //qDebug() << "fwSpeed: " << fwSpeed << " - rotSpeed: " << rotSpeed;

    double omega0, omega1;
    double wheel_rad_m = _wheel_rad_mm/1000.0;
    double L = _wheel_base_mm/1000.0;

    double rad = rotSpeed*DEG2RAD;

    omega0 = (2.0*fwSpeed-rad*L)/(2.0*wheel_rad_m);
    omega1 = (2.0*fwSpeed+rad*L)/(2.0*wheel_rad_m);

    if( omega0 > 32.0 )
        omega0 = 32.0;
    else if( omega0 < -32.0 )
        omega0 = -32.0;

    if( omega1 > 32.0 )
        omega1 = 32.0;
    else if( omega1 < -32.0 )
        omega1 = -32.0;


    int16_t rot_speed0 = (int16_t)(omega0 * 1000);
    int16_t rot_speed1 = (int16_t)(omega1 * 1000);

    //qDebug() << "omega0: " << omega0 << " - omega1: " << omega1;
    //qDebug() << "rot_speed0: " << rot_speed0 << " - rot_speed1: " << rot_speed1;

    bool ok0 = sendMotorSpeed( 0, rot_speed0 );
    bool ok1 = sendMotorSpeed( 1, rot_speed1 );

    return (ok0 & ok1);
}

bool MainWindow::stopMotor( quint8 motorIdx )
{
    //if( !_connected )
    //    return false;

    return sendMotorSpeed( motorIdx, 0 );
}

bool MainWindow::getMotorStatus(quint8 motIdx)
{
    //if( !_connected )
    //    return false;

    try
    {
        quint8 command=MOTOR_L;
        if(motIdx==0)
            command = MOTOR_L;
        else
            command = MOTOR_R;

        information_packet_t send = _uNav->createPacket( command, REQUEST, HASHMAP_MOTION);
        packet_t received = _uNav->sendSyncPacket( _uNav->encoder(send), 3, boost::posix_time::millisec(200) );

        // parse packet
        vector<information_packet_t> list = _uNav->parsing(received);
        //get first packet
        information_packet_t first = list.at(0);

        if(first.option == DATA)
        {
            if(first.type == HASHMAP_MOTION)
            {
                motor_t motor0, motor1;

                switch(first.command)
                {
                case MOTOR_L:
                    motor0 = first.packet.motor;

                    _current_value_0 = ((double)motor0.measure_vel)/1000.0;
                    _current_setPoint_0 = ((double)motor0.refer_vel)/1000.0;
                    _current_error_0 = _current_setPoint_0-_current_value_0;
                    _current_control_0 = ((double)motor0.control_vel)/1000.0;

                    break;

                case MOTOR_R:
                    motor1 = first.packet.motor;

                    _current_value_1 = ((double)motor1.measure_vel)/1000.0;
                    _current_setPoint_1 = ((double)motor1.refer_vel)/1000.0;
                    _current_error_1 = _current_setPoint_1-_current_value_1;
                    _current_control_1 = ((double)motor1.control_vel)/1000.0;

                    break;
                }
            }
        }

    }
    catch( parser_exception& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg(e.what());

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch(...)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: Unknown error");

        throw;
        return false;
    }

    return true;
}



bool MainWindow::sendMotorSpeed( quint8 motorIdx, int16_t speed )
{

    try
    {
        motor_control_t motor_ref = (int16_t) speed;

        quint8 command;
        if(motorIdx==0)
            command = VEL_MOTOR_L;
        else
            command = VEL_MOTOR_R;

        _uNav->parserSendPacket(_uNav->createDataPacket( command, HASHMAP_MOTION, (abstract_message_u*) & motor_ref), 3, boost::posix_time::millisec(200));

    }
    catch( parser_exception& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg(e.what());

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch(...)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: Unknown error");

        throw;
        return false;
    }

    return true;
}

bool MainWindow::sendMotorSpeeds( int16_t speed0, int16_t speed1 )
{
    bool ok0 = sendMotorSpeed( 0, speed0 );
    bool ok1 = sendMotorSpeed( 1, speed1 );

    return (ok0 & ok1);
}

bool MainWindow::sendPIDGains( quint8 motorIdx, double kp, double ki, double kd )
{
    //if( !_connected )
    //    return false;

    try
    {
        pid_control_t pid;
        pid.kp = kp;
        pid.ki = ki;
        pid.kd = kd;

        quint8 command;
        if(motorIdx==0)
            command = PID_CONTROL_L;
        else
            command = PID_CONTROL_R;


        _uNav->parserSendPacket(_uNav->createDataPacket( command, HASHMAP_MOTION, (abstract_message_u*) & pid), 3, boost::posix_time::millisec(200));
    }
    catch( parser_exception& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg(e.what());

        throw e;
        return false;
    }
    catch( boost::system::system_error& e)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: %1").arg( e.what() );

        throw e;
        return false;
    }
    catch(...)
    {
        qDebug() << Q_FUNC_INFO << tr("Serial error: Unknown error");

        throw;
        return false;
    }

    return true;
}

