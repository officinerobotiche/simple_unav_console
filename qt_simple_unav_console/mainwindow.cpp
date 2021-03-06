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

#define MEAN_COUNT 10 // Number of values of speed to be averaged

#define MAX_COMM_ERROR 5 // MAxximun number of consequtive communication error allowed

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    _uNavComm(NULL)
{
    ui->setupUi(this);

    _vel_idx = 0;
    _vel_vec_0.resize(MEAN_COUNT);
    _vel_vec_1.resize(MEAN_COUNT);

    updateSerialPortList();
}

MainWindow::~MainWindow()
{


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
    g_settings->loadMotorParams( _cpr, _ratio, _wheel_rad_mm, _wheel_base_mm,
                                 _versus_left, _versus_right, _enable_mode, _enc_pos, _bridge_V );

    if( !_uNavComm )
        return;

    _uNavComm->sendMotorParams( 0, _cpr, _ratio, _versus_left, _enable_mode, _enc_pos, _bridge_V );
    // TODO error handling

    _uNavComm->sendMotorParams( 1, _cpr, _ratio, _versus_right, _enable_mode, _enc_pos, _bridge_V );
    // TODO error handling
}

void MainWindow::on_pushButton_connect_clicked(bool checked)
{
    if( checked )
    {
        if( !connectSerial() )
        {
            ui->pushButton_connect->setChecked(false);
            QMessageBox::warning( this, tr("Connection error"),
                                  tr("<p>Please verify the correctness of the serial port: <b>%1</b> </p>"
                                     "<p>If the port name is correct, verify the cable and the power of the board</p>"
                                     "<p><i><b>Note:</b> if the port name of the serial port that you are using is not available, you can edit the field to add it manually</i></p>")
                                  .arg( ui->comboBox_serial_port->currentText() ) );
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

        _uNavComm->disconnect();

        delete _uNavComm;
        _uNavComm = NULL;
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
    if( _uNavComm )
        delete _uNavComm;

    string serialPort = ui->comboBox_serial_port->currentText().toStdString();

    _uNavComm = new UNavInterface();

    if( !_uNavComm->connect( serialPort, 115200 ) )
        return false;

    _prevFwSpeed = 0.0;
    _prevRotSpeed = 0.0;
    _fwSpeed = 0.0;
    _rotSpeed = 0.0;

    _commError = 0;

    // >>>>> Qt signals connecting
    connect( ui->widget, SIGNAL(newJoypadValues(float,float)),
             this, SLOT(onNewJoypadValues(float,float) ) );
    connect( &_commandSendTimer, SIGNAL(timeout()),
             this, SLOT(onCommandTimerTimeout()) );
    connect( &_statusUpdateTimer, SIGNAL(timeout()),
             this, SLOT(onStatusTimerTimeout()) );
    // <<<<< Qt signals connecting

    // >>>>> Robot params updating
    g_settings->loadMotorParams( _cpr, _ratio, _wheel_rad_mm, _wheel_base_mm,
                                 _versus_left, _versus_right, _enable_mode, _enc_pos, _bridge_V );

    if( !_uNavComm->sendMotorParams( 0, _cpr, _ratio, _versus_left, _enable_mode, _enc_pos, _bridge_V ) )
        return false;
    // TODO error handling

    if( !_uNavComm->sendMotorParams( 1, _cpr, _ratio, _versus_right, _enable_mode, _enc_pos, _bridge_V ) )
        return false;
    // TODO error handling
    // <<<<< Robot params updating

    if( !_uNavComm->enableSpeedControl( 0, true ) )
        return false;
    if( !_uNavComm->enableSpeedControl( 1, true ) )
        return false;

    if( !_uNavComm->sendPIDGains( 0, 0.05, 0.2, 0.45 ) )
        return false;
    if( !_uNavComm->sendPIDGains( 1, 0.05, 0.2, 0.45 ) )
        return false;

    // >>>>> Start timers
    _commandSendTimer.start( CMD_SEND_PERIOD );
    _statusUpdateTimer.start( STATUS_UPDATE_PERIOD );
    // <<<<< Start timers

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

    ui->horizontalSlider_fixed_rot_speed->setValue( X/maxAxis*100 + 100 );
    ui->verticalSlider_fixed_fw_speed->setValue( Y/maxAxis*100 + 100 );
}

void MainWindow::onStatusTimerTimeout()
{
    if( !_uNavComm )
        return;

    double vel0,vel1;

    bool ok0 = _uNavComm->getMotorSpeed( 0, vel0 );
    bool ok1 = _uNavComm->getMotorSpeed( 1, vel1 );

    if( !ok0 || !ok1 )
    {
        _commError++;

        if( _commError==MAX_COMM_ERROR )
        {
            disconnectSerial();
            QMessageBox::warning( this,
                                  tr("Connection error"), tr("Please verify the correctness of the connection to the board") );

            ui->pushButton_connect->setChecked(false);
            ui->pushButton_connect->setText( tr("Connect") );
        }

        return;
    }

    _commError=0;

    qDebug() << tr("R - Motor 0: %1").arg(vel0);
    qDebug() << tr("R - Motor 1: %1").arg(vel1);

    // >>>>> Averaging
    double mean_vel_0, mean_vel_1;

    _vel_vec_0[_vel_idx] = vel0;
    _vel_vec_1[_vel_idx] = vel1;
    _vel_idx++;
    _vel_count++;

    int stop_idx;

    stop_idx = qMin( MEAN_COUNT, _vel_idx );

    double sum0=0.0;
    double sum1=0.0;

    for( int i=0; i<stop_idx; i++)
    {
        sum0+=_vel_vec_0[i];
        sum1+=_vel_vec_1[i];
    }
    mean_vel_0 = sum0/stop_idx;
    mean_vel_1 = sum1/stop_idx;

    _vel_idx %= MEAN_COUNT;
    // <<<<< Averaging

    double wheel_rad_m = _wheel_rad_mm/1000.0;
    double L = _wheel_base_mm/1000.0;

    double robot_speed_fw = 0.5*(mean_vel_0 + mean_vel_1)*wheel_rad_m;
    double robot_speed_rot = (mean_vel_0 - mean_vel_1)*wheel_rad_m/L;

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

    if( !sendRobotSpeeds( _fwSpeed, _rotSpeed ) )
    {
        _commError++;

        if( _commError==MAX_COMM_ERROR )
        {
            disconnectSerial();
            QMessageBox::warning( this, tr("Connection error"), tr("Please verify the correctness of the connection to the board") );


            ui->pushButton_connect->setChecked(false);
            ui->pushButton_connect->setText( tr("Connect") );
        }
    }
    else
        _commError=0;
}

bool MainWindow::sendRobotSpeeds( double fwSpeed, double rotSpeed )
{
    //qDebug() << "fwSpeed: " << fwSpeed << " - rotSpeed: " << rotSpeed;

    double omega0, omega1;
    double wheel_rad_m = _wheel_rad_mm/1000.0;
    double L = _wheel_base_mm/1000.0;

    double rad = rotSpeed*DEG2RAD;

    omega0 = (2.0*fwSpeed+rad*L)/(2.0*wheel_rad_m);
    omega1 = (2.0*fwSpeed-rad*L)/(2.0*wheel_rad_m);

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

    /*bool ok0 = _uNavComm->sendMotorSpeed( 0, rot_speed0 );
    bool ok1 = _uNavComm->sendMotorSpeed( 1, rot_speed1 );

    return (ok0 & ok1); */

    return _uNavComm->sendMotorSpeeds( rot_speed0, rot_speed1 );
}

bool MainWindow::stopMotor( quint8 motorIdx )
{
    return _uNavComm->sendMotorSpeed( motorIdx, 0 );
}

bool MainWindow::getMotorSpeeds()
{
    return false;
}

void MainWindow::on_verticalSlider_fixed_fw_speed_sliderMoved(int position)
{
    double pos = (double)(position-100)/100.0;
    _fwSpeed  = (((float)ui->verticalSlider_max_fw_speed->value())/1000.0f)*pos;
}

void MainWindow::on_horizontalSlider_fixed_rot_speed_sliderMoved(int position)
{
    double pos = (double)(position-100)/100.0;
    _rotSpeed = (((float)ui->verticalSlider_max_rot_speed->value())/10.0f)*pos;
}

void MainWindow::on_pushButton_stop_clicked()
{
    _fwSpeed = 0.0;
    _rotSpeed = 0.0;

    ui->verticalSlider_fixed_fw_speed->setValue(100);
    ui->horizontalSlider_fixed_rot_speed->setValue(100);
}
