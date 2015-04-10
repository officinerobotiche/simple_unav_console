#include "robotparamscalculatedialog.h"
#include "ui_robotparamscalculatedialog.h"

#include "QMessageBox"
#include "qmath.h"

#include "csettings.h"

RobotParamsCalculateDialog::RobotParamsCalculateDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RobotParamsCalculateDialog)
{
    ui->setupUi(this);

    _k_ang = 0.0;
    _k_vel = 0.0;
    _calculated = false;

    int cpr;
    double ratio, wheel_rad_mm, wheel_base;
    double k_ang, k_vel;
    qint8 versus_left, versus_right;
    quint8 enable_mode;

    g_settings->loadMotorParams( cpr, ratio, wheel_rad_mm, wheel_base, k_ang, k_vel,
                                versus_left, versus_right, enable_mode  );

    ui->lineEdit_enc_cpr->setText( tr("%1").arg(cpr) );
    ui->lineEdit_motor_ratio->setText( tr("%1").arg(ratio) );
    ui->lineEdit_wheel_rad_mm->setText( tr("%1").arg(wheel_rad_mm) );
    ui->lineEdit_wheelbase_mm->setText( tr("%1").arg(wheel_base)  );
    ui->lineEdit_k_ang_left->setText( tr("%1").arg(k_ang) );
    ui->lineEdit_k_ang_right->setText( tr("%1").arg(k_ang) );
    ui->lineEdit_k_vel_left->setText( tr("%1").arg(k_vel) );
    ui->lineEdit_k_vel_right->setText( tr("%1").arg(k_vel) );

    ui->checkBox_invert_mot_left->setChecked( versus_left==-1?true:false );
    ui->checkBox_invert_mot_right->setChecked( versus_right==-1?true:false );

    ui->radioButton_polarity_high->setChecked( enable_mode==1?true:false );
    ui->radioButton_polarity_low->setChecked( enable_mode==0?true:false );
}

RobotParamsCalculateDialog::~RobotParamsCalculateDialog()
{
    delete ui;
}

bool RobotParamsCalculateDialog::getParams(double& k_ang, double& k_vel, qint8& versus_left, qint8 &versus_right, quint8& enable_mode )
{
    if( !_calculated )
        return false;

    k_ang = _k_ang;
    k_vel = _k_vel;

    versus_left = ui->checkBox_invert_mot_left->isChecked()?-1:1;
    versus_right = ui->checkBox_invert_mot_right->isChecked()?-1:1;

    enable_mode = ui->radioButton_polarity_high->isChecked()?1:0;

    return true;
}

void RobotParamsCalculateDialog::on_pushButton_clicked()
{
    _k_ang = 0.0;
    _k_vel = 0.0;
    _calculated = false;

    double freq = 80e6;

    bool ok;
    double cpr = ui->lineEdit_enc_cpr->text().toDouble( &ok );

    if(!ok)
    {
        QMessageBox::warning( this, tr("Warning"), tr("Please insert a correct value for Encoder CPR"));
        return;
    }

    double ratio = ui->lineEdit_motor_ratio->text().toDouble( &ok);

    if(!ok)
    {
        QMessageBox::warning( this, tr("Warning"), tr("Please insert a correct value for Motor Reduction ratio"));
        return;
    }

    double k_ang = (2.0*M_PI)/(4.0*ratio*cpr);

    double d_rad = k_ang * 2.0;

    double k_vel = 1000.0*d_rad*freq;

    ui->lineEdit_k_ang_left->setText( tr("%1").arg(k_ang,12,'f') );
    ui->lineEdit_k_ang_right->setText( tr("%1").arg(k_ang,12,'f') );

    ui->lineEdit_k_vel_left->setText( tr("%1").arg(k_vel,12,'f') );
    ui->lineEdit_k_vel_right->setText( tr("%1").arg(k_vel,12,'f') );

    _k_ang = k_ang;
    _k_vel = k_vel;
    _calculated = true;
}

void RobotParamsCalculateDialog::on_buttonBox_accepted()
{
    if( !_calculated )
        on_pushButton_clicked();

    int versus_left = ui->checkBox_invert_mot_left->isChecked()?-1:1;
    int versus_right = ui->checkBox_invert_mot_right->isChecked()?-1:1;

    int enable_mode = ui->radioButton_polarity_high->isChecked()?1:0;

    g_settings->saveMotorParams(
                ui->lineEdit_enc_cpr->text().toInt(),
                ui->lineEdit_motor_ratio->text().toDouble(),
                ui->lineEdit_wheel_rad_mm->text().toDouble(),
                ui->lineEdit_wheelbase_mm->text().toDouble(),
                _k_ang, _k_vel,
                versus_left, versus_right, enable_mode );

    emit accept();
}

void RobotParamsCalculateDialog::on_lineEdit_enc_cpr_textEdited(const QString &arg1)
{
    Q_UNUSED( arg1 );

    ui->lineEdit_k_ang_left->clear();
    ui->lineEdit_k_ang_right->clear();
    ui->lineEdit_k_vel_left->clear();
    ui->lineEdit_k_vel_right->clear();
}

void RobotParamsCalculateDialog::on_lineEdit_motor_ratio_textEdited(const QString &arg1)
{
    Q_UNUSED( arg1 );

    ui->lineEdit_k_ang_left->clear();
    ui->lineEdit_k_ang_right->clear();
    ui->lineEdit_k_vel_left->clear();
    ui->lineEdit_k_vel_right->clear();
}
