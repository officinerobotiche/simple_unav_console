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

    int cpr;
    double ratio, wheel_rad_mm, wheel_base;
    qint8 versus_left, versus_right;
    quint8 enable_mode;
    quint8 enc_pos;
    qint16 bridge_V;

    g_settings->loadMotorParams( cpr, ratio, wheel_rad_mm, wheel_base,
                                versus_left, versus_right, enable_mode, enc_pos, bridge_V );

    ui->lineEdit_enc_cpr->setText( tr("%1").arg(cpr) );
    ui->lineEdit_motor_ratio->setText( tr("%1").arg(ratio) );
    ui->lineEdit_wheel_rad_mm->setText( tr("%1").arg(wheel_rad_mm) );
    ui->lineEdit_wheelbase_mm->setText( tr("%1").arg(wheel_base)  );

    ui->checkBox_invert_mot_left->setChecked( versus_left==-1?true:false );
    ui->checkBox_invert_mot_right->setChecked( versus_right==-1?true:false );

    ui->radioButton_polarity_high->setChecked( enable_mode==1?true:false );
    ui->radioButton_polarity_low->setChecked( enable_mode==0?true:false );

    ui->radioButton_enc_wheel->setChecked( enc_pos==0?true:false );
    ui->radioButton_enc_shaft->setChecked( enc_pos==1?true:false );

    ui->lineEdit_bridge->setText( tr("%1").arg(((float)bridge_V)/1000.0f) );
}

RobotParamsCalculateDialog::~RobotParamsCalculateDialog()
{
    delete ui;
}

bool RobotParamsCalculateDialog::getParams(float& wheel_radius, float& wheel_base, float& ratio,
                                           qint8& versus_left, qint8 &versus_right, quint8& enable_mode , quint8 &enc_pos, qint16 &bridge_V)
{
    versus_left = ui->checkBox_invert_mot_left->isChecked()?-1:1;
    versus_right = ui->checkBox_invert_mot_right->isChecked()?-1:1;

    enable_mode = ui->radioButton_polarity_high->isChecked()?1:0;

    enc_pos = ui->radioButton_enc_wheel->isChecked()?0:1;

    wheel_radius = ui->lineEdit_wheelbase_mm->text().toFloat();
    wheel_base = ui->lineEdit_wheelbase_mm->text().toFloat();

    ratio = ui->lineEdit_motor_ratio->text().toFloat();

    bridge_V = (qint16)(ui->lineEdit_bridge->text().toFloat()*1000.0f);

    return true;
}

void RobotParamsCalculateDialog::on_buttonBox_accepted()
{
    qint8 versus_left = ui->checkBox_invert_mot_left->isChecked()?-1:1;
    qint8 versus_right = ui->checkBox_invert_mot_right->isChecked()?-1:1;

    quint8 enable_mode = ui->radioButton_polarity_high->isChecked()?1:0;

    quint8 enc_pos = ui->radioButton_enc_wheel->isChecked()?0:1;

    qint16 bridge_v = (int)(ui->lineEdit_bridge->text().toFloat()*1000.0f);

    g_settings->saveMotorParams(
                ui->lineEdit_enc_cpr->text().toInt(),
                ui->lineEdit_motor_ratio->text().toDouble(),
                ui->lineEdit_wheel_rad_mm->text().toDouble(),
                ui->lineEdit_wheelbase_mm->text().toDouble(),                
                versus_left, versus_right, enable_mode, enc_pos, bridge_v );

    emit accept();
}

void RobotParamsCalculateDialog::on_lineEdit_enc_cpr_textEdited(const QString &arg1)
{
    Q_UNUSED( arg1 );
}

void RobotParamsCalculateDialog::on_lineEdit_motor_ratio_textEdited(const QString &arg1)
{
    Q_UNUSED( arg1 );
}
