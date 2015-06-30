#include "csettings.h"

#include <QApplication>

CSettings* g_settings=NULL;

CSettings::CSettings():
    mSettings(NULL)
{
    mSettingsFile = QObject::tr( "%1/%2.ini")
            .arg(QApplication::applicationDirPath() )
            .arg(QApplication::applicationName() );

    mSettings = new QSettings( mSettingsFile, QSettings::IniFormat );

}


CSettings::~CSettings()
{
    if(mSettings)
        delete mSettings;
}

bool CSettings::saveMotorParams(int cpr, double ratio, double wheel_rad_mm, double wheelbase_mm,
        double k_ang, double k_vel , qint8 versus_left, qint8 versus_right, quint8 enable_mode )
{
    if(!mSettings)
        return false;

    mSettings->beginGroup( MOTOR_GROUP );

    mSettings->setValue( ENCODER_CPR, cpr );
    mSettings->setValue( MOTOR_RATIO, ratio );
    mSettings->setValue( WHEEL_RAD, wheel_rad_mm );
    mSettings->setValue( WHEELBASE, wheelbase_mm );
    mSettings->setValue( K_ANG, k_ang );
    mSettings->setValue( K_VEL, k_vel );
    mSettings->setValue( DIRECTION_L, versus_left );
    mSettings->setValue( DIRECTION_R, versus_right );
    mSettings->setValue( EN_POLARITY, enable_mode );

    mSettings->endGroup();
    mSettings->sync();

    return true;
}

bool CSettings::loadMotorParams(int& cpr, double &ratio, double &wheel_rad_mm, double &wheelbase_mm,
                                double& k_ang, double& k_vel , qint8& versus_left, qint8& versus_right,
                                quint8& enable_mode )
{
    if(!mSettings)
        return false;

    mSettings->beginGroup( MOTOR_GROUP );

    cpr = mSettings->value( ENCODER_CPR, "400" ).toInt();
    ratio = mSettings->value( MOTOR_RATIO, "18.33" ).toDouble();
    wheel_rad_mm = mSettings->value( WHEEL_RAD, "33.5" ).toDouble();
    wheelbase_mm = mSettings->value( WHEELBASE, "400.0" ).toDouble();
    k_ang = mSettings->value( K_ANG, "0.000214" ).toDouble();
    k_vel = mSettings->value( K_VEL, "34278152.248661" ).toDouble();
    versus_left = mSettings->value( DIRECTION_L, "1" ).toInt();
    versus_right = mSettings->value( DIRECTION_R, "1" ).toInt();
    enable_mode = mSettings->value( EN_POLARITY, "0" ).toInt();

    mSettings->setValue( ENCODER_CPR, cpr );
    mSettings->setValue( MOTOR_RATIO, ratio );
    mSettings->setValue( WHEEL_RAD, wheel_rad_mm );
    mSettings->setValue( WHEELBASE, wheelbase_mm );
    mSettings->setValue( K_ANG, k_ang );
    mSettings->setValue( K_VEL, k_vel );
    mSettings->setValue( DIRECTION_L, versus_left );
    mSettings->setValue( DIRECTION_R, versus_right );
    mSettings->setValue( EN_POLARITY, enable_mode );

    mSettings->endGroup();

    mSettings->sync();

    return true;
}
