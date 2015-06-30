#ifndef CSETTINGS_H
#define CSETTINGS_H

#include <QSettings>

#define MOTOR_GROUP "Motor"
#define ENCODER_CPR "encoder_cpr"
#define MOTOR_RATIO "motor_ratio"
#define WHEEL_RAD   "wheel_radius_mm"
#define WHEELBASE   "wheelbase_mm"
#define EN_POLARITY "enable_polarity"
#define DIRECTION_L "direction_left"
#define DIRECTION_R "direction_right"
#define K_VEL       "k_vel"
#define K_ANG       "k_ang"

class CSettings
{
public:
    CSettings();
    ~CSettings();

public:
    bool saveMotorParams(int cpr, double ratio, double wheel_rad_mm, double wheelbase_mm,
                         double k_ang, double k_vel , qint8 versus_left, qint8 versus_right, quint8 enable_mode );
    bool loadMotorParams(int& cpr, double &ratio, double &wheel_rad_mm, double &wheelbase_mm,
                          double& k_ang, double& k_vel , qint8& versus_left, qint8& versus_right,
                          quint8& enable_mode );


private:
    QSettings* mSettings;
    QString mSettingsFile;
};

extern CSettings* g_settings;

#endif // CSETTINGS_H
