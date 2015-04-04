#ifndef ROBOTPARAMSCALCULATEDIALOG_H
#define ROBOTPARAMSCALCULATEDIALOG_H

#include <QDialog>

namespace Ui {
class RobotParamsCalculateDialog;
}

class RobotParamsCalculateDialog : public QDialog
{
    Q_OBJECT

public:
    explicit RobotParamsCalculateDialog(QWidget *parent = 0);
    ~RobotParamsCalculateDialog();

    bool getParams(double &k_ang, double &k_vel , qint8 &versus_left, qint8 &versus_right, quint8 &enable_mode);

private slots:
    void on_pushButton_clicked();

    void on_buttonBox_accepted();

    void on_lineEdit_enc_cpr_textEdited(const QString &arg1);

    void on_lineEdit_motor_ratio_textEdited(const QString &arg1);

private:
    Ui::RobotParamsCalculateDialog *ui;

    double _k_ang;
    double _k_vel;
    bool _calculated;
};

#endif // ROBOTPARAMSCALCULATEDIALOG_H
