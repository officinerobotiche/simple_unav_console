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

    bool getParams(float& wheel_radius, float& wheel_base, float& ratio,
                   qint8& versus_left, qint8 &versus_right, quint8& enable_mode , quint8 &enc_pos, qint16 &bridge_V);

private slots:
    void on_buttonBox_accepted();
    void on_lineEdit_enc_cpr_textEdited(const QString &arg1);
    void on_lineEdit_motor_ratio_textEdited(const QString &arg1);



private:
    Ui::RobotParamsCalculateDialog *ui;
};

#endif // ROBOTPARAMSCALCULATEDIALOG_H
