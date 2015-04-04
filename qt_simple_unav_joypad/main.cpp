#include "mainwindow.h"
#include <QApplication>
#include "csettings.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    g_settings = new CSettings();

    MainWindow w;
    w.show();

    return a.exec();
}
