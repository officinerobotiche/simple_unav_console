#include "mainwindow.h"
#include <QApplication>
#include "csettings.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    g_settings = new CSettings();

    MainWindow w;
    w.show();

    int res;
    try
    {
        res = a.exec();
    }
    catch(...)
    {
        delete g_settings;
        exit( EXIT_FAILURE );
    }

    delete g_settings;

    return res;
}
