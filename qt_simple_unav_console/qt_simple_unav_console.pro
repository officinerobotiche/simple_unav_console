#-------------------------------------------------
#
# Project created by QtCreator 2015-04-01T17:58:00
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

include(../../orblibcpp/orblibcpp/orblibcpp.pri)

TARGET = qt_simple_unav_console
TEMPLATE = app


SOURCES += \
    main.cpp\
    mainwindow.cpp \
    joypad/qjoypad.cpp \
    csettings.cpp \
    robotparamscalculatedialog.cpp \
    OrblibcppInterface/orblibcppinterface.cpp

HEADERS  += \
    mainwindow.h \
    joypad/qjoypad.h \
    csettings.h \
    robotparamscalculatedialog.h \
    OrblibcppInterface/orblibcppinterface.h

FORMS    += mainwindow.ui \
    robotparamscalculatedialog.ui

RESOURCES += \
    resources.qrc
