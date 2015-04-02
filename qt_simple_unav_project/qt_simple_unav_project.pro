#-------------------------------------------------
#
# Project created by QtCreator 2015-04-01T17:58:00
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

include(../../orblibcpp/orblibcpp/orblibcpp.pri)

TARGET = qt_simple_unav_project
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    joypad/qjoypad.cpp

HEADERS  += mainwindow.h \
    joypad/qjoypad.h

FORMS    += mainwindow.ui

RESOURCES += \
    resources.qrc
