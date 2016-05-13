#-------------------------------------------------
#
# Project created by QtCreator 2016-05-11T19:00:41
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = MyProjectBpp
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    webcam.cpp \
    camerainterface.cpp \
    mykinect.cpp \
    kinectinterface.cpp

HEADERS  += mainwindow.h \
    webcam.h \
    camerainterface.h \
    mykinect.h \
    kinectinterface.h

FORMS    += mainwindow.ui
