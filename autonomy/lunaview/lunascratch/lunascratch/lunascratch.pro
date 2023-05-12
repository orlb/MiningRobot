TEMPLATE = app

QT += core gui qml quick widgets printsupport

TARGET = lunascratch

CONFIG += c++17

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    qcustomplot.cpp

HEADERS += \
    mainwindow.h \
    qcustomplot.h

RESOURCES += qml.qrc

FORMS += \
    mainwindow.ui
