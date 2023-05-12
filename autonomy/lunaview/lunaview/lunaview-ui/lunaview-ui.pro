QT += qml quick
TEMPLATE = app

CONFIG += c++17

INCLUDEPATH += source

SOURCES += \
    source/main.cpp

RESOURCES += views.qrc

QML_IMPORT_PATH = $$PWD
