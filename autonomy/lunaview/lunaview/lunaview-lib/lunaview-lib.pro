QT -= gui
TARGET = lunaview-lib.pro
TEMPLATE = lib
CONFIG += c++17
DEFINES += LUNAVIEWLIB_LIBRARY
INCLUDEPATH += source

SOURCES += \
    source/models/client.cpp

HEADERS += \
    source/lunaview-lib_global.h \
    source/models/client.h

# Default rules for deployment.
unix {
    target.path = /usr/lib
}
!isEmpty(target.path): INSTALLS += target
