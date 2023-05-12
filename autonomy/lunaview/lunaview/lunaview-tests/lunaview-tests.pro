QT += testlib
QT -= gui
TARGET = client-tests
TEMPLATE = app

CONFIG += c++17
CONFIG += qt console warn_on depend_includepath testcase
CONFIG -= app_bundle

INCLUDEPATH += source

SOURCES +=  \
    source/models/client-tests.cpp
