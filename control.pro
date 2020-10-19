TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt


SOURCES += \
    main.cpp \
    src/lidarscanner.cpp \
    src/fuzzycontrol.cpp \
    src/location.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv4

INCLUDEPATH += /home/kristian/Documents/sdu/fuzzylite-6.0/fuzzylite/

LIBS += /home/kristian/Documents/sdu/fuzzylite-6.0/fuzzylite/release/bin/libfuzzylite-static.a
LIBS += /home/kristian/Documents/sdu/fuzzylite-6.0/fuzzylite/release/bin/libfuzzylite.so.6.0
LIBS += /home/kristian/Documents/sdu/fuzzylite-6.0/fuzzylite/release/bin/libfuzzylite.so

HEADERS += \
    inc/lidarscanner.h \
    inc/fuzzycontrol.h \
    inc/location.h

DISTFILES += \
    obstacleAvoidance.fll

