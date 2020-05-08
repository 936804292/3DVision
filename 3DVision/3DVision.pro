#-------------------------------------------------
#
# Project created by QtCreator 2020-04-30T09:40:10
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = 3DVision
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
    MyCloud.cpp \
    Tools.cpp \
    MeshProcessing.cpp \
    FileIO.cpp \
    CloudViewer.cpp \
    MyAlgorithm.cpp

HEADERS += \
    MyCloud.h \
    Tools.h \
    MeshProcessing.h \
    GBK.h \
    FileIO.h \
    CloudViewer.h \
    MyAlgorithm.h

FORMS += \
    CloudViewer.ui \
    CloudViewer.ui

INCLUDEPATH += /usr/include/eigen3

INCLUDEPATH += /usr/include/vtk-6.3
LIBS += /usr/lib/x86_64-linux-gnu/libvtk*.so

INCLUDEPATH += /usr/include/boost
LIBS += /usr/lib/x86_64-linux-gnu/libboost_*.so

#INCLUDEPATH += /usr/include/pcl-1.9
INCLUDEPATH += /usr/include/pcl-1.8
LIBS += /usr/lib/x86_64-linux-gnu/libpcl_*.so

win32: {
    include("e:/opencv/opencv.pri")
}

unix: !macx{
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv
}

unix: macx{
INCLUDEPATH += /usr/local/include
LIBS += -L"/usr/local/lib" \
    -lopencv_world
}

DISTFILES += \
    Resources/images/Calibration.png \
    Resources/images/Camera.png \
    Resources/images/Clean.png \
    Resources/images/CN.png \
    Resources/images/Email.png \
    Resources/images/EN.png \
    Resources/images/Exit.png \
    Resources/images/Floder.png \
    Resources/images/Help.png \
    Resources/images/Language.png \
    Resources/images/Save.png \
    Resources/images/Setting.png \
    Resources/images/Theme.png \
    Resources/clouds/rabbit.pcd \
    Resources/clouds/table.pcd \
    Resources/clouds/zz.ply \
    Resources/qss/Darcula.qss \
    Resources/qss/Windows.qss \
    Resources/images/Calibration.png \
    Resources/images/Camera.png \
    Resources/images/Clean.png \
    Resources/images/CN.png \
    Resources/images/Email.png \
    Resources/images/EN.png \
    Resources/images/Exit.png \
    Resources/images/Floder.png \
    Resources/images/Help.png \
    Resources/images/Language.png \
    Resources/images/Save.png \
    Resources/images/Setting.png \
    Resources/images/Theme.png \
    Resources/clouds/rabbit.pcd \
    Resources/clouds/table.pcd \
    Resources/clouds/zz.ply \
    Resources/qss/Darcula.qss \
    Resources/qss/Windows.qss \
    Resources/images/logo.png

RESOURCES += \
    resources.qrc
