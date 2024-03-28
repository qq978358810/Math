QT       += core gui
QT += printsupport
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
INCLUDEPATH += D:\Qt\eigen-3.4.0\eigen-3.4.0\Eigen
CONFIG += c++11

        #INCLUDEPATH += D:\OpenCV4.5.1\opencv\opencv-build\install\include

       # LIBS += D:\OpenCV4.5.1\opencv\opencv-build\lib\libopencv_*.a
# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
win32{
    QMAKE_CXXFLAGS += /source-charset:utf-8 /execution-charset:utf-8
}
SOURCES += \
    cameracalibrationalgorithm.cpp \
    cameracalibrationwidget.cpp \
    circlefit2widget.cpp \
    circlefit3widget.cpp \
    circlefitwidget.cpp \
    globlealgorithm.cpp \
    imagecorrectionwidget.cpp \
    linefitwidget.cpp \
    main.cpp \
    mainwindow.cpp \
    npointcalibrationwidget.cpp \
    qcustomplot.cpp \
    viewtransform.cpp

HEADERS += \
    cameracalibrationalgorithm.h \
    cameracalibrationwidget.h \
    circlefit2widget.h \
    circlefit3widget.h \
    circlefitwidget.h \
    globlealgorithm.h \
    imagecorrectionwidget.h \
    linefitwidget.h \
    mainwindow.h \
    npointcalibrationwidget.h \
    qcustomplot.h \
    viewtransform.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
