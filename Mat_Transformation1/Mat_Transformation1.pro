QT       += core gui
QT += printsupport
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

QMAKE_CXXFLAGS+=/openmp  #导入支持OpenMp库
#CONFIG += c++11

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
    curvedwidget.cpp \
    globlealgorithm.cpp \
    hyperboloidmapping.cpp \
    imagecorrectionwidget.cpp \
    imageview.cpp \
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
    curvedwidget.h \
    globlealgorithm.h \
    hyperboloidmapping.h \
    imagecorrectionwidget.h \
    imageview.h \
    linefitwidget.h \
    mainwindow.h \
    npointcalibrationwidget.h \
    qcustomplot.h \
    viewtransform.h

FORMS += \
    curvedwidget.ui \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target




#添加Eigen库
INCLUDEPATH += D:\Qt\eigen-3.4.0\eigen-3.4.0
#添加Eigen 扩展库
INCLUDEPATH += D:\Qt\eigen-3.4.0\eigen-3.4.0\unsupported

##添加autodiff库 注意：需要 使用c++17

#INCLUDEPATH += ..\autodiff\autodiff-1.0.3

#OpenCV 4.8.1
win32:CONFIG(release, debug|release): LIBS += -LD:/OpenCV4.8.1/BUILD_VS2019_X64_GPU_CUDA10.1/install/x64/vc16/lib/ -lopencv_world481
else:win32:CONFIG(debug, debug|release): LIBS += -LD:/OpenCV4.8.1/BUILD_VS2019_X64_GPU_CUDA10.1/install/x64/vc16/lib/ -lopencv_world481d
INCLUDEPATH += D:/OpenCV4.8.1/BUILD_VS2019_X64_GPU_CUDA10.1/install/include
DEPENDPATH += D:/OpenCV4.8.1/BUILD_VS2019_X64_GPU_CUDA10.1/install/include
