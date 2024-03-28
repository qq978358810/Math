#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "circlefitwidget.h"
#include "linefitwidget.h"
#include "circlefit2widget.h"
#include "circlefit3widget.h"

#include "npointcalibrationwidget.h"
#include "cameracalibrationwidget.h"
#include "imagecorrectionwidget.h"
#include "viewtransform.h"
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


private slots:

private:
    Ui::MainWindow *ui;

    CircleFitWidget * m_circleFitWidget;
    CircleFit2Widget  * m_circle2FitWidget;
    CircleFit3Widget * m_circle3FitWidget;
    LineFitWidget * m_lineFitWidget;
    NPointCalibrationWidget * m_NPointCalibrationWidget;
    CameraCalibrationWidget * m_cameraCalibrationWidget;
    ImageCorrectionWidget * m_imageCorrectionWidget;
    ViewTransform* m_viewTransform;
};
#endif // MAINWINDOW_H
