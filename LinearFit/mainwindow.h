#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "circlefitwidget.h"
#include "linefitwidget.h"
#include "circlefit2widget.h"
#include "npointcalibrationwidget.h"

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
    LineFitWidget * m_lineFitWidget;
    NPointCalibrationWidget * m_NPointCalibrationWidget;

};
#endif // MAINWINDOW_H
