#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
using namespace std;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_circleFitWidget = new CircleFitWidget(ui->tabWidget);
    ui->tabWidget->addTab(m_circleFitWidget,QString("拟合圆"));
    m_circle2FitWidget  = new CircleFit2Widget(ui->tabWidget);
    ui->tabWidget->addTab(m_circle2FitWidget,QString("拟合圆2"));
    m_circle3FitWidget  = new CircleFit3Widget(ui->tabWidget);
    ui->tabWidget->addTab(m_circle3FitWidget,QString("拟合圆3"));
    m_lineFitWidget = new LineFitWidget(ui->tabWidget);
    ui->tabWidget->addTab(m_lineFitWidget,QString("拟合线"));
    m_NPointCalibrationWidget = new NPointCalibrationWidget(ui->tabWidget);
    ui->tabWidget->addTab(m_NPointCalibrationWidget,QString("N点标定"));
    m_cameraCalibrationWidget = new CameraCalibrationWidget(ui->tabWidget);
    ui->tabWidget->addTab(m_cameraCalibrationWidget,QString("单目相机标定"));
    m_imageCorrectionWidget = new ImageCorrectionWidget(ui->tabWidget);
    ui->tabWidget->addTab(m_imageCorrectionWidget,QString("图片矫正"));
    m_viewTransform = new ViewTransform(ui->tabWidget);
    ui->tabWidget->addTab(m_viewTransform,QString("图像视角转换(暂未)"));
}

MainWindow::~MainWindow()
{
    delete ui;
}


