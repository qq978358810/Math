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
    m_lineFitWidget = new LineFitWidget(ui->tabWidget);
    ui->tabWidget->addTab(m_lineFitWidget,QString("拟合线"));
    m_NPointCalibrationWidget = new NPointCalibrationWidget(ui->tabWidget);
    ui->tabWidget->addTab(m_NPointCalibrationWidget,QString("N点标定"));
}

MainWindow::~MainWindow()
{
    delete ui;
}


