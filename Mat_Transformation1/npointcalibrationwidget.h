#ifndef NPOINTCALIBRATIONWIDGET_H
#define NPOINTCALIBRATIONWIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QDebug>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QTableWidget>
#include <QTextEdit>
#include <QHBoxLayout>
#include <QLabel>
#include <QButtonGroup>
#include <QFileDialog>
#include <QMessageBox>
#include "globlealgorithm.h"
class NPointCalibrationWidget : public QWidget
{
    Q_OBJECT
public:
    explicit NPointCalibrationWidget(QWidget *parent = nullptr);

signals:

protected slots:

    void slot_setClick();
    void slot_executeClick();
    void slot_openFileClick();
    void slot_tableWidgetItemChanged(QTableWidgetItem *item);

private:

    QPushButton *m_set;//设置
    QPushButton *m_execute;//执行
    QPushButton *m_openFile;//从文件加载数据
    QSpinBox *m_fitCount;//设置拟合个数
    QCheckBox *m_OLS;//最小二乘
    QCheckBox *m_IRLS;//迭代重加权最小二乘（IRLS）
    QSpinBox *m_iterationCount;//迭代个数
    QDoubleSpinBox *m_normCount;//Lp范数

    QTableWidget *m_tableWidget;//设置数据
    QTextEdit *m_textEdit;//显示记录

    QString old_text;
};

#endif // NPOINTCALIBRATIONWIDGET_H
