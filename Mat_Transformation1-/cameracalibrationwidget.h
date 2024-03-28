#ifndef CAMERACALIBRATIONWIDGET_H
#define CAMERACALIBRATIONWIDGET_H

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
#include <QLineEdit>
#include <QButtonGroup>
#include <QFileDialog>
#include <QMessageBox>
#include <QRegExp>
#include <QComboBox>
#include "globlealgorithm.h"
#include "cameracalibrationalgorithm.h"
class CameraCalibrationWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CameraCalibrationWidget(QWidget *parent = nullptr);

    //生成数据表格
    void createDataTable(int imageCount,int pointCount);

signals:

protected slots:

    void slot_loadToTable();
    void slot_setClick();
    void slot_executeClick();
    void slot_tableWidgetItemChanged(QTableWidgetItem *item);

private:

    QPushButton *m_set;//设置
    QPushButton *m_execute;//执行
    QSpinBox *m_pointCount;//设置每张图片点个数
    QSpinBox *m_imagesCount;//设置图片个数
    QLineEdit *m_splitChar;//分割符
    QPushButton *m_load;//加载到表格
    QCheckBox *m_row;//按行加载数据
    QCheckBox *m_col;//按列加载数据
    QComboBox *m_comboBoxDis;//畸变系数个数
    QComboBox *m_comboBoxInP;//内参个数
    QSpinBox *m_iterationCount;//迭代个数

    QTableWidget *m_tableWidget;//设置数据
    QTextEdit *m_addTextEdit;//添加数据记录
    QTextEdit *m_showTextEdit;//显示记录

    QString old_text;
};
#endif // CAMERACALIBRATIONWIDGET_H
