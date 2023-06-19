#ifndef CIRCLEFITWIDGET_H
#define CIRCLEFITWIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QPaintEvent>
#include <QDebug>
#include <QPalette>
#include <QPainter>
#include <QWheelEvent>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QTableWidget>
#include <QTextEdit>
#include <QHBoxLayout>
#include <QLabel>
#include <QButtonGroup>
#include "qcustomplot.h"
#include "globlealgorithm.h"

#define  ALIGN_NUM 10
class CircleFitWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CircleFitWidget(QWidget *parent = nullptr);

    bool eventFilter(QObject *watched, QEvent *event) ;
    //获取圆的中心和半径,---多点拟合
    _arcConfig get_ArcCenter(QList<QPointF>);

    //绘制点
    void draw_PointF(QSharedPointer<QCPGraphDataContainer> &);

    //绘制圆
    void draw_Ellipse(QSharedPointer<QCPGraphDataContainer> &);
signals:

protected slots:

    void slot_clearCustomPlotClick();
    void slot_zoomInClick();
    void slot_zoomOutClick();
    void slot_originClick();
    void slot_setClick();
    void slot_executeClick();


private:
    QPushButton *m_clearCustomPlot;//清除界面
    QPushButton *m_zoomIn;//放大
    QPushButton *m_zoomOut;//缩小
    QPushButton *m_origin;//原点
    QPushButton *m_set;//设置
    QPushButton *m_execute;//执行
    QSpinBox *m_fitCount;//设置拟合个数
    QCheckBox *m_OLS;//最小二乘
    QCheckBox *m_IRLS;//迭代重加权最小二乘（IRLS）
    QSpinBox *m_iterationCount;//迭代个数
    QDoubleSpinBox *m_normCount;//Lp范数

    QTableWidget *m_tableWidget;//设置数据
    QTextEdit *m_textEdit;//显示记录
    QCustomPlot *m_customPlot;//显示绘图

    QSharedPointer<QCPGraphDataContainer> m_globlePoint;//点坐标
    QList<QCPItemEllipse *> m_itemEllipse;//画圆类

    QSize m_customPlotOldSize; //保存窗口变化前大小
};


#endif // CIRCLEFITWIDGET_H
