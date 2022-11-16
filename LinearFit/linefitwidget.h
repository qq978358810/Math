#ifndef LINEFITWIDGET_H
#define LINEFITWIDGET_H

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
#include <QComboBox>
#include <QInputDialog>
#include "qcustomplot.h"
#include "globlefunction.h"

#define  ALIGN_NUM 10
#define  MORE QString("...")
class LineFitWidget : public QWidget
{
    Q_OBJECT
public:
    explicit LineFitWidget(QWidget *parent = nullptr);

    bool eventFilter(QObject *watched, QEvent *event) ;
    //获取圆的中心和半径,---多点拟合
    QList<double> get_Line(QList<QPointF> listP);

    //绘制点
    void draw_PointF(QSharedPointer<QCPGraphDataContainer> &);

    //绘制线
    void draw_Line(QSharedPointer<QCPGraphDataContainer> &);
signals:

protected slots:

    void slot_clearCustomPlotClick();
    void slot_zoomInClick();
    void slot_zoomOutClick();
    void slot_originClick();
    void slot_setClick();
    void slot_executeClick();
    void slot_activated(const QString &text);


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
    QComboBox *m_comboBox;//保存公式

    qint32 m_maxPower;//保存最大幂项

    QSharedPointer<QCPGraphDataContainer> m_globlePoint;//点
    QList<QCPCurve *> m_itemLine;//画线类

    QSize m_customPlotOldSize; //保存窗口变化前大小
};

#endif // LINEFITWIDGET_H
