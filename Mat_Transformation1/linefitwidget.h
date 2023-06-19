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
#include "globlealgorithm.h"

#define  ALIGN_NUM 10
#define  MORE QString("...")


#define DERIV_STEP 1e-5


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
    QCheckBox *m_GN;//高斯牛顿法（GaussNewton）
    QCheckBox *m_LM;//列文伯格-马夸尔特（Levenberg-Marquardt）
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


//线拟合残差值向量
class LineFitResidualsVector
{
public:
    Eigen::VectorXd  operator()(const Eigen::VectorXd& parameter,const QList<Eigen::MatrixXd> &otherArgs)
    {
        Eigen::MatrixXd inValue = otherArgs.at(0);
        Eigen::VectorXd outValue = otherArgs.at(1);
        int dataCount = inValue.rows();
        int paramsCount = parameter.rows();
        //保存残差值
        Eigen::VectorXd residual = Eigen::VectorXd::Zero(dataCount);
        //获取预测偏差值 r= ^y(预测值) - y(实际值)
        for(int i=0;i<dataCount;++i)
        {
            for(int j=0;j<paramsCount;++j)
            {
                //这里使用曲线方程 y = a1*x^0 + a2*x^1 + a3*x^2 + ...      根据参数(a1,a2,a3,...)个数来设置
                residual(i) += parameter(j) * inValue(i,j);
            }

        }

        return residual - outValue;
    }

};


//求线拟合雅克比矩阵 -- 通过计算求偏导
class LineFitJacobi
{
    //求偏导
    double PartialDeriv(const Eigen::VectorXd& parameter,int paraIndex,const Eigen::MatrixXd &inValue,int objIndex)
    {
        Eigen::VectorXd para1 = parameter;
        Eigen::VectorXd para2 = parameter;
        para1(paraIndex) -= DERIV_STEP;
        para2(paraIndex) += DERIV_STEP;

        //逻辑
        double obj1 = 0;
        double obj2 = 0;
        for(int i=0;i<parameter.rows();++i)
        {
            //这里使用曲线方程 y = a1*x^0 + a2*x^1 + a3*x^2 + ...      根据参数(a1,a2,a3,...)个数来设置
            obj1 += para1(i) * inValue(objIndex,i);
        }

        for(int i=0;i<parameter.rows();++i)
        {
            //这里使用曲线方程 y = a1*x^0 + a2*x^1 + a3*x^2 + ...      根据参数(a1,a2,a3,...)个数来设置
            obj2 += para2(i) * inValue(objIndex,i);
        }

        return (obj2 - obj1) / (2 * DERIV_STEP);
    }

public:

    Eigen::MatrixXd operator()(const Eigen::VectorXd& parameter,const QList<Eigen::MatrixXd> &otherArgs)
    {
        Eigen::MatrixXd inValue = otherArgs.at(0);
        int rowNum = inValue.rows();
        int paramsCount = parameter.rows();

        Eigen::MatrixXd Jac(rowNum, paramsCount);

        for (int i = 0; i < rowNum; i++)
        {
            for (int j = 0; j < paramsCount; j++)
            {
                Jac(i,j) = PartialDeriv(parameter,j,inValue,i);
            }
        }
        return Jac;
    }
};
#endif // LINEFITWIDGET_H
