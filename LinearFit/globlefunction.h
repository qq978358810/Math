#ifndef GLOBLEFUNCTION_H
#define GLOBLEFUNCTION_H

#include <QObject>
#include <Eigen>
#include <QPointF>
#include <QtMath>
#include <QThread>
using namespace Eigen;



//保存从界面获取的坐标数据
struct _coordList{
    QList<QPointF> pixPoines;//保存像素坐标
    QList<QPointF> physicsPoines;//保存物理坐标
};

//保存最小二乘法三个未知量(a,b,c)的结果
struct _squareLaw{
    double aa=0;
    double bb=0;
    double cc=0;
    double ab=0;
    double ac=0;
    double bc=0;
    double _a=0;
    double _b=0;
    double _c=0;
};
//导数3x4
struct _differentialCoefficient{
    QList<double> diff_a;
    QList<double> diff_b;
    QList<double> diff_c;
};

//矩阵3X3
struct _mat{
    double a=0;
    double b=0;
    double c=0;
    double d=0;
    double e=0;
    double f=0;
    double g=0;
    double h=0;
    double i=0;
};












//保存圆的参数
struct _arcConfig{
    double raio;//保存半径
    QPointF center;//保存圆心
    QPointF center2;//保存圆心2
};

//保存线的参数
struct _lineConfig{
    double a;//直线斜率
    double b;//直线截距
};


class GlobleFunction
{

public:
    explicit GlobleFunction();

    //获取圆的中心和半径，---两点和两点之间的角度 -- 代数推导
    static _arcConfig get_ArcCenter2(QPointF,QPointF,double rad);

    //获取圆的中心和半径，---两点和两点之间的角度 -- 几何推导
    static _arcConfig get_ArcCenter3(QPointF,QPointF,double rad);


    //以下算法只适用于超定方程组 方程个数大于未知数

    /* 普通最小二乘(OLS) Ax = B
     * (A^T * A) * x = A^T * B
     * x = (A^T * A)^-1 * A^T * B
     */
    static Array<double,Dynamic,1> leastSquares(Matrix<double,Dynamic,Dynamic> A, Matrix<double,Dynamic,1> B);

    /* 加权最小二乘（WLS）  W为对角线矩阵
     * W²(Ax - B) = 0
     * W²Ax = W²B
     * (A^T * W^T * W * A) * x = A^T * W^T * W * B
     * x = (A^T * W^T * W * A)^-1 * A^T * W^T * W * B
     */
    static Array<double,Dynamic,1> reweightedLeastSquares(Matrix<double,Dynamic,Dynamic> A, Matrix<double,Dynamic,1> B,Array<double,Dynamic,1> vectorW = Array<double,Dynamic,1>());

    /* 迭代重加权最小二乘（IRLS）  W为为对角线矩阵,p为范数
     * e = Ax - B
     * W = e^(p−2)/2
     * W²(Ax - B) = 0
     * W²Ax = W²B
     * (A^T * W^T * W * A) * x = A^T * W^T * W * B
     * x = (A^T * W^T * W * A)^-1 * A^T * W^T * W * B
     * 参考论文地址：https://www.semanticscholar.org/paper/Iterative-Reweighted-Least-Squares-%E2%88%97-Burrus/9b9218e7233f4d0b491e1582c893c9a099470a73
     */
    static Array<double,Dynamic,1> iterativeReweightedLeastSquares(Matrix<double,Dynamic,Dynamic> A, Matrix<double,Dynamic,1> B,double p = 2 ,int kk = 1);



    //N点标定算法
    static _mat calibration(_coordList);
};

#endif // GLOBLEFUNCTION_H
