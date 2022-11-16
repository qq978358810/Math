#include "globlefunction.h"

#include <QDebug>
GlobleFunction::GlobleFunction()
{

}

//获取圆的中心和半径，---两点和两点之间的角度 -- 代数推导
_arcConfig GlobleFunction::get_ArcCenter2(QPointF point1,QPointF point2,double rad)
{
    /* 存在两个解：
     * 解1：
     * (1-cosβ)a + sinβ*b = x2 - x1*cosβ + y1*sinβ
     * sinβ*a + (cosβ-1)b = -y2 + x1*sinβ + y1*cosβ
     * 解2：
     * * (1-cosβ)a + sinβ*b = x1 - x2*cosβ + y2*sinβ
     * sinβ*a + (cosβ-1)b = -y1 + x2*sinβ + y2*cosβ
     * 利用克拉默法则解行列式
     * 求center(a,b)
     *
     */
    QPointF center,center2;
    _arcConfig config;
    Matrix<double,2,2> matD,matD1,matD2,matD3,matD4;
    matD<<(1 - qCos(rad)),qSin(rad),
            qSin(rad),(qCos(rad)-1);

    matD1<<point2.x() - (point1.x()*qCos(rad))+(point1.y()*qSin(rad)) ,  qSin(rad),
            point1.x()*qSin(rad) + point1.y()*qCos(rad) -point2.y() , (qCos(rad)-1);
    matD2<<(1 - qCos(rad )),point2.x() - (point1.x()*qCos(rad))+(point1.y()*qSin(rad)),
            qSin(rad ),point1.x()*qSin(rad) + point1.y()*qCos(rad) -point2.y() ;

    matD3<<point1.x() - (point2.x()*qCos(rad))+(point2.y()*qSin(rad)) ,  qSin(rad),
            point2.x()*qSin(rad) + point2.y()*qCos(rad) -point1.y() , (qCos(rad)-1);
    matD4<<(1 - qCos(rad )),point1.x() - (point2.x()*qCos(rad))+(point2.y()*qSin(rad)),
            qSin(rad ),point2.x()*qSin(rad) + point2.y()*qCos(rad) -point1.y() ;

    double D = matD.determinant();
    double D1 = matD1.determinant();
    double D2 = matD2.determinant();
    double D3 = matD3.determinant();
    double D4 = matD4.determinant();
    center.setX(D1 / D);
    center.setY(D2 / D);
    center2.setX(D3 / D);
    center2.setY(D4 / D);

    double raio = qSqrt(qPow(point1.x()-center.x(),2)+qPow(point1.y()-center.y(),2));

    config.raio =raio;
    config.center = center;
    config.center2 = center2;

    return config;

}
//获取圆的中心和半径，---两点和两点之间的角度 -- 几何推导
_arcConfig GlobleFunction::get_ArcCenter3(QPointF point1,QPointF point2,double rad)
{
    /* 存在两个解：
     * 解1：
     * x = (x1+x2)/2 + (y1-y2) / 2tan(β/2)
     * y = (y1+y2)/2 + (x2-x1) / 2tan(β/2)
     * 解2：
     * x = (x1+x2)/2 - (y1-y2) / 2tan(β/2)
     * y = (y1+y2)/2 - (x2-x1) / 2tan(β/2)
     *
     * 直接求解
     *
     */
    QPointF center,center2;
    _arcConfig config;


    center.setX((point1.x() + point2.x()) / 2 + (point1.y()-point2.y()) / (2*qTan(rad/2)));
    center.setY((point1.y() + point2.y()) / 2 + (point2.x()-point1.x()) / (2*qTan(rad/2)));

    center2.setX((point1.x() + point2.x()) / 2 - (point1.y()-point2.y()) / (2*qTan(rad/2)));
    center2.setY((point1.y() + point2.y()) / 2 - (point2.x()-point1.x()) / (2*qTan(rad/2)));
    double raio = qSqrt(qPow(point1.x()-center.x(),2)+qPow(point1.y()-center.y(),2));

    config.raio =raio;
    config.center = center;
    config.center2 = center2;

    return config;
}
/* 普通最小二乘 Ax = B
 * (A^T * A) * x = A^T * B
 * x = (A^T * A)^-1 * A^T * B
 */
Array<double,Dynamic,1> GlobleFunction::leastSquares(Matrix<double,Dynamic,Dynamic> A, Matrix<double,Dynamic,1> B)
{
    //获取矩阵的行数和列数
    int rows =  A.rows();
    int col = A.cols();
    //A的转置矩阵
    Matrix<double,Dynamic,Dynamic> AT;
    AT.resize(col,rows);

    //x矩阵
    Array<double,Dynamic,1> x;
    x.resize(col,1);

    //转置 AT
    AT = A.transpose();

    //x = (A^T * A)^-1 * A^T * B
    x = ((AT * A).inverse()) * (AT * B);
    return x;

}
/* 加权最小二乘（WLS）  W为对角线矩阵
 * W²(Ax - B) = 0
 * W²Ax = W²B
 * (A^T * W^T * W * A) * x = A^T * W^T * W * B
 * x = (A^T * W^T * W * A)^-1 * A^T * W^T * W * B
 */
Array<double,Dynamic,1> GlobleFunction::reweightedLeastSquares(Matrix<double,Dynamic,Dynamic> A, Matrix<double,Dynamic,1> B,Array<double,Dynamic,1> vectorW)
{
    //获取矩阵的行数和列数
    int rows =  A.rows();
    int col = A.cols();
    //vectorW为空,默认构建对角线矩阵1
    //qDebug()<<"vectorW.isZero():"<<vectorW.isZero();
    if(vectorW.isZero())
    {
        vectorW.resize(rows,1);
        for(int i=0;i<rows;++i)
        {
            vectorW(i,0) = 1;
        }
    }



    //A的转置矩阵
    Matrix<double,Dynamic,Dynamic> AT;
    AT.resize(col,rows);

    //x矩阵
    Array<double,Dynamic,1> x;
    x.resize(col,1);

    //W的转置矩阵
    Matrix<double,Dynamic,Dynamic> WT,W;
    W.resize(rows,rows);
    WT.resize(rows,rows);

    //生成对角线矩阵
    W = vectorW.matrix().asDiagonal();
    //转置
    WT = W.transpose();
    //转置 AT
    AT = A.transpose();

    // x = (A^T * W^T * W * A)^-1 * A^T * W^T * W * B
    x = ((AT * WT * W * A).inverse()) * (AT * WT * W * B);
    return x;
}

/* 迭代重加权最小二乘（IRLS）  W为权重,p为范数
 * e = Ax - B
 * W = e^(p−2)/2
 * W²(Ax - B) = 0
 * W²Ax = W²B
 * (A^T * W^T * W * A) * x = A^T * W^T * W * B
 * x = (A^T * W^T * W * A)^-1 * A^T * W^T * W * B
 * 参考论文地址：https://www.semanticscholar.org/paper/Iterative-Reweighted-Least-Squares-%E2%88%97-Burrus/9b9218e7233f4d0b491e1582c893c9a099470a73
 */
Array<double,Dynamic,1> GlobleFunction::iterativeReweightedLeastSquares(Matrix<double,Dynamic,Dynamic> A, Matrix<double,Dynamic,1> B,double p,int kk)
{


    /* x(k) = q x1(k) + (1-q)x(k-1)
     * q = 1 / (p-1)
     */
    //获取矩阵的行数和列数
    int rows =  A.rows();
    int col = A.cols();

    double pk = 2;//初始同伦值
    double K = 1.5;

    double epsilong = 10e-9; // ε
    double delta = 10e-15; // δ
    Array<double,Dynamic,1> x,_x,x1,e,w;
    x.resize(col,1);
    _x.resize(col,1);
    x1.resize(col,1);
    e.resize(rows,1);
    w.resize(rows,1);
    //初始x  对角矩阵w=1
    x = reweightedLeastSquares(A,B);

    //迭代  最大迭代次数kk
    for(int i=0;i<kk;++i)
    {
        //保留前一个x值,用作最后比较确定收敛
        _x = x;

        if(p>=2)
        {
            pk = qMin(p,K*pk);
        }
        else
        {
            pk = qMax(p,K*pk);
        }
        //偏差
        e = (A * x.matrix()) - B;
        //偏差的绝对值//  求矩阵绝对值 ：e = e.cwiseAbs(); 或 e.array().abs().matrix()
        e = e.abs();
        //对每个偏差值小于delta,用delta赋值给它
        for(int i=0;i<e.rows();++i)
        {
            e(i,0) = qMax(delta,e(i,0));
        }
        //对每个偏差值进行幂操作
        w = e.pow(p/2.0-1);
        w = w / w.sum();

        x1 = reweightedLeastSquares(A,B,w);

        double q = 1 / (pk-1);
        if(p>2)
        {
            x = x1*q + x*(1-q);
        }
        else
        {
            x = x1;
        }
        //达到精度,结束
        if((x-_x).abs().sum()<epsilong)
        {
            return x;
        }
    }
    return x;

}


//标定
_mat GlobleFunction::calibration(_coordList coordL)
{
    //保存未知量（a,b,c）的系数
    _squareLaw squareLaw_x;
    _squareLaw squareLaw_y;
    for(int i=0;i<coordL.pixPoines.count();i++) {
        squareLaw_x.aa += pow(coordL.pixPoines.at(i).x(),2)*2;
        squareLaw_x.bb += pow(coordL.pixPoines.at(i).y(),2)*2;
        squareLaw_x.cc += 1*2;
        squareLaw_x.ab += coordL.pixPoines.at(i).x()*coordL.pixPoines.at(i).y()*2;
        squareLaw_x.ac += coordL.pixPoines.at(i).x()*2;
        squareLaw_x.bc += coordL.pixPoines.at(i).y()*2;
        squareLaw_x._a += coordL.pixPoines.at(i).x()*coordL.physicsPoines.at(i).x()*2;
        squareLaw_x._b += coordL.pixPoines.at(i).y()*coordL.physicsPoines.at(i).x()*2;
        squareLaw_x._c += coordL.physicsPoines.at(i).x()*2;
    }

    for(int i=0;i<coordL.pixPoines.count();i++) {
        squareLaw_y.aa += pow(coordL.pixPoines.at(i).x(),2)*2;
        squareLaw_y.bb += pow(coordL.pixPoines.at(i).y(),2)*2;
        squareLaw_y.cc += 1*2;
        squareLaw_y.ab += coordL.pixPoines.at(i).x()*coordL.pixPoines.at(i).y()*2;
        squareLaw_y.ac += coordL.pixPoines.at(i).x()*2;
        squareLaw_y.bc += coordL.pixPoines.at(i).y()*2;
        squareLaw_y._a += coordL.pixPoines.at(i).x()*coordL.physicsPoines.at(i).y()*2;
        squareLaw_y._b += coordL.pixPoines.at(i).y()*coordL.physicsPoines.at(i).y()*2;
        squareLaw_y._c += coordL.physicsPoines.at(i).y()*2;
    }

    //求导数  3元方程组系数
    _differentialCoefficient diff_x;
    _differentialCoefficient diff_y;
    diff_x.diff_a.append(squareLaw_x.aa);
    diff_x.diff_a.append(squareLaw_x.ab);
    diff_x.diff_a.append(squareLaw_x.ac);
    diff_x.diff_a.append(squareLaw_x._a);
    diff_x.diff_b.append(squareLaw_x.ab);
    diff_x.diff_b.append(squareLaw_x.bb);
    diff_x.diff_b.append(squareLaw_x.bc);
    diff_x.diff_b.append(squareLaw_x._b);
    diff_x.diff_c.append(squareLaw_x.ac);
    diff_x.diff_c.append(squareLaw_x.bc);
    diff_x.diff_c.append(squareLaw_x.cc);
    diff_x.diff_c.append(squareLaw_x._c);

    diff_y.diff_a.append(squareLaw_y.aa);
    diff_y.diff_a.append(squareLaw_y.ab);
    diff_y.diff_a.append(squareLaw_y.ac);
    diff_y.diff_a.append(squareLaw_y._a);
    diff_y.diff_b.append(squareLaw_y.ab);
    diff_y.diff_b.append(squareLaw_y.bb);
    diff_y.diff_b.append(squareLaw_y.bc);
    diff_y.diff_b.append(squareLaw_y._b);
    diff_y.diff_c.append(squareLaw_y.ac);
    diff_y.diff_c.append(squareLaw_y.bc);
    diff_y.diff_c.append(squareLaw_y.cc);
    diff_y.diff_c.append(squareLaw_y._c);

    //解方程 使用克莱默法则解方程
    //a11​a22​a33​+a12​a23​a31​+a13​a21​a32​ −a13​a22​a31​−a11​a23​a32​−a12​a21​a33​

    double D_x,Da,Db,Dc, D_y,Dd,De,Df;
    D_x = diff_x.diff_a.at(0)*diff_x.diff_b.at(1)*diff_x.diff_c.at(2) + diff_x.diff_a.at(1)*diff_x.diff_b.at(2)*diff_x.diff_c.at(0) +diff_x.diff_a.at(2)*diff_x.diff_b.at(0)*diff_x.diff_c.at(1)
            -diff_x.diff_a.at(2)*diff_x.diff_b.at(1)*diff_x.diff_c.at(0) - diff_x.diff_a.at(0)*diff_x.diff_b.at(2)*diff_x.diff_c.at(1) -diff_x.diff_a.at(1)*diff_x.diff_b.at(0)*diff_x.diff_c.at(2);
    Da = diff_x.diff_a.at(3)*diff_x.diff_b.at(1)*diff_x.diff_c.at(2) + diff_x.diff_a.at(1)*diff_x.diff_b.at(2)*diff_x.diff_c.at(3) +diff_x.diff_a.at(2)*diff_x.diff_b.at(3)*diff_x.diff_c.at(1)
            -diff_x.diff_a.at(2)*diff_x.diff_b.at(1)*diff_x.diff_c.at(3) - diff_x.diff_a.at(3)*diff_x.diff_b.at(2)*diff_x.diff_c.at(1) -diff_x.diff_a.at(1)*diff_x.diff_b.at(3)*diff_x.diff_c.at(2);
    Db = diff_x.diff_a.at(0)*diff_x.diff_b.at(3)*diff_x.diff_c.at(2) + diff_x.diff_a.at(3)*diff_x.diff_b.at(2)*diff_x.diff_c.at(0) +diff_x.diff_a.at(2)*diff_x.diff_b.at(0)*diff_x.diff_c.at(3)
            -diff_x.diff_a.at(2)*diff_x.diff_b.at(3)*diff_x.diff_c.at(0) - diff_x.diff_a.at(0)*diff_x.diff_b.at(2)*diff_x.diff_c.at(3) -diff_x.diff_a.at(3)*diff_x.diff_b.at(0)*diff_x.diff_c.at(2);
    Dc = diff_x.diff_a.at(0)*diff_x.diff_b.at(1)*diff_x.diff_c.at(3) + diff_x.diff_a.at(1)*diff_x.diff_b.at(3)*diff_x.diff_c.at(0) +diff_x.diff_a.at(3)*diff_x.diff_b.at(0)*diff_x.diff_c.at(1)
            -diff_x.diff_a.at(3)*diff_x.diff_b.at(1)*diff_x.diff_c.at(0) - diff_x.diff_a.at(0)*diff_x.diff_b.at(3)*diff_x.diff_c.at(1) -diff_x.diff_a.at(1)*diff_x.diff_b.at(0)*diff_x.diff_c.at(3);
    D_y = diff_y.diff_a.at(0)*diff_y.diff_b.at(1)*diff_y.diff_c.at(2) + diff_y.diff_a.at(1)*diff_y.diff_b.at(2)*diff_y.diff_c.at(0) +diff_y.diff_a.at(2)*diff_y.diff_b.at(0)*diff_y.diff_c.at(1)
            -diff_y.diff_a.at(2)*diff_y.diff_b.at(1)*diff_y.diff_c.at(0) - diff_y.diff_a.at(0)*diff_y.diff_b.at(2)*diff_y.diff_c.at(1) -diff_y.diff_a.at(1)*diff_y.diff_b.at(0)*diff_y.diff_c.at(2);
    Dd = diff_y.diff_a.at(3)*diff_y.diff_b.at(1)*diff_y.diff_c.at(2) + diff_y.diff_a.at(1)*diff_y.diff_b.at(2)*diff_y.diff_c.at(3) +diff_y.diff_a.at(2)*diff_y.diff_b.at(3)*diff_y.diff_c.at(1)
            -diff_y.diff_a.at(2)*diff_y.diff_b.at(1)*diff_y.diff_c.at(3) - diff_y.diff_a.at(3)*diff_y.diff_b.at(2)*diff_y.diff_c.at(1) -diff_y.diff_a.at(1)*diff_y.diff_b.at(3)*diff_y.diff_c.at(2);
    De = diff_y.diff_a.at(0)*diff_y.diff_b.at(3)*diff_y.diff_c.at(2) + diff_y.diff_a.at(3)*diff_y.diff_b.at(2)*diff_y.diff_c.at(0) +diff_y.diff_a.at(2)*diff_y.diff_b.at(0)*diff_y.diff_c.at(3)
            -diff_y.diff_a.at(2)*diff_y.diff_b.at(3)*diff_y.diff_c.at(0) - diff_y.diff_a.at(0)*diff_y.diff_b.at(2)*diff_y.diff_c.at(3) -diff_y.diff_a.at(3)*diff_y.diff_b.at(0)*diff_y.diff_c.at(2);
    Df = diff_y.diff_a.at(0)*diff_y.diff_b.at(1)*diff_y.diff_c.at(3) + diff_y.diff_a.at(1)*diff_y.diff_b.at(3)*diff_y.diff_c.at(0) +diff_y.diff_a.at(3)*diff_y.diff_b.at(0)*diff_y.diff_c.at(1)
            -diff_y.diff_a.at(3)*diff_y.diff_b.at(1)*diff_y.diff_c.at(0) - diff_y.diff_a.at(0)*diff_y.diff_b.at(3)*diff_y.diff_c.at(1) -diff_y.diff_a.at(1)*diff_y.diff_b.at(0)*diff_y.diff_c.at(3);

    _mat mat;
    mat.a = Da / D_x;
    mat.b = Db / D_x;
    mat.c = Dc / D_x;
    mat.d = Dd / D_y;
    mat.e = De / D_y;
    mat.f = Df / D_y;
    mat.g = 0;
    mat.h = 0;
    mat.i = 1;
    return mat;
}




//        for(int i=0;i<rows;++i)
//        {

//            qDebug()<<"e:"<<e(i,0)<<"     _e:"<<_e(i,0);
//            qDebug()<<"w:"<<w(i,0)<<"     _w:"<<_w(i,0);
//        }
//        for(int i=0;i<x.rows();i++)
//        {
//            qDebug()<<"x:"<<x(i,0)<<"     _x:"<<_x(i,0);
//        }

//        qDebug()<<"(x-_x).abs().sum():"<<(x-_x).abs().sum()<<"  cishu:"<<ss++;


//    //获取矩阵的行数和列数
//    int rows =  A.rows();
//    int col = A.cols();

//    double epsilong = 10e-9; // ε
//    double delta = 10e-15; // δ
//    Array<double,Dynamic,1> x,_x,e,w;
//    x.resize(col,1);
//    _x.resize(col,1);
//    e.resize(rows,1);
//    w.resize(rows,1);
//    //初始x  对角矩阵w=1
//    x = reweightedLeastSquares(A,B);

//    //迭代  最大迭代次数kk
//    for(int i=0;i<kk;++i)
//    {
//        //保留前一个x值,用作最后比较确定收敛
//        _x = x;
//        //偏差
//        e = (A * x.matrix()) - B;
//        //偏差的绝对值//  求矩阵绝对值 ：e = e.cwiseAbs(); 或 e.array().abs().matrix()
//        e = e.abs();
//        //对每个偏差值小于delta,用delta赋值给它
//        for(int i=0;i<e.rows();++i)
//        {
//            e(i,0) = qMax(delta,e(i,0));
//        }
//        //对每个偏差值进行幂操作
//        w = e.pow(p/2.0-1);
//        w = w / w.sum();

//        x = reweightedLeastSquares(A,B,w);

//        //达到精度,结束
//        if((x-_x).abs().sum()<epsilong)
//        {
//            return x;
//        }
//    }
//    return x;


//    for(int i=0;i<e.rows();++i)
//    {
//        if(e(i,0) < eta)
//        {
//            //e(i,0) = pow(e(i,0),2)/2;
//            e(i,0) = pow(1 - pow(e(i,0),2) / pow(eta,2),2);
//        }
//        else
//        {
//            //e(i,0) = eta * (e(i,0) - eta/2) ;
//            e(i,0) = 0;
//        }
//    }
