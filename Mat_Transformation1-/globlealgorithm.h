#ifndef GLOBLEALGORITHM_H
#define GLOBLEALGORITHM_H

#include <QObject>
#include <Eigen>
#include <QPointF>
#include <QtMath>
#include <QThread>
#include <QAtomicPointer>
#include <QMutex>
#include <QDebug>
#include <iostream>
#include <QElapsedTimer>
//#define DERIV_STEP 0.01
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


//全局算法类
class GlobleAlgorithm
{

public:
    explicit GlobleAlgorithm();

    //旋转矩阵 --> 旋转向量    :罗德里格斯公式逆变换
    Vector3d Rodrigues(const Matrix3d& R)
    {
        Eigen::AngleAxisd rotAA2(R);
        Vector3d r{rotAA2.angle() * rotAA2.axis()};


        //        double theta = acos((R.trace() - 1) * 0.5);
        //        Matrix3d r_hat = (R - R.transpose()) * 0.5 / sin(theta);
        //        Vector3d r1;
        //        r1(0) = theta*(r_hat(2,1) - r_hat(1,2))*0.5;
        //        r1(1) = theta*(r_hat(0,2) - r_hat(2,0))*0.5;
        //        r1(2) = theta*(r_hat(1,0) - r_hat(0,1))*0.5;
        //        std::cout<<"R.trace():"<<R.trace()<<"  theta: "<<theta<<std::endl<<"r:"<< r <<std::endl<<std::endl<<"r1:"<< r1 <<std::endl;
        return r;
    }
    //旋转向量 --> 旋转矩阵  :罗德里格斯公式
    Matrix3d Rodrigues(const Vector3d& _r)
    {
        // 第1种方法
        Eigen::AngleAxisd  rotAA{_r.norm(), _r.normalized()};
        Matrix3d R {rotAA.toRotationMatrix()};

        //    // 第2种方法
        //    double theta = _r.lpNorm<2>();
        //    Vector3d r = _r / theta;
        //    Matrix3d r_hat;
        //    r_hat << 0, -r[2], r[1],
        //            r[2], 0, -r[0],
        //            -r[1], r[0], 0;

        //    Matrix3d R = cos(theta) * Matrix3d::Identity() + (1 - cos(theta)) * r * r.transpose() + sin(theta) * r_hat;
        //    std::cout << "R :" << R << std::endl;
        return R;
    }
    //获取圆的中心和半径，---三点拟合圆 -- 代数推导
    _arcConfig Get_ArcCenter3(QPointF,QPointF,QPointF);

    //获取圆的中心和半径，---两点和两点之间的角度 -- 代数推导
    _arcConfig Get_ArcCenter1(QPointF,QPointF,double rad);

    //获取圆的中心和半径，---两点和两点之间的角度 -- 几何推导
    _arcConfig Get_ArcCenter2(QPointF,QPointF,double rad);


    //矫正图像 根据内参和畸变系数矫正
    Eigen::MatrixXi RectifiedImage(Eigen::MatrixXi src,Eigen::Matrix3d intrinsicParam , Eigen::VectorXd distortionCoeff );

    //以下算法只适用于超定方程组 方程个数大于未知数

    /* 普通最小二乘(OLS) Ax = B
     * (A^T * A) * x = A^T * B
     * x = (A^T * A)^-1 * A^T * B
     */
    Eigen::VectorXd LeastSquares(Eigen::MatrixXd A, Eigen::VectorXd B);

    /* 加权最小二乘（WLS）  W为对角线矩阵
     * W²(Ax - B) = 0
     * W²Ax = W²B
     * (A^T * W^T * W * A) * x = A^T * W^T * W * B
     * x = (A^T * W^T * W * A)^-1 * A^T * W^T * W * B
     */
    Eigen::VectorXd ReweightedLeastSquares(Eigen::MatrixXd A, Eigen::VectorXd B,Eigen::VectorXd vectorW = Eigen::VectorXd());

    /* 迭代重加权最小二乘（IRLS）  W为为对角线矩阵,p为范数
     * e = Ax - B
     * W = e^(p−2)/2
     * W²(Ax - B) = 0
     * W²Ax = W²B
     * (A^T * W^T * W * A) * x = A^T * W^T * W * B
     * x = (A^T * W^T * W * A)^-1 * A^T * W^T * W * B
     * 参考论文地址：https://www.semanticscholar.org/paper/Iterative-Reweighted-Least-Squares-%E2%88%97-Burrus/9b9218e7233f4d0b491e1582c893c9a099470a73
     */
    Eigen::VectorXd IterativeReweightedLeastSquares(Eigen::MatrixXd A, Eigen::VectorXd B,double p = 2 ,int kk = 1);

    //N点标定算法
    _mat calibration(_coordList);
    /* 高斯牛顿法(GNA) 解决非线性最小二乘问题 确定目标函数和约束来对现有的参数优化
     */
    template <class _T,class _ResidualsVector,class _JacobiMat>
    Eigen::VectorXd GaussNewtonAlgorithm(Eigen::VectorXd params,_T otherArgs, _ResidualsVector ResidualsVector,_JacobiMat JacobiMat,double _epsilon = 1e-10,quint32 _maxIteCount = 99)
    {
        quint32 iterCount=0;
        double currentEpsilon =0.;
        QElapsedTimer eTimer;

        int k=0;
        // ε 终止条件
        double epsilon = _epsilon;
        //迭代次数
        int maxIteCount = _maxIteCount;

        //found 为true 结束循环
        bool found = false;
        eTimer.restart();
        while(!found && k<maxIteCount)
        {
            //迭代增加
            k++;
            //获取预测偏差值 r= ^y(预测值) - y(实际值)
            //保存残差值
            Eigen::VectorXd residual = ResidualsVector(params,otherArgs);

            //求雅可比矩阵
            Eigen::MatrixXd Jac = JacobiMat(params,otherArgs);

            // Δx = - (Jac^T * Jac)^-1 * Jac^T * r
            Eigen::VectorXd delta_x =  -  (((Jac .transpose() * Jac ).inverse()) * Jac.transpose() * residual).array();

            //qDebug()<<QString("高斯牛顿法：第 %1 次迭代 --- 精度：%2  ").arg(k).arg(delta_x.array().abs().sum());
            //达到精度,结束
            if(delta_x.array().abs().sum() < epsilon)
            {
                found = true;
            }

            //x(k+1) = x(k) + Δx
            params = params + delta_x;

            iterCount=k;
            currentEpsilon = delta_x.array().abs().sum();
            //发送 当前迭代次数,当前精度，迭代一次需要的时长
            qDebug()<<QString("当前迭代次数: %1 ,收敛精度: %2 ,迭代时长: %3 ").arg(iterCount).arg(currentEpsilon).arg(eTimer.restart());
        }
        return params;
    }

    /* 列文伯格马夸尔特法(LMA) ==  使用信赖域的高斯牛顿法，鲁棒性更好， 确定目标函数和约束来对现有的参数优化
     * params 初始参数,待优化
     * otherArgs 其他参数
     * _ResidualsVector 自定义函数：获取预测值和实际值的差值
     * _JacobiMat 自定义函数：获取当前的雅可比矩阵
     * _epsilon 收敛精度
     * _maxIteCount 最大迭代次数
     * _epsilon 和 _maxIteCount 达到任意一个条件就停止返回
     */
    template <class _T,class _ResidualsVector,class _JacobiMat>
    Eigen::VectorXd LevenbergMarquardtAlgorithm(Eigen::VectorXd params,_T otherArgs, _ResidualsVector ResidualsVector,_JacobiMat JacobiMat,double _epsilon = 1e-12,quint32 _maxIteCount = 99)
    {
        quint32 iterCount=0;
        double currentEpsilon =0.;
        QElapsedTimer eTimer;

        // ε 终止条件
        double epsilon = _epsilon;
        double _currentEpsilon=0.0;
        // τ
        double tau = 1e-6;

        //迭代次数
        quint32 maxIteCount = _maxIteCount;
        quint32 k=0;
        int v=2;

        //求雅可比矩阵
        Eigen::MatrixXd Jac = JacobiMat(params,otherArgs);

        //用雅可比矩阵近似黑森矩阵
        Eigen::MatrixXd Hessen = Jac .transpose() * Jac ;

        //获取预测偏差值 r= ^y(预测值) - y(实际值)
        //保存残差值
        Eigen::VectorXd residual = ResidualsVector(params,otherArgs);
        //梯度
        Eigen::MatrixXd g = Jac.transpose() * residual;

        //found 为true 结束循环
        bool found = ( g.lpNorm<Eigen::Infinity>() <= epsilon );

        //阻尼参数μ
        double mu =  tau * Hessen.diagonal().maxCoeff();
        eTimer.restart();
        while(!found && k<maxIteCount)
        {
            k++;
            //LM方向  uI => I 用黑森矩阵对角线代替
            //Eigen::MatrixXd delta_x = - (Hessen + mu*Hessen.asDiagonal().diagonal()).inverse() * g;

            Eigen::VectorXd delta_x = - (Hessen + mu*Eigen::MatrixXd::Identity(Hessen.cols(), Hessen.cols())).inverse() * g;


            if( delta_x.lpNorm<2>() <= epsilon * (params.lpNorm<2>() + epsilon ))
            {
                currentEpsilon = delta_x.lpNorm<2>();
                found = true;
            }
            else
            {
                Eigen::VectorXd newParams = params + delta_x;
                //L(0) - L(delta) = 0.5*(delta^-1)*(μ*delta - g)
                //ρ     =    (F(x) - F(x_new)) / (L(0) - L(delta));
                double rho = (ResidualsVector(params,otherArgs).array().pow(2).sum() - ResidualsVector(newParams,otherArgs).array().pow(2).sum())
                        / (0.5*delta_x.transpose()*(mu * delta_x - g)).sum();

                if(rho>0)
                {
                    params = newParams;
                    Jac = JacobiMat(params,otherArgs);
                    Hessen = Jac.transpose() * Jac ;
                    //获取预测偏差值 r= ^y(预测值) - y(实际值)
                    residual = ResidualsVector(params,otherArgs);
                    g = Jac.transpose() * residual;
                    _currentEpsilon = g.lpNorm<Eigen::Infinity>();
                    found = (_currentEpsilon  <= epsilon );
                    mu = mu* qMax(1/3.0 , 1-qPow(2*rho -1,3));
                    v=2;
                }
                else
                {
                    mu = mu*v;
                    v = 2*v;
                }
            }
            iterCount=k;
            currentEpsilon = _currentEpsilon;
            //发送 当前迭代次数,当前精度，迭代一次需要的时长
            qDebug()<<QString("当前迭代次数: %1 ,收敛精度: %2 ,迭代时长: %3 ").arg(iterCount).arg(currentEpsilon).arg(eTimer.restart());
        }
        return params;
    }

protected:
    //    //求雅克比矩阵
    //    template <class FunctionPredictedValue>
    //    Eigen::MatrixXd Jacobi(const Eigen::MatrixXd& inValue, const Eigen::VectorXd& outValue,const Eigen::VectorXd& params,FunctionPredictedValue funPV=Function())
    //    {
    //        int rowNum = inValue.rows();
    //        int colNum = params.rows();

    //        Eigen::MatrixXd Jac(rowNum, colNum);

    //        for (int i = 0; i < rowNum; i++)
    //        {
    //            for (int j = 0; j < colNum; j++)
    //            {
    //                Jac(i, j) = Deriv(inValue,outValue, i, params, j,funPV);
    //            }
    //        }
    //        return Jac;
    //    }
    //    //残差值向量
    //    template <class FunctionPredictedValue>
    //    Eigen::VectorXd ResidualsVector(const Eigen::MatrixXd& inValue, const Eigen::VectorXd& outValue,const Eigen::VectorXd& params,FunctionPredictedValue funPV=Function())
    //    {
    //        int dataCount = inValue.rows();
    //        //保存残差值
    //        Eigen::VectorXd residual(dataCount);
    //        //获取预测偏差值 r= ^y(预测值) - y(实际值)
    //        for(int i=0;i<dataCount;++i)
    //        {
    //            //获取预测值
    //            double predictedValue = funPV(inValue,outValue,i,params);
    //            residual(i) = predictedValue - outValue(i);
    //        }
    //        return residual;
    //    }

    //    //求导
    //    template <class FunctionPredictedValue>
    //    double Deriv(const Eigen::MatrixXd& inValue, const Eigen::VectorXd& outValue,int objIndex, const Eigen::VectorXd& params,int paraIndex,FunctionPredictedValue funPV=Function())
    //    {
    //        Eigen::VectorXd para1 = params;
    //        Eigen::VectorXd para2 = params;
    //        para1(paraIndex) -= DERIV_STEP;
    //        para2(paraIndex) += DERIV_STEP;

    //        double obj1 = funPV(inValue,outValue,objIndex,para1);
    //        double obj2 = funPV(inValue,outValue,objIndex,para2);

    //        return (obj2 - obj1) / (2 * DERIV_STEP);
    //    }


public:
    //创建单例
    static GlobleAlgorithm * getInstance();

private:
    static QAtomicPointer<GlobleAlgorithm> m_instance;//单例对象
    static QMutex m_mutex;

};



#endif // GLOBLEALGORITHM_H
