#ifndef CAMERACALIBRATIONALGORITHM_H
#define CAMERACALIBRATIONALGORITHM_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#define DERIV_STEP 1e-6  //微小变化量,用于中心差分法求导

/* 普通最小二乘 Ax = B
 * (A^T * A) * x = A^T * B
 * x = (A^T * A)^-1 * A^T * B
 */
Eigen::VectorXd LeastSquares(Eigen::MatrixXd A, Eigen::VectorXd B);
//旋转矩阵 --> 旋转向量    :罗德里格斯公式逆变换
Eigen::Vector3d Rodrigues(const Eigen::Matrix3d& R);
//旋转向量 --> 旋转矩阵  :罗德里格斯公式
Eigen::Matrix3d Rodrigues(const Eigen::Vector3d& _r);

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
    Eigen::VectorXd LevenbergMarquardtAlgorithm(Eigen::VectorXd params,_T otherArgs, _ResidualsVector ResidualsVector,_JacobiMat JacobiMat,double _epsilon = 1e-12,int _maxIteCount = 99)
    {
        int iterCount=0;
        double currentEpsilon =0.0;
        // ε 终止条件
        double epsilon = _epsilon;
        double _currentEpsilon=0.0;
        // τ
        double tau = 1e-6;

        //迭代次数
        int maxIteCount = _maxIteCount;
        int k=0;
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
                    mu = mu* std::max(1/3.0 , 1-std::pow(2*rho -1,3));
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
            // 当前迭代次数,当前精度
            std::cout<<"当前迭代次数:"<<std::to_string(iterCount)<< "收敛精度:"<<std::to_string(currentEpsilon)<<std::endl;
        }
        return params;
    }





//用于求相机参数的其他参数
typedef struct _CameraOtherParameter
{
    std::vector<Eigen::MatrixXd>  srcL;              //物体点
    std::vector<Eigen::MatrixXd>  dstL;              //图像点
    int intrinsicCount;                //内参个数
    int disCount;                       //畸变个数 //畸变系数 2:k1,k2,(4:)p1,p2,[(5:)k3]
    int imageCount;                     // 图像个数

}S_CameraOtherParameter;

//相机参数
typedef struct _CameraParameter
{
    //内参
    Eigen::Matrix3d intrinsicParameter;

    //畸变系数
    Eigen::VectorXd distortionCoeff;

    //外参
    std::vector<Eigen::MatrixXd> externalParams;

    //每张图片单应性矩阵
    std::vector<Eigen::Matrix3d> homographyList;

    //每张图片重投影误差
    std::vector<double> reprojErrL;

    //总重投影误差
    double reprojErr;
}S_CameraP;


//存放坐标
struct PointF
{
    double _x;
    double _y;

    PointF() {}

    double x()
    {
        return _x;
    }

    double y()
    {
        return _y;
    }

};
///////////////////////////////////////////////////////////////
/// \brief The CameraCalibration class
/// 相机标定类
class CameraCalibration
{
public:
    explicit CameraCalibration(std::vector<std::vector<PointF> > objectPointsL, std::vector<std::vector<PointF> > imagePointsL,int _disCount=5,int _intrinsicCount=5,double _epsilon = 1e-12,int _maxIteCount = 99);

    //求单应性矩阵H
    Eigen::Matrix3d  GetHomography(const Eigen::MatrixXd& src, const Eigen::MatrixXd& dst);
    //求相机内参
    Eigen::Matrix3d  GetIntrinsicParameter(const std::vector<Eigen::Matrix3d>& HList);

    //求相机外参
    std::vector<Eigen::MatrixXd>  GetExternalParameter(const std::vector<Eigen::Matrix3d>& HList,const Eigen::Matrix3d& intrinsicParam);

    //获取畸变系数 k1,k2,[p1,p2,[k3]]
    void GetDistortionCoeff(const std::vector<Eigen::MatrixXd>&  srcL, const std::vector<Eigen::MatrixXd>&  dstL,const Eigen::Matrix3d& intrinsicParam ,const std::vector<Eigen::MatrixXd>& externalParams,Eigen::VectorXd & disCoeff);

    //优化所有参数 (内参,畸变系数,外参) 返回重投影误差值
    double OptimizeParameter(const std::vector<Eigen::MatrixXd>&  srcL,const std::vector<Eigen::MatrixXd>&  dstL, Eigen::Matrix3d& intrinsicParam , Eigen::VectorXd& distortionCoeff, std::vector<Eigen::MatrixXd>& externalParams);

    //获取相机参数
    S_CameraP GetCameraParameter();
protected:

    //物体坐标和像素坐标转换矩阵形式
    std::pair<std::vector<Eigen::MatrixXd>,std::vector<Eigen::MatrixXd>> PointsToMat(std::vector<std::vector<PointF> > objectPointsL, std::vector<std::vector<PointF> > imagePointsL);

    //根据单应性矩阵H返回pq位置对应的v向量
    Eigen::VectorXd CreateV(int p, int q,const Eigen::Matrix3d &H);

    //获取初始矩阵H
    Eigen::VectorXd GetInitialH (const Eigen::MatrixXd& srcNormal, const Eigen::MatrixXd& dstNormal );


    //整合所有参数(内参,畸变系数,外参)到一个向量中
    Eigen::VectorXd ComposeParameter(const Eigen::Matrix3d& intrinsicParam ,const Eigen::VectorXd& distortionCoeff,const std::vector<Eigen::MatrixXd>& externalParams);

    //分解所有参数  得到对应的内参,畸变矫正系数,外参
    void DecomposeParamter(const Eigen::VectorXd &P, Eigen::Matrix3d& intrinsicParam , Eigen::VectorXd& distortionCoeff, std::vector<Eigen::MatrixXd>& externalParams);

    //获取标准差
    double StdDiffer(const Eigen::VectorXd& data);
    // 归一化
    Eigen::Matrix3d Normalization (const Eigen::MatrixXd& P);

private:
    //多张物体点
    std::vector<std::vector<Eigen::MatrixXd> > m_srcPointsL;
    //多张像素点
    std::vector<std::vector<Eigen::MatrixXd> > m_dstPointsL;

    //内参
    Eigen::Matrix3d m_intrinsicParameter;

    //畸变系数
    Eigen::VectorXd m_distortionCoeff;

    //外参
    std::vector<Eigen::MatrixXd> m_externalParams;

    //每张图片单应性矩阵
    std::vector<Eigen::Matrix3d> m_homographyList;

    //每张图片重投影误差
    std::vector<double> m_reprojErrL;

    //重投影误差
    double m_reprojErr;

    //内参个数
    int m_intrinsicCount;
    //畸变个数
    int m_disCount;
    //收敛精度
    double m_epsilon ;
    //最大迭代次数
    int m_maxIteCount;

};

//单应性残差值向量
class HomographyResidualsVector
{
public:
    Eigen::VectorXd  operator()(const Eigen::VectorXd& parameter,const std::vector<Eigen::MatrixXd> &otherArgs)
    {
        Eigen::MatrixXd inValue = otherArgs.at(0);
        Eigen::MatrixXd outValue = otherArgs.at(1);
        int dataCount = inValue.rows();
        //保存残差值
        Eigen::VectorXd residual(dataCount*2) ,residualNew(dataCount*2);
        Eigen::Matrix3d HH =  parameter.reshaped<Eigen::RowMajor>(3,3);
        //获取预测偏差值 r= ^y(预测值) - y(实际值)
        for(int i=0;i<dataCount;++i)
        {
            //转换齐次坐标
            Eigen::VectorXd singleRealCoor(3),U(3);
            singleRealCoor(0,0) = inValue(i,0);
            singleRealCoor(1,0) = inValue(i,1);
            singleRealCoor(2,0) = 1;
            U = HH * singleRealCoor;
            U /= U(2);

            residual(i*2) = U(0);
            residual(i*2+1) = U(1);

            residualNew(i*2) = outValue(i,0);
            residualNew(i*2+1) = outValue(i,1);
        }

        residual -= residualNew;
        return residual;
    }

};



//求单应性雅克比矩阵
class HomographyJacobi
{
    //求偏导1
    double PartialDeriv_1(const Eigen::VectorXd& parameter,int paraIndex,const Eigen::MatrixXd &inValue,int objIndex)
    {
        Eigen::VectorXd para1 = parameter;
        Eigen::VectorXd para2 = parameter;
        para1(paraIndex) -= DERIV_STEP;
        para2(paraIndex) += DERIV_STEP;

        //逻辑
        double obj1 = ((para1(0))*inValue(objIndex,0) + para1(1)*inValue(objIndex,1) + para1(2)) / (para1(6)*inValue(objIndex,0) + para1(7)*inValue(objIndex,1) + para1(8));
        double obj2 = ((para2(0))*inValue(objIndex,0) + para2(1)*inValue(objIndex,1) + para2(2)) / (para2(6)*inValue(objIndex,0) + para2(7)*inValue(objIndex,1) + para2(8));;

        return (obj2 - obj1) / (2 * DERIV_STEP);
    }

    //求偏导2
    double PartialDeriv_2(const Eigen::VectorXd& parameter,int paraIndex,const Eigen::MatrixXd &inValue,int objIndex)
    {
        Eigen::VectorXd para1 = parameter;
        Eigen::VectorXd para2 = parameter;
        para1(paraIndex) -= DERIV_STEP;
        para2(paraIndex) += DERIV_STEP;

        //逻辑
        double obj1 = ((para1(3))*inValue(objIndex,0) + para1(4)*inValue(objIndex,1) + para1(5)) / (para1(6)*inValue(objIndex,0) + para1(7)*inValue(objIndex,1) + para1(8));
        double obj2 = ((para2(3))*inValue(objIndex,0) + para2(4)*inValue(objIndex,1) + para2(5)) / (para2(6)*inValue(objIndex,0) + para2(7)*inValue(objIndex,1) + para2(8));;

        return (obj2 - obj1) / (2 * DERIV_STEP);
    }
public:

    Eigen::MatrixXd operator()(const Eigen::VectorXd& parameter,const std::vector<Eigen::MatrixXd> &otherArgs)
    {
        Eigen::MatrixXd inValue = otherArgs.at(0);
        int rowNum = inValue.rows();

        Eigen::MatrixXd Jac(rowNum*2, parameter.rows());

        for (int i = 0; i < rowNum; i++)
        {
            //            //第一种方法：直接求偏导
            //            double sx = parameter(0)*inValue(i,0) + parameter(1)*inValue(i,1) + parameter(2);
            //            double sy = parameter(3)*inValue(i,0) + parameter(4)*inValue(i,1) + parameter(5);
            //            double w = parameter(6)*inValue(i,0) + parameter(7)*inValue(i,1) + parameter(8);
            //            double w2 = w*w;

            //            Jac(i*2,0) = inValue(i,0)/w;
            //            Jac(i*2,1) = inValue(i,1)/w;
            //            Jac(i*2,2) = 1/w;
            //            Jac(i*2,3) = 0;
            //            Jac(i*2,4) = 0;
            //            Jac(i*2,5) = 0;
            //            Jac(i*2,6) = -sx*inValue(i,0)/w2;
            //            Jac(i*2,7) = -sx*inValue(i,1)/w2;
            //            Jac(i*2,8) = -sx/w2;

            //            Jac(i*2+1,0) = 0;
            //            Jac(i*2+1,1) = 0;
            //            Jac(i*2+1,2) = 0;
            //            Jac(i*2+1,3) = inValue(i,0)/w;
            //            Jac(i*2+1,4) = inValue(i,1)/w;
            //            Jac(i*2+1,5) = 1/w;
            //            Jac(i*2+1,6) = -sy*inValue(i,0)/w2;
            //            Jac(i*2+1,7) = -sy*inValue(i,1)/w2;
            //            Jac(i*2+1,8) = -sy/w2;

            //第二种方法: 计算求偏导

            Jac(i*2,0) = PartialDeriv_1(parameter,0,inValue,i);
            Jac(i*2,1) = PartialDeriv_1(parameter,1,inValue,i);
            Jac(i*2,2) = PartialDeriv_1(parameter,2,inValue,i);
            Jac(i*2,3) = 0;
            Jac(i*2,4) = 0;
            Jac(i*2,5) = 0;
            Jac(i*2,6) = PartialDeriv_1(parameter,6,inValue,i);
            Jac(i*2,7) = PartialDeriv_1(parameter,7,inValue,i);
            Jac(i*2,8) = PartialDeriv_1(parameter,8,inValue,i);

            Jac(i*2+1,0) = 0;
            Jac(i*2+1,1) = 0;
            Jac(i*2+1,2) = 0;
            Jac(i*2+1,3) = PartialDeriv_2(parameter,3,inValue,i);
            Jac(i*2+1,4) = PartialDeriv_2(parameter,4,inValue,i);
            Jac(i*2+1,5) = PartialDeriv_2(parameter,5,inValue,i);
            Jac(i*2+1,6) = PartialDeriv_2(parameter,6,inValue,i);
            Jac(i*2+1,7) = PartialDeriv_2(parameter,7,inValue,i);
            Jac(i*2+1,8) = PartialDeriv_2(parameter,8,inValue,i);

        }
        return Jac;
    }
};


//相机标定残差值向量 -- 返回所有点的真实世界坐标映射到图像坐标 与 真实图像坐标的残差
class CalibrationResidualsVector
{
    //返回从真实世界坐标映射的图像坐标
    Eigen::Vector3d getMapCoor(const Eigen::Matrix3d& intrinsicParam ,const Eigen::VectorXd& distortionCoeff,const  Eigen::MatrixXd& externalParam,const Eigen::Vector3d& XYZ)
    {
        //畸变个数
        int disCount = distortionCoeff.rows();
        //转换齐次坐标
        Eigen::VectorXd singleCoor(4);
        singleCoor(0) = XYZ(0);
        singleCoor(1) = XYZ(1);
        singleCoor(2) = XYZ(2);
        singleCoor(3) = 1;

        //归一化坐标
        Eigen::Vector3d normCoor = externalParam * singleCoor;
        normCoor /= normCoor(2);

        double r = std::sqrt(std::pow(normCoor(0),2) + std::pow(normCoor(1),2));


        Eigen::Vector3d uv;
        uv(0)=0;
        uv(1)=0;
        uv(2)=1;

        //无畸变参数
        if(disCount == 0)
        {
            uv(0) = normCoor(0);
            uv(1) = normCoor(1);
        }
        double u_2=0,v_2=0,u_4=0,v_4=0,u_5=0,v_5=0,u_8=0,v_8=0,u_12=0,v_12=0;
        //k1,k2
        if(disCount >= 2)
        {
            u_2 = normCoor(0)*(1+std::pow(r,2)*distortionCoeff(0) + std::pow(r,4) * distortionCoeff(1));
            v_2 = normCoor(1)*(1+std::pow(r,2)*distortionCoeff(0) + std::pow(r,4) * distortionCoeff(1));
            uv(0) += u_2;
            uv(1) += v_2;
        }
        //k1,k2,p1,p2
        if(disCount >= 4)
        {
            u_4 = (2*normCoor(0)*normCoor(1)*distortionCoeff(2) + (2*std::pow(normCoor(0),2) + std::pow(r,2))*distortionCoeff(3));
            v_4 = ((2*std::pow(normCoor(1),2) + std::pow(r,2))*distortionCoeff(2) + normCoor(0)*normCoor(1)*2*distortionCoeff(3));
            uv(0) += u_4;
            uv(1) += v_4;
        }
        //k1,k2,p1,p2,k3
        if(disCount >= 5)
        {
            u_5 = normCoor(0)*std::pow(r,6)*distortionCoeff(4);
            v_5 = normCoor(1)*std::pow(r,6)*distortionCoeff(4);
            uv(0) += u_5;
            uv(1) += v_5;
        }
        //k1,k2,p1,p2,k3,k4,k5,k6
        if(disCount >= 8)
        {
            u_8 = (u_2 + u_5) / (1+std::pow(r,2)*distortionCoeff(5) + std::pow(r,4) * distortionCoeff(6) + std::pow(r,6)*distortionCoeff(7)) + u_4;
            v_8 = (v_2 + v_5) / (1+std::pow(r,2)*distortionCoeff(5) + std::pow(r,4) * distortionCoeff(6) + std::pow(r,6)*distortionCoeff(7)) + v_4;
            uv(0) = u_8;
            uv(1) = v_8;
        }
        //k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
        if(disCount >= 12)
        {
            u_12 = std::pow(r,2)*distortionCoeff(8) + std::pow(r,4)*distortionCoeff(9);
            v_12 = std::pow(r,2)*distortionCoeff(10) + std::pow(r,4)*distortionCoeff(11);
            uv(0) += u_12;
            uv(1) += v_12;
        }
        uv = intrinsicParam * uv;

        return uv;
    }

public:
    Eigen::VectorXd  operator()(const Eigen::VectorXd& parameter,const S_CameraOtherParameter &otherArgs)
    {
        //获取数据总个数
        int allCount=0;
        for(int i=0;i<otherArgs.imageCount;++i)
        {
            allCount += otherArgs.srcL.at(i).rows();
        }
        Eigen::VectorXd real_uv(allCount*2),map_uv(allCount*2);

        //内参
        Eigen::Matrix3d intrinsicParam;

        if(otherArgs.intrinsicCount == 4)
        {
            intrinsicParam<<parameter(0),0,parameter(2),
                0,parameter(1),parameter(3),
                0,0,1;
        }
        else
        {
            intrinsicParam<<parameter(0),parameter(1),parameter(3),
                0,parameter(2),parameter(4),
                0,0,1;
        }




        //畸变系数
        Eigen::VectorXd distortionCoeff(otherArgs.disCount);
        for(int i=0;i<otherArgs.disCount;++i)
        {
            distortionCoeff(i) = parameter(otherArgs.intrinsicCount+i);
        }
        //索引k存放数据
        int k=0;
        for(int i=0;i<otherArgs.imageCount;++i)
        {
            Eigen::MatrixXd src = otherArgs.srcL.at(i);
            Eigen::MatrixXd dst = otherArgs.dstL.at(i);
            int srcCount = src.rows();

            //外参
            Eigen::MatrixXd W(3,4);
            Eigen::Vector3d r ;
            r(0) = parameter(otherArgs.intrinsicCount+otherArgs.disCount+i*6);
            r(1) = parameter(otherArgs.intrinsicCount+otherArgs.disCount+i*6+1);
            r(2) = parameter(otherArgs.intrinsicCount+otherArgs.disCount+i*6+2);
            W.block(0,0,3,3) = Rodrigues(r);
            W(0,3) = parameter(otherArgs.intrinsicCount+otherArgs.disCount+i*6+3);
            W(1,3) = parameter(otherArgs.intrinsicCount+otherArgs.disCount+i*6+4);
            W(2,3) = parameter(otherArgs.intrinsicCount+otherArgs.disCount+i*6+5);

            //遍历当前图片数据点
            for(int j=0;j<srcCount;++j)
            {
                //物体坐标
                Eigen::Vector3d XYZ;
                XYZ<<src(j,0),
                        src(j,1),
                        0;

                Eigen::Vector3d uv = getMapCoor(intrinsicParam,distortionCoeff,W,XYZ);
                map_uv(k) = uv(0);
                map_uv(k+1) = uv(1);

                real_uv(k) = dst(j,0);
                real_uv(k+1) = dst(j,1);

                k += 2;
            }
        }
        //获取预测偏差值 r=   ^y(预测值) - y(实际值)
        return map_uv - real_uv;
    }
};
//求相机标定雅克比矩阵
class CalibrationJacobi
{
    //求偏导1
    double PartialDeriv_1(const Eigen::VectorXd& parameter,int paraIndex, const S_CameraOtherParameter &otherArgs,int i,int j)
    {
        Eigen::VectorXd para1 = parameter;
        Eigen::VectorXd para2 = parameter;
        para1(paraIndex) -= DERIV_STEP;
        para2(paraIndex) += DERIV_STEP;


        double obj1 =0,obj2 =0;
        //坐标
        double x = otherArgs.srcL.at(i)(j,0);
        double y = otherArgs.srcL.at(i)(j,1);
        double z = 0;
        //旋转向量
        double r_1 = para1(otherArgs.intrinsicCount+otherArgs.disCount+i*6);
        double r_2 = para1(otherArgs.intrinsicCount+otherArgs.disCount+i*6+1);
        double r_3 = para1(otherArgs.intrinsicCount+otherArgs.disCount+i*6+2);
        //平移向量
        double t_1 = para1(otherArgs.intrinsicCount+otherArgs.disCount+i*6+3);
        double t_2 = para1(otherArgs.intrinsicCount+otherArgs.disCount+i*6+4);
        double t_3 = para1(otherArgs.intrinsicCount+otherArgs.disCount+i*6+5);


        double x1 = (((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_1*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*x+(sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_2*z-r_3*y))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+t_1)/((sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_1*y-r_2*x))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_3*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*z+t_3);
        double y1 = ((sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_3*x-r_1*z))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_2*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*y+t_2)/((sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_1*y-r_2*x))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_3*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*z+t_3);
        double r = std::sqrt(std::pow(x1,2)+std::pow(y1,2));

        double x11=0,y11=0;
        //无畸变参数
        if(otherArgs.disCount == 0)
        {
            x11 = x1;
            y11 = y1;
        }

        double u_2=0,v_2=0,u_4=0,v_4=0,u_5=0,v_5=0,u_8=0,v_8=0,u_12=0,v_12=0;
        //k1,k2
        if(otherArgs.disCount >= 2)
        {
            u_2 = x1*(1+std::pow(r,2)*para1(otherArgs.intrinsicCount) + std::pow(r,4) * para1(otherArgs.intrinsicCount+1));
            v_2 = y1*(1+std::pow(r,2)*para1(otherArgs.intrinsicCount) + std::pow(r,4) * para1(otherArgs.intrinsicCount+1));
            x11 += u_2;
            y11 += v_2;
        }
        //k1,k2,p1,p2
        if(otherArgs.disCount >= 4)
        {
            u_4 = (2*x1*y1*para1(otherArgs.intrinsicCount+2) + (2*std::pow(x1,2) + std::pow(r,2))*para1(otherArgs.intrinsicCount+3));
            v_4 = ((2*std::pow(y1,2) + std::pow(r,2))*para1(otherArgs.intrinsicCount+2) + x1*y1*2*para1(otherArgs.intrinsicCount+3));
            x11 += u_4;
            y11 += v_4;
        }
        //k1,k2,p1,p2,k3
        if(otherArgs.disCount >= 5)
        {
            u_5 = x1*std::pow(r,6)*para1(otherArgs.intrinsicCount+4);
            v_5 = y1*std::pow(r,6)*para1(otherArgs.intrinsicCount+4);
            x11 += u_5;
            y11 += v_5;
        }
        //k1,k2,p1,p2,k3,k4,k5,k6
        if(otherArgs.disCount >= 8)
        {
            u_8 = (u_2 + u_5) / (1+std::pow(r,2)*para1(otherArgs.intrinsicCount+5) + std::pow(r,4) * para1(otherArgs.intrinsicCount+6) + std::pow(r,6)*para1(otherArgs.intrinsicCount+7)) + u_4;
            v_8 = (v_2 + v_5) / (1+std::pow(r,2)*para1(otherArgs.intrinsicCount+5) + std::pow(r,4) * para1(otherArgs.intrinsicCount+6) + std::pow(r,6)*para1(otherArgs.intrinsicCount+7)) + v_4;
            x11 = u_8;
            y11 = v_8;
        }
        //k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
        if(otherArgs.disCount >= 12)
        {
            u_12 = std::pow(r,2)*para1(otherArgs.intrinsicCount+8) + std::pow(r,4)*para1(otherArgs.intrinsicCount+9);
            v_12 = std::pow(r,2)*para1(otherArgs.intrinsicCount+10) + std::pow(r,4)*para1(otherArgs.intrinsicCount+11);
            x11 += u_12;
            y11 += v_12;
        }

        if(otherArgs.intrinsicCount == 4)
        {
            double f_x = para1(0);
            //double f_y = para1(1);
            double u_0 = para1(2);
            //double v_0 = para1(3);
            obj1 = f_x*x11+u_0;
        }
        else
        {
            double f_x = para1(0);
            double gam = para1(1);
            //double f_y = para1(2);
            double u_0 = para1(3);
            //double v_0 = para1(4);
            obj1 = f_x*x11+gam*y11+u_0;
        }


        {
            //旋转向量
            double r_1 = para2(otherArgs.intrinsicCount+otherArgs.disCount+i*6);
            double r_2 = para2(otherArgs.intrinsicCount+otherArgs.disCount+i*6+1);
            double r_3 = para2(otherArgs.intrinsicCount+otherArgs.disCount+i*6+2);
            //平移向量
            double t_1 = para2(otherArgs.intrinsicCount+otherArgs.disCount+i*6+3);
            double t_2 = para2(otherArgs.intrinsicCount+otherArgs.disCount+i*6+4);
            double t_3 = para2(otherArgs.intrinsicCount+otherArgs.disCount+i*6+5);


            double x1 = (((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_1*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*x+(sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_2*z-r_3*y))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+t_1)/((sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_1*y-r_2*x))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_3*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*z+t_3);
            double y1 = ((sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_3*x-r_1*z))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_2*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*y+t_2)/((sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_1*y-r_2*x))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_3*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*z+t_3);
            double r = std::sqrt(std::pow(x1,2)+std::pow(y1,2));

            double x11=0,y11=0;
            //无畸变参数
            if(otherArgs.disCount == 0)
            {
                x11 = x1;
                y11 = y1;
            }

            double u_2=0,v_2=0,u_4=0,v_4=0,u_5=0,v_5=0,u_8=0,v_8=0,u_12=0,v_12=0;
            //k1,k2
            if(otherArgs.disCount >= 2)
            {
                u_2 = x1*(1+std::pow(r,2)*para2(otherArgs.intrinsicCount) + std::pow(r,4) * para2(otherArgs.intrinsicCount+1));
                v_2 = y1*(1+std::pow(r,2)*para2(otherArgs.intrinsicCount) + std::pow(r,4) * para2(otherArgs.intrinsicCount+1));
                x11 += u_2;
                y11 += v_2;
            }
            //k1,k2,p1,p2
            if(otherArgs.disCount >= 4)
            {
                u_4 = (2*x1*y1*para2(otherArgs.intrinsicCount+2) + (2*std::pow(x1,2) + std::pow(r,2))*para2(otherArgs.intrinsicCount+3));
                v_4 = ((2*std::pow(y1,2) + std::pow(r,2))*para2(otherArgs.intrinsicCount+2) + x1*y1*2*para2(otherArgs.intrinsicCount+3));
                x11 += u_4;
                y11 += v_4;
            }
            //k1,k2,p1,p2,k3
            if(otherArgs.disCount >= 5)
            {
                u_5 = x1*std::pow(r,6)*para2(otherArgs.intrinsicCount+4);
                v_5 = y1*std::pow(r,6)*para2(otherArgs.intrinsicCount+4);
                x11 += u_5;
                y11 += v_5;
            }
            //k1,k2,p1,p2,k3,k4,k5,k6
            if(otherArgs.disCount >= 8)
            {
                u_8 = (u_2 + u_5) / (1+std::pow(r,2)*para2(otherArgs.intrinsicCount+5) + std::pow(r,4) * para2(otherArgs.intrinsicCount+6) + std::pow(r,6)*para2(otherArgs.intrinsicCount+7)) + u_4;
                v_8 = (v_2 + v_5) / (1+std::pow(r,2)*para2(otherArgs.intrinsicCount+5) + std::pow(r,4) * para2(otherArgs.intrinsicCount+6) + std::pow(r,6)*para2(otherArgs.intrinsicCount+7)) + v_4;
                x11 = u_8;
                y11 = v_8;
            }
            //k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
            if(otherArgs.disCount >= 12)
            {
                u_12 = std::pow(r,2)*para2(otherArgs.intrinsicCount+8) + std::pow(r,4)*para2(otherArgs.intrinsicCount+9);
                v_12 = std::pow(r,2)*para2(otherArgs.intrinsicCount+10) + std::pow(r,4)*para2(otherArgs.intrinsicCount+11);
                x11 += u_12;
                y11 += v_12;
            }


            if(otherArgs.intrinsicCount == 4)
            {

                double f_x = para2(0);
                //double f_y = para2(1);
                double u_0 = para2(2);
                // double v_0 = para2(3);

                obj2 = f_x*x11+u_0;
            }
            else
            {

                double f_x = para2(0);
                double gam = para2(1);
                //double f_y = para2(2);
                double u_0 = para2(3);
                // double v_0 = para2(4);

                obj2 = f_x*x11+gam*y11+u_0;
            }

        }

        return (obj2 - obj1) / (2 * DERIV_STEP);
    }

    //求偏导2
    double PartialDeriv_2(const Eigen::VectorXd& parameter,int paraIndex, const S_CameraOtherParameter &otherArgs,int i,int j)
    {
        Eigen::VectorXd para1 = parameter;
        Eigen::VectorXd para2 = parameter;
        para1(paraIndex) -= DERIV_STEP;
        para2(paraIndex) += DERIV_STEP;


        double obj1 =0,obj2 =0;
        //坐标
        double x = otherArgs.srcL.at(i)(j,0);
        double y = otherArgs.srcL.at(i)(j,1);
        double z = 0;
        //旋转向量
        double r_1 = para1(otherArgs.intrinsicCount+otherArgs.disCount+i*6);
        double r_2 = para1(otherArgs.intrinsicCount+otherArgs.disCount+i*6+1);
        double r_3 = para1(otherArgs.intrinsicCount+otherArgs.disCount+i*6+2);
        //平移向量
        double t_1 = para1(otherArgs.intrinsicCount+otherArgs.disCount+i*6+3);
        double t_2 = para1(otherArgs.intrinsicCount+otherArgs.disCount+i*6+4);
        double t_3 = para1(otherArgs.intrinsicCount+otherArgs.disCount+i*6+5);


        double x1 = (((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_1*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*x+(sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_2*z-r_3*y))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+t_1)/((sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_1*y-r_2*x))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_3*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*z+t_3);
        double y1 = ((sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_3*x-r_1*z))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_2*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*y+t_2)/((sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_1*y-r_2*x))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_3*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*z+t_3);
        double r = std::sqrt(std::pow(x1,2)+std::pow(y1,2));

        double x11=0,y11=0;
        //无畸变参数
        if(otherArgs.disCount == 0)
        {
            x11 = x1;
            y11 = y1;
        }
        double u_2=0,v_2=0,u_4=0,v_4=0,u_5=0,v_5=0,u_8=0,v_8=0,u_12=0,v_12=0;
        //k1,k2
        if(otherArgs.disCount >= 2)
        {
            u_2 = x1*(1+std::pow(r,2)*para1(otherArgs.intrinsicCount) + std::pow(r,4) * para1(otherArgs.intrinsicCount+1));
            v_2 = y1*(1+std::pow(r,2)*para1(otherArgs.intrinsicCount) + std::pow(r,4) * para1(otherArgs.intrinsicCount+1));
            x11 += u_2;
            y11 += v_2;
        }
        //k1,k2,p1,p2
        if(otherArgs.disCount >= 4)
        {
            u_4 = (2*x1*y1*para1(otherArgs.intrinsicCount+2) + (2*std::pow(x1,2) + std::pow(r,2))*para1(otherArgs.intrinsicCount+3));
            v_4 = ((2*std::pow(y1,2) + std::pow(r,2))*para1(otherArgs.intrinsicCount+2) + x1*y1*2*para1(otherArgs.intrinsicCount+3));
            x11 += u_4;
            y11 += v_4;
        }
        //k1,k2,p1,p2,k3
        if(otherArgs.disCount >= 5)
        {
            u_5 = x1*std::pow(r,6)*para1(otherArgs.intrinsicCount+4);
            v_5 = y1*std::pow(r,6)*para1(otherArgs.intrinsicCount+4);
            x11 += u_5;
            y11 += v_5;
        }
        //k1,k2,p1,p2,k3,k4,k5,k6
        if(otherArgs.disCount >= 8)
        {
            u_8 = (u_2 + u_5) / (1+std::pow(r,2)*para1(otherArgs.intrinsicCount+5) + std::pow(r,4) * para1(otherArgs.intrinsicCount+6) + std::pow(r,6)*para1(otherArgs.intrinsicCount+7)) + u_4;
            v_8 = (v_2 + v_5) / (1+std::pow(r,2)*para1(otherArgs.intrinsicCount+5) + std::pow(r,4) * para1(otherArgs.intrinsicCount+6) + std::pow(r,6)*para1(otherArgs.intrinsicCount+7)) + v_4;
            x11 = u_8;
            y11 = v_8;
        }
        //k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
        if(otherArgs.disCount >= 12)
        {
            u_12 = std::pow(r,2)*para1(otherArgs.intrinsicCount+8) + std::pow(r,4)*para1(otherArgs.intrinsicCount+9);
            v_12 = std::pow(r,2)*para1(otherArgs.intrinsicCount+10) + std::pow(r,4)*para1(otherArgs.intrinsicCount+11);
            x11 += u_12;
            y11 += v_12;
        }

        if(otherArgs.intrinsicCount == 4)
        {
            //double f_x = para1(0);
            double f_y = para1(1);
            //double u_0 = para1(2);
            double v_0 = para1(3);

            obj1 = f_y*y11+v_0;
        }
        else
        {

            //double f_x = para1(0);
            // double gam = para1(1);
            double f_y = para1(2);
            //double u_0 = para1(3);
            double v_0 = para1(4);

            obj1 = f_y*y11+v_0;
        }





        {
            //旋转向量
            double r_1 = para2(otherArgs.intrinsicCount+otherArgs.disCount+i*6);
            double r_2 = para2(otherArgs.intrinsicCount+otherArgs.disCount+i*6+1);
            double r_3 = para2(otherArgs.intrinsicCount+otherArgs.disCount+i*6+2);
            //平移向量
            double t_1 = para2(otherArgs.intrinsicCount+otherArgs.disCount+i*6+3);
            double t_2 = para2(otherArgs.intrinsicCount+otherArgs.disCount+i*6+4);
            double t_3 = para2(otherArgs.intrinsicCount+otherArgs.disCount+i*6+5);


            double x1 = (((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_1*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*x+(sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_2*z-r_3*y))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+t_1)/((sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_1*y-r_2*x))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_3*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*z+t_3);
            double y1 = ((sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_3*x-r_1*z))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_2*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*y+t_2)/((sin(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*(r_1*y-r_2*x))/std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+((1-cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))))*r_3*(r_1*x+r_3*z+r_2*y))/(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2))+cos(std::sqrt(std::pow(r_1,2)+std::pow(r_2,2)+std::pow(r_3,2)))*z+t_3);
            double r = std::sqrt(std::pow(x1,2)+std::pow(y1,2));

            double x11=0,y11=0;
            //无畸变参数
            if(otherArgs.disCount == 0)
            {
                x11 = x1;
                y11 = y1;
            }
            double u_2=0,v_2=0,u_4=0,v_4=0,u_5=0,v_5=0,u_8=0,v_8=0,u_12=0,v_12=0;
            //k1,k2
            if(otherArgs.disCount >= 2)
            {
                u_2 = x1*(1+std::pow(r,2)*para2(otherArgs.intrinsicCount) + std::pow(r,4) * para2(otherArgs.intrinsicCount+1));
                v_2 = y1*(1+std::pow(r,2)*para2(otherArgs.intrinsicCount) + std::pow(r,4) * para2(otherArgs.intrinsicCount+1));
                x11 += u_2;
                y11 += v_2;
            }
            //k1,k2,p1,p2
            if(otherArgs.disCount >= 4)
            {
                u_4 = (2*x1*y1*para2(otherArgs.intrinsicCount+2) + (2*std::pow(x1,2) + std::pow(r,2))*para2(otherArgs.intrinsicCount+3));
                v_4 = ((2*std::pow(y1,2) + std::pow(r,2))*para2(otherArgs.intrinsicCount+2) + x1*y1*2*para2(otherArgs.intrinsicCount+3));
                x11 += u_4;
                y11 += v_4;
            }
            //k1,k2,p1,p2,k3
            if(otherArgs.disCount >= 5)
            {
                u_5 = x1*std::pow(r,6)*para2(otherArgs.intrinsicCount+4);
                v_5 = y1*std::pow(r,6)*para2(otherArgs.intrinsicCount+4);
                x11 += u_5;
                y11 += v_5;
            }
            //k1,k2,p1,p2,k3,k4,k5,k6
            if(otherArgs.disCount >= 8)
            {
                u_8 = (u_2 + u_5) / (1+std::pow(r,2)*para2(otherArgs.intrinsicCount+5) + std::pow(r,4) * para2(otherArgs.intrinsicCount+6) + std::pow(r,6)*para2(otherArgs.intrinsicCount+7)) + u_4;
                v_8 = (v_2 + v_5) / (1+std::pow(r,2)*para2(otherArgs.intrinsicCount+5) + std::pow(r,4) * para2(otherArgs.intrinsicCount+6) + std::pow(r,6)*para2(otherArgs.intrinsicCount+7)) + v_4;
                x11 = u_8;
                y11 = v_8;
            }
            //k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
            if(otherArgs.disCount >= 12)
            {
                u_12 = std::pow(r,2)*para2(otherArgs.intrinsicCount+8) + std::pow(r,4)*para2(otherArgs.intrinsicCount+9);
                v_12 = std::pow(r,2)*para2(otherArgs.intrinsicCount+10) + std::pow(r,4)*para2(otherArgs.intrinsicCount+11);
                x11 += u_12;
                y11 += v_12;
            }



            if(otherArgs.intrinsicCount == 4)
            {
                //double f_x = para2(0);
                double f_y = para2(1);
                //double u_0 = para2(2);
                double v_0 = para2(3);

                obj2 = f_y*y11+v_0;
            }
            else
            {
                //double f_x = para2(0);
                //double gam = para2(1);
                double f_y = para2(2);
                //double u_0 = para2(3);
                double v_0 = para2(4);

                obj2 = f_y*y11+v_0;
            }

        }

        return (obj2 - obj1) / (2 * DERIV_STEP);
    }
public:

    Eigen::MatrixXd  operator()(const Eigen::VectorXd& parameter,const S_CameraOtherParameter &otherArgs)
    {
        //获取数据总个数
        int allCount=0;
        for(int i=0;i<otherArgs.imageCount;++i)
        {
            allCount += otherArgs.srcL.at(i).rows();
        }

        //初始化雅可比矩阵都为0
        Eigen::MatrixXd Jac = Eigen::MatrixXd::Zero(allCount*2,parameter.rows());

        int k=0;
        for(int i=0;i<otherArgs.imageCount;++i)
        {
            Eigen::MatrixXd src = otherArgs.srcL.at(i);
            int srcCount = src.rows();


            //遍历当前图片数据点
            for(int j=0;j<srcCount;++j)
            {

                //内参偏导
                if(otherArgs.intrinsicCount == 4)
                {
                    Jac(k,0) = PartialDeriv_1(parameter,0,otherArgs,i,j);
                    Jac(k,1) = 0;
                    Jac(k,2) = 1;
                    Jac(k,3) = 0;

                    Jac(k+1,0) = 0;
                    Jac(k+1,1) = PartialDeriv_2(parameter,1,otherArgs,i,j);
                    Jac(k+1,2) = 0;
                    Jac(k+1,3) = 1;
                }
                else
                {
                    Jac(k,0) = PartialDeriv_1(parameter,0,otherArgs,i,j);
                    Jac(k,1) = PartialDeriv_1(parameter,1,otherArgs,i,j);
                    Jac(k,2) = 0;
                    Jac(k,3) = 1;
                    Jac(k,4) = 0;

                    Jac(k+1,0) = 0;
                    Jac(k+1,1) = 0;
                    Jac(k+1,2) = PartialDeriv_2(parameter,2,otherArgs,i,j);
                    Jac(k+1,3) = 0;
                    Jac(k+1,4) = 1;
                }



                //畸变偏导
                //k1,k2
                if(otherArgs.disCount >= 2)
                {
                    Jac(k,otherArgs.intrinsicCount) = PartialDeriv_1(parameter,otherArgs.intrinsicCount,otherArgs,i,j);
                    Jac(k,otherArgs.intrinsicCount+1) = PartialDeriv_1(parameter,otherArgs.intrinsicCount+1,otherArgs,i,j);

                    Jac(k+1,otherArgs.intrinsicCount) = PartialDeriv_2(parameter,otherArgs.intrinsicCount,otherArgs,i,j);
                    Jac(k+1,otherArgs.intrinsicCount+1) = PartialDeriv_2(parameter,otherArgs.intrinsicCount+1,otherArgs,i,j);
                }
                //k1,k2,p1,p2
                if(otherArgs.disCount >= 4)
                {
                    Jac(k,otherArgs.intrinsicCount+2) = PartialDeriv_1(parameter,otherArgs.intrinsicCount+2,otherArgs,i,j);
                    Jac(k,otherArgs.intrinsicCount+3) = PartialDeriv_1(parameter,otherArgs.intrinsicCount+3,otherArgs,i,j);

                    Jac(k+1,otherArgs.intrinsicCount+2) = PartialDeriv_2(parameter,otherArgs.intrinsicCount+2,otherArgs,i,j);
                    Jac(k+1,otherArgs.intrinsicCount+3) = PartialDeriv_2(parameter,otherArgs.intrinsicCount+3,otherArgs,i,j);
                }
                //k1,k2,p1,p2,k3
                if(otherArgs.disCount >= 5)
                {
                    Jac(k,otherArgs.intrinsicCount+4) = PartialDeriv_1(parameter,otherArgs.intrinsicCount+4,otherArgs,i,j);

                    Jac(k+1,otherArgs.intrinsicCount+4) = PartialDeriv_2(parameter,otherArgs.intrinsicCount+4,otherArgs,i,j);
                }
                //k1,k2,p1,p2,k3,k4,k5,k6
                if(otherArgs.disCount >= 8)
                {
                    Jac(k,otherArgs.intrinsicCount+5) = PartialDeriv_1(parameter,otherArgs.intrinsicCount+5,otherArgs,i,j);
                    Jac(k,otherArgs.intrinsicCount+6) = PartialDeriv_1(parameter,otherArgs.intrinsicCount+6,otherArgs,i,j);
                    Jac(k,otherArgs.intrinsicCount+7) = PartialDeriv_1(parameter,otherArgs.intrinsicCount+7,otherArgs,i,j);

                    Jac(k+1,otherArgs.intrinsicCount+5) = PartialDeriv_2(parameter,otherArgs.intrinsicCount+5,otherArgs,i,j);
                    Jac(k+1,otherArgs.intrinsicCount+6) = PartialDeriv_2(parameter,otherArgs.intrinsicCount+6,otherArgs,i,j);
                    Jac(k+1,otherArgs.intrinsicCount+7) = PartialDeriv_2(parameter,otherArgs.intrinsicCount+7,otherArgs,i,j);
                }
                //k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
                if(otherArgs.disCount >= 12)
                {
                    Jac(k,otherArgs.intrinsicCount+8) = PartialDeriv_1(parameter,otherArgs.intrinsicCount+8,otherArgs,i,j);
                    Jac(k,otherArgs.intrinsicCount+9) = PartialDeriv_1(parameter,otherArgs.intrinsicCount+9,otherArgs,i,j);

                    Jac(k+1,otherArgs.intrinsicCount+10) = PartialDeriv_2(parameter,otherArgs.intrinsicCount+10,otherArgs,i,j);
                    Jac(k+1,otherArgs.intrinsicCount+11) = PartialDeriv_2(parameter,otherArgs.intrinsicCount+11,otherArgs,i,j);
                }

                //外参偏导 r1,r2,r3,t1,t2,t3

                Jac(k,otherArgs.intrinsicCount+otherArgs.disCount + i*6) = PartialDeriv_1(parameter,otherArgs.intrinsicCount+otherArgs.disCount+i*6,otherArgs,i,j);
                Jac(k,otherArgs.intrinsicCount+otherArgs.disCount + i*6 + 1) = PartialDeriv_1(parameter,otherArgs.intrinsicCount+otherArgs.disCount+i*6+1,otherArgs,i,j);
                Jac(k,otherArgs.intrinsicCount+otherArgs.disCount + i*6 + 2) = PartialDeriv_1(parameter,otherArgs.intrinsicCount+otherArgs.disCount+i*6+2,otherArgs,i,j);
                Jac(k,otherArgs.intrinsicCount+otherArgs.disCount + i*6 + 3) = PartialDeriv_1(parameter,otherArgs.intrinsicCount+otherArgs.disCount+i*6+3,otherArgs,i,j);
                Jac(k,otherArgs.intrinsicCount+otherArgs.disCount + i*6 + 4) = 0;
                Jac(k,otherArgs.intrinsicCount+otherArgs.disCount + i*6 + 5) = PartialDeriv_1(parameter,otherArgs.intrinsicCount+otherArgs.disCount+i*6+5,otherArgs,i,j);

                Jac(k+1,otherArgs.intrinsicCount+otherArgs.disCount + i*6) = PartialDeriv_2(parameter,otherArgs.intrinsicCount+otherArgs.disCount+i*6,otherArgs,i,j);
                Jac(k+1,otherArgs.intrinsicCount+otherArgs.disCount + i*6 + 1) = PartialDeriv_2(parameter,otherArgs.intrinsicCount+otherArgs.disCount+i*6+1,otherArgs,i,j);;
                Jac(k+1,otherArgs.intrinsicCount+otherArgs.disCount + i*6 + 2) = PartialDeriv_2(parameter,otherArgs.intrinsicCount+otherArgs.disCount+i*6+2,otherArgs,i,j);;
                Jac(k+1,otherArgs.intrinsicCount+otherArgs.disCount + i*6 + 3) = 0;
                Jac(k+1,otherArgs.intrinsicCount+otherArgs.disCount + i*6 + 4) = PartialDeriv_2(parameter,otherArgs.intrinsicCount+otherArgs.disCount+i*6+4,otherArgs,i,j);;
                Jac(k+1,otherArgs.intrinsicCount+otherArgs.disCount + i*6 + 5) = PartialDeriv_2(parameter,otherArgs.intrinsicCount+otherArgs.disCount+i*6+5,otherArgs,i,j);;



                k += 2;
            }
        }
        return Jac;
    }
};

#endif // CAMERACALIBRATIONALGORITHM_H
