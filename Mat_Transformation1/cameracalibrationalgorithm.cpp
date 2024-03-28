#include "cameracalibrationalgorithm.h"


/* 普通最小二乘 Ax = B
 * (A^T * A) * x = A^T * B
 * x = (A^T * A)^-1 * A^T * B
 */
Eigen::VectorXd LeastSquares(Eigen::MatrixXd A, Eigen::VectorXd B)
{
    //获取矩阵的行数和列数
    int rows =  A.rows();
    int col = A.cols();
    //A的转置矩阵
    Eigen::MatrixXd AT(col,rows);
    //AT.resize(col,rows);

    //x矩阵
    Eigen::VectorXd x(col,1);
    //x.resize(col,1);

    //转置 AT
    AT = A.transpose();

    //x = (A^T * A)^-1 * A^T * B
    x = ((AT * A).inverse()) * (AT * B);
    return x;

}
//旋转矩阵 --> 旋转向量    :罗德里格斯公式逆变换
Eigen::Vector3d Rodrigues(const Eigen::Matrix3d& R)
{
    Eigen::AngleAxisd rotAA2(R);
    Eigen::Vector3d r{rotAA2.angle() * rotAA2.axis()};


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
Eigen::Matrix3d Rodrigues(const Eigen::Vector3d& _r)
{
    // 第1种方法
    Eigen::AngleAxisd  rotAA{_r.norm(), _r.normalized()};
    Eigen::Matrix3d R {rotAA.toRotationMatrix()};

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



CameraCalibration::CameraCalibration(std::vector<std::vector<PointF> > objectPointsL, std::vector<std::vector<PointF> > imagePointsL,int _disCount,int _intrinsicCount,double _epsilon ,int _maxIteCount)
{
    //内参个数
    m_intrinsicCount = _intrinsicCount;
    //畸变个数
    m_disCount = _disCount;

    //收敛精度
    m_epsilon = _epsilon ;
    //最大迭代次数
    m_maxIteCount = _maxIteCount;
    std::vector<Eigen::Matrix3d> HList;
    //点转矩阵形式
    std::pair<std::vector<Eigen::MatrixXd>,std::vector<Eigen::MatrixXd>> pair = PointsToMat(objectPointsL,imagePointsL);
    std::vector<Eigen::MatrixXd> srcL = pair.first;
    std::vector<Eigen::MatrixXd> dstL = pair.second;
    //获取单应性矩阵
    for(int i=0;i<srcL.size();++i)
    {
        Eigen::Matrix3d H = GetHomography(srcL[i],dstL[i]);
        HList.push_back(H);
    }

    //内参 -  需要单应性矩阵
    Eigen::Matrix3d K = GetIntrinsicParameter(HList);

    //外参 -  需要单应性矩阵 和 内参
    std::vector<Eigen::MatrixXd> W = GetExternalParameter(HList,K);

    //无畸变优化 - 可有可无，对结果无影响
    //Eigen::VectorXd disCoeff1(0);
    //OptimizeParameter(srcL,dstL,K,disCoeff1,W);

    //带畸变优化
    Eigen::VectorXd disCoeff = Eigen::VectorXd::Zero(m_disCount);
    GetDistortionCoeff(srcL,dstL,K,W,disCoeff);
    double reprojErr = OptimizeParameter(srcL,dstL,K,disCoeff,W);

    //内参
    m_intrinsicParameter = K;

    //畸变系数
    m_distortionCoeff = disCoeff;

    //外参
    m_externalParams = W;

    //每张图片单应性矩阵
    m_homographyList = HList;

    //重投影误差
    m_reprojErr = reprojErr;

}
//获取相机参数
S_CameraP CameraCalibration::GetCameraParameter()
{
    S_CameraP CP;
    CP.intrinsicParameter = m_intrinsicParameter;
    CP.distortionCoeff = m_distortionCoeff;
    CP.externalParams = m_externalParams;
    CP.homographyList = m_homographyList;
    CP.reprojErr = m_reprojErr;
    CP.reprojErrL = m_reprojErrL;
    return CP;
}
//物体坐标和像素坐标转换矩阵形式
std::pair<std::vector<Eigen::MatrixXd>,std::vector<Eigen::MatrixXd>> CameraCalibration::PointsToMat(std::vector<std::vector<PointF> > objectPointsL, std::vector<std::vector<PointF> > imagePointsL)
{
    int size = objectPointsL.size();
    std::vector<Eigen::MatrixXd> srcL ,dstL;

    //判断数据个数
    for(int i=0;i<size;++i)
    {
        if(objectPointsL.at(i).size() != imagePointsL.at(i).size())
        {
            //报错:终止
            std::cout<<"错误:第"<< std::to_string(i)<<"张物体点和图像点数据个数不匹配!"<<std::endl;

            return {srcL,dstL};
        }
    }
    //构造数据矩阵
    for(int i=0;i<size;++i)
    {
        int dataCount = objectPointsL.at(i).size();
        std::vector<PointF> objectPoints = objectPointsL.at(i);
        std::vector<PointF> imagePoints = imagePointsL.at(i);

        Eigen::MatrixXd src(dataCount,2) ,dst(dataCount,2);
        for(int j=0;j<dataCount;++j)
        {
            src(j,0) = objectPoints.at(j).x();
            src(j,1) = objectPoints.at(j).y();

            dst(j,0) = imagePoints.at(j).x();
            dst(j,1) = imagePoints.at(j).y();
        }
        srcL.push_back(src);
        dstL.push_back(dst);
    }
    return {srcL,dstL};
}
//求单应性矩阵H
Eigen::Matrix3d  CameraCalibration::GetHomography(const Eigen::MatrixXd& src,const Eigen::MatrixXd& dst)
{

    //获取初始单应性矩阵 -- svd
    Eigen::VectorXd H = GetInitialH(src,dst);
    std::vector<Eigen::MatrixXd> otherValue{src,dst};


    //非线性优化 H 参数 -- LM算法
    //这段代码对单应性矩阵进行提前优化，也可以注释掉，不影响结果，最后会统一优化。
    {
        H =LevenbergMarquardtAlgorithm(H,otherValue,HomographyResidualsVector(),HomographyJacobi());
        H /= H(8);
    }

    return  H.reshaped<Eigen::RowMajor>(3,3);
}
//根据单应性矩阵H返回pq位置对应的v向量
Eigen::VectorXd CameraCalibration::CreateV(int p, int q,const Eigen::Matrix3d& H)
{
    Eigen::VectorXd v(6,1);

    v << H(0, p) * H(0, q),
            H(0, p) * H(1, q) + H(1, p) * H(0, q),
            H(1, p) * H(1, q),
            H(2, p) * H(0, q) + H(0, p) * H(2, q),
            H(2, p) * H(1, q) + H(1, p) * H(2, q),
            H(2, p) * H(2, q);
    return v;

}
//求相机内参
Eigen::Matrix3d  CameraCalibration::GetIntrinsicParameter(const std::vector<Eigen::Matrix3d>& HList)
{
    int HCount = HList.size();
    //构建V矩阵
    Eigen::MatrixXd V(HCount*2,6);
    for(int i=0;i<HCount;++i)
    {
        V.row(i*2) = CreateV(0, 1, HList.at(i)).transpose();
        V.row(i*2+1) = (CreateV(0, 0, HList.at(i)) - CreateV(1, 1, HList.at(i))).transpose();
    }

    //Vb = 0
    //svd分解求x
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(V, Eigen::ComputeFullU | Eigen::ComputeFullV);
    //获取V矩阵最后一列就是b的值
    Eigen::VectorXd b = svd.matrixV().rightCols(1);

    double B11 = b(0);
    double B12 = b(1);
    double B22 = b(2);
    double B13 = b(3);
    double B23 = b(4);
    double B33 = b(5);

    double v0 = (B12*B13 - B11*B23) /  (B11*B22 - B12*B12);
    double lambda = B33 - (B13*B13 + v0*(B12*B13 - B11*B23))/B11;
    double alpha = std::sqrt(lambda / B11);
    double beta = std::sqrt(lambda*B11 / (B11*B22 - B12 *B12));
    double gamma = (- B12*alpha*alpha*beta) / lambda;
    double u0 = (gamma*v0 / beta) - (B13 * alpha * alpha / lambda);

    Eigen::Matrix3d K;

    K<<alpha,(m_intrinsicCount == 4 ? 0 : gamma),u0,
            0,beta,v0,
            0,0,1;

    return  K;
}


//求相机外参
std::vector<Eigen::MatrixXd>  CameraCalibration::GetExternalParameter(const std::vector<Eigen::Matrix3d>& HList,const Eigen::Matrix3d& intrinsicParam)
{
    std::vector<Eigen::MatrixXd> exterParame;
    //内参逆矩阵
    Eigen::Matrix3d intrinsicParamInv = intrinsicParam.inverse();
    int HCount = HList.size();
    for(int i=0;i<HCount;++i)
    {
        Eigen::Matrix3d H = HList.at(i);
        Eigen::Vector3d h0,h1,h2;
        h0 = H.col(0);
        h1 = H.col(1);
        h2 = H.col(2);

        Eigen::Vector3d  r0,r1,r2,t;
        //比例因子λ
        double scaleFactor = 1.0 / (intrinsicParamInv * h0).lpNorm<2>();
        r0 = scaleFactor * (intrinsicParamInv * h0);
        r1 = scaleFactor * (intrinsicParamInv * h1);
        r2 = r0.cross(r1);
        t = scaleFactor * (intrinsicParamInv * h2);
        Eigen::MatrixXd Rt(3,4);
        Rt.col(0) = r0;
        Rt.col(1) = r1;
        Rt.col(2) = r2;
        Rt.col(3) = t;
        exterParame.push_back(Rt);
    }

    return exterParame;
}


//获取标准差
double CameraCalibration::StdDiffer(const Eigen::VectorXd& data)
{
    //获取平均值
    double mean = data.mean();
    //std::sqrt((Σ(x-_x)²) / n)
    return std::sqrt((data.array() - mean).pow(2).sum() / data.rows());
}

// 归一化
Eigen::Matrix3d CameraCalibration::Normalization (const Eigen::MatrixXd& P)
{
    Eigen::Matrix3d T;
    double cx = P.col ( 0 ).mean();
    double cy = P.col ( 1 ).mean();

    double stdx = StdDiffer(P.col(0));
    double stdy = StdDiffer(P.col(1));;


    double sqrt_2 = std::sqrt ( 2 );
    double scalex = sqrt_2 / stdx;
    double scaley = sqrt_2 / stdy;

    T << scalex, 0, -scalex*cx,
            0, scaley, -scaley*cy,
            0, 0, 1;
    return T;
}

//获取初始矩阵H
Eigen::VectorXd CameraCalibration::GetInitialH (const Eigen::MatrixXd& srcNormal,const Eigen::MatrixXd& dstNormal )
{
    Eigen::Matrix3d realNormMat = Normalization(srcNormal);
    Eigen::Matrix3d picNormMat = Normalization(dstNormal);

    int n = srcNormal.rows();
    // 2. DLT
    Eigen::MatrixXd input ( 2*n, 9 );

    for ( int i=0; i<n; ++i )
    {
        //转换齐次坐标
        Eigen::MatrixXd singleSrcCoor(3,1),singleDstCoor(3,1);
        singleSrcCoor(0,0) = srcNormal(i,0);
        singleSrcCoor(1,0) = srcNormal(i,1);
        singleSrcCoor(2,0) = 1;
        singleDstCoor(0,0) = dstNormal(i,0);
        singleDstCoor(1,0) = dstNormal(i,1);
        singleDstCoor(2,0) = 1;


        //坐标归一化
        Eigen::MatrixXd realNorm(3,1),picNorm(3,1);
        realNorm = realNormMat * singleSrcCoor;
        picNorm = picNormMat * singleDstCoor;

        input ( 2*i, 0 ) = realNorm ( 0, 0 );
        input ( 2*i, 1 ) = realNorm ( 1, 0 );
        input ( 2*i, 2 ) = 1.;
        input ( 2*i, 3 ) = 0.;
        input ( 2*i, 4 ) = 0.;
        input ( 2*i, 5 ) = 0.;
        input ( 2*i, 6 ) = -picNorm ( 0, 0 ) * realNorm ( 0, 0 );
        input ( 2*i, 7 ) = -picNorm ( 0, 0 ) * realNorm ( 1, 0 );
        input ( 2*i, 8 ) = -picNorm ( 0, 0 );

        input ( 2*i+1, 0 ) = 0.;
        input ( 2*i+1, 1 ) = 0.;
        input ( 2*i+1, 2 ) = 0.;
        input ( 2*i+1, 3 ) = realNorm ( 0, 0 );
        input ( 2*i+1, 4 ) = realNorm ( 1, 0 );
        input ( 2*i+1, 5 ) = 1.;
        input ( 2*i+1, 6 ) = -picNorm ( 1, 0 ) * realNorm ( 0, 0 );
        input ( 2*i+1, 7 ) = -picNorm ( 1, 0 ) * realNorm ( 1, 0 );
        input ( 2*i+1, 8 ) = -picNorm ( 1, 0 );
    }

    // 3. SVD分解
    Eigen::JacobiSVD<Eigen::MatrixXd> svdSolver ( input, Eigen::ComputeFullU | Eigen::ComputeFullV );
    Eigen::MatrixXd V = svdSolver.matrixV();
    Eigen::Matrix3d H = V.rightCols(1).reshaped<Eigen::RowMajor>(3,3);
    //去归一化
    H = (picNormMat.inverse() * H) * realNormMat;
    H /= H(2,2);
    return H.reshaped<Eigen::RowMajor>(9,1);
}

//获取畸变系数 k1,k2,[p1,p2,[k3]]
void CameraCalibration::GetDistortionCoeff(const std::vector<Eigen::MatrixXd>&  srcL,const  std::vector<Eigen::MatrixXd>&  dstL,const Eigen::Matrix3d& intrinsicParam ,const std::vector<Eigen::MatrixXd>& externalParams,Eigen::VectorXd & disCoeff)
{
    //按照畸变个数获取参数
    int disCount = disCoeff.rows();

    if(!(disCount == 2 || disCount == 4 || disCount == 5 || disCount == 8 || disCount == 12))
    {
        std::cout<<"畸变参数个数按照数组大小为2或者4或者5,请重新设置数组大小！"<<std::endl;
        return;
    }
    int size = srcL.size();
    double u0 = intrinsicParam(0,2);
    double v0 = intrinsicParam(1,2);
    int rowS = 0;
    int k =  0;
    //获取数据个数
    for(int i=0;i<size;++i)
    {
        rowS += srcL.at(i).rows();
    }
    //初始化数据大小
    Eigen::MatrixXd D(rowS*2,disCount),d(rowS*2,1);
    for(int i=0;i<size;++i)
    {
        Eigen::MatrixXd src = srcL.at(i);
        int dataCount = src.rows();
        Eigen::MatrixXd dst = dstL.at(i);
        Eigen::MatrixXd externalParam = externalParams.at(i);

        for(int j=0;j<dataCount;++j)
        {
            //转换齐次坐标
            Eigen::VectorXd singleCoor(4);
            singleCoor(0) = src(j,0);
            singleCoor(1) = src(j,1);
            singleCoor(2) = 0.0;
            singleCoor(3) = 1.0;

            //用现有的内参及外参求估计图像坐标
            Eigen::VectorXd imageCoorEstimate = intrinsicParam * externalParam * singleCoor;
            //归一化图像坐标
            double u_estimate = imageCoorEstimate(0)/imageCoorEstimate(2);
            double v_estimate = imageCoorEstimate(1)/imageCoorEstimate(2);

            //相机坐标系下的坐标
            Eigen::VectorXd normCoor = externalParam * singleCoor;
            //归一化坐标
            normCoor /= normCoor(2);

            double r = std::sqrt(std::pow(normCoor(0),2) + std::pow(normCoor(1),2));

            //k1,k2
            if(disCount >= 2)
            {
                D(k,0) = (u_estimate - u0)*std::pow(r,2);
                D(k,1) = (u_estimate - u0)*std::pow(r,4);

                D(k+1,0) = (v_estimate - v0)*std::pow(r,2);
                D(k+1,1) = (v_estimate - v0)*std::pow(r,4);
            }
            //k1,k2,p1,p2
            if(disCount >= 4)
            {
                D(k,2) = (u_estimate - u0)*(v_estimate - v0)*2;
                D(k,3) = 2 * std::pow((u_estimate - u0),2) + std::pow(r,2);

                D(k+1,2) = 2 * std::pow((v_estimate - v0),2) + std::pow(r,2);
                D(k+1,3) = (u_estimate - u0)*(v_estimate - v0)*2;
            }
            //k1,k2,p1,p2,k3
            if(disCount >= 5)
            {
                D(k,4) = (u_estimate - u0)*std::pow(r,6);

                D(k+1,4) = (v_estimate - v0)*std::pow(r,6);
            }
            d(k,0) = dst(j,0) - u_estimate;
            d(k+1,0) = dst(j,1) - v_estimate;
            k += 2;
        }
    }
    // 最小二乘求解畸变系数
    disCoeff = LeastSquares(D,d);

}



//整合所有参数(内参,畸变系数,外参)到一个向量中
Eigen::VectorXd CameraCalibration::ComposeParameter(const Eigen::Matrix3d& intrinsicParam ,const Eigen::VectorXd& distortionCoeff,const std::vector<Eigen::MatrixXd>& externalParams)
{
    //畸变参数个数
    int disCount = distortionCoeff.rows();

    //外参个数
    int exterCount=0;
    for(int i=0;i<externalParams.size();++i)
    {
        //一张图片的外参个数 R->r(9->3) + t 3 = 6
        exterCount += 6;
    }

    Eigen::VectorXd P(m_intrinsicCount+disCount+exterCount);

    if(m_intrinsicCount == 4)
    {
        //整合内参
        P(0) = intrinsicParam(0,0);
        P(1) = intrinsicParam(1,1);
        P(2) = intrinsicParam(0,2);
        P(3) = intrinsicParam(1,2);
    }
    else
    {
        //整合内参
        P(0) = intrinsicParam(0,0);
        P(1) = intrinsicParam(0,1);
        P(2) = intrinsicParam(1,1);
        P(3) = intrinsicParam(0,2);
        P(4) = intrinsicParam(1,2);
    }



    //整合畸变
    for(int i=0;i<disCount;++i)
    {
        P(m_intrinsicCount+i) = distortionCoeff(i);
    }

    //整合外参
    for(int i=0;i<externalParams.size();++i)
    {
        Eigen::Matrix3d R = externalParams.at(i).block(0,0,3,3);
        //旋转矩阵转旋转向量
        Eigen::Vector3d r = Rodrigues(R);
        Eigen::Vector3d t = externalParams.at(i).col(3);

        P(m_intrinsicCount+disCount+i*6) = r(0);
        P(m_intrinsicCount+disCount+i*6+1) = r(1);
        P(m_intrinsicCount+disCount+i*6+2) = r(2);

        P(m_intrinsicCount+disCount+i*6+3) = t(0);
        P(m_intrinsicCount+disCount+i*6+4) = t(1);
        P(m_intrinsicCount+disCount+i*6+5) = t(2);
    }

    return P;
}
//分解所有参数  得到对应的内参,畸变矫正系数,外参
void CameraCalibration::DecomposeParamter(const Eigen::VectorXd &P, Eigen::Matrix3d& intrinsicParam , Eigen::VectorXd& distortionCoeff, std::vector<Eigen::MatrixXd>& externalParams)
{
    if(m_intrinsicCount == 4)
    {
        //内参
        intrinsicParam << P(0),0,P(2),
            0,P(1),P(3),
            0,0,1;
    }
    else
    {
        //内参
        intrinsicParam << P(0),P(1),P(3),
            0,P(2),P(4),
            0,0,1;
    }




    //畸变
    for(int i =0;i<distortionCoeff.rows();++i)
    {
        distortionCoeff(i) = P(m_intrinsicCount+i);
    }
    //外参
    for(int i=0;i<externalParams.size();++i)
    {
        Eigen::Vector3d r,t;
        r(0) = P(m_intrinsicCount+distortionCoeff.rows()+i*6);
        r(1) =  P(m_intrinsicCount+distortionCoeff.rows()+i*6+1) ;
        r(2) =  P(m_intrinsicCount+distortionCoeff.rows()+i*6+2);

        t(0) =  P(m_intrinsicCount+distortionCoeff.rows()+i*6+3) ;
        t(1) =  P(m_intrinsicCount+distortionCoeff.rows()+i*6+4);
        t(2) =  P(m_intrinsicCount+distortionCoeff.rows()+i*6+5) ;

        Eigen::Matrix3d R = Rodrigues(r);
        externalParams[i].block(0,0,3,3) = R;
        externalParams[i].col(3) = t;
    }
}


//优化所有参数 (内参,畸变系数,外参) 返回重投影误差值
double CameraCalibration::OptimizeParameter(const std::vector<Eigen::MatrixXd>&  srcL,const std::vector<Eigen::MatrixXd>&  dstL, Eigen::Matrix3d& intrinsicParam , Eigen::VectorXd& distortionCoeff, std::vector<Eigen::MatrixXd>& externalParams)
{
    //整合参数
    Eigen::VectorXd P = ComposeParameter(intrinsicParam,distortionCoeff,externalParams);
    S_CameraOtherParameter cameraParam;
    cameraParam.dstL = dstL;
    cameraParam.srcL = srcL;
    cameraParam.imageCount = dstL.size();
    cameraParam.intrinsicCount = m_intrinsicCount;
    cameraParam.disCount = distortionCoeff.rows();

    Eigen::VectorXd P1 = LevenbergMarquardtAlgorithm(P,cameraParam,CalibrationResidualsVector(),CalibrationJacobi(),m_epsilon,m_maxIteCount);


    //分解参数
    DecomposeParamter(P1,intrinsicParam,distortionCoeff,externalParams);


    //计算重投影误差
    CalibrationResidualsVector reV;

    //每张图片重投影误差
    m_reprojErrL.clear();
    Eigen::VectorXd PP(m_intrinsicCount+distortionCoeff.rows()+6);
    PP.block(0,0,m_intrinsicCount+distortionCoeff.rows(),1) = P1.block(0,0,m_intrinsicCount+distortionCoeff.rows(),1);
    for(int i=0;i<externalParams.size();++i)
    {
        PP(m_intrinsicCount+distortionCoeff.rows())= P1(m_intrinsicCount+distortionCoeff.rows()+i*6);
        PP(m_intrinsicCount+distortionCoeff.rows()+1)= P1(m_intrinsicCount+distortionCoeff.rows()+i*6+1);
        PP(m_intrinsicCount+distortionCoeff.rows()+2)= P1(m_intrinsicCount+distortionCoeff.rows()+i*6+2);
        PP(m_intrinsicCount+distortionCoeff.rows()+3)= P1(m_intrinsicCount+distortionCoeff.rows()+i*6+3);
        PP(m_intrinsicCount+distortionCoeff.rows()+4)= P1(m_intrinsicCount+distortionCoeff.rows()+i*6+4);
        PP(m_intrinsicCount+distortionCoeff.rows()+5)= P1(m_intrinsicCount+distortionCoeff.rows()+i*6+5);

        S_CameraOtherParameter cameraParam1;
        cameraParam1.dstL.push_back(dstL.at(i));
        cameraParam1.srcL.push_back(srcL.at(i));
        cameraParam1.imageCount = 1;
        cameraParam1.intrinsicCount = m_intrinsicCount;
        cameraParam1.disCount = distortionCoeff.rows();

        Eigen::VectorXd reV1 = reV(PP,cameraParam1);

        int pointCount = reV1.rows()/2;
        Eigen::VectorXd errorV(pointCount);
        for(int i=0,k=0;i<pointCount;++i,k+=2)
        {
            errorV(i) = std::sqrt(std::pow(reV1(k),2) + std::pow(reV1(k+1),2));
        }

        m_reprojErrL.push_back(std::sqrt(errorV.sum()/pointCount));

    }

    //总重投影误差
    Eigen::VectorXd reV1 = reV(P1,cameraParam);
    int pointCount = reV1.rows()/2;
    Eigen::VectorXd errorV(pointCount);
    for(int i=0,k=0;i<pointCount;++i,k+=2)
    {
        errorV(i) = std::sqrt(std::pow(reV1(k),2) + std::pow(reV1(k+1),2));
    }

    return std::sqrt(errorV.sum()/pointCount);
}
