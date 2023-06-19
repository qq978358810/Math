#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}

//// 函数值
//double RosenBrock(const Eigen::VectorXd x)
//{
//    double res = 0.0;
//    for(int i = 0; i <= int(x.size()/2 - 1); ++i)
//    {
//        res += 100 * pow(pow(x(2*i),2) - x(2*i+1), 2) + pow(x(2*i) - 1, 2);
//    }
//    return res;
//}
//// 算函数梯度
//Eigen::VectorXd gradient(const Eigen::VectorXd x)
//{
//    Eigen::VectorXd res(x.size());
//    for(int i = 0; i <= int(x.size()/2 - 1); ++i)
//    {
//        res(2*i) = 400 * (pow(x(2*i), 3) - x(2*i)*x(2*i+1)) + 2 * (x(2*i) - 1);
//        res(2*i+1) = -200 * (pow(x(2*i), 2) - x(2*i+1));
//    }
//    return res;
//}
//// Hessian
//Eigen::MatrixXd Hessian(const Eigen::VectorXd x)
//{
//    Eigen::MatrixXd H(x.size(), x.size());
//    for(int i = 0; i < x.size()/2; ++i){
//        H(2*i,2*i) = 400 * (3 * pow(x(2*i), 2) - x(2*i+1)) + 2;
//        H(2*i,2*i+1) = -400 * x(2*i);
//        H(2*i+1,2*i) = -400 * x(2*i);
//        H(2*i+1,2*i+1) = 200;
//    }
//    return H;
//}

//int main() {
//    Eigen::VectorXd x(2);
//    x << 1.5,3;
//    qDebug() <<"初始函数值：" << RosenBrock(x) ;

//    // iteration initialization
//    double t = 1.0;
//    double c = 0.5;
//    double sigma = 1e-5;
//    Eigen::VectorXd gra = gradient(x);
//    Eigen::VectorXd d = gra * (-1);
//    qDebug() <<"初始梯度值：";
//    for(int i = 0; i < x.size(); ++i) qDebug() << gra(i) << " ";

//    qDebug() << RosenBrock(x + d) ;
//    int cnt = 0;

//    while(sqrt(gra.adjoint()*gra) > sigma)
//    {
//        Eigen::VectorXd tt = gra * c *t;
//        double tmp = d.adjoint() * tt;
//        while(RosenBrock(x + d*t) > RosenBrock(x) + tmp)
//        {
//            // update
//            t /= 2;
//            tt = gra * c *t;
//            tmp = d.adjoint() * tt;
//        }
//        x += d*t;
//        Eigen::MatrixXd M = Hessian(x);
//        gra = gradient(x);
//        d = M.inverse() * gra * (-1);
//        ++ cnt;
//    }
//    qDebug() <<"运行迭代次数: "<<  cnt ;
//    qDebug() << "最终函数值：" << RosenBrock(x) ;
//    qDebug() << "最终步长 " << t <<endl;
//    qDebug() << "最终收敛时对应的x：";
//    for(int i = 0; i < x.size(); ++i)
//        qDebug() << x(i) << " ";
//    return 1;
//}


//#include <iostream>
//#include <iomanip>
//#include <math.h>

//using namespace std;

//const double DERIV_STEP = 1e-5;
//const int MAX_ITER = 100;

//#define max(a,b) (((a)>(b))?(a):(b))

//double func(const VectorXd & input, const VectorXd& output, const VectorXd& params, int objIndex)
//{
//    // obj = A * sin(Bx) + C * cos(D*x) - F
//    double x1 = params(0);
//    double x2 = params(1);
//    double x3 = params(2);
//    double x4 = params(3);

//    double t = input(objIndex);
//    double f = output(objIndex);

//    return x1 * sin(x2 * t) + x3 * cos(x4 * t) - f;
//}

////return vector make up of func() element.
//VectorXd objF(const VectorXd& input, const VectorXd& output, const VectorXd& params)
//{
//    VectorXd obj(input.rows());
//    for (int i = 0; i < input.rows(); i++)
//        obj(i) = func(input, output, params, i);

//    return obj;
//}

////F = (f ^t * f)/2
//double Func(const VectorXd& obj)
//{
//    //平方和，所有误差的平方和
//    return obj.squaredNorm() / 2;
//}

//double Deriv(const VectorXd& input, const VectorXd& output, int objIndex, const VectorXd& params,
//    int paraIndex)
//{
//    VectorXd para1 = params;
//    VectorXd para2 = params;

//    para1(paraIndex) -= DERIV_STEP;
//    para2(paraIndex) += DERIV_STEP;

//    double obj1 = func(input, output, para1, objIndex);
//    double obj2 = func(input, output, para2, objIndex);

//    return (obj2 - obj1) / (2 * DERIV_STEP);
//}

//MatrixXd Jacobin(const VectorXd& input, const VectorXd& output, const VectorXd& params)
//{
//    int rowNum = input.rows();
//    int colNum = params.rows();

////    MatrixXd Jac(rowNum, colNum);

////    for (int i = 0; i < rowNum; i++)
////    {
////        for (int j = 0; j < colNum; j++)
////        {
////            Jac(i, j) = Deriv(input, output, i, params, j);
////        }
////    }

//       // int rows = rowNum.count();
//        Matrix<double,Dynamic,Dynamic> Jac;
//        Jac.resize(rowNum,4);
//        for(int i=0;i<rowNum;++i)
//        {
//            Jac(i,0)=sin(params(1) * input(i));
//            Jac(i,1)= params(0)*input(i)*cos(params(1) * input(i));
//            Jac(i,2)=cos(params(3) * input(i));
//            Jac(i,3)=params(2)* input(i)*-sin(params(3) * input(i));
//        }
//    return Jac;
//}

//void gaussNewton(const VectorXd& input, const VectorXd& output, VectorXd& params)
//{
//    int errNum = input.rows();      //error  num
//    int paraNum = params.rows();    //parameter  num

//    VectorXd obj(errNum);

//    double last_sum = 0;

//    int iterCnt = 0;
//    while (iterCnt < 1000)
//    {
//        //得到误差
//        obj = objF(input, output, params);

//        double sum = 0;
//        //误差平方和
//        sum = Func(obj);

//        cout << "Iterator index: " << iterCnt << endl;
//        cout << "parameter: " << endl << params << endl;
//        cout << "error sum: " << endl << sum << endl << endl;

//        //如果两次之间的误差小于1e-12，就算结束。
//        if (fabs(sum - last_sum) <= 1e-20)
//            break;
//        //否则代替上次误差继续迭代
//        last_sum = sum;

//        MatrixXd Jac = Jacobin(input, output, params);
//        VectorXd delta(paraNum);
//        delta = (Jac.transpose() * Jac).inverse() * Jac.transpose() * obj;

//        params -= delta;
//        iterCnt++;
//    }
//}
////int main(int argc, char* argv[])
////{
////    // obj = A * sin(Bx) + C * cos(D*x) - F
////    //there are 4 parameter: A, B, C, D.
////    int num_params = 4;

////    //generate random data using these parameter
////    int total_data = 100;

////    VectorXd input(total_data);
////    VectorXd output(total_data);

////    double A = 5, B = 1, C = 10, D = 2;
////    //load observation data
////    for (int i = 0; i < total_data; i++)
////    {
////        //generate a random variable [-10 10]
////        double x = 20.0 * ((rand() % 1000) / 1000.0) - 10.0;
////        double deltaY = 2.0 * (rand() % 1000) / 1000.0;
////        double y = A * sin(B*x) + C * cos(D*x) + deltaY;

////        input(i) = x;
////        output(i) = y;
////    }

////    //gauss the parameters
////    VectorXd params_gaussNewton(num_params);
////    //init gauss
////    params_gaussNewton << 1.8, 1.2, 9, 1.9;

////    gaussNewton(input, output, params_gaussNewton);

////    cout << "gauss newton parameter: " << endl << params_gaussNewton << endl << endl << endl;
////}
