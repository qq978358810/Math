#ifndef HYPERBOLOIDMAPPING_H
#define HYPERBOLOIDMAPPING_H

#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <cassert>
#include <QImage>
struct Point
{
    float x = -1;
    float y = -1;
};

using MatrixP = Eigen::Matrix<Point,Eigen::Dynamic,Eigen::Dynamic>;
class HyperboloidMapping
{
public:
    //1 2
    //4 3
    HyperboloidMapping(const Eigen::Vector2f src_points[4], const Eigen::Vector2f dst_points[4],int src_height,int src_width,int dst_height = -1,int dst_width = -1)
        :_height(src_height),_width(src_width)
    {
        _transform_matrix = get_perspective_transform(src_points,dst_points);

        //找出目标边界
        _min_width = std::min(std::min(dst_points[0](0),dst_points[1](0)),std::min(dst_points[2](0),dst_points[3](0)));
        _min_height = std::min(std::min(dst_points[0](1),dst_points[1](1)),std::min(dst_points[2](1),dst_points[3](1)));

        _max_width = std::max(std::max(dst_points[0](0),dst_points[1](0)),std::max(dst_points[2](0),dst_points[3](0)));
        _max_height = std::max(std::max(dst_points[0](1),dst_points[1](1)),std::max(dst_points[2](1),dst_points[3](1)));

        if(dst_height <= 0)
        {
            dst_height = _height;
        }

        if(dst_width <= 0)
        {
            dst_width = _width;
        }
        _perspective_mapping_table = get_perspective_mapping_table(_transform_matrix,dst_height,dst_width);
    }

    //应用双曲
    /////
    /// \brief warp_hyperbola
    /// \param src_image    //输入数据
    /// \param dst_image    //输出数据
    /// \param radian       //曲面弧度 限制在 [0, 1] 范围内
    /// \param crop         //裁剪有效数据到dst_image
    ///
    void warp_hyperbola(const Eigen::MatrixXf (&src_image)[3],Eigen::MatrixXf (&dst_image)[3],float radian = 0.1,bool crop = false)
    {

        assert(src_image[0].rows() == _height);
        assert(src_image[1].rows() == _height);
        assert(src_image[2].rows() == _height);

        assert(src_image[0].cols() == _width);
        assert(src_image[1].cols() == _width);
        assert(src_image[2].cols() == _width);

        Eigen::MatrixXf perspective_dst_image[3];
        //先进行透视变换
        warp_perspective(src_image,perspective_dst_image,_perspective_mapping_table);

        int src_rows = perspective_dst_image[0].rows();
        int src_cols = perspective_dst_image[0].cols();
        for(int i = 0;i < 3; ++i)
        {
            dst_image[i] = Eigen::MatrixXf::Zero(src_rows,src_cols);
        }

        //用于记录当前位置的累计像素个数 ，最后求平局像素值
        Eigen::MatrixXf accumulative_total = Eigen::MatrixXf::Zero(src_rows,src_cols);

        //坐标原点
        int center_x = src_cols >> 1;
        int center_y = (src_rows+1) >> 1;

        //radian 的值限制在 [0, 1] 范围内
        radian = std::min(1.0f, radian);
        radian = std::max(0.0f, radian);

        //内缩
        int retract = center_y * radian;

        //系数
        float coefficient = (float)retract / (center_x * center_x);
        //系数步长
        float coefficient_step = coefficient / center_y;

        // 计算映射表 y位置
        for (int y = 0; y < center_y; ++y)
        {
            //变量x步长
            float x_step = y * coefficient_step;
            //变量常数大小
            float y_step = (center_y - y) - ((center_x) * (center_x) * (coefficient - x_step));


            //获取映射y坐标
            for (int x = center_x; x < src_cols; ++x)
            {
                //获取第一象限y位置
                int y1 = center_y - static_cast<float>((x - center_x) * (x - center_x) * (coefficient - x_step) + (y_step));
                accumulative_total(y1, x)++;

                //获取第二象限y位置
                int x2 = src_cols - x - 1;
                int y2 = y1;
                accumulative_total(y2, x2)++;

                //获取第三象限y位置
                int x3 = x2;
                int y3 = src_rows - y1 - 1;
                accumulative_total(y3, x3)++;

                //获取第四象限y位置
                int x4 = x;
                int y4 = y3;
                accumulative_total(y4, x4)++;

                for(int i = 0;i < 3; ++i)
                {
                    //获取第一象限y位置
                    dst_image[i](y1, x) += perspective_dst_image[i](y,x);

                    //获取第二象限y位置
                    dst_image[i](y2, x2) += perspective_dst_image[i](y,x2);

                    //获取第三象限y位置
                    dst_image[i](y3, x3) += perspective_dst_image[i](src_rows - y - 1,x3);

                    //获取第四象限y位置
                    dst_image[i](y4, x4) += perspective_dst_image[i](src_rows - y - 1,x4);
                }
            }
        }

        for(int i = 0;i < 3; ++i)
        {
            // 对位相除
            dst_image[i] = dst_image[i].cwiseQuotient(accumulative_total);
        }

        //裁剪
        if(crop)
        {
            int min_width = _min_width > src_cols ? 0 : _min_width;
            int max_width = _max_width > src_cols ? src_cols : _max_width;
            int min_height = _min_height > src_rows ? 0 : _min_height;
            int max_height = _max_height > src_rows ? src_rows : _max_height;
            for(int i = 0;i < 3; ++i)
            {
                Eigen::MatrixXf temp = dst_image[i].block(min_height, min_width, max_height-min_height, max_width-min_width);
                dst_image[i] = temp;
            }
        }
    }

    //qimage 转 Matrix
    static void image_2_matrix(const QImage& image,Eigen::MatrixXf (&matrix)[3])
    {
        // 将图像转换为 RGB888 格式
        QImage convertedImage = image.convertToFormat(QImage::Format_RGB888);

        for(int i=0;i<3;++i)
        {
            matrix[i] = Eigen::MatrixXf(convertedImage.height(), convertedImage.width());
        }

        // 从 QImage 中提取每个通道的数据
        for (int i = 0; i < convertedImage.height(); ++i)
        {
            for (int j = 0; j < convertedImage.width(); ++j)
            {
                // 获取像素的 RGB 值
                QRgb pixel = convertedImage.pixel(j, i);

                // 将 RGB 值拆分为每个通道的值，并保存到对应的 Eigen Matrix 中
                matrix[0](i, j) = qRed(pixel);
                matrix[1](i, j) = qGreen(pixel);
                matrix[2](i, j) = qBlue(pixel);
            }
        }

    }

    // 将三个 Eigen Matrix 转换为 QImage
    static void matrix_2_image(const Eigen::MatrixXf (&matrix)[3],QImage& image)
    {
        // 创建一个空的 QImage
        image = QImage(matrix[0].cols(), matrix[0].rows(), QImage::Format_RGB888);

        // 设置图像的像素值
        for (int i = 0; i < image.height(); ++i)
        {
            for (int j = 0; j < image.width(); ++j)
            {
                // 创建 QColor 对象并设置像素值
                QColor color(static_cast<int>(matrix[0](i, j)), static_cast<int>(matrix[1](i, j)), static_cast<int>(matrix[2](i, j)));
                image.setPixel(j, i, color.rgb());
            }
        }
    }
protected:
    //src_points
    //1 2
    //4 3
    //获取透视变化矩阵
    Eigen::Matrix3f get_perspective_transform(const Eigen::Vector2f src_points[4], const Eigen::Vector2f dst_points[4])
    {
        Eigen::Matrix3f perspective_matrix;

        // 构造线性方程组
        Eigen::Matrix<float, 8, 8> A;
        Eigen::Matrix<float, 8, 1> b;
        for (int i = 0; i < 4; ++i) {
            A.row(i * 2) << src_points[i].x(), src_points[i].y(), 1, 0, 0, 0, -dst_points[i].x() * src_points[i].x(), -dst_points[i].x() * src_points[i].y();
            A.row(i * 2 + 1) << 0, 0, 0, src_points[i].x(), src_points[i].y(), 1, -dst_points[i].y() * src_points[i].x(), -dst_points[i].y() * src_points[i].y();
            b.row(i * 2) << dst_points[i].x();
            b.row(i * 2 + 1) << dst_points[i].y();
        }

        // 解线性方程组
        /* 在 Eigen 库中，`colPivHouseholderQr()` 是用于执行列主元素高斯-约当消元法的方法，
         * 用于解线性方程组。它返回一个对象，该对象提供了一种求解线性方程组的方式。
         * 在你的代码中，`A.colPivHouseholderQr().solve(b)` 表示对矩阵 `A` 应用列主元素高斯-约当消元法，
         * 并解出线性方程组 `Ax = b`，其中 `b` 是右侧的常数向量，`x` 是未知向量。解出的向量 `x` 包含了方程组的解。
         */
        Eigen::Matrix<float, 8, 1> x = A.colPivHouseholderQr().solve(b);

        // 构造透视变换矩阵
        perspective_matrix << x[0], x[1], x[2],
            x[3], x[4], x[5],
            x[6], x[7], 1;

        return perspective_matrix;
    }

    // 定义双线性插值函数 - 通过(x,y)坐标获取像素值
    float bilinear_interpolation(const Eigen::MatrixXf& image, float x, float y)
    {
        // 获取图像的宽度和高度
        int width = image.cols();
        int height = image.rows();

        // 计算四个最近的像素的坐标
        int x0 = static_cast<int>(x);
        int y0 = static_cast<int>(y);
        //不能超出图像边界
        int x1 = std::min(x0 + 1, width - 1);
        int y1 = std::min(y0 + 1, height - 1);

        // 计算双线性插值系数
        float alpha = x - x0;
        float beta = y - y0;

        //1 3
        //2 4
        // 计算四个最近的像素的灰度值
        float f00 = image(y0, x0); //1
        float f10 = image(y1, x0); //2
        float f01 = image(y0, x1); //3
        float f11 = image(y1, x1); //4

        // 执行双线性插值
        return ((1 - alpha) * (1 - beta) * f00 +
                (1 - alpha) * beta * f10 +
                alpha * (1 - beta) * f01 +
                alpha * beta * f11);
    }


    //透视变换映射表
    MatrixP get_perspective_mapping_table(const Eigen::Matrix3f& transform,int dst_height,int dst_width)
    {
        // // 构建齐次坐标向量
        // //1 3
        // //2 4
        // Eigen::Vector3f src_point_0(0, 0, 1);
        // Eigen::Vector3f src_point_1(0, src_height, 1);
        // Eigen::Vector3f src_point_2(src_width, 0, 1);
        // Eigen::Vector3f src_point_3(src_width, src_height, 1);

        // // 应用透视变换
        // Eigen::Vector3f dst_point_0 = transform * src_point_0;
        // Eigen::Vector3f dst_point_1 = transform * src_point_1;
        // Eigen::Vector3f dst_point_2 = transform * src_point_2;
        // Eigen::Vector3f dst_point_3 = transform * src_point_3;
        // // 归一化坐标
        // int u_0 = (int)(dst_point_0[0] / dst_point_0[2] + 0.5);
        // int v_0 = (int)(dst_point_0[1] / dst_point_0[2] + 0.5);

        // int u_1 = (int)(dst_point_1[0] / dst_point_1[2] + 0.5);
        // int v_1 = (int)(dst_point_1[1] / dst_point_1[2] + 0.5);

        // int u_2 = (int)(dst_point_2[0] / dst_point_2[2] + 0.5);
        // int v_2 = (int)(dst_point_2[1] / dst_point_2[2] + 0.5);

        // int u_3 = (int)(dst_point_3[0] / dst_point_3[2] + 0.5);
        // int v_3 = (int)(dst_point_3[1] / dst_point_3[2] + 0.5);

        //找出目标边界
        int dst_min_width = _min_width;
        int dst_min_height = _min_height;

        int dst_max_width = _max_width;
        int dst_max_height = _max_height;


        //求逆
        Eigen::Matrix3f transform_inv = transform.inverse();

        //映射表
        MatrixP matp(dst_height,dst_width);

        // 遍历目标图像的每个像素，并进行透视变换
        for (int y = dst_min_height; y < dst_max_height; ++y)
        {
            for (int x = dst_min_width; x < dst_max_width; ++x)
            {
                //在规定的范围呢
                if(y < dst_height && x < dst_width)
                {
                    // 构建齐次坐标向量
                    Eigen::Vector3f src_point(x, y, 1);

                    // 应用透视变换
                    Eigen::Vector3f dst_point = transform_inv * src_point;

                    // 归一化坐标
                    float u = dst_point[0] / dst_point[2];
                    float v = dst_point[1] / dst_point[2];


                    //构建映射表
                    matp(y,x) = {u,v};
                }

            }
        }
        return matp;
    }

    //应用透视
    void warp_perspective(const Eigen::MatrixXf (&src_image)[3],Eigen::MatrixXf (&dst_image)[3],const MatrixP& mapping_table)
    {
        int src_rows = src_image[0].rows();
        int src_cols = src_image[0].cols();
        for(int i = 0;i < 3; ++i)
        {
            dst_image[i] = Eigen::MatrixXf::Zero(mapping_table.rows(),mapping_table.cols());
        }
        // 遍历目标图像的每个像素，并进行透视变换
        for (int y = 0; y < mapping_table.rows(); ++y)
        {
            for (int x = 0; x < mapping_table.cols(); ++x)
            {
                Point point = mapping_table(y,x);
                // 对坐标进行边界检查s
                if (point.x >= 0 && point.x < src_cols && point.y >= 0 && point.y < src_rows)
                {
                    for(int i = 0;i < 3; ++i)
                    {
                        dst_image[i](y,x) = bilinear_interpolation(src_image[i],point.x,point.y);
                    }
                }
            }
        }

    }

private:
    //输入图大小
    int _height;
    int _width;

    //有效数据范围
    int _min_width;
    int _max_width;
    int _min_height;
    int _max_height;

    Eigen::Matrix3f _transform_matrix; //透视变换矩阵
    MatrixP _perspective_mapping_table; //透视映射表
};

#endif // HYPERBOLOIDMAPPING_H





















// #ifndef HYPERBOLOIDMAPPING_H
// #define HYPERBOLOIDMAPPING_H

// #include <Eigen/Dense>
// #include <iostream>
// #include <cmath>
// #include <cassert>

// struct Point
// {
//     float x;
//     float y;
// };

// using MatrixP = Eigen::Matrix<Point,Eigen::Dynamic,Eigen::Dynamic>;
// class HyperboloidMapping
// {
// public:


//     HyperboloidMapping(int height,int width):_height(height),_width(width)
//     {

//     }


//     //src_points
//     //1 2
//     //4 3
//     //获取透视变化矩阵
//     Eigen::Matrix3f get_perspective_transform(const Eigen::Vector2f src_points[4], const Eigen::Vector2f dst_points[4])
//     {
//         Eigen::Matrix3f perspective_matrix;

//         // 构造线性方程组
//         Eigen::Matrix<float, 8, 8> A;
//         Eigen::Matrix<float, 8, 1> b;
//         for (int i = 0; i < 4; ++i) {
//             A.row(i * 2) << src_points[i].x(), src_points[i].y(), 1, 0, 0, 0, -dst_points[i].x() * src_points[i].x(), -dst_points[i].x() * src_points[i].y();
//             A.row(i * 2 + 1) << 0, 0, 0, src_points[i].x(), src_points[i].y(), 1, -dst_points[i].y() * src_points[i].x(), -dst_points[i].y() * src_points[i].y();
//             b.row(i * 2) << dst_points[i].x();
//             b.row(i * 2 + 1) << dst_points[i].y();
//         }

//         // 解线性方程组
//         Eigen::Matrix<float, 8, 1> x = A.colPivHouseholderQr().solve(b);

//         // 构造透视变换矩阵
//         perspective_matrix << x[0], x[1], x[2],
//             x[3], x[4], x[5],
//             x[6], x[7], 1;

//         return perspective_matrix;
//     }

//     // 定义双线性插值函数 - 通过(x,y)坐标获取像素值
//     float bilinear_interpolation(const Eigen::MatrixXf& image, float x, float y)
//     {
//         // 获取图像的宽度和高度
//         int width = image.cols();
//         int height = image.rows();

//         // 计算四个最近的像素的坐标
//         int x0 = static_cast<int>(x);
//         int y0 = static_cast<int>(y);
//         //不能超出图像边界
//         int x1 = std::min(x0 + 1, width - 1);
//         int y1 = std::min(y0 + 1, height - 1);

//         // 计算双线性插值系数
//         float alpha = x - x0;
//         float beta = y - y0;

//         //1 3
//         //2 4
//         // 计算四个最近的像素的灰度值
//         float f00 = image(y0, x0); //1
//         float f10 = image(y1, x0); //2
//         float f01 = image(y0, x1); //3
//         float f11 = image(y1, x1); //4

//         // 执行双线性插值
//         return ((1 - alpha) * (1 - beta) * f00 +
//                (1 - alpha) * beta * f10 +
//                alpha * (1 - beta) * f01 +
//                alpha * beta * f11);
//     }

//     //获取掩码矩阵
//     Eigen::MatrixXi mask_matrix(const Eigen::MatrixXf (&src_image)[3],const Eigen::Matrix3f& transform)
//     {
//         int src_rows = src_image[0].rows();
//         int src_cols = src_image[0].cols();
//         // 构建齐次坐标向量
//         //1 3
//         //2 4
//         Eigen::Vector3f src_point_0(0, 0, 1);
//         Eigen::Vector3f src_point_1(0, src_rows, 1);
//         Eigen::Vector3f src_point_2(src_cols, 0, 1);
//         Eigen::Vector3f src_point_3(src_cols, src_rows, 1);

//         // 应用透视变换
//         Eigen::Vector3f dst_point_0 = transform * src_point_0;
//         Eigen::Vector3f dst_point_1 = transform * src_point_1;
//         Eigen::Vector3f dst_point_2 = transform * src_point_2;
//         Eigen::Vector3f dst_point_3 = transform * src_point_3;
//         // 归一化坐标
//         int u_0 = (int)(dst_point_0[0] / dst_point_0[2] + 0.5);
//         int v_0 = (int)(dst_point_0[1] / dst_point_0[2] + 0.5);

//         int u_1 = (int)(dst_point_1[0] / dst_point_1[2] + 0.5);
//         int v_1 = (int)(dst_point_1[1] / dst_point_1[2] + 0.5);

//         int u_2 = (int)(dst_point_2[0] / dst_point_2[2] + 0.5);
//         int v_2 = (int)(dst_point_2[1] / dst_point_2[2] + 0.5);

//         int u_3 = (int)(dst_point_3[0] / dst_point_3[2] + 0.5);
//         int v_3 = (int)(dst_point_3[1] / dst_point_3[2] + 0.5);

//         //找出目标边界
//         int dst_width = std::max(std::max(u_0,u_1),std::max(u_2,u_3));
//         int dst_height = std::max(std::max(v_0,v_1),std::max(v_2,v_3));

//         Eigen::MatrixXi mask = Eigen::MatrixXi::Zero(dst_height,dst_width);

//         // 遍历目标图像的每个像素，生成掩码矩阵
//         for (int y = 0; y < src_rows; ++y)
//         {
//             for (int x = 0; x < src_cols; ++x)
//             {
//                 // 构建齐次坐标向量
//                 Eigen::Vector3f src_point(x, y, 1);

//                 // 应用透视变换
//                 Eigen::Vector3f dst_point = transform * src_point;

//                 // 归一化坐标
//                 float u = dst_point[0] / dst_point[2];
//                 float v = dst_point[1] / dst_point[2];
//                 // 对坐标进行边界检查
//                 if (u >= 0 && u < dst_width && v >= 0 && v < dst_height)
//                 {
//                     // 计算四个最近的像素的坐标
//                     int x0 = static_cast<int>(u);
//                     int y0 = static_cast<int>(v);
//                     //不能超出图像边界
//                     int x1 = std::min(x0 + 1, dst_width - 1);
//                     int y1 = std::min(y0 + 1, dst_height - 1);

//                     //1 3
//                     //2 4
//                     mask(y0,x0) = 1;
//                     mask(y1,x0) = 1;
//                     mask(y0,x1) = 1;
//                     mask(y1,x1) = 1;
//                 }
//             }
//         }
//         return mask;
//     }

//     //透视变换映射表
//     MatrixP get_perspective_mapping_table(const Eigen::MatrixXi& mask_matrix,const Eigen::Matrix3f& transform)
//     {
//         //求逆
//         Eigen::Matrix3f transform_inv = transform.inverse();

//         MatrixP matp(mask_matrix.rows(),mask_matrix.cols());

//         // 遍历目标图像的每个像素，并进行透视变换
//         for (int y = 0; y < mask_matrix.rows(); ++y)
//         {
//             for (int x = 0; x < mask_matrix.cols(); ++x)
//             {
//                 matp(y,x) = {-1,-1};
//                 if(mask_matrix(y,x) != 0)
//                 {
//                     // 构建齐次坐标向量
//                     Eigen::Vector3f src_point(x, y, 1);

//                     // 应用透视变换
//                     Eigen::Vector3f dst_point = transform_inv * src_point;

//                     // 归一化坐标
//                     float u = dst_point[0] / dst_point[2];
//                     float v = dst_point[1] / dst_point[2];

//                     //构建映射表
//                     matp(y,x) = {u,v};
//                 }
//             }
//         }
//         return matp;
//     }

//     //应用透视
//     void warp_perspective(const Eigen::MatrixXf (&src_image)[3],Eigen::MatrixXf (&dst_image)[3],const MatrixP& mapping_table)
//     {

//         int src_rows = src_image[0].rows();
//         int src_cols = src_image[0].cols();
//         for(int i = 0;i < 3; ++i)
//         {
//             dst_image[i] = Eigen::MatrixXf::Zero(mapping_table.rows(),mapping_table.cols());
//         }
//         // 遍历目标图像的每个像素，并进行透视变换
//         for (int y = 0; y < mapping_table.rows(); ++y)
//         {
//             for (int x = 0; x < mapping_table.cols(); ++x)
//             {
//                 Point point = mapping_table(y,x);
//                 // 对坐标进行边界检查s
//                 if (point.x >= 0 && point.x < src_cols && point.y >= 0 && point.y < src_rows)
//                 {
//                     for(int i = 0;i < 3; ++i)
//                     {
//                         dst_image[i](y,x) = bilinear_interpolation(src_image[i],point.x,point.y);
//                     }
//                 }
//             }
//         }

//     }

//     //获取曲面映射表
//     MatrixP hyperbola_mapping(const Eigen::MatrixXi& mask_matrix,const Eigen::Matrix3f& transform)
//     {

//         // 创建映射表
//         cv::Mat map_x(image.size(), CV_32FC1);
//         cv::Mat map_y(image.size(), CV_32FC1);
//         //初始化，建立映射表

//         int left_center_x = (image.cols) >> 1;
//         int right_center_x = (image.cols + 1) >> 1;

//         int up_center_y = (image.rows) >> 1;
//         int down_center_y = (image.rows + 1) >> 1;


//         //内缩
//         int retract = 100;

//         //系数
//         double coefficient = (double)retract / (left_center_x * left_center_x);
//         //系数步长
//         double coefficient_step = coefficient / up_center_y;

//         // 计算映射表 x位置保持不变
//         for (int y = 0; y < image.rows; ++y)
//         {
//             for (int x = 0; x < image.cols; ++x)
//             {
//                 map_x.at<float>(y, x) = x;
//                 //map_y.at<float>(y, x) = -1;
//             }
//         }
//         // 计算映射表 y位置
//         for (int y = 0; y < up_center_y; ++y)
//         {
//             //变量x步长
//             double x_step = y * coefficient_step;
//             //变量常数大小
//             double y_step = (up_center_y - y) - ((left_center_x) * (left_center_x) * (coefficient - x_step));


//             //获取映射y坐标
//             for (int x = right_center_x; x < image.cols; ++x)
//             {

//                 //获取第一象限y位置
//                 double y1 = up_center_y - static_cast<double>((x - left_center_x) * (x - left_center_x) * (coefficient - x_step) + (y_step)) + 0.5;
//                 map_y.at<float>(static_cast<int>(y1), x) = y;

//                 //获取第二象限y位置
//                 double x2 = left_center_x - (x - left_center_x);
//                 double y2 = y1;
//                 map_y.at<float>(static_cast<int>(y2), x2) = y;

//                 //获取第三象限y位置
//                 double x3 = left_center_x - (x - left_center_x);
//                 double y3 = image.rows - y1;
//                 map_y.at<float>(static_cast<int>(y3), x3) = image.rows - y;

//                 //获取第四象限y位置
//                 double x4 = x;
//                 double y4 = image.rows - y1;
//                 map_y.at<float>(static_cast<int>(y4), x4) = image.rows - y;

//             }
//         }

//     }
//     //向前映射
//     void forward_mapping(const Eigen::MatrixXf (&src_image)[3],Eigen::MatrixXf (&dst_image)[3],const Eigen::Matrix3f& transform,int dst_height = -1,int dst_width = -1)
//     {

//         assert(src_image[0].rows() == _height);
//         assert(src_image[1].rows() == _height);
//         assert(src_image[2].rows() == _height);

//         assert(src_image[0].cols() == _width);
//         assert(src_image[1].cols() == _width);
//         assert(src_image[2].cols() == _width);


//         if(dst_height <= 0)
//         {
//             dst_height = _height;
//         }

//         if(dst_width <= 0)
//         {
//             dst_width = _width;
//         }

//         //创建目标图像 分配内存
//         dst_image[0] = Eigen::MatrixXf::Zero(dst_height,dst_width);
//         dst_image[1] = Eigen::MatrixXf::Zero(dst_height,dst_width);
//         dst_image[2] = Eigen::MatrixXf::Zero(dst_height,dst_width);
//         //计算两个方向上的(目标图像/原图像)系数比
//         float height_coefficient = static_cast<float>(dst_height) / _height;
//         float width_coefficient = static_cast<float>(dst_width) / _width;


//         // 遍历目标图像的每个像素，并进行透视变换
//         for (int y = 0; y < _height; ++y)
//         {
//             for (int x = 0; x < _width; ++x)
//             {
//                 // 构建齐次坐标向量
//                 Eigen::Vector3f src_point(x, y, 1);

//                 // 应用透视变换
//                 Eigen::Vector3f dst_point = transform * src_point;

//                 // 归一化坐标
//                 float u = dst_point[0] / dst_point[2] * width_coefficient;
//                 float v = dst_point[1] / dst_point[2] * height_coefficient;

//                 // 对坐标进行边界检查
//                 if (u >= 0 && u < dst_width && v >= 0 && v < dst_height)
//                 {
//                     // 双线性插值获取变换后的像素值

//                     // 计算四个最近的像素的坐标
//                     int x0 = static_cast<int>(u);
//                     int y0 = static_cast<int>(v);
//                     //不能超出图像边界
//                     int x1 = std::min(x0 + 1, dst_width - 1);
//                     int y1 = std::min(y0 + 1, dst_height - 1);

//                     // 计算双线性插值系数
//                     float alpha = u - x0;
//                     float beta = v - y0;

//                     for(int i=0;i<3;++i)
//                     {
//                         //1 3
//                         //2 4
//                         // 计算四个最近的像素的灰度值
//                         dst_image[i](y0, x0) += src_image[i](y,x) * (1 - alpha) * (1 - beta); //1
//                         dst_image[i](y1, x0) += src_image[i](y,x) * (1 - alpha) * beta; //2
//                         dst_image[i](y0, x1) += src_image[i](y,x) * alpha * (1 - beta); //3
//                         dst_image[i](y1, x1) += src_image[i](y,x) * alpha * beta; //4
//                     }
//                 }
//             }
//         }

//     }

//     //向后映射
//     void backward_mapping(const Eigen::MatrixXf (&src_image)[3],Eigen::MatrixXf (&dst_image)[3],const Eigen::Matrix3f& transform,int dst_height = -1,int dst_width = -1)
//     {

//         assert(src_image[0].rows() == _height);
//         assert(src_image[1].rows() == _height);
//         assert(src_image[2].rows() == _height);

//         assert(src_image[0].cols() == _width);
//         assert(src_image[1].cols() == _width);
//         assert(src_image[2].cols() == _width);


//         if(dst_height <= 0)
//         {
//             dst_height = _height;
//         }

//         if(dst_width <= 0)
//         {
//             dst_width = _width;
//         }

//         //创建目标图像 分配内存
//         dst_image[0] = Eigen::MatrixXf::Zero(dst_height,dst_width);
//         dst_image[1] = Eigen::MatrixXf::Zero(dst_height,dst_width);
//         dst_image[2] = Eigen::MatrixXf::Zero(dst_height,dst_width);
//         //计算两个方向上的(目标图像/原图像)系数比
//         float height_coefficient = static_cast<float>(dst_height) / _height;
//         float width_coefficient = static_cast<float>(dst_width) / _width;


//         // 遍历目标图像的每个像素，并进行透视变换
//         for (int y = 0; y < dst_height; ++y)
//         {
//             for (int x = 0; x < dst_width; ++x)
//             {
//                 // 构建齐次坐标向量
//                 Eigen::Vector3f dst_point(x, y, 1);

//                 // 应用透视变换
//                 Eigen::Vector3f src_point = transform * dst_point;

//                 // 归一化坐标
//                 float u = src_point[0] / src_point[2] * width_coefficient;
//                 float v = src_point[1] / src_point[2] * height_coefficient;

//                 // 对坐标进行边界检查
//                 if (u >= 0 && u < _width && v >= 0 && v < _height)
//                 {
//                     // 双线性插值获取变换后的像素值

//                     // 计算四个最近的像素的坐标
//                     int x0 = static_cast<int>(u);
//                     int y0 = static_cast<int>(v);
//                     //不能超出图像边界
//                     int x1 = std::min(x0 + 1, _width - 1);
//                     int y1 = std::min(y0 + 1, _height - 1);

//                     // 计算双线性插值系数
//                     float alpha = u - x0;
//                     float beta = v - y0;
//                     for(int i=0;i<3;++i)
//                     {
//                         //1 3
//                         //2 4
//                         // 计算四个最近的像素的灰度值
//                         float f00 = src_image[i](y0, x0); //1
//                         float f10 = src_image[i](y1, x0); //2
//                         float f01 = src_image[i](y0, x1); //3
//                         float f11 = src_image[i](y1, x1); //4

//                         // 执行双线性插值
//                         dst_image[i](y, x) = ((1 - alpha) * (1 - beta) * f00 +
//                                               (1 - alpha) * beta * f10 +
//                                               alpha * (1 - beta) * f01 +
//                                               alpha * beta * f11);
//                     }
//                 }
//             }
//         }

//     }

// private:
//     int _height;
//     int _width;
// };

// #endif // HYPERBOLOIDMAPPING_H
