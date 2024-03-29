#include "cameracalibrationwidget.h"

CameraCalibrationWidget::CameraCalibrationWidget(QWidget *parent) : QWidget(parent)
{

    //创建控件
    m_set = new QPushButton(QString("设置"));
    m_execute = new QPushButton(QString("执行"));
    m_pointCount = new QSpinBox();
    m_imagesCount = new QSpinBox();
    m_load = new QPushButton(QString("加载到表格"));
    m_loadImages = new QPushButton(QString("加载图像"));
    m_executeOpenCV = new QPushButton(QString("执行OpenCV"));
    m_splitChar = new QLineEdit();
    m_splitChar->setMaximumWidth(100);
    m_splitChar->setText(",");
    m_row = new QCheckBox(QString("按行加载"));
    m_col = new QCheckBox(QString("按列加载"));
    m_comboBoxDis = new QComboBox();
    m_comboBoxDis->addItem(QString(""));
    m_comboBoxDis->addItem(QString("2: k1,k2"));
    m_comboBoxDis->addItem(QString("4: k1,k2,p1,p2"));
    m_comboBoxDis->addItem(QString("5: k1,k2,p1,p2,k3"));
    m_comboBoxInP = new QComboBox();
    m_comboBoxInP->addItem(QString("5: fx,γ,fy,u0,v0"));
    m_comboBoxInP->addItem(QString("4: fx,fy,u0,v0"));
    m_iterationCount = new QSpinBox();
    m_iterationCount->setMinimum(0);
    m_iterationCount->setMaximum(99999999);
    m_iterationCount->setValue(99);

    m_rows = new QSpinBox();
    m_rows->setMinimum(0);
    m_rows->setMaximum(9999);
    m_rows->setValue(8);

    m_cols = new QSpinBox();
    m_cols->setMinimum(0);
    m_cols->setMaximum(9999);
    m_cols->setValue(11);

    m_tableWidget = new QTableWidget();
    m_tableWidget->setMinimumWidth(550);
    m_addTextEdit = new QTextEdit();
    //不自动换行
    m_addTextEdit->setLineWrapMode(QTextEdit::NoWrap);
    m_showTextEdit = new QTextEdit();
    //不自动换行
    m_showTextEdit->setLineWrapMode(QTextEdit::NoWrap);


    QHBoxLayout * HLayout_0 = new QHBoxLayout();
    HLayout_0->addWidget(new QLabel(QString("标定版行数:")));
    HLayout_0->addWidget(m_rows);
    HLayout_0->addWidget(new QLabel(QString("标定版列数:")));
    HLayout_0->addWidget(m_cols);
    HLayout_0->addWidget(m_loadImages);
    HLayout_0->addWidget(m_executeOpenCV);
    HLayout_0->addStretch();

    QHBoxLayout * HLayout = new QHBoxLayout();
    HLayout->addWidget(m_load);
    HLayout->addWidget(new QLabel(QString("分割数据符:")));
    HLayout->addWidget(m_splitChar);
    HLayout->addWidget(m_row);
    HLayout->addWidget(m_col);
    HLayout->addWidget(new QLabel(QString("畸变系数:")));
    HLayout->addWidget(m_comboBoxDis);
    HLayout->addWidget(new QLabel(QString("内参个数:")));
    HLayout->addWidget(m_comboBoxInP);
    HLayout->addWidget(new QLabel(QString("最大迭代次数:")));
    HLayout->addWidget(m_iterationCount);
    HLayout->addStretch();


    QVBoxLayout* VLayout = new QVBoxLayout();
    VLayout->addWidget(m_tableWidget);
    VLayout->addWidget(m_showTextEdit);

    QGridLayout* GLayout_2  = new QGridLayout();
    GLayout_2->addWidget(new QLabel(QString("image:")),0,0);
    GLayout_2->addWidget(m_imagesCount,0,1);
    GLayout_2->addWidget(new QLabel(QString("point:")),0,2);
    GLayout_2->addWidget(m_pointCount,0,3);

    GLayout_2->addWidget(m_set,0,4);
    GLayout_2->setColumnStretch(5,5);
    GLayout_2->addWidget(m_execute,0,6);
    GLayout_2->addLayout(VLayout,1,0,-1,-1);


    QGridLayout* GLayout_1  = new QGridLayout();
    GLayout_1->addLayout(HLayout_0,0,0,1,-1);
    GLayout_1->addLayout(HLayout,1,0,1,-1);
    GLayout_1->addWidget(m_addTextEdit,2,0);
    GLayout_1->addLayout(GLayout_2,2,1,-1,-1);

    this->setLayout(GLayout_1);

    //连接信号和槽
    connect(m_set,&QPushButton::clicked,this,&CameraCalibrationWidget::slot_setClick);
    connect(m_execute,&QPushButton::clicked,this,&CameraCalibrationWidget::slot_executeClick);
    connect(m_tableWidget,&QTableWidget::itemChanged,this,&CameraCalibrationWidget::slot_tableWidgetItemChanged);
    connect(m_load,&QPushButton::clicked,this,&CameraCalibrationWidget::slot_loadToTable);
    connect(m_loadImages,&QPushButton::clicked,this,&CameraCalibrationWidget::slot_loadImages);
    connect(m_executeOpenCV,&QPushButton::clicked,this,&CameraCalibrationWidget::slot_executeOpenCV);



    QButtonGroup *buttonGroup = new QButtonGroup(this);
    //互锁
    buttonGroup->addButton(m_row);
    buttonGroup->addButton(m_col);
    buttonGroup->setExclusive(true);
    m_row->setChecked(true);

    m_pointCount->setMinimum(0);
    m_pointCount->setMaximum(9999);
    m_pointCount->setValue(0);

    m_imagesCount->setMinimum(0);
    m_imagesCount->setMaximum(9999);
    m_imagesCount->setValue(0);

}

//数据加载到表格
void CameraCalibrationWidget::slot_loadToTable()
{
    QString splitChar = m_splitChar->text();
    QString str = m_addTextEdit->toPlainText();
    QStringList strL = str.split('\n');

    QRegExp rx("\\S");//匹配任何非空白字符：\S 或  [^\s]，
    strL = strL.filter(rx);


    //判断数据个数是否一致 和数据是否为double类型
    int pointCount=0;
    for(int i=0;i<strL.count();++i)
    {
        pointCount = strL.at(0).split(splitChar,QString::SkipEmptyParts).count();
        QStringList next = strL.at(i).split(splitChar,QString::SkipEmptyParts);
        if(pointCount != next.count())
        {
            QMessageBox::information(this,"错误",QString("第 %1 行数据个数不匹配!").arg(i));
            return;
        }
        for(int j=0;j<next.count();++j)
        {
            bool isDouble;
            next.at(j).toDouble(&isDouble);
            if(!isDouble)
            {
                QMessageBox::information(this,"错误",QString("第 %1 行,第 %2 个数据转换【double】失败!").arg(i).arg(j));
                return;
            }
        }
    }


    //按行读取
    if(m_row->isChecked())
    {
        if(strL.count()%4 != 0)
        {
           QMessageBox::information(this,"错误","总体缺少行数据!");
           return;
        }
        int imageCount = strL.count()/4;
        createDataTable(imageCount,pointCount);

        for(int i=0;i<strL.count();++i)
        {
            QStringList data = strL.at(i).split(splitChar,QString::SkipEmptyParts);
            for(int j=0;j<data.count();++j)
            {
                m_tableWidget->setItem(j,i,new QTableWidgetItem(data[j].remove(QRegExp("\\s"))));
            }
        }
    }
    //按列读取
    if(m_col->isChecked())
    {
        if(pointCount%4 != 0)
        {
           QMessageBox::information(this,"错误","总体缺少列数据!");
           return;
        }
        int imageCount = pointCount/4;
        createDataTable(imageCount,strL.count());

        for(int i=0;i<strL.count();++i)
        {
            QStringList data = strL.at(i).split(splitChar,QString::SkipEmptyParts);
            for(int j=0;j<data.count();++j)
            {
                m_tableWidget->setItem(i,j,new QTableWidgetItem(data[j].remove(QRegExp("\\s"))));
            }

        }
    }
}

//生成数据表格
void CameraCalibrationWidget::createDataTable(int imageCount,int pointCount)
{
    //设置标题
    //表头标题用QStringList来表示
    QStringList headerText;


    for(int i=0;i<imageCount;++i)
    {
        headerText<<QString("像素X %1").arg(i+1)<<QString("像素Y %1").arg(i+1)<<QString("物理X %1").arg(i+1)<<QString("物理Y %1").arg(i+1);
    }

    int oldRow = m_tableWidget->rowCount();
    int oldCol = m_tableWidget->columnCount();
    //设置行数
    m_tableWidget->setRowCount(pointCount);
    //设置列数
    m_tableWidget->setColumnCount(4*imageCount);
    m_tableWidget->setHorizontalHeaderLabels(headerText);

    int newRow = m_tableWidget->rowCount();
    int newCol = m_tableWidget->columnCount();
    //生成表格，默认值为0

    for(int i = 0;i<newRow;i++)
    {
        for (int j=0;j<newCol;j++)
        {
            if(!(i<oldRow && j<oldCol))
            {
                m_tableWidget->setItem(i,j,new QTableWidgetItem("0"));
            }

        }
    }

}
void CameraCalibrationWidget::slot_setClick()
{
    //获取图片的个数
    quint32 imageCount = m_imagesCount->value();
    //获取点的个数
    quint32 pointCount = m_pointCount->value();
    createDataTable(imageCount,pointCount);


}
//执行标定
void CameraCalibrationWidget::slot_executeClick()
{
    std::vector<std::vector<PointF> > objectPointsL, imagePointsL;

    if(m_tableWidget->size().width() ==0 || m_tableWidget->size().height() ==0 )
    {
        return;
    }
    //获取界面坐标数据
    for(int i=0;i<m_tableWidget->columnCount()/4;i++)
    {
        std::vector<PointF> object,image;
        for(int j=0;j<m_tableWidget->rowCount();j++)
        {
            PointF pairPix;
            PointF pairPhy;
            pairPix._x = (m_tableWidget->item(j,i*4)->text().toDouble());
            pairPix._y = (m_tableWidget->item(j,i*4+1)->text().toDouble());
            pairPhy._x = (m_tableWidget->item(j,i*4+2)->text().toDouble());
            pairPhy._y = (m_tableWidget->item(j,i*4+3)->text().toDouble());
            object.push_back(pairPhy);
            image.push_back(pairPix);
        }
        objectPointsL.push_back(object);
        imagePointsL.push_back(image);
    }

    //获取畸变个数
    int disCount;
    QString str = m_comboBoxDis->currentText();
    if(str.isEmpty())
    {
        disCount=0;
    }
    else
    {
        bool isOk;
        disCount = str.split(":",QString::SkipEmptyParts).at(0).toInt(&isOk);
        if(!isOk)
        {
            disCount=0;
        }
    }
    //内参个数
    int inPCount = m_comboBoxInP->currentText().split(":",QString::SkipEmptyParts).at(0).toInt();
    int maxCount = m_iterationCount->value();
    CameraCalibration CameraCalib(objectPointsL, imagePointsL,disCount,inPCount,1e-15,maxCount);

    S_CameraP CameraP = CameraCalib.GetCameraParameter();






    //显示标定结果
    m_showTextEdit->append(QString("标定结果:"));
    m_showTextEdit->append(QString("内参:"));
    m_showTextEdit->append(QString::number(CameraP.intrinsicParameter(0,0),'f',15)+" , "+QString::number(CameraP.intrinsicParameter(0,1),'f',15)+" , "+QString::number(CameraP.intrinsicParameter(0,2),'f',15));
    m_showTextEdit->append(QString::number(CameraP.intrinsicParameter(1,0),'f',15)+" , "+QString::number(CameraP.intrinsicParameter(1,1),'f',15)+" , "+QString::number(CameraP.intrinsicParameter(1,2),'f',15));
    m_showTextEdit->append(QString::number(CameraP.intrinsicParameter(2,0),'f',15)+" , "+QString::number(CameraP.intrinsicParameter(2,1),'f',15)+" , "+QString::number(CameraP.intrinsicParameter(2,2),'f',15));
    m_showTextEdit->append("\r\n");
    m_showTextEdit->append(QString("畸变系数:"));
    for(int i=0;i<CameraP.distortionCoeff.rows();++i)
    {
        switch (i) {
        case 0:
            m_showTextEdit->append("k1: ");
            break;
        case 1:
            m_showTextEdit->append("k2: ");
            break;
        case 2:
            m_showTextEdit->append("p1: ");
            break;
        case 3:
            m_showTextEdit->append("p2: ");
            break;
        case 4:
            m_showTextEdit->append("k3: ");
            break;
        default:
            m_showTextEdit->append(QString::number(i));

        }

        m_showTextEdit->insertPlainText(QString::number(CameraP.distortionCoeff(i),'f',15));
    }
    m_showTextEdit->append("\r\n");

    m_showTextEdit->append(QString("外参:"));
    for(int i=0;i<CameraP.externalParams.size();++i)
    {
        m_showTextEdit->append(QString("外参矩阵: %1").arg(i));
        Eigen::MatrixXd external = CameraP.externalParams.at(i);
        m_showTextEdit->append(QString::number(external(0,0),'f',15)+" , "+QString::number(external(0,1),'f',15)+" , "+QString::number(external(0,2),'f',15)+" , "+QString::number(external(0,3),'f',15));
        m_showTextEdit->append(QString::number(external(1,0),'f',15)+" , "+QString::number(external(1,1),'f',15)+" , "+QString::number(external(1,2),'f',15)+" , "+QString::number(external(1,3),'f',15));
        m_showTextEdit->append(QString::number(external(2,0),'f',15)+" , "+QString::number(external(2,1),'f',15)+" , "+QString::number(external(2,2),'f',15)+" , "+QString::number(external(2,3),'f',15));

    }
    m_showTextEdit->append("\r\n");
    m_showTextEdit->append(QString("单应性:"));
    for(int i=0;i<CameraP.homographyList.size();++i)
    {
        m_showTextEdit->append(QString("单应性矩阵: %1").arg(i));
        Eigen::Matrix3d homography = CameraP.homographyList.at(i);
        m_showTextEdit->append(QString::number(homography(0,0),'f',15)+" , "+QString::number(homography(0,1),'f',15)+" , "+QString::number(homography(0,2),'f',15));
        m_showTextEdit->append(QString::number(homography(1,0),'f',15)+" , "+QString::number(homography(1,1),'f',15)+" , "+QString::number(homography(1,2),'f',15));
        m_showTextEdit->append(QString::number(homography(2,0),'f',15)+" , "+QString::number(homography(2,1),'f',15)+" , "+QString::number(homography(2,2),'f',15));

    }
    m_showTextEdit->append("\r\n");
    for(int i=0;i<CameraP.reprojErrL.size();++i)
    {
        m_showTextEdit->append(QString("第 %1 张图片重投影误差值: %2").arg(i).arg(QString::number(CameraP.reprojErrL.at(i),'f',15)));
    }
    m_showTextEdit->append("\r\n");
    m_showTextEdit->append(QString("总体重投影误差值: %1").arg(QString::number(CameraP.reprojErr,'f',15)));
    m_showTextEdit->append("\r\n");
    m_showTextEdit->append("\r\n");

}

//item没有字符时，双击触发
void CameraCalibrationWidget::slot_tableWidgetItemChanged(QTableWidgetItem *item)
{
    //2、匹配正负整数、正负浮点数
    QString Pattern("(-?[1-9][0-9]+)|(-?[0-9])|(-?[1-9]\\d+\\.\\d+)|(-?[0-9]\\.\\d+)");
    //QString Pattern("/^[+-]?[\\d]+([\\.][\\d]+)?([Ee][+-]?[\\d]+)?$/");
    QRegExp  reg(Pattern);

    //3.获取修改的新的单元格内容
    QString str=item->text();

    if(str.isEmpty()) {
        return;
    }
    //匹配失败，返回原来的字符
    if(!reg.exactMatch(str)){
        QMessageBox::information(this,"匹配失败","请输入小数和整数!");
        item->setText(old_text);  //更换之前的内容
    }
    //1、记录旧的单元格内容
    old_text = item->text();
}

void CameraCalibrationWidget::slot_loadImages()
{
    // 打开文件对话框以选择图像文件
    QStringList filePaths = QFileDialog::getOpenFileNames(nullptr,
                                                    QObject::tr("选择图像文件"),
                                                    QDir::currentPath(),
                                                    QObject::tr("Images (*.png *.jpg *.bmp)"));

    if(filePaths.isEmpty())
    {
        return;
    }
    //读取所有图像
    std::vector<cv::Mat> imgs;
    for(const auto& path : filePaths)
    {
        // 使用OpenCV读取所选图像文件
        cv::Mat image = cv::imread(path.toStdString());
        if (image.empty()) {
            std::cout << "图像打开失败。。。" << std::endl;
            return;
        }
        imgs.push_back(image);
    }
    std::cout << "imgs:" <<imgs.size()<< std::endl;


    //定义角点大小和图像上的角点一致 --  根据实际可修改
    cv::Size boardSize = cv::Size(m_cols->value(), m_rows->value());

    //生成棋盘格每个内角点三维坐标
    cv::Size squareSize = cv::Size(15, 15);
    std::vector<std::vector<cv::Point3f>> objectPoints;

    std::vector<cv::Point3f> tempPoints;
    for (int j = 0; j < boardSize.height; ++j)
    {
        for (int k = 0; k < boardSize.width; ++k)
        {
            cv::Point3f realPoint;
            //假设标定板为世界坐标系的z平面，即z=0
            realPoint.x = k * squareSize.width;
            realPoint.y = j * squareSize.height;
            realPoint.z = 0;
            tempPoints.push_back(realPoint);
        }
    }



    std::vector<std::vector<cv::Point2f>> imgsPoints;
    //获取n张图像像素角点
    for (int i = 0; i < imgs.size(); ++i)
    {
        cv::Mat img1 = imgs[i];
        cv::Mat gray1;
        cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> img1Points;
        //计算棋盘格角点
        bool patternFound = cv::findChessboardCorners(gray1, boardSize, img1Points, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FILTER_QUADS);
        //计算圆心
        //bool patternFound = cv::findCirclesGrid(gray1, boardSize, img1Points);
        // std::cout << patternFound << std::endl;
        if (patternFound)
        {
            // 如果找到了角点，则绘制它们并显示图像
            //优化角点，精确到亚像素
            cv::find4QuadCornerSubpix(gray1, img1Points, cv::Size(5, 5));

            //绘制角点
            cv::drawChessboardCorners(img1, boardSize, img1Points, patternFound);

            imgsPoints.push_back(img1Points);


            QStringList xList;
            QStringList yList;
            //整理合成字符串显示到界面
            for(const auto& point : img1Points)
            {
                //图像像素坐标
                xList.append(QString::number(point.x));
                yList.append(QString::number(point.y));
            }
            m_addTextEdit->append(xList.join(","));
            m_addTextEdit->append(yList.join(","));

            QStringList xwList;
            QStringList ywList;
            for(const auto& point : tempPoints)
            {
                //图像世界坐标
                xwList.append(QString::number(point.x));
                ywList.append(QString::number(point.y));
            }
            m_addTextEdit->append(xwList.join(","));
            m_addTextEdit->append(ywList.join(","));

        }
        else
        {
            QMessageBox::information(this,QString("失败"),QString("没找到角点"));
            // 如果未找到角点，则输出提示信息
            std::cout << "未找到棋盘格角点" << std::endl;
        }
    }

    for (int i = 0; i < imgsPoints.size(); ++i)
    {
        objectPoints.push_back(tempPoints);
    }


    m_imgs = imgs;
    m_objectPoints = objectPoints;
    m_imgsPoints = imgsPoints;
    //cv:: waitKey(0);


    slot_loadToTable();

}
void CameraCalibrationWidget::slot_executeOpenCV()
{

    if(m_imgs.empty())
    {
        return;
    }
    //图像尺寸
    cv::Size imageSize(m_imgs[0].cols,m_imgs[0].rows);

    //相机内参 ， 畸变系数
    cv::Mat cameraMatrix , distCoeffs;

    //每张图形的旋转向量
    std::vector<cv::Mat> rvecs;
    //每张图像的平移向量
    std::vector<cv::Mat> tvecs;
    double reprojError = cv::calibrateCamera(m_objectPoints, m_imgsPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, 0);

    m_showTextEdit->append(QString("reprojError:") + QString::number(reprojError));
    std::cout << "reprojError:" << reprojError << std::endl;

    // 创建一个字符串流对象
    std::ostringstream oss;
    // 将整数输出到字符串流中
    oss << cameraMatrix;
    // 从字符串流中获取字符串
    std::string result = oss.str();

    m_showTextEdit->append(QString("cameraMatrix:") + QString::fromStdString(result));
    std::cout << "cameraMatrix:" << std::endl << cameraMatrix << std::endl;


    // 创建一个字符串流对象
    std::ostringstream oss1;
    // 将整数输出到字符串流中
    oss1 << distCoeffs;
    // 从字符串流中获取字符串
    std::string result1 = oss1.str();
    m_showTextEdit->append(QString("distCoeffs:") + QString::fromStdString(result1));
    std::cout << "distCoeffs:" << std::endl << distCoeffs << std::endl;
    //std::cout << "旋转向量：" << std::endl;
    // for (int i = 0; i < rvecs.size(); ++i)
    // {
    //     std::cout << rvecs[i] << std::endl;
    // }
    // std::cout << "平移向量：" << std::endl;
    // for (int i = 0; i < tvecs.size(); ++i)
    // {
    //     std::cout << tvecs[i] << std::endl;
    // }

    // std::vector<cv::Point2f> imgPoints;
    // cv::Mat Jac;
    // //模型投影 -- 根据世界坐标 根据 内参和畸变系数 投影到图像坐标
    // cv::projectPoints(tempPoints, rvecs[0], tvecs[0], cameraMatrix, distCoeffs, imgPoints, Jac);

    // //cout << "雅可比矩阵：" << endl << Jac << endl;

    // // //根据内参和畸变系数对图像去畸变
    // // for (int i = 0; i < imgs.size(); ++i)
    // // {
    // //     cv::Mat srcImg = imgs[i];
    // //     cv::Mat dstImg;
    // //     cv::undistort(srcImg, dstImg, cameraMatrix, distCoeffs);

    // //     //显示每一个图像
    // //     std::string srcS = "srcImg" + std::to_string(i);
    // //     std::string dstS = "dstImg" + std::to_string(i);
    // //     cv::namedWindow(srcS, cv::WINDOW_NORMAL);
    // //     cv::namedWindow(dstS, cv::WINDOW_NORMAL);
    // //     cv::imshow(srcS, srcImg);
    // //     cv::imshow(dstS, dstImg);
    // // }


    // //求世界坐标和当前图像坐标的相机的位姿
    // //旋转向量
    // cv::Mat rvecs1;
    // //平移向量
    // cv::Mat tvecs1;
    // cv::solvePnP(objectPoints[0], imgsPoints[0], cameraMatrix, distCoeffs, rvecs1, tvecs1);

    // //solvePnP(objectPoints[0], imgsPoints[0], cameraMatrix, distCoeffs, rvecs[0], tvecs[0], true);
    // std::cout << "rvecs1:" << rvecs1<<std::endl;
    // std::cout << "tvecs1:" << tvecs1 << std::endl;
}
