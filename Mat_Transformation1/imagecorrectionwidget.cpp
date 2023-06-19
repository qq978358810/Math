#include "imagecorrectionwidget.h"

ImageCorrectionWidget::ImageCorrectionWidget(QWidget *parent) : QWidget(parent)
{

    m_alpha = new QLineEdit();
    m_gamma = new QLineEdit();
    m_beta = new QLineEdit();
    m_u0 = new QLineEdit();
    m_v0 = new QLineEdit();
    m_0 = new QLineEdit();
    m_1 = new QLineEdit();
    m_00 = new QLineEdit();
    m_000 = new QLineEdit();
    m_alpha->setPlaceholderText("0");
    m_gamma->setPlaceholderText("0");
    m_beta->setPlaceholderText("0");
    m_u0->setPlaceholderText("0");
    m_v0->setPlaceholderText("0");
    m_0->setPlaceholderText("0");
    m_00->setPlaceholderText("0");
    m_000->setPlaceholderText("0");
    m_1->setPlaceholderText("1");
    m_0->setReadOnly(true);
    m_1->setReadOnly(true);
    m_00->setReadOnly(true);
    m_000->setReadOnly(true);

    m_k1 = new QLineEdit();
    m_k2 = new QLineEdit();
    m_p1 = new QLineEdit();
    m_p2 = new QLineEdit();
    m_k3 = new QLineEdit();
    m_k1->setPlaceholderText("0");
    m_k2->setPlaceholderText("0");
    m_p1->setPlaceholderText("0");
    m_p2->setPlaceholderText("0");
    m_k3->setPlaceholderText("0");


    //使用正则匹配正负整数、正负浮点数
    QString Pattern("(-?[1-9][0-9]+)|(-?[0-9])|(-?[1-9]\\d+\\.\\d+)|(-?[0-9]\\.\\d+)");
    QRegExp  reg(Pattern);
    QRegExpValidator *pReg = new QRegExpValidator(reg, this);
    m_alpha->setValidator(pReg);
    m_gamma->setValidator(pReg);
    m_beta->setValidator(pReg);
    m_u0->setValidator(pReg);
    m_v0->setValidator(pReg);

    m_k1->setValidator(pReg);
    m_k2->setValidator(pReg);
    m_p1->setValidator(pReg);
    m_p2->setValidator(pReg);
    m_k3->setValidator(pReg);

    m_execute = new QPushButton(QString("执行"));
    m_execute->setMinimumHeight(100);
    m_execute->setMinimumWidth(100);

    m_src = new QPushButton(QString("源地址"));
    m_dst = new QPushButton(QString("目标地址"));
    m_inputTextEdit = new QTextEdit();
    m_outTextEdit = new QTextEdit();
    m_inputTextEdit->setReadOnly(true);
    m_outTextEdit->setReadOnly(true);

    QGridLayout * GLayout_1 = new QGridLayout();
    GLayout_1->addWidget(new QLabel(QString("内参矩阵(3×3)")),0,0,1,-1);
    GLayout_1->addWidget(m_alpha,1,0);
    GLayout_1->addWidget(m_gamma,1,1);
    GLayout_1->addWidget(m_u0,1,2);
    GLayout_1->addWidget(m_0,2,0);
    GLayout_1->addWidget(m_beta,2,1);
    GLayout_1->addWidget(m_v0,2,2);
    GLayout_1->addWidget(m_00,3,0);
    GLayout_1->addWidget(m_000,3,1);
    GLayout_1->addWidget(m_1,3,2);

    QGridLayout * GLayout_2 = new QGridLayout();
    GLayout_2->addWidget(new QLabel(QString("畸变系数")),0,0,1,-1);
    GLayout_2->addWidget(new QLabel(QString("k1:")),1,0);
    GLayout_2->addWidget(m_k1,1,1);
    GLayout_2->addWidget(new QLabel(QString("k2:")),2,0);
    GLayout_2->addWidget(m_k2,2,1);
    GLayout_2->addWidget(new QLabel(QString("p1:")),3,0);
    GLayout_2->addWidget(m_p1,3,1);
    GLayout_2->addWidget(new QLabel(QString("p2:")),4,0);
    GLayout_2->addWidget(m_p2,4,1);
    GLayout_2->addWidget(new QLabel(QString("k3:")),5,0);
    GLayout_2->addWidget(m_k3,5,1);

    QHBoxLayout* HLayout = new QHBoxLayout();
    HLayout->addLayout(GLayout_1);
    HLayout->addLayout(GLayout_2);

    QHBoxLayout* HLayout_1 = new QHBoxLayout();
    HLayout_1->addWidget(m_src);
    HLayout_1->addWidget(m_dst);

    QHBoxLayout* HLayout_2 = new QHBoxLayout();
    HLayout_2->addWidget(m_inputTextEdit);
    HLayout_2->addWidget(m_outTextEdit);

    QGridLayout* GLayout = new QGridLayout();
    GLayout->addLayout(HLayout,0,0);
    GLayout->addWidget(m_execute,0,1);
    GLayout->addLayout(HLayout_1,1,0,1,-1);
    GLayout->addLayout(HLayout_2,2,0,-1,-1);


    this->setLayout(GLayout);

    connect(m_execute,&QPushButton::clicked,this,&ImageCorrectionWidget::slot_executeClicked);
    connect(m_src,&QPushButton::clicked,this,[=](){

        //获取桌面路径
        QString desktop_path = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);

        QStringList files = QFileDialog::getOpenFileNames(
                    this,
                    tr("请选择1张或多张图片！"),
                    desktop_path,
                    "Images (*.png *.xpm *.jpg)");
        if(!files.isEmpty())
        {
            m_inputTextEdit->clear();
            m_files.clear();
            //显示
            for(int i=0;i<files.count();++i)
            {
                QString file = QDir::toNativeSeparators(files.at(i));
                m_files.append(file);
                m_inputTextEdit->append(file);
            }
        }
    });
    connect(m_dst,&QPushButton::clicked,this,[=](){
        //获取桌面路径
        QString desktop_path = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);

        QString dir = QFileDialog::getExistingDirectory(this, tr("保存"),
                                                         desktop_path,
                                                         QFileDialog::ShowDirsOnly
                                                         | QFileDialog::DontResolveSymlinks);
        if(!dir.isEmpty())
        {
            m_savePath = QDir::toNativeSeparators(dir);
            m_outTextEdit->append(QString("图片矫正后保存路径: ") + m_savePath);
        }
    });

}
//执行
void ImageCorrectionWidget::slot_executeClicked()
{
    //获取内参和畸变
    Eigen::Matrix3d intrinsicParameter;
    intrinsicParameter<<m_alpha->text().toDouble(),m_gamma->text().toDouble(),m_u0->text().toDouble(),
            0,m_beta->text().toDouble(),m_v0->text().toDouble(),
            0,0,1;
    Eigen::VectorXd distortionCoeff(5);
    distortionCoeff(0) = m_k1->text().toDouble();
    distortionCoeff(1) = m_k2->text().toDouble();
    distortionCoeff(2) = m_p1->text().toDouble();
    distortionCoeff(3) = m_p2->text().toDouble();
    distortionCoeff(4) = m_k3->text().toDouble();

    for(int i=0;i<m_files.count();++i)
    {

        QImage srcImage(m_files.at(i));

        QImage dstImage = srcImage.scaled(1280,1706, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);

        //QImage dstImage = ddImage.convertToFormat(QImage::Format_RGB32,Qt::ColorOnly);
       // QImage dstImage(640,480,QImage::Format_RGB32);
       // QImage dstImage = RectifiedImage(ddImage,intrinsicParameter,distortionCoeff);
        QString fileName = m_files.at(i).split(QDir::separator()).last();
        QString path = m_savePath+QDir::separator()+QDateTime::currentDateTime().toString("yyyy.MM.dd-hh.mm.ss.zzz")+"_"+fileName;
        if(dstImage.save(path))
        {
            m_outTextEdit->append(path+" : 保存成功!");
        }
        else
        {
            m_outTextEdit->append(path+" : 保存失败!");
        }
    }
}
//修复图片
QImage ImageCorrectionWidget::RectifiedImage(const QImage& srcImage, const Eigen::Matrix3d& intrinsicParameter,const Eigen::VectorXd& distortionCoeff)
{

    int width = srcImage.width ();                               // 图像宽度
    int height = srcImage.height ();                             // 图像高度
    //qDebug()<< srcImage.depth() <<srcImage.allGray()<<"  width:"<<width<<"  height:"<<height;
    //获取rgb数值
    Eigen::MatrixXi srcR(height,width),srcG(height,width),srcB(height,width);
    for (int i = 0; i < height; i++)                        // 遍历每一行
    {
        for ( int j = 0; j < width; j++ )                   // 遍历每一列
        {
            QColor color = srcImage.pixelColor(j,i);
            srcR(i,j) = color.red();
            srcG(i,j) = color.green();
            srcB(i,j) = color.blue();

        }
    }
    //rgb数值转换去畸变
    Eigen::MatrixXi dstR = GlobleAlgorithm::getInstance()->RectifiedImage(srcR,intrinsicParameter,distortionCoeff);
    Eigen::MatrixXi dstG = GlobleAlgorithm::getInstance()->RectifiedImage(srcG,intrinsicParameter,distortionCoeff);
    Eigen::MatrixXi dstB = GlobleAlgorithm::getInstance()->RectifiedImage(srcB,intrinsicParameter,distortionCoeff);
    QImage dstImage(width,height,srcImage.format());
    //填充去畸变图像
    for (int i = 0; i < height; i++)                        // 遍历每一行
    {
        for ( int j = 0; j < width; j++ )                   // 遍历每一列
        {
            dstImage.setPixelColor(j,i,QColor(dstR(i,j),dstG(i,j),dstB(i,j)));
        }
    }

    return dstImage;

}
