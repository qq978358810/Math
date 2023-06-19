#include "viewtransform.h"

ViewTransform::ViewTransform(QWidget *parent) : QWidget(parent)
{

    m_src = new QPushButton(QString("源地址"));
    m_src->setMaximumWidth(100);
    m_dst = new QPushButton(QString("目标地址"));
    m_dst->setMaximumWidth(100);

    m_srcView = new ImageViewer();
    m_dstView = new ImageViewer();

    connect(m_src,&QPushButton::clicked,this,[=](){

        //获取桌面路径
        QString desktop_path = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);

        QString file = QFileDialog::getOpenFileName(
                    this,
                    tr("请选择1张图片！"),
                    desktop_path,
                    "Image (*.png *.xpm *.jpg)");
        if(!file.isEmpty())
        {
            
        }

    });
    connect(m_dst,&QPushButton::clicked,this,[=](){
        //获取桌面路径
        QString desktop_path = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);

        QString dir = QFileDialog::getExistingDirectory(this, tr("保存"),
                                                         desktop_path,
                                                         QFileDialog::ShowDirsOnly
                                                         | QFileDialog::DontResolveSymlinks);

    });


    QHBoxLayout  *hLayout = new QHBoxLayout();
    hLayout->addWidget(m_src);
    hLayout->addStretch();
    hLayout->addWidget(m_dst);


    //QVBoxLayout *layout = new QVBoxLayout();
    QSplitter *splitter = new QSplitter(Qt::Horizontal, this);
    //splitter->setStyleSheet();
    QWidget *leftWidget = new QWidget(splitter);
    QHBoxLayout *leftLayout = new QHBoxLayout(leftWidget);
    leftLayout->addWidget(m_srcView);
    QWidget *rightWidget = new QWidget(splitter);
    QHBoxLayout *rightLayout = new QHBoxLayout(rightWidget);
    rightLayout->addWidget(m_dstView);
    splitter->addWidget(leftWidget);
    splitter->addWidget(rightWidget);
    //layout->addWidget(splitter);

    QGridLayout  *GLayout = new QGridLayout();
    GLayout->addWidget(m_src,0,0);
    GLayout->addWidget(m_dst,0,1);
    GLayout->addWidget(splitter,1,0);

    this->setLayout(GLayout);



}
