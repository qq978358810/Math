#include "curvedwidget.h"
#include "ui_curvedwidget.h"

CurvedWidget::CurvedWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::CurvedWidget)
{
    ui->setupUi(this);





    // //QPushButton *ss = new QPushButton(this);
    // // 创建场景
    // QGraphicsScene* scene= new QGraphicsScene(this);

    // // 添加图像项到场景
    // QPixmap pixmap("D:/桌面/简历/Snipaste_2024-02-28_17-55-30.jpg");
    // QGraphicsPixmapItem *pixmapItem = scene->addPixmap(pixmap);

    // // 设置图像项的位置
    // pixmapItem->setPos(100, 100);

    // // 创建视图并设置场景
    // QGraphicsView view(scene);
    // view.setWindowTitle("Qt Graphics View Example");

    // // 设置视图的大小
    // view.resize(400, 300);

    // // 显示视图
    // view.show();



    // QGraphicsScene *s1 = new QGraphicsScene(this);
    // s1->addLine(10, 10, 300, 300);
    //  QPixmap pixmap("D:/桌面/简历/Snipaste_2024-02-28_17-55-30.jpg");
    // QGraphicsPixmapItem *pixmapItem = s1->addPixmap(pixmap);
    //  // 添加曲面图像项到场景
    // // CurvedPixmapItem *curvedItem = new CurvedPixmapItem(pixmap);
    // s1->addItem(pixmapItem);


    // QGraphicsView *v1 = new QGraphicsView(this);
    // v1->setScene(s1);
    // v1->setRenderHint(QPainter::Antialiasing);
    // //v1->move(0,50);
    // //v1->resize(1000,1000);
    // v1->setWindowTitle("Graphics View");
    // v1->show();
}


CurvedWidget::~CurvedWidget()
{
    delete ui;
}



//加载
void CurvedWidget::on_pushButton_clicked()
{
    QString imageFile = QFileDialog::getOpenFileName(this, "Open Image", "./", tr("Images (*.png *.xpm *.jpg)"));

    QFile file(imageFile);
    if (!file.exists())
        return;

    m_Image.load(imageFile);
    ui->widget->loadImage(m_Image);

    ui->spinBox->setValue(0);ui->spinBox_2->setValue(0);
    ui->spinBox_5->setValue(m_Image.width() - 1);ui->spinBox_6->setValue(0);
    ui->spinBox_9->setValue(m_Image.width() - 1); ui->spinBox_10->setValue(m_Image.height() - 1);
    ui->spinBox_13->setValue(0); ui->spinBox_14->setValue(m_Image.height() - 1);

}

//执行
void CurvedWidget::on_pushButton_2_clicked()
{
    // 读取图像
    if(m_Image.isNull())
        return;
    Eigen::MatrixXf src_image[3];
    HyperboloidMapping::image_2_matrix(m_Image,src_image);

    Eigen::Vector2f src_points[4];
    Eigen::Vector2f dst_points[4];
    if(ui->checkBox_2->checkState()!=Qt::Checked)
    {

        // 定义原始图像的四个角点和目标图像的四个角点
        src_points[0] = Eigen::Vector2f(0, 0);
        src_points[1] = Eigen::Vector2f(src_image[0].cols() - 1, 0);
        src_points[2] = Eigen::Vector2f(src_image[0].cols() - 1, src_image[0].rows() - 1);
        src_points[3] = Eigen::Vector2f(0, src_image[0].rows() - 1);

        dst_points[0] = Eigen::Vector2f(0, 0);
        dst_points[1] = Eigen::Vector2f(src_image[0].cols() - 1, 0);
        dst_points[2] = Eigen::Vector2f(src_image[0].cols() - 1, src_image[0].rows() - 1);
        dst_points[3] = Eigen::Vector2f(0, src_image[0].rows() - 1);


    }
    else
    {
        // 定义原始图像的四个角点和目标图像的四个角点
        src_points[0] =  Eigen::Vector2f((float)ui->spinBox->value(), (float)ui->spinBox_2->value());
        src_points[1] =  Eigen::Vector2f((float)ui->spinBox_5->value(), (float)ui->spinBox_6->value());
        src_points[2] =  Eigen::Vector2f((float)ui->spinBox_9->value(), (float)ui->spinBox_10->value());
        src_points[3] =  Eigen::Vector2f((float)ui->spinBox_13->value(), (float)ui->spinBox_14->value());

        dst_points[0] =  Eigen::Vector2f((float)ui->spinBox_3->value(), (float)ui->spinBox_4->value());
        dst_points[1] =  Eigen::Vector2f((float)ui->spinBox_7->value(),(float)ui->spinBox_8->value());
        dst_points[2] =  Eigen::Vector2f((float)ui->spinBox_11->value(), (float)ui->spinBox_12->value());
        dst_points[3] =  Eigen::Vector2f((float)ui->spinBox_15->value(), (float)ui->spinBox_16->value());

    }


    HyperboloidMapping hy(src_points, dst_points,src_image[0].rows(),src_image[0].cols(),(int)ui->spinBox_19->value(),(int)ui->spinBox_20->value());
    Eigen::MatrixXf dst_image[3];

    // 开始计时
    auto start = std::chrono::high_resolution_clock::now();


    hy.warp_hyperbola(src_image,dst_image,(float)ui->doubleSpinBox->value(),ui->checkBox->checkState()==Qt::Checked ? true : false);
    // 结束计时
    auto end = std::chrono::high_resolution_clock::now();
    // 计算持续时间
    auto duration = end - start;

    auto durationInMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
    std::cout << "::" << durationInMilliseconds.count() << "ms" << std::endl;

    //转换image
    QImage dst_image_2;
    HyperboloidMapping::matrix_2_image(dst_image,dst_image_2);

    ui->widget_2->loadImage(dst_image_2);
}

