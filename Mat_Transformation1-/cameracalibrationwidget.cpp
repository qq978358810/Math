#include "cameracalibrationwidget.h"

CameraCalibrationWidget::CameraCalibrationWidget(QWidget *parent) : QWidget(parent)
{

    //创建控件
    m_set = new QPushButton(QString("设置"));
    m_execute = new QPushButton(QString("执行"));
    m_pointCount = new QSpinBox();
    m_imagesCount = new QSpinBox();
    m_load = new QPushButton(QString("加载到表格"));
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

    m_tableWidget = new QTableWidget();
    m_tableWidget->setMinimumWidth(550);
    m_addTextEdit = new QTextEdit();
    //不自动换行
    m_addTextEdit->setLineWrapMode(QTextEdit::NoWrap);
    m_showTextEdit = new QTextEdit();
    //不自动换行
    m_showTextEdit->setLineWrapMode(QTextEdit::NoWrap);

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
    GLayout_1->addLayout(HLayout,0,0,1,-1);
    GLayout_1->addWidget(m_addTextEdit,1,0);
    GLayout_1->addLayout(GLayout_2,1,1,-1,-1);

    this->setLayout(GLayout_1);

    //连接信号和槽
    connect(m_set,&QPushButton::clicked,this,&CameraCalibrationWidget::slot_setClick);
    connect(m_execute,&QPushButton::clicked,this,&CameraCalibrationWidget::slot_executeClick);
    connect(m_tableWidget,&QTableWidget::itemChanged,this,&CameraCalibrationWidget::slot_tableWidgetItemChanged);
    connect(m_load,&QPushButton::clicked,this,&CameraCalibrationWidget::slot_loadToTable);



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
    QList<QList<QPointF> > objectPointsL, imagePointsL;

    //获取界面坐标数据
    for(int i=0;i<m_tableWidget->columnCount()/4;i++)
    {
        QList<QPointF> object,image;
        for(int j=0;j<m_tableWidget->rowCount();j++)
        {
            QPointF pairPix;
            QPointF pairPhy;
            pairPix.setX(m_tableWidget->item(j,i*4)->text().toDouble());
            pairPix.setY(m_tableWidget->item(j,i*4+1)->text().toDouble());
            pairPhy.setX(m_tableWidget->item(j,i*4+2)->text().toDouble());
            pairPhy.setY(m_tableWidget->item(j,i*4+3)->text().toDouble());
            object.append(pairPhy);
            image.append(pairPix);
        }
        objectPointsL.append(object);
        imagePointsL.append(image);
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
    CameraCalibration CameraCalib(objectPointsL, imagePointsL,disCount,inPCount,1e-12,maxCount);

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
    for(int i=0;i<CameraP.externalParams.count();++i)
    {
        m_showTextEdit->append(QString("外参矩阵: %1").arg(i));
        Eigen::MatrixXd external = CameraP.externalParams.at(i);
        m_showTextEdit->append(QString::number(external(0,0),'f',15)+" , "+QString::number(external(0,1),'f',15)+" , "+QString::number(external(0,2),'f',15)+" , "+QString::number(external(0,3),'f',15));
        m_showTextEdit->append(QString::number(external(1,0),'f',15)+" , "+QString::number(external(1,1),'f',15)+" , "+QString::number(external(1,2),'f',15)+" , "+QString::number(external(1,3),'f',15));
        m_showTextEdit->append(QString::number(external(2,0),'f',15)+" , "+QString::number(external(2,1),'f',15)+" , "+QString::number(external(2,2),'f',15)+" , "+QString::number(external(2,3),'f',15));

    }
    m_showTextEdit->append("\r\n");
    m_showTextEdit->append(QString("单应性:"));
    for(int i=0;i<CameraP.homographyList.count();++i)
    {
        m_showTextEdit->append(QString("单应性矩阵: %1").arg(i));
        Eigen::Matrix3d homography = CameraP.homographyList.at(i);
        m_showTextEdit->append(QString::number(homography(0,0),'f',15)+" , "+QString::number(homography(0,1),'f',15)+" , "+QString::number(homography(0,2),'f',15));
        m_showTextEdit->append(QString::number(homography(1,0),'f',15)+" , "+QString::number(homography(1,1),'f',15)+" , "+QString::number(homography(1,2),'f',15));
        m_showTextEdit->append(QString::number(homography(2,0),'f',15)+" , "+QString::number(homography(2,1),'f',15)+" , "+QString::number(homography(2,2),'f',15));

    }
    m_showTextEdit->append("\r\n");
    for(int i=0;i<CameraP.reprojErrL.count();++i)
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

