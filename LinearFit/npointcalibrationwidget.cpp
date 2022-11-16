#include "npointcalibrationwidget.h"

NPointCalibrationWidget::NPointCalibrationWidget(QWidget *parent) : QWidget(parent)
{

    //创建控件
    m_set = new QPushButton(QString("设置"));
    m_execute = new QPushButton(QString("执行"));
    m_openFile = new QPushButton(QString("打开"));
    m_fitCount = new QSpinBox();
    m_OLS = new QCheckBox(QString("最小二乘"));
    m_IRLS = new QCheckBox(QString("迭代重加权最小二乘"));
    m_iterationCount = new QSpinBox();
    m_normCount = new QDoubleSpinBox();



    m_tableWidget = new QTableWidget();
    m_tableWidget->setMinimumWidth(550);
    m_textEdit = new QTextEdit();

    QHBoxLayout * HLayout = new QHBoxLayout();
    HLayout->addWidget(m_OLS);
    HLayout->addStretch();
    HLayout->addWidget(m_IRLS);
    HLayout->addWidget(new QLabel(QString("迭代次数：")));
    HLayout->addWidget(m_iterationCount);
    HLayout->addWidget(new QLabel(QString("L(p)范数：")));
    HLayout->addWidget(m_normCount);
    HLayout->addStretch();



    QGridLayout* GLayout_2  = new QGridLayout();
    GLayout_2->addWidget(m_fitCount,0,0);
    GLayout_2->addWidget(new QLabel(QString("点拟合")),0,1);
    GLayout_2->addWidget(m_set,0,2);
    GLayout_2->setColumnStretch(3,0);
    GLayout_2->addWidget(m_openFile,0,4);
    GLayout_2->addWidget(m_execute,0,5);
    GLayout_2->addWidget(m_tableWidget,1,0,-1,-1);


    QGridLayout* GLayout_1  = new QGridLayout();
    GLayout_1->addLayout(HLayout,0,0);
    GLayout_1->addWidget(m_textEdit,1,0,-1,-1);
    GLayout_1->addLayout(GLayout_2,1,1,-1,-1);

    this->setLayout(GLayout_1);

    //连接信号和槽
    connect(m_set,&QPushButton::clicked,this,&NPointCalibrationWidget::slot_setClick);
    connect(m_execute,&QPushButton::clicked,this,&NPointCalibrationWidget::slot_executeClick);
    connect(m_openFile,&QPushButton::clicked,this,&NPointCalibrationWidget::slot_openFileClick);
    connect(m_tableWidget,&QTableWidget::itemChanged,this,&NPointCalibrationWidget::slot_tableWidgetItemChanged);

    //设置列数
    m_tableWidget->setColumnCount(4);
    //设置标题
    //表头标题用QStringList来表示
    QStringList headerText;
    headerText<<"像素X坐标"<<"像素Y坐标"<<"物理X坐标"<<"物理Y坐标";
    m_tableWidget->setHorizontalHeaderLabels(headerText);




    QButtonGroup *buttonGroup = new QButtonGroup(this);
    //互锁
    buttonGroup->addButton(m_OLS);
    buttonGroup->addButton(m_IRLS);
    buttonGroup->setExclusive(true);
    m_OLS->setChecked(true);

    m_fitCount->setMinimum(3);
    m_fitCount->setMaximum(9999);
    m_fitCount->setValue(3);

    m_iterationCount->setMinimum(0);
    m_iterationCount->setMaximum(99999999);
    m_iterationCount->setValue(999);

    m_normCount->setMinimum(0);
    m_normCount->setMaximum(999);
    m_normCount->setValue(1);

}





void NPointCalibrationWidget::slot_setClick()
{
    //获取拟合的个数
    quint32 transitionCount = m_fitCount->value();

    int oldRow = m_tableWidget->rowCount();
    //设置行数
    m_tableWidget->setRowCount(transitionCount);

    int newRow = m_tableWidget->rowCount();
    //生成表格，默认值为0
    if(newRow>oldRow) {
        for(int i = oldRow;i<newRow;i++) {
            for (int j=0;j<4;j++) {
                m_tableWidget->setItem(i,j,new QTableWidgetItem("0"));
            }
        }
    }
}
//执行标定
void NPointCalibrationWidget::slot_executeClick()
{
    _coordList coordL;
    //获取界面坐标数据
    for(int i=0;i<m_tableWidget->rowCount();i++) {
        QPointF pairPix;
        QPointF pairPhy;

        for(int j=0;j<4;j++) {
            switch (j) {
            case 0:
                pairPix.setX(m_tableWidget->item(i,j)->text().toDouble());
                break;
            case 1:
                pairPix.setY(m_tableWidget->item(i,j)->text().toDouble());
                break;
            case 2:
                pairPhy.setX(m_tableWidget->item(i,j)->text().toDouble());
                break;
            case 3:
                pairPhy.setY(m_tableWidget->item(i,j)->text().toDouble());
                break;
            default:
                break;

            }
        }
        coordL.pixPoines.append(pairPix);
        coordL.physicsPoines.append(pairPhy);
    }

    //执行标定，获取结果
    _mat mat = GlobleFunction::calibration(coordL);


    //显示标定结果
    m_textEdit->append(QString("内部算法【1】标定: 矩阵结果:"));
    m_textEdit->append(QString::number(mat.a,'e',15)+" , "+QString::number(mat.b,'e',15)+" , "+QString::number(mat.c,'e',15));
    m_textEdit->append(QString::number(mat.d,'e',15)+" , "+QString::number(mat.e,'e',15)+" , "+QString::number(mat.f,'e',15));
    m_textEdit->append(QString::number(mat.g,'e',15)+" , "+QString::number(mat.h,'e',15)+" , "+QString::number(mat.i,'e',15));
    m_textEdit->append("\r\n");


    int rows = coordL.pixPoines.count();

    Matrix<double,3,1> abc_3_1,abc_3_2;
    Matrix<double,Dynamic,1> matB_x,matB_y;
    matB_x.resize(rows,1);
    matB_y.resize(rows,1);
    //创建动态n行，3列矩阵
    Matrix<double,Dynamic,3> matA;
    matA.resize(rows,3);
    //构建矩阵
    for(int i=0;i<rows;++i)
    {
        matA(i,0) = coordL.pixPoines.at(i).x();
        matA(i,1) = coordL.pixPoines.at(i).y();
        matA(i,2) = 1;
        matB_x(i,0) = coordL.physicsPoines.at(i).x();
        matB_y(i,0) = coordL.physicsPoines.at(i).y();
    }

    //勾选最小二乘
    if(m_OLS->isChecked())
    {
        abc_3_1 = GlobleFunction::leastSquares(matA,matB_x);
        abc_3_2 = GlobleFunction::leastSquares(matA,matB_y);
    }
    //勾选迭代加权最小二乘
    else if(m_IRLS->isChecked())
    {
        int kk= m_iterationCount->value();
        double p = m_normCount->value();
        abc_3_1 =GlobleFunction::iterativeReweightedLeastSquares(matA,matB_x,p,kk);
        abc_3_2 =GlobleFunction::iterativeReweightedLeastSquares(matA,matB_y,p,kk);
    }
    else
    {
        return ;
    }

    //显示标定结果
    m_textEdit->append(QString("外部算法【2】标定: 矩阵结果:"));
    m_textEdit->append(QString::number(abc_3_1(0,0),'e',15)+" , "+QString::number(abc_3_1(1,0),'e',15)+" , "+QString::number(abc_3_1(2,0),'e',15));
    m_textEdit->append(QString::number(abc_3_2(0,0),'e',15)+" , "+QString::number(abc_3_2(1,0),'e',15)+" , "+QString::number(abc_3_2(2,0),'e',15));
    m_textEdit->append(QString::number(0,'e',15)+" , "+QString::number(0,'e',15)+" , "+QString::number(1,'e',15));
    m_textEdit->append("\r\n");
}
//打开文件-》从文件读取坐标
void NPointCalibrationWidget::slot_openFileClick()
{
    QString fileName = QFileDialog::getOpenFileName(this,tr("文件对话框！"), "",tr("文件(*.csv *.txt)"));
    if(!fileName.isEmpty()) {
        QFile file(fileName);
        quint32 line=0;
        if(file.open(QIODevice::ReadOnly|QIODevice::Text)) {
            for (;;) {
                QString data = file.readLine();
                if(data.isEmpty()) {
                    QMessageBox::information(this,"完成","数据读取完成!");
                    return;
                }
                QStringList sL = data.split(",");
                if(sL.count()!=4) {
                    QMessageBox::information(this,"错误","请确定每行数据为4个并且以,分割!");
                    return;
                }
                for(int i=0;i<sL.count();i++) {
                    bool isOk;
                    sL.at(i).toDouble(&isOk);
                    sL.at(i).toULongLong(&isOk);
                    if(isOk) {
                        //设置行数
                        m_tableWidget->setRowCount(line+1);
                        m_tableWidget->setItem(line,i,new QTableWidgetItem(sL.at(i)));
                    }else {
                        QMessageBox::information(this,"错误","数据转换double失败!");
                    }
                }

                line++;
            }
        }
    }
}
//item没有字符时，双击触发
void NPointCalibrationWidget::slot_tableWidgetItemChanged(QTableWidgetItem *item)
{
    //2、匹配正负整数、正负浮点数
    QString Pattern("(-?[1-9][0-9]+)|(-?[0-9])|(-?[1-9]\\d+\\.\\d+)|(-?[0-9]\\.\\d+)");
    QRegExp  reg(Pattern);

    //3.获取修改的新的单元格内容
    QString str=item->text();

    if(str.isEmpty()) {
        return;
    }
    //匹配失败，返回原来的字符
    if(!reg.exactMatch(str)){
        QMessageBox::information(this,"匹配失败","请输入小数和整数!");
        item->setText("0");  //更换之前的内容
    }
    //1、记录旧的单元格内容
    old_text = item->text();
}



