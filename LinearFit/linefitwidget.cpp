#include "linefitwidget.h"

LineFitWidget::LineFitWidget(QWidget *parent) : QWidget(parent)
{
    //绘图初始默认大小
    double customPlotSize = 800;

    //创建控件
    m_clearCustomPlot = new QPushButton(QString("清除界面"));
    m_zoomIn = new QPushButton(QString("放大"));
    m_zoomOut = new QPushButton(QString("缩小"));
    m_origin = new QPushButton(QString("原点"));
    m_set = new QPushButton(QString("设置"));
    m_execute = new QPushButton(QString("执行"));
    m_fitCount = new QSpinBox();
    m_OLS = new QCheckBox(QString("最小二乘"));
    m_IRLS = new QCheckBox(QString("迭代重加权最小二乘"));
    m_iterationCount = new QSpinBox();
    m_normCount = new QDoubleSpinBox();



    m_tableWidget = new QTableWidget();
    m_tableWidget->setMaximumWidth(500);
    m_textEdit = new QTextEdit();
    m_textEdit->setMaximumWidth(500);
    m_customPlot = new QCustomPlot();
    m_customPlot->setMinimumWidth(500);
    m_customPlot->setMinimumHeight(500);
    m_customPlot->resize(customPlotSize,customPlotSize);
    m_comboBox = new QComboBox();
    m_comboBox->setMinimumWidth(100);
    //悬浮提示
    m_comboBox->setToolTip(       "下列数字代表代表多项式函数最高次幂\n"
                                  "比如 1 => y = a0 * x^0 + a1 * x^1  求未知系数a0和a1\n"
                                  "比如 2 => y = a0 * x^0 + a1 * x^1 + a2 * x^2  求未知系数a0和a1和a2\n"
                                  "比如 3 => y = a0 * x^0 + a1 * x^1 + a2 * x^2 + a3 * x^3  求未知系数a0和a1和a2和a3\n"
                                  ".....\n"
                                  "依次递推");
    m_comboBox->addItem(MORE);
    for(int i=0;i<10;++i)
    {
        m_comboBox->addItem(QString::number(i+1));
    }
    m_maxPower = 0;


    QHBoxLayout * HLayout = new QHBoxLayout();
    HLayout->addWidget(m_OLS);
    HLayout->addStretch();
    HLayout->addWidget(m_IRLS);
    HLayout->addWidget(new QLabel(QString("迭代次数：")));
    HLayout->addWidget(m_iterationCount);
    HLayout->addWidget(new QLabel(QString("L(p)范数：")));
    HLayout->addWidget(m_normCount);
    HLayout->addStretch();

    QVBoxLayout* VLayout = new QVBoxLayout();
    VLayout->addWidget(m_tableWidget);
    VLayout->addWidget(m_textEdit);

    QGridLayout* GLayout_2  = new QGridLayout();
    GLayout_2->addWidget(m_fitCount,0,0);
    GLayout_2->addWidget(new QLabel(QString("点拟合")),0,1);
    GLayout_2->addWidget(m_set,0,2);
    GLayout_2->setColumnStretch(3,0);
    GLayout_2->addWidget(m_execute,0,4);
    GLayout_2->addLayout(VLayout,1,0,-1,-1);


    QGridLayout* GLayout  = new QGridLayout();
    GLayout->addWidget(m_clearCustomPlot,0,0);
    GLayout->addWidget(m_origin,0,1);
    GLayout->addWidget(m_zoomIn,0,2);
    GLayout->addWidget(m_zoomOut,0,3);
    GLayout->setColumnStretch(4,1);//设置列比例系数
    GLayout->addWidget(m_comboBox,0,5);
    GLayout->addWidget(m_customPlot,1,0,-1,-1);

    QGridLayout* GLayout_1  = new QGridLayout();
    GLayout_1->addLayout(HLayout,0,0);
    GLayout_1->addLayout(GLayout,1,0);
    GLayout_1->addLayout(GLayout_2,1,1);

    this->setLayout(GLayout_1);


    //安装事件过滤器
    m_customPlot->installEventFilter(this);
    this->installEventFilter(this);


    //连接信号和槽
    connect(m_clearCustomPlot,&QPushButton::clicked,this,&LineFitWidget::slot_clearCustomPlotClick);
    connect(m_zoomIn,&QPushButton::clicked,this,&LineFitWidget::slot_zoomInClick);
    connect(m_zoomOut,&QPushButton::clicked,this,&LineFitWidget::slot_zoomOutClick);
    connect(m_origin,&QPushButton::clicked,this,&LineFitWidget::slot_originClick);
    connect(m_set,&QPushButton::clicked,this,&LineFitWidget::slot_setClick);
    connect(m_execute,&QPushButton::clicked,this,&LineFitWidget::slot_executeClick);
    connect(m_comboBox, QOverload<const QString &>::of(&QComboBox::activated),this,&LineFitWidget::slot_activated);





    //设置列数
    m_tableWidget->setColumnCount(2);
    //设置标题
    //表头标题用QStringList来表示
    QStringList headerText;
    headerText<<"X坐标"<<"Y坐标";
    m_tableWidget->setHorizontalHeaderLabels(headerText);

    //设置绘图界面可拖动的
    m_customPlot->setInteraction(QCP::iRangeDrag,true);
    //设置绘图界面可通过滑轮缩放 -- 有时会失灵
    //m_customPlot->setInteraction(QCP::iRangeZoom,true);


    QSharedPointer<QCPGraphDataContainer> globlePoint(new QCPGraphDataContainer);
    m_globlePoint = globlePoint;


    QButtonGroup *buttonGroup = new QButtonGroup(this);
    //互锁
    buttonGroup->addButton(m_OLS);
    buttonGroup->addButton(m_IRLS);
    buttonGroup->setExclusive(true);
    m_OLS->setChecked(true);

    m_fitCount->setMinimum(0);
    m_fitCount->setMaximum(9999);
    m_fitCount->setValue(0);

    m_iterationCount->setMinimum(0);
    m_iterationCount->setMaximum(99999999);
    m_iterationCount->setValue(999);

    m_normCount->setMinimum(0);
    m_normCount->setMaximum(999);
    m_normCount->setValue(1);

    m_customPlotOldSize = m_customPlot->size();
    m_customPlot->xAxis->setRange(0,customPlotSize);
    m_customPlot->yAxis->setRange(0,customPlotSize);

}


bool LineFitWidget::eventFilter(QObject *watched, QEvent *event)
{
    //主窗体大小发生改变
    if (watched == this && event->type() == QEvent::Resize)
    {
        /*获取坐标轴最低值
         * 获取窗口大小
         * 坐标轴最低值不发生改变，x轴最大值设置窗口宽度+x轴最低值，y轴最大值设置窗口宽度+y轴最低值
         */
        //获取当前坐标系显示最小值
        double xLower = m_customPlot->xAxis->range().lower;
        double yLower = m_customPlot->yAxis->range().lower;
        //获取当前坐标系显示最大值
        double xUpper = m_customPlot->xAxis->range().upper;
        double yUpper = m_customPlot->yAxis->range().upper;

        QSize newSize = m_customPlot->size();

        //计算变化前坐标刻度和像素的比例
        double xRatio = (xUpper-xLower) / m_customPlotOldSize.width();
        double yRatio = (yUpper-yLower) / m_customPlotOldSize.height();

        m_customPlot->xAxis->setRangeUpper((newSize.width() - m_customPlotOldSize.width()) * xRatio +xUpper);
        m_customPlot->yAxis->setRangeUpper((newSize.height() - m_customPlotOldSize.height())* yRatio+yUpper);

        m_customPlotOldSize = newSize;
        //重新绘制
        m_customPlot->replot();

    }
    //鼠标在坐标系中按下抬起的时候发生改变
    if (watched == m_customPlot && static_cast<QMouseEvent *>(event)->button() == Qt::RightButton  && event->type() == QEvent::MouseButtonRelease)
    {
        QPoint point = m_customPlot->mapFrom(this,mapFromGlobal(QCursor().pos()));
        double x_val = m_customPlot->xAxis->pixelToCoord(point.x());
        double y_val = m_customPlot->yAxis->pixelToCoord(point.y());

        QString str = QString("像素坐标:(%1,%2)   ---   转画板坐标:(%3,%4)\r\n").arg(point.x()).arg(point.y()).arg(x_val).arg(y_val);
        m_textEdit->append(str);
        m_globlePoint->add(QCPGraphData(x_val,y_val));
        //画点
        draw_PointF(m_globlePoint);

        //画圆
        draw_Line(m_globlePoint);

        //重新绘制
        m_customPlot->replot();

    }

    //鼠标滑轮在坐标系中滚动的时候发生改变
    if (watched == m_customPlot && event->type() == QEvent::Wheel )
    {
        QWheelEvent *wheel_event=static_cast<QWheelEvent *>(event);
        if(wheel_event->angleDelta().y()<0)
        {
            double dCenter = m_customPlot->xAxis->range().center();
            // (缩小 plottables 鼠标向内滚动)
            m_customPlot->xAxis->scaleRange(1.5,dCenter);
            m_customPlot->yAxis->scaleRange(1.5,dCenter);

        }
        else
        {
            double dCenter = m_customPlot->xAxis->range().center();
            // (放大 plotTables 鼠标向外滚动)
            m_customPlot->xAxis->scaleRange(0.5,dCenter);
            m_customPlot->yAxis->scaleRange(0.5,dCenter);

        }
        //重新绘制
        m_customPlot->replot();
    }

    return QWidget::eventFilter(watched, event);
}

//获取线参数
QList<double> LineFitWidget::get_Line(QList<QPointF> listP)
{
    /* y = a0 * x^0 + a1 * x^1  求未知系数a0和a1\n"
         * y = a0 * x^0 + a1 * x^1 + a2 * x^2  求未知系数a0和a1和a2\n"
         * y = a0 * x^0 + a1 * x^1 + a2 * x^2 + a3 * x^3  求未知系数a0和a1和a2和a3\n"
         *
         * 矩阵描述
         *  _               _ _   _     _ _
         * |1   x1  x1^2 ...| | a0 |   |y1 |
         * |1   x2  x2^2 ...| | a1 |  =|y2 |
         * |1   x3  x3^2 ...| | a2 |   |y3 |
         * |1   x4  x4^2 ...| | .. |   |y4 |
         * |....         ...|  - -     |...|
         *  -              -            - -
         *  Ax = B
         */

    QList<double> coeffL;
    //数据个数
    int rows = listP.count();
    int col = m_maxPower + 1;
    //m_maxPower
    Matrix<double,Dynamic,1> vector_x;

    //创建动态n行，3列矩阵
    Matrix<double,Dynamic,Dynamic> matA;
    matA.resize(rows,col);
    Matrix<double,Dynamic,1> matB;
    matB.resize(rows,1);

    //构建矩阵
    for(int i=0;i<rows;++i)
    {
        //A
        for(int j=0;j<col;++j)
        {
            matA(i,j) = qPow(listP.at(i).x(),j);
        }
        //B
        matB(i,0) = listP.at(i).y();
    }


    //勾选最小二乘
    if(m_OLS->isChecked())
    {
        vector_x = GlobleFunction::leastSquares(matA,matB);
    }
    //勾选迭代加权最小二乘
    else if(m_IRLS->isChecked())
    {
        int kk= m_iterationCount->value();
        double p = m_normCount->value();
        vector_x =GlobleFunction::iterativeReweightedLeastSquares(matA,matB,p,kk);
    }
    else
    {
        return QList<double>();
    }

    for(int i=0;i < col;++i)
    {
        coeffL.append(vector_x(i,0));
    }
    return coeffL;
}

//绘制点
void LineFitWidget::draw_PointF(QSharedPointer<QCPGraphDataContainer> & globlePoint)
{
    //画点
    m_customPlot->addGraph();
    m_customPlot->graph()->setPen(QPen(QBrush(Qt::red),6));
    //设置绘制点的数据
    m_customPlot->graph()->setData(globlePoint);

    //不显示连接线
    m_customPlot->graph()->setLineStyle((QCPGraph::LineStyle::lsNone));
    m_customPlot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));
}

//绘制线
void LineFitWidget::draw_Line(QSharedPointer<QCPGraphDataContainer> &globlePoint)
{
    if(m_maxPower == 0)
        return;
    //大于等于2个点画线
    if(globlePoint->size()>=2)
    {
        for(int i=0;i<m_itemLine.count();i++)
        {
            m_itemLine.at(i)->setPen(QPen(QBrush(Qt::green),2));
        }

        QList<QPointF> listP;
        //获取数据到列表，参数匹配
        for(int i=0;i<globlePoint->size();i++)
        {
            listP.append(QPointF(globlePoint->at(i)->key,globlePoint->at(i)->value));
        }


        //获取f(x)系数
        QList<double> config = get_Line(listP);

        //获取x最大值和最小值之间的区间大小
        QList<double> tempL;
        for(int i=0;i<listP.count();++i)
        {
            tempL.append(listP.at(i).x());
        }
        std::sort(tempL.begin(), tempL.end());
        double min = tempL.first()-1000;
        double max = tempL.last()+1000;
        double interval = max - min; //区间大小
        int drawCoordCount = 1000; //需要绘制坐标点的个数
        double step = interval / drawCoordCount;//步长

        QString str =  QString("多项式函数: f(x) = ");
        for(int i=0;i < config.count();++i)
        {
            str.append(QString("%1*x^%2 + ").arg(QString::number(config.at(i))).arg(QString::number(i)));
        }
        str.chop(2);//去掉最后"+ "两个字符
        m_textEdit->append(str+"\r\n");

        //绘制
        QCPCurve * qcpCurve=new  QCPCurve(m_customPlot->xAxis,m_customPlot->yAxis);
        qcpCurve->setPen(QPen(QBrush(Qt::blue),4));
        QVector<double> tV;
        QVector<double> keyV;
        QVector<double> valueV;
        for(int i=0;i<drawCoordCount;++i)
        {
            double x = min + i * step;
            double y =0;
            for(int i=0;i<config.count();++i)
            {
                y += config.at(i) * pow(x,i);
            }
            tV.append(i);
            keyV.append(x);
            valueV.append(y);
        }
        qcpCurve->setData(tV,keyV,valueV);
        m_itemLine.append(qcpCurve);
        //qcpitemellipse->setBrush(QBrush(QColor(0,255,255)));//填充圆的颜色

    }
}

void LineFitWidget::slot_clearCustomPlotClick()
{
    m_customPlot->axisRect()->setRangeZoomFactor(1,1);
    //清除 旧数据
    m_globlePoint->clear();
    m_customPlot->clearItems();
    m_customPlot->clearPlottables();
    m_itemLine.clear();
    m_customPlot->replot();
}
void LineFitWidget::slot_zoomInClick()
{
    double dCenter = m_customPlot->xAxis->range().center();
    // (放大 plotTables 鼠标向外滚动)
    m_customPlot->xAxis->scaleRange(0.5,dCenter);
    m_customPlot->yAxis->scaleRange(0.5,dCenter);
    m_customPlot->replot();
}
void LineFitWidget::slot_zoomOutClick()
{
    double dCenter = m_customPlot->xAxis->range().center();
    // (缩小 plottables 鼠标向内滚动)
    m_customPlot->xAxis->scaleRange(1.5,dCenter);
    m_customPlot->yAxis->scaleRange(1.5,dCenter);
    m_customPlot->replot();
}
void LineFitWidget::slot_originClick()
{
    QSize size = m_customPlot->size();
    m_customPlot->xAxis->setRange(0,size.width());
    m_customPlot->yAxis->setRange(0,size.height());
    m_customPlot->replot();
}
void LineFitWidget::slot_setClick()
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
            for (int j=0;j<2;j++) {
                m_tableWidget->setItem(i,j,new QTableWidgetItem("0"));
            }
        }
    }
}
void LineFitWidget::slot_executeClick()
{
    //获取界面坐标数据加入点坐标
    for(int i=0;i<m_tableWidget->rowCount();i++) {
        QCPGraphData GraphData(m_tableWidget->item(i,0)->text().toDouble(),m_tableWidget->item(i,1)->text().toDouble());

        m_globlePoint->add(GraphData);
    }
    //画点
    draw_PointF(m_globlePoint);

    //画圆
    draw_Line(m_globlePoint);
    m_customPlot->replot();
}
void LineFitWidget::slot_activated(const QString &text)
{
    if(text == MORE)
    {
        //自定义
        QString dlgTitle=QString("对话框");//对话框标题
        QString txtLabel=QString("请输入最高幂：");//对话框Label显示内容

        bool ok=false;
        int i = QInputDialog::getInt(this,dlgTitle,txtLabel,2, 0, 2147483647, 1, &ok,Qt::MSWindowsFixedSizeDialogHint | Qt::WindowCloseButtonHint);
        if(ok)
        {
            QString str = QString("添加【%1】次幂方程\r\n").arg(QString::number(i));
            m_textEdit->append(str);
            m_comboBox->addItem(QString::number(i));
        }
        m_maxPower = 0;

    }
    else
    {
        m_maxPower = text.toUInt();
        QString str = QString("切换到【%1】次幂方程\r\n").arg(m_maxPower);
        m_textEdit->append(str);
    }

}
