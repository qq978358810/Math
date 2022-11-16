#include "circlefitwidget.h"

CircleFitWidget::CircleFitWidget(QWidget *parent) : QWidget(parent)
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
    connect(m_clearCustomPlot,&QPushButton::clicked,this,&CircleFitWidget::slot_clearCustomPlotClick);
    connect(m_zoomIn,&QPushButton::clicked,this,&CircleFitWidget::slot_zoomInClick);
    connect(m_zoomOut,&QPushButton::clicked,this,&CircleFitWidget::slot_zoomOutClick);
    connect(m_origin,&QPushButton::clicked,this,&CircleFitWidget::slot_originClick);
    connect(m_set,&QPushButton::clicked,this,&CircleFitWidget::slot_setClick);
    connect(m_execute,&QPushButton::clicked,this,&CircleFitWidget::slot_executeClick);




    //设置列数
    m_tableWidget->setColumnCount(2);
    //设置标题
    //表头标题用QStringList来表示
    QStringList headerText;
    headerText<<"X坐标"<<"Y坐标";
    m_tableWidget->setHorizontalHeaderLabels(headerText);

    //设置可拖动的
    m_customPlot->setInteraction(QCP::iRangeDrag,true);

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


bool CircleFitWidget::eventFilter(QObject *watched, QEvent *event)
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
        draw_Ellipse(m_globlePoint);


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

//获取圆的中心和半径,多点拟合
_arcConfig CircleFitWidget::get_ArcCenter(QList<QPointF> listP)
{
    /* R² = (x-x0)²+(y-y0)²
         * x²+y²-ax-by+c=0
         * 矩阵描述：
         *  _      _   _ _     _       _
         * |x1 y1 -1| | a |   |x1²+y1²|
         * |x2 y2 -1| | b |  =|x2²+y2²|
         * |x3 y3 -1| | c |   |x3²+y3²|
         * |x4 y4 -1|  - -    |x4²+y4²|
         * |....    |         |...    |
         *  -      -           -       -
         *
         */
    _arcConfig config;

    int rows = listP.count();

    Matrix<double,3,1> abc_3_1;
    Matrix<double,Dynamic,1> matB;
    matB.resize(rows,1);
    //创建动态n行，3列矩阵
    Matrix<double,Dynamic,3> matA;
    matA.resize(rows,3);
    //构建矩阵
    for(int i=0;i<rows;++i)
    {
        matA(i,0) = listP.at(i).x();
        matA(i,1) = listP.at(i).y();
        matA(i,2) = -1.0;
        matB(i,0) = pow(listP.at(i).x(),2)+pow(listP.at(i).y(),2);

    }


    //勾选最小二乘
    if(m_OLS->isChecked())
    {
        abc_3_1 = GlobleFunction::leastSquares(matA,matB);
    }
    //勾选迭代加权最小二乘
    else if(m_IRLS->isChecked())
    {
        int kk= m_iterationCount->value();
        double p = m_normCount->value();
        abc_3_1 =GlobleFunction::iterativeReweightedLeastSquares(matA,matB,p,kk);
    }
    else
    {
        return _arcConfig();
    }

    //求半径R
    config.raio = 0.5 * sqrt(pow(abc_3_1(0,0),2)+pow(abc_3_1(1,0),2) - 4*abc_3_1(2,0));

    //求圆心
    config.center.setX(abc_3_1(0,0) / 2.0);
    config.center.setY(abc_3_1(1,0) / 2.0);


    qDebug()<<"半径:"<<config.raio;
    qDebug()<<"圆心坐标:"<<config.center;
    return config;

}

//绘制点
void CircleFitWidget::draw_PointF(QSharedPointer<QCPGraphDataContainer> & globlePoint)
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

//绘制圆
void CircleFitWidget::draw_Ellipse(QSharedPointer<QCPGraphDataContainer> &globlePoint)
{
    //大于等于3个点画圆
    if(globlePoint->size()>=3)
    {
        for(int i=0;i<m_itemEllipse.count();i++)
        {
            m_itemEllipse.at(i)->setPen(QPen(QBrush(Qt::green),2));
        }

        QList<QPointF> listP;
        //获取数据到列表，参数匹配
        for(int i=0;i<globlePoint->size();i++)
        {
            listP.append(QPointF(globlePoint->at(i)->key,globlePoint->at(i)->value));
        }
        //获取圆心和半径
        _arcConfig config = get_ArcCenter(listP);
        m_textEdit->append(QString("圆心坐标: (%1,%2) --- 圆的半径: %3\r\n")
                           .arg(QString::number(config.center.x()).leftJustified(ALIGN_NUM,' ',true))
                           .arg(QString::number(config.center.y()).leftJustified(ALIGN_NUM,' ',true))
                           .arg(config.raio));


        QCPItemEllipse*qcpitemellipse=new  QCPItemEllipse(m_customPlot);
        qcpitemellipse->setPen(QPen(QBrush(Qt::blue),4));
        qcpitemellipse->topLeft->setCoords(config.center.x() - config.raio,config.center.y() + config.raio);//圆左上角位置
        qcpitemellipse->bottomRight->setCoords(config.center.x() + config.raio,config.center.y() - config.raio);//圆右下角位置
        m_itemEllipse.append(qcpitemellipse);
        //qcpitemellipse->setBrush(QBrush(QColor(0,255,255)));//填充圆的颜色

    }
}

void CircleFitWidget::slot_clearCustomPlotClick()
{
    m_customPlot->axisRect()->setRangeZoomFactor(1,1);
    //清除 旧数据
    m_globlePoint->clear();
    m_customPlot->clearItems();
    m_customPlot->clearGraphs();
    m_itemEllipse.clear();
    m_customPlot->replot();
}
void CircleFitWidget::slot_zoomInClick()
{
    double dCenter = m_customPlot->xAxis->range().center();
    // (放大 plotTables 鼠标向外滚动)
    m_customPlot->xAxis->scaleRange(0.5,dCenter);
    m_customPlot->yAxis->scaleRange(0.5,dCenter);
    m_customPlot->replot();
}
void CircleFitWidget::slot_zoomOutClick()
{
    double dCenter = m_customPlot->xAxis->range().center();
    // (缩小 plottables 鼠标向内滚动)
    m_customPlot->xAxis->scaleRange(1.5,dCenter);
    m_customPlot->yAxis->scaleRange(1.5,dCenter);
    m_customPlot->replot();
}
void CircleFitWidget::slot_originClick()
{
    QSize size = m_customPlot->size();
    m_customPlot->xAxis->setRange(0,size.width());
    m_customPlot->yAxis->setRange(0,size.height());
    m_customPlot->replot();
}
void CircleFitWidget::slot_setClick()
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
void CircleFitWidget::slot_executeClick()
{
    //获取界面坐标数据加入点坐标
    for(int i=0;i<m_tableWidget->rowCount();i++) {
        QCPGraphData GraphData(m_tableWidget->item(i,0)->text().toDouble(),m_tableWidget->item(i,1)->text().toDouble());

        m_globlePoint->add(GraphData);
    }
    //画点
    draw_PointF(m_globlePoint);

    //画圆
    draw_Ellipse(m_globlePoint);
    m_customPlot->replot();
}
