#include "mywidget.h"

myWidget::myWidget(QWidget *parent) : QWidget(parent)
{

}
void myWidget::paintEvent(QPaintEvent *)
{

    QPainter painter(ui->widget);

    //画点
    for(int i=0;i<m_globlePoint.count();i++)
    {
        painter.setPen(QPen(QBrush(Qt::red),5));
        painter.drawPoint(m_globlePoint.at(i));
    }

    //大于画圆
    if(m_globlePoint.count()>2)
    {
        painter.setPen(QPen(QBrush(Qt::green),3));
        _arcConfig config = get_ArcCenter(m_globlePoint);
        painter.drawEllipse(config.center,config.raio,config.raio); //画大圆
    }

}
