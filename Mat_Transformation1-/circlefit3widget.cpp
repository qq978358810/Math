#include "circlefit3widget.h"


CircleFit3Widget::CircleFit3Widget(QWidget *parent) : QWidget(parent)
{
    m_pointx = new QLineEdit();
    m_pointy = new QLineEdit();
    m_pointx2 = new QLineEdit();
    m_pointy2 = new QLineEdit();
    m_pointx3 = new QLineEdit();
    m_pointy3 = new QLineEdit();
    //使用正则匹配正负整数、正负浮点数
    QString Pattern("(-?[1-9][0-9]+)|(-?[0-9])|(-?[1-9]\\d+\\.\\d+)|(-?[0-9]\\.\\d+)");
    QRegExp  reg(Pattern);
    QRegExpValidator *pReg = new QRegExpValidator(reg, this);
    m_pointx->setValidator(pReg);
    m_pointy->setValidator(pReg);
    m_pointx2->setValidator(pReg);
    m_pointy2->setValidator(pReg);
    m_pointx3->setValidator(pReg);
    m_pointy3->setValidator(pReg);

    m_execute = new QPushButton(QString("执行"));
    m_execute->setMinimumHeight(100);
    m_execute->setMinimumWidth(100);
    m_textEdit = new QTextEdit();

    QHBoxLayout * HLayout = new QHBoxLayout();
    HLayout->addWidget(new QLabel(QString("坐标x1：")));
    HLayout->addWidget(m_pointx);
    HLayout->addWidget(new QLabel(QString("坐标y1：")));
    HLayout->addWidget(m_pointy);

    QHBoxLayout * HLayout2 = new QHBoxLayout();
    HLayout2->addWidget(new QLabel(QString("坐标x2：")));
    HLayout2->addWidget(m_pointx2);
    HLayout2->addWidget(new QLabel(QString("坐标y2：")));
    HLayout2->addWidget(m_pointy2);

    QHBoxLayout * HLayout3 = new QHBoxLayout();
    HLayout3->addWidget(new QLabel(QString("坐标x3：")));
    HLayout3->addWidget(m_pointx3);
    HLayout3->addWidget(new QLabel(QString("坐标y3：")));
    HLayout3->addWidget(m_pointy3);

    QVBoxLayout* VLayout = new QVBoxLayout();
    VLayout->addLayout(HLayout);
    VLayout->addLayout(HLayout2);
    VLayout->addLayout(HLayout3);

    QGridLayout* GLayout = new QGridLayout();
    GLayout->addLayout(VLayout,0,0);
    GLayout->addWidget(m_execute,0,1);
    GLayout->addWidget(m_textEdit,1,0,-1,-1);

    this->setLayout(GLayout);

    connect(m_execute,&QPushButton::clicked,this,&CircleFit3Widget::slot_executeClicked);
}
//执行
void CircleFit3Widget::slot_executeClicked()
{
    double x1 = m_pointx->text().toDouble();
    double y1 = m_pointy->text().toDouble();
    double x2 = m_pointx2->text().toDouble();
    double y2 = m_pointy2->text().toDouble();
    double x3 = m_pointx3->text().toDouble();
    double y3 = m_pointy3->text().toDouble();

    QPointF coord1(x1,y1);
    QPointF coord2(x2,y2);
    QPointF coord3(x3,y3);

    //代数求解
    _arcConfig config  = GlobleAlgorithm::getInstance()->Get_ArcCenter3(coord1 ,coord2, coord3);
    m_textEdit->append(QString("代数推导: 圆心1坐标: (%1,%2)  --- 圆的半径: %3")
                           .arg(QString::number(config.center.x()).leftJustified(ALIGN_NUM,' ',true))
                           .arg(QString::number(config.center.y()).leftJustified(ALIGN_NUM,' ',true))
                           .arg(config.raio));

}
