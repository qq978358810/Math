#include "circlefit2widget.h"

CircleFit2Widget::CircleFit2Widget(QWidget *parent) : QWidget(parent)
{
    m_pointx = new QLineEdit();
    m_pointy = new QLineEdit();
    m_pointx2 = new QLineEdit();
    m_pointy2 = new QLineEdit();
    m_angle = new QLineEdit();
    //使用正则匹配正负整数、正负浮点数
    QString Pattern("(-?[1-9][0-9]+)|(-?[0-9])|(-?[1-9]\\d+\\.\\d+)|(-?[0-9]\\.\\d+)");
    QRegExp  reg(Pattern);
    QRegExpValidator *pReg = new QRegExpValidator(reg, this);
    m_pointx->setValidator(pReg);
    m_pointy->setValidator(pReg);
    m_pointx2->setValidator(pReg);
    m_pointy2->setValidator(pReg);
    m_angle->setValidator(pReg);

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
    HLayout3->addWidget(new QLabel(QString("角度(θ)：")));
    HLayout3->addWidget(m_angle);

    QVBoxLayout* VLayout = new QVBoxLayout();
    VLayout->addLayout(HLayout);
    VLayout->addLayout(HLayout2);
    VLayout->addLayout(HLayout3);

    QGridLayout* GLayout = new QGridLayout();
    GLayout->addLayout(VLayout,0,0);
    GLayout->addWidget(m_execute,0,1);
    GLayout->addWidget(m_textEdit,1,0,-1,-1);

    this->setLayout(GLayout);

    connect(m_execute,&QPushButton::clicked,this,&CircleFit2Widget::slot_executeClicked);
}
//执行
void CircleFit2Widget::slot_executeClicked()
{
    double x1 = m_pointx->text().toDouble();
    double y1 = m_pointy->text().toDouble();
    double x2 = m_pointx2->text().toDouble();
    double y2 = m_pointy2->text().toDouble();
    double angle = m_angle->text().toDouble();
    QPointF coord1(x1,y1);
    QPointF coord2(x2,y2);
    double rad = angle * M_PI / 180;
    //代数求解
    _arcConfig config  = GlobleFunction::get_ArcCenter2(coord1 ,coord2, rad);
    m_textEdit->append(QString("代数推导: 圆心1坐标: (%1,%2) --- 圆心2坐标: (%3,%4) --- 圆的半径: %5")
                           .arg(QString::number(config.center.x()).leftJustified(ALIGN_NUM,' ',true))
                           .arg(QString::number(config.center.y()).leftJustified(ALIGN_NUM,' ',true))
                           .arg(QString::number(config.center2.x()).leftJustified(ALIGN_NUM,' ',true))
                           .arg(QString::number(config.center2.y()).leftJustified(ALIGN_NUM,' ',true))
                           .arg(config.raio));
    //几何求解
    _arcConfig config1  =GlobleFunction::get_ArcCenter3(coord1 ,coord2, rad);
    m_textEdit->append(QString("几何推导: 圆心1坐标: (%1,%2) --- 圆心2坐标: (%3,%4) --- 圆的半径: %5\r\n")
                           .arg(QString::number(config1.center.x()).leftJustified(ALIGN_NUM,' ',true))
                           .arg(QString::number(config1.center.y()).leftJustified(ALIGN_NUM,' ',true))
                           .arg(QString::number(config1.center2.x()).leftJustified(ALIGN_NUM,' ',true))
                           .arg(QString::number(config1.center2.y()).leftJustified(ALIGN_NUM,' ',true))
                           .arg(config1.raio));
}
