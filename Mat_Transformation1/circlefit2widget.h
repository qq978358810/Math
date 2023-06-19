#ifndef CIRCLEFIT2WIDGET_H
#define CIRCLEFIT2WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QDebug>
#include <QTextEdit>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QRegExpValidator>
#include <QtMath>
#include "globlealgorithm.h"

#define  ALIGN_NUM 10
class CircleFit2Widget : public QWidget
{
    Q_OBJECT
public:
    explicit CircleFit2Widget(QWidget *parent = nullptr);

signals:

protected slots:
    void slot_executeClicked();
private:
    QLineEdit *m_pointx;//x1
    QLineEdit *m_pointy;//y1
    QLineEdit *m_pointx2;//x2
    QLineEdit *m_pointy2;//y2
    QLineEdit *m_angle;//角度
    QPushButton *m_execute;
    QTextEdit *m_textEdit;

};

#endif // CIRCLEFIT2WIDGET_H
