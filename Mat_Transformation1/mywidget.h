#ifndef MYWIDGET_H
#define MYWIDGET_H

#include <QWidget>

class myWidget : public QWidget
{
    Q_OBJECT
public:
    explicit myWidget(QWidget *parent = nullptr);

    void paintEvent(QPaintEvent *);
signals:

};

#endif // MYWIDGET_H
