#ifndef CURVEDWIDGET_H
#define CURVEDWIDGET_H
#include <QWidget>
#include <QtWidgets>

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QPixmap>
#include <QPainter>
#include <QTransform>
#include <cmath>



#include "hyperboloidmapping.h"
#include "imageview.h"
namespace Ui {
class CurvedWidget;
}

class CurvedWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CurvedWidget(QWidget *parent = nullptr);
    ~CurvedWidget();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::CurvedWidget *ui;
    QImage m_Image;
};

#endif // CURVEDWIDGET_H
