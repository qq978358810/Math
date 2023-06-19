#ifndef VIEWTRANSFORM_H
#define VIEWTRANSFORM_H

#include <QtWidgets>
#include <QPushButton>
#include <QStandardPaths>
#include <QFileDialog>
#include "globlealgorithm.h"


class ImageViewer : public QWidget
{
public:
    ImageViewer(QWidget *parent = nullptr)
        : QWidget(parent), pixmap("image.jpg")
    {
        setFixedSize(pixmap.size());
    }

protected:
    void mousePressEvent(QMouseEvent *event) override {
        poly << event->pos();
        update();
    }

    void mouseMoveEvent(QMouseEvent *event) override {
        poly.replace(poly.size() - 1, event->pos());
        update();
    }

    void mouseReleaseEvent(QMouseEvent *event) override {
        poly.replace(poly.size() - 1, event->pos());
        update();
    }

    void paintEvent(QPaintEvent *event) override {
        QPainter painter(this);
        painter.drawPixmap(0, 0, pixmap);
        painter.setPen(QPen(Qt::green, 3));
        painter.setBrush(QBrush(Qt::green, Qt::Dense4Pattern));
        painter.drawPolygon(poly);
    }

    void keyPressEvent(QKeyEvent *event) override {
        if (event->key() == Qt::Key_Enter || event->key() == Qt::Key_Return) {
            captureImage();
        }
    }

private:
    void captureImage() {
        QImage image = pixmap.toImage();
        QImage croppedImage = image.copy(poly.boundingRect().toRect());
        croppedImage.save("croppedImage.png");
    }

private:
    QPixmap pixmap;
    QPolygonF poly;
};
class ViewTransform : public QWidget
{
    Q_OBJECT
public:
    explicit ViewTransform(QWidget *parent = nullptr);

signals:

private:
    QPushButton *m_src;
    QPushButton *m_dst;

    ImageViewer *m_srcView;
    ImageViewer *m_dstView;

};

#endif // VIEWTRANSFORM_H
