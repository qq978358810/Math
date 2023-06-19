#ifndef IMAGECORRECTIONWIDGET_H
#define IMAGECORRECTIONWIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QDebug>
#include <QTextEdit>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QRegExpValidator>
#include <QtMath>
#include <QFileDialog>
#include <QImage>
#include <QDateTime>
#include <QStandardPaths>
#include "globlealgorithm.h"

#define  ALIGN_NUM 10
class ImageCorrectionWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ImageCorrectionWidget(QWidget *parent = nullptr);

    //修复图片
    QImage RectifiedImage(const QImage& srcImage,const Eigen::Matrix3d& intrinsicParameter,const Eigen::VectorXd& distortionCoeff);

signals:

protected slots:
    void slot_executeClicked();
private:
    QLineEdit *m_alpha;//(0,0)
    QLineEdit *m_gamma;//(0,1)
    QLineEdit *m_beta;//(1,1)
    QLineEdit *m_u0;//(0,2)
    QLineEdit *m_v0;//(1,2)
    QLineEdit *m_0;//0
    QLineEdit *m_00;//0
    QLineEdit *m_000;//0
    QLineEdit *m_1;//1

    QLineEdit *m_k1;//(0)
    QLineEdit *m_k2;//(1)
    QLineEdit *m_p1;//(2)
    QLineEdit *m_p2;//(3)
    QLineEdit *m_k3;//(4)

    QPushButton *m_src;
    QPushButton *m_dst;
    QPushButton *m_execute;
    QTextEdit *m_inputTextEdit;
    QTextEdit *m_outTextEdit;

    QStringList m_files;//文件
    QString m_savePath;//保存文件路径
};

#endif // IMAGECORRECTIONWIDGET_H
