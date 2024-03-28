/********************************************************************************
** Form generated from reading UI file 'curvedwidget.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CURVEDWIDGET_H
#define UI_CURVEDWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <imageview.h>

QT_BEGIN_NAMESPACE

class Ui_CurvedWidget
{
public:
    QGridLayout *gridLayout_3;
    QGridLayout *gridLayout_2;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_22;
    QSpinBox *spinBox_19;
    QLabel *label_23;
    QSpinBox *spinBox_20;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QLabel *label_18;
    QLabel *label_19;
    QGridLayout *gridLayout;
    QSpinBox *spinBox_16;
    QSpinBox *spinBox_9;
    QSpinBox *spinBox_2;
    QSpinBox *spinBox;
    QSpinBox *spinBox_8;
    QLabel *label_6;
    QSpinBox *spinBox_3;
    QLabel *label_8;
    QLabel *label_7;
    QSpinBox *spinBox_13;
    QLabel *label_14;
    QLabel *label_11;
    QLabel *label_9;
    QLabel *label_13;
    QLabel *label_4;
    QLabel *label_10;
    QSpinBox *spinBox_7;
    QLabel *label_15;
    QSpinBox *spinBox_6;
    QSpinBox *spinBox_4;
    QLabel *label_5;
    QLabel *label_12;
    QSpinBox *spinBox_14;
    QLabel *label_2;
    QLabel *label_16;
    QSpinBox *spinBox_10;
    QSpinBox *spinBox_12;
    QSpinBox *spinBox_15;
    QSpinBox *spinBox_11;
    QLabel *label_3;
    QLabel *label_17;
    QSpinBox *spinBox_5;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *pushButton;
    QLabel *label;
    QDoubleSpinBox *doubleSpinBox;
    QCheckBox *checkBox_2;
    QCheckBox *checkBox;
    QPushButton *pushButton_2;
    QSpacerItem *horizontalSpacer;
    QHBoxLayout *horizontalLayout_2;
    ImageView *widget;
    ImageView *widget_2;

    void setupUi(QWidget *CurvedWidget)
    {
        if (CurvedWidget->objectName().isEmpty())
            CurvedWidget->setObjectName(QString::fromUtf8("CurvedWidget"));
        CurvedWidget->resize(799, 556);
        gridLayout_3 = new QGridLayout(CurvedWidget);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_22 = new QLabel(CurvedWidget);
        label_22->setObjectName(QString::fromUtf8("label_22"));

        horizontalLayout_5->addWidget(label_22);

        spinBox_19 = new QSpinBox(CurvedWidget);
        spinBox_19->setObjectName(QString::fromUtf8("spinBox_19"));
        spinBox_19->setMaximum(99999999);

        horizontalLayout_5->addWidget(spinBox_19);

        label_23 = new QLabel(CurvedWidget);
        label_23->setObjectName(QString::fromUtf8("label_23"));

        horizontalLayout_5->addWidget(label_23);

        spinBox_20 = new QSpinBox(CurvedWidget);
        spinBox_20->setObjectName(QString::fromUtf8("spinBox_20"));
        spinBox_20->setMaximum(99999999);

        horizontalLayout_5->addWidget(spinBox_20);


        gridLayout_2->addLayout(horizontalLayout_5, 1, 0, 1, 1);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setSizeConstraint(QLayout::SetFixedSize);
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label_18 = new QLabel(CurvedWidget);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        verticalLayout->addWidget(label_18);

        label_19 = new QLabel(CurvedWidget);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        verticalLayout->addWidget(label_19);


        horizontalLayout->addLayout(verticalLayout);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        spinBox_16 = new QSpinBox(CurvedWidget);
        spinBox_16->setObjectName(QString::fromUtf8("spinBox_16"));
        spinBox_16->setMaximum(99999999);
        spinBox_16->setSingleStep(1);

        gridLayout->addWidget(spinBox_16, 3, 7, 1, 1);

        spinBox_9 = new QSpinBox(CurvedWidget);
        spinBox_9->setObjectName(QString::fromUtf8("spinBox_9"));
        spinBox_9->setMaximum(99999999);

        gridLayout->addWidget(spinBox_9, 0, 5, 1, 1);

        spinBox_2 = new QSpinBox(CurvedWidget);
        spinBox_2->setObjectName(QString::fromUtf8("spinBox_2"));
        spinBox_2->setMaximum(99999999);

        gridLayout->addWidget(spinBox_2, 1, 1, 1, 1);

        spinBox = new QSpinBox(CurvedWidget);
        spinBox->setObjectName(QString::fromUtf8("spinBox"));
        spinBox->setMaximum(99999999);

        gridLayout->addWidget(spinBox, 0, 1, 1, 1);

        spinBox_8 = new QSpinBox(CurvedWidget);
        spinBox_8->setObjectName(QString::fromUtf8("spinBox_8"));
        spinBox_8->setMaximum(99999999);

        gridLayout->addWidget(spinBox_8, 3, 3, 1, 1);

        label_6 = new QLabel(CurvedWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout->addWidget(label_6, 0, 4, 1, 1);

        spinBox_3 = new QSpinBox(CurvedWidget);
        spinBox_3->setObjectName(QString::fromUtf8("spinBox_3"));
        spinBox_3->setMaximum(99999999);

        gridLayout->addWidget(spinBox_3, 2, 1, 1, 1);

        label_8 = new QLabel(CurvedWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        gridLayout->addWidget(label_8, 0, 6, 1, 1);

        label_7 = new QLabel(CurvedWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout->addWidget(label_7, 1, 4, 1, 1);

        spinBox_13 = new QSpinBox(CurvedWidget);
        spinBox_13->setObjectName(QString::fromUtf8("spinBox_13"));
        spinBox_13->setMaximum(99999999);

        gridLayout->addWidget(spinBox_13, 0, 7, 1, 1);

        label_14 = new QLabel(CurvedWidget);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        gridLayout->addWidget(label_14, 2, 4, 1, 1);

        label_11 = new QLabel(CurvedWidget);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout->addWidget(label_11, 3, 0, 1, 1);

        label_9 = new QLabel(CurvedWidget);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout->addWidget(label_9, 1, 6, 1, 1);

        label_13 = new QLabel(CurvedWidget);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        gridLayout->addWidget(label_13, 3, 2, 1, 1);

        label_4 = new QLabel(CurvedWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout->addWidget(label_4, 0, 2, 1, 1);

        label_10 = new QLabel(CurvedWidget);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout->addWidget(label_10, 2, 0, 1, 1);

        spinBox_7 = new QSpinBox(CurvedWidget);
        spinBox_7->setObjectName(QString::fromUtf8("spinBox_7"));
        spinBox_7->setMaximum(99999999);

        gridLayout->addWidget(spinBox_7, 2, 3, 1, 1);

        label_15 = new QLabel(CurvedWidget);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        gridLayout->addWidget(label_15, 3, 4, 1, 1);

        spinBox_6 = new QSpinBox(CurvedWidget);
        spinBox_6->setObjectName(QString::fromUtf8("spinBox_6"));
        spinBox_6->setMaximum(99999999);

        gridLayout->addWidget(spinBox_6, 1, 3, 1, 1);

        spinBox_4 = new QSpinBox(CurvedWidget);
        spinBox_4->setObjectName(QString::fromUtf8("spinBox_4"));
        spinBox_4->setMaximum(99999999);

        gridLayout->addWidget(spinBox_4, 3, 1, 1, 1);

        label_5 = new QLabel(CurvedWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout->addWidget(label_5, 1, 2, 1, 1);

        label_12 = new QLabel(CurvedWidget);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        gridLayout->addWidget(label_12, 2, 2, 1, 1);

        spinBox_14 = new QSpinBox(CurvedWidget);
        spinBox_14->setObjectName(QString::fromUtf8("spinBox_14"));
        spinBox_14->setMaximum(99999999);

        gridLayout->addWidget(spinBox_14, 1, 7, 1, 1);

        label_2 = new QLabel(CurvedWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 0, 0, 1, 1);

        label_16 = new QLabel(CurvedWidget);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        gridLayout->addWidget(label_16, 2, 6, 1, 1);

        spinBox_10 = new QSpinBox(CurvedWidget);
        spinBox_10->setObjectName(QString::fromUtf8("spinBox_10"));
        spinBox_10->setMaximum(99999999);

        gridLayout->addWidget(spinBox_10, 1, 5, 1, 1);

        spinBox_12 = new QSpinBox(CurvedWidget);
        spinBox_12->setObjectName(QString::fromUtf8("spinBox_12"));
        spinBox_12->setMaximum(99999999);

        gridLayout->addWidget(spinBox_12, 3, 5, 1, 1);

        spinBox_15 = new QSpinBox(CurvedWidget);
        spinBox_15->setObjectName(QString::fromUtf8("spinBox_15"));
        spinBox_15->setMaximum(99999999);

        gridLayout->addWidget(spinBox_15, 2, 7, 1, 1);

        spinBox_11 = new QSpinBox(CurvedWidget);
        spinBox_11->setObjectName(QString::fromUtf8("spinBox_11"));
        spinBox_11->setMaximum(99999999);

        gridLayout->addWidget(spinBox_11, 2, 5, 1, 1);

        label_3 = new QLabel(CurvedWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 1, 0, 1, 1);

        label_17 = new QLabel(CurvedWidget);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        gridLayout->addWidget(label_17, 3, 6, 1, 1);

        spinBox_5 = new QSpinBox(CurvedWidget);
        spinBox_5->setObjectName(QString::fromUtf8("spinBox_5"));
        spinBox_5->setMaximum(99999999);

        gridLayout->addWidget(spinBox_5, 0, 3, 1, 1);


        horizontalLayout->addLayout(gridLayout);


        gridLayout_2->addLayout(horizontalLayout, 2, 0, 1, 1);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        pushButton = new QPushButton(CurvedWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        horizontalLayout_3->addWidget(pushButton);

        label = new QLabel(CurvedWidget);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_3->addWidget(label);

        doubleSpinBox = new QDoubleSpinBox(CurvedWidget);
        doubleSpinBox->setObjectName(QString::fromUtf8("doubleSpinBox"));
        doubleSpinBox->setMaximum(1.000000000000000);
        doubleSpinBox->setSingleStep(0.100000000000000);

        horizontalLayout_3->addWidget(doubleSpinBox);

        checkBox_2 = new QCheckBox(CurvedWidget);
        checkBox_2->setObjectName(QString::fromUtf8("checkBox_2"));

        horizontalLayout_3->addWidget(checkBox_2);

        checkBox = new QCheckBox(CurvedWidget);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));

        horizontalLayout_3->addWidget(checkBox);

        pushButton_2 = new QPushButton(CurvedWidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        horizontalLayout_3->addWidget(pushButton_2);


        gridLayout_2->addLayout(horizontalLayout_3, 0, 0, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer, 2, 1, 1, 1);


        gridLayout_3->addLayout(gridLayout_2, 0, 0, 1, 1);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        widget = new ImageView(CurvedWidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(widget->sizePolicy().hasHeightForWidth());
        widget->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(widget);

        widget_2 = new ImageView(CurvedWidget);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        sizePolicy.setHeightForWidth(widget_2->sizePolicy().hasHeightForWidth());
        widget_2->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(widget_2);


        gridLayout_3->addLayout(horizontalLayout_2, 1, 0, 1, 1);


        retranslateUi(CurvedWidget);

        QMetaObject::connectSlotsByName(CurvedWidget);
    } // setupUi

    void retranslateUi(QWidget *CurvedWidget)
    {
        CurvedWidget->setWindowTitle(QCoreApplication::translate("CurvedWidget", "Form", nullptr));
        label_22->setText(QCoreApplication::translate("CurvedWidget", "\350\276\223\345\207\272\345\233\276\345\203\217\351\253\230:", nullptr));
        label_23->setText(QCoreApplication::translate("CurvedWidget", "\350\276\223\345\207\272\345\233\276\345\203\217\345\256\275:", nullptr));
        label_18->setText(QCoreApplication::translate("CurvedWidget", "\350\276\223\345\205\245\347\202\271\357\274\232", nullptr));
        label_19->setText(QCoreApplication::translate("CurvedWidget", "\350\276\223\345\207\272\347\202\271\357\274\232", nullptr));
        label_6->setText(QCoreApplication::translate("CurvedWidget", "x3:", nullptr));
        label_8->setText(QCoreApplication::translate("CurvedWidget", "x4:", nullptr));
        label_7->setText(QCoreApplication::translate("CurvedWidget", "y3:", nullptr));
        label_14->setText(QCoreApplication::translate("CurvedWidget", "x3:", nullptr));
        label_11->setText(QCoreApplication::translate("CurvedWidget", "y1:", nullptr));
        label_9->setText(QCoreApplication::translate("CurvedWidget", "y4:", nullptr));
        label_13->setText(QCoreApplication::translate("CurvedWidget", "y2:", nullptr));
        label_4->setText(QCoreApplication::translate("CurvedWidget", "x2:", nullptr));
        label_10->setText(QCoreApplication::translate("CurvedWidget", "x1:", nullptr));
        label_15->setText(QCoreApplication::translate("CurvedWidget", "y3:", nullptr));
        label_5->setText(QCoreApplication::translate("CurvedWidget", "y2:", nullptr));
        label_12->setText(QCoreApplication::translate("CurvedWidget", "x2:", nullptr));
        label_2->setText(QCoreApplication::translate("CurvedWidget", "x1:", nullptr));
        label_16->setText(QCoreApplication::translate("CurvedWidget", "x4:", nullptr));
        label_3->setText(QCoreApplication::translate("CurvedWidget", "y1:", nullptr));
        label_17->setText(QCoreApplication::translate("CurvedWidget", "y4:", nullptr));
        pushButton->setText(QCoreApplication::translate("CurvedWidget", "\345\212\240\350\275\275\345\233\276\345\203\217", nullptr));
        label->setText(QCoreApplication::translate("CurvedWidget", "\347\274\251\346\224\276:", nullptr));
        checkBox_2->setText(QCoreApplication::translate("CurvedWidget", "\345\272\224\347\224\250\351\200\217\350\247\206\345\217\230\346\215\242", nullptr));
        checkBox->setText(QCoreApplication::translate("CurvedWidget", "\345\211\252\345\210\207", nullptr));
        pushButton_2->setText(QCoreApplication::translate("CurvedWidget", "\346\211\247\350\241\214", nullptr));
    } // retranslateUi

};

namespace Ui {
    class CurvedWidget: public Ui_CurvedWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CURVEDWIDGET_H
