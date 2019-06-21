/********************************************************************************
** Form generated from reading UI file 'Qt_gui.ui'
**
** Created by: Qt User Interface Compiler version 5.12.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QT_GUI_H
#define UI_QT_GUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_QtGuiClass
{
public:
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout_2;
    QFrame *frame;
    QVBoxLayout *verticalLayout_2;
    QFrame *frame_3;
    QVBoxLayout *verticalLayout;
    QPushButton *pb_simulation;
    QPushButton *pb_start;
    QFrame *frame_2;
    QHBoxLayout *horizontalLayout;
    QSlider *horizontalSlider;
    QLabel *label;
    QFrame *frame_4;
    QVBoxLayout *verticalLayout_3;
    QPushButton *pb_optimization;
    QPushButton *pb_force_opti_end;
    QFrame *frame_5;
    QVBoxLayout *verticalLayout_4;
    QPushButton *pb_setup_learning;
    QPushButton *pb_test_default;
    QPushButton *pb_run_learning;
    QTabWidget *tabWidget;

    void setupUi(QMainWindow *QtGuiClass)
    {
        if (QtGuiClass->objectName().isEmpty())
            QtGuiClass->setObjectName(QString::fromUtf8("QtGuiClass"));
        QtGuiClass->setEnabled(true);
        QtGuiClass->resize(200, 348);
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(QtGuiClass->sizePolicy().hasHeightForWidth());
        QtGuiClass->setSizePolicy(sizePolicy);
        QtGuiClass->setFocusPolicy(Qt::NoFocus);
        QtGuiClass->setAutoFillBackground(false);
        centralWidget = new QWidget(QtGuiClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        centralWidget->setFocusPolicy(Qt::NoFocus);
        horizontalLayout_2 = new QHBoxLayout(centralWidget);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        frame = new QFrame(centralWidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        QSizePolicy sizePolicy1(QSizePolicy::Maximum, QSizePolicy::Maximum);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy1);
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        verticalLayout_2 = new QVBoxLayout(frame);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        frame_3 = new QFrame(frame);
        frame_3->setObjectName(QString::fromUtf8("frame_3"));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(frame_3->sizePolicy().hasHeightForWidth());
        frame_3->setSizePolicy(sizePolicy2);
        frame_3->setMinimumSize(QSize(156, 40));
        frame_3->setMaximumSize(QSize(156, 16777215));
        frame_3->setFrameShape(QFrame::StyledPanel);
        frame_3->setFrameShadow(QFrame::Plain);
        verticalLayout = new QVBoxLayout(frame_3);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        pb_simulation = new QPushButton(frame_3);
        pb_simulation->setObjectName(QString::fromUtf8("pb_simulation"));
        pb_simulation->setEnabled(true);
        QSizePolicy sizePolicy3(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(pb_simulation->sizePolicy().hasHeightForWidth());
        pb_simulation->setSizePolicy(sizePolicy3);
        pb_simulation->setLayoutDirection(Qt::LeftToRight);

        verticalLayout->addWidget(pb_simulation, 0, Qt::AlignHCenter);

        pb_start = new QPushButton(frame_3);
        pb_start->setObjectName(QString::fromUtf8("pb_start"));
        pb_start->setEnabled(false);
        sizePolicy3.setHeightForWidth(pb_start->sizePolicy().hasHeightForWidth());
        pb_start->setSizePolicy(sizePolicy3);
        pb_start->setLayoutDirection(Qt::LeftToRight);

        verticalLayout->addWidget(pb_start, 0, Qt::AlignHCenter);

        frame_2 = new QFrame(frame_3);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        horizontalLayout = new QHBoxLayout(frame_2);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalSlider = new QSlider(frame_2);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setEnabled(false);
        sizePolicy3.setHeightForWidth(horizontalSlider->sizePolicy().hasHeightForWidth());
        horizontalSlider->setSizePolicy(sizePolicy3);
        horizontalSlider->setMinimum(-10);
        horizontalSlider->setMaximum(4);
        horizontalSlider->setSingleStep(1);
        horizontalSlider->setPageStep(10);
        horizontalSlider->setValue(0);
        horizontalSlider->setSliderPosition(0);
        horizontalSlider->setOrientation(Qt::Horizontal);
        horizontalSlider->setTickPosition(QSlider::TicksBelow);
        horizontalSlider->setTickInterval(1);

        horizontalLayout->addWidget(horizontalSlider);

        label = new QLabel(frame_2);
        label->setObjectName(QString::fromUtf8("label"));
        label->setEnabled(false);
        sizePolicy3.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy3);
        label->setText(QString::fromUtf8("Speed : 1"));

        horizontalLayout->addWidget(label);


        verticalLayout->addWidget(frame_2);


        verticalLayout_2->addWidget(frame_3);

        frame_4 = new QFrame(frame);
        frame_4->setObjectName(QString::fromUtf8("frame_4"));
        frame_4->setFrameShape(QFrame::StyledPanel);
        frame_4->setFrameShadow(QFrame::Plain);
        verticalLayout_3 = new QVBoxLayout(frame_4);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        pb_optimization = new QPushButton(frame_4);
        pb_optimization->setObjectName(QString::fromUtf8("pb_optimization"));
        sizePolicy3.setHeightForWidth(pb_optimization->sizePolicy().hasHeightForWidth());
        pb_optimization->setSizePolicy(sizePolicy3);
        pb_optimization->setLayoutDirection(Qt::LeftToRight);

        verticalLayout_3->addWidget(pb_optimization, 0, Qt::AlignHCenter);

        pb_force_opti_end = new QPushButton(frame_4);
        pb_force_opti_end->setObjectName(QString::fromUtf8("pb_force_opti_end"));
        pb_force_opti_end->setEnabled(false);
        sizePolicy3.setHeightForWidth(pb_force_opti_end->sizePolicy().hasHeightForWidth());
        pb_force_opti_end->setSizePolicy(sizePolicy3);
        pb_force_opti_end->setLayoutDirection(Qt::LeftToRight);

        verticalLayout_3->addWidget(pb_force_opti_end, 0, Qt::AlignHCenter);


        verticalLayout_2->addWidget(frame_4);

        frame_5 = new QFrame(frame);
        frame_5->setObjectName(QString::fromUtf8("frame_5"));
        frame_5->setFrameShape(QFrame::StyledPanel);
        frame_5->setFrameShadow(QFrame::Plain);
        verticalLayout_4 = new QVBoxLayout(frame_5);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        pb_setup_learning = new QPushButton(frame_5);
        pb_setup_learning->setObjectName(QString::fromUtf8("pb_setup_learning"));
        sizePolicy3.setHeightForWidth(pb_setup_learning->sizePolicy().hasHeightForWidth());
        pb_setup_learning->setSizePolicy(sizePolicy3);
        pb_setup_learning->setLayoutDirection(Qt::LeftToRight);

        verticalLayout_4->addWidget(pb_setup_learning, 0, Qt::AlignHCenter);

        pb_test_default = new QPushButton(frame_5);
        pb_test_default->setObjectName(QString::fromUtf8("pb_test_default"));
        pb_test_default->setEnabled(false);

        verticalLayout_4->addWidget(pb_test_default);

        pb_run_learning = new QPushButton(frame_5);
        pb_run_learning->setObjectName(QString::fromUtf8("pb_run_learning"));
        pb_run_learning->setEnabled(false);

        verticalLayout_4->addWidget(pb_run_learning);


        verticalLayout_2->addWidget(frame_5);


        horizontalLayout_2->addWidget(frame, 0, Qt::AlignVCenter);

        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setEnabled(false);
        QSizePolicy sizePolicy4(QSizePolicy::Ignored, QSizePolicy::Ignored);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(tabWidget->sizePolicy().hasHeightForWidth());
        tabWidget->setSizePolicy(sizePolicy4);

        horizontalLayout_2->addWidget(tabWidget);

        QtGuiClass->setCentralWidget(centralWidget);

        retranslateUi(QtGuiClass);
        QObject::connect(pb_simulation, SIGNAL(clicked()), QtGuiClass, SLOT(setup_simulation()));
        QObject::connect(pb_optimization, SIGNAL(clicked()), QtGuiClass, SLOT(setup_optimization()));
        QObject::connect(pb_force_opti_end, SIGNAL(clicked()), QtGuiClass, SLOT(stop_opti()));
        QObject::connect(horizontalSlider, SIGNAL(valueChanged(int)), QtGuiClass, SLOT(set_speed_value(int)));
        QObject::connect(pb_setup_learning, SIGNAL(clicked()), QtGuiClass, SLOT(setup_learning()));
        QObject::connect(pb_test_default, SIGNAL(clicked()), QtGuiClass, SLOT(test_default_config()));
        QObject::connect(pb_run_learning, SIGNAL(clicked()), QtGuiClass, SLOT(run_learning()));

        tabWidget->setCurrentIndex(-1);


        QMetaObject::connectSlotsByName(QtGuiClass);
    } // setupUi

    void retranslateUi(QMainWindow *QtGuiClass)
    {
        QtGuiClass->setWindowTitle(QApplication::translate("QtGuiClass", "QtGui", nullptr));
        pb_simulation->setText(QApplication::translate("QtGuiClass", "Setup Simulation", nullptr));
        pb_start->setText(QApplication::translate("QtGuiClass", "Start", nullptr));
        pb_optimization->setText(QApplication::translate("QtGuiClass", "Setup/Start Optimization", nullptr));
        pb_force_opti_end->setText(QApplication::translate("QtGuiClass", "Force Optimization end", nullptr));
        pb_setup_learning->setText(QApplication::translate("QtGuiClass", "Setup Learning framework", nullptr));
        pb_test_default->setText(QApplication::translate("QtGuiClass", "Test default config", nullptr));
        pb_run_learning->setText(QApplication::translate("QtGuiClass", "Run learning", nullptr));
    } // retranslateUi

};

namespace Ui {
    class QtGuiClass: public Ui_QtGuiClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QT_GUI_H
