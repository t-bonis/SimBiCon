/********************************************************************************
** Form generated from reading UI file 'Qt_gui.ui'
**
** Created by: Qt User Interface Compiler version 5.11.3
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
    QPushButton *pb_simulation;
    QPushButton *pb_optimization;
    QPushButton *pb_force_opti_end;
    QPushButton *pb_start_learning;
    QPushButton *pb_start;
    QFrame *frame_2;
    QHBoxLayout *horizontalLayout;
    QSlider *horizontalSlider;
    QLabel *label;
    QTabWidget *tabWidget;

    void setupUi(QMainWindow *QtGuiClass)
    {
        if (QtGuiClass->objectName().isEmpty())
            QtGuiClass->setObjectName(QStringLiteral("QtGuiClass"));
        QtGuiClass->setEnabled(true);
        QtGuiClass->resize(200, 455);
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(QtGuiClass->sizePolicy().hasHeightForWidth());
        QtGuiClass->setSizePolicy(sizePolicy);
        QtGuiClass->setFocusPolicy(Qt::NoFocus);
        QtGuiClass->setAutoFillBackground(false);
        centralWidget = new QWidget(QtGuiClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        centralWidget->setFocusPolicy(Qt::NoFocus);
        horizontalLayout_2 = new QHBoxLayout(centralWidget);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        frame = new QFrame(centralWidget);
        frame->setObjectName(QStringLiteral("frame"));
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
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        pb_simulation = new QPushButton(frame);
        pb_simulation->setObjectName(QStringLiteral("pb_simulation"));
        pb_simulation->setEnabled(true);
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(pb_simulation->sizePolicy().hasHeightForWidth());
        pb_simulation->setSizePolicy(sizePolicy2);
        pb_simulation->setLayoutDirection(Qt::LeftToRight);

        verticalLayout_2->addWidget(pb_simulation, 0, Qt::AlignHCenter);

        pb_optimization = new QPushButton(frame);
        pb_optimization->setObjectName(QStringLiteral("pb_optimization"));
        sizePolicy2.setHeightForWidth(pb_optimization->sizePolicy().hasHeightForWidth());
        pb_optimization->setSizePolicy(sizePolicy2);
        pb_optimization->setLayoutDirection(Qt::LeftToRight);

        verticalLayout_2->addWidget(pb_optimization, 0, Qt::AlignHCenter);

        pb_force_opti_end = new QPushButton(frame);
        pb_force_opti_end->setObjectName(QStringLiteral("pb_force_opti_end"));
        sizePolicy2.setHeightForWidth(pb_force_opti_end->sizePolicy().hasHeightForWidth());
        pb_force_opti_end->setSizePolicy(sizePolicy2);
        pb_force_opti_end->setLayoutDirection(Qt::LeftToRight);

        verticalLayout_2->addWidget(pb_force_opti_end, 0, Qt::AlignHCenter);

        pb_start_learning = new QPushButton(frame);
        pb_start_learning->setObjectName(QStringLiteral("pb_start_learning"));
        sizePolicy2.setHeightForWidth(pb_start_learning->sizePolicy().hasHeightForWidth());
        pb_start_learning->setSizePolicy(sizePolicy2);
        pb_start_learning->setLayoutDirection(Qt::LeftToRight);

        verticalLayout_2->addWidget(pb_start_learning, 0, Qt::AlignHCenter);

        pb_start = new QPushButton(frame);
        pb_start->setObjectName(QStringLiteral("pb_start"));
        sizePolicy2.setHeightForWidth(pb_start->sizePolicy().hasHeightForWidth());
        pb_start->setSizePolicy(sizePolicy2);
        pb_start->setLayoutDirection(Qt::LeftToRight);

        verticalLayout_2->addWidget(pb_start, 0, Qt::AlignHCenter);

        frame_2 = new QFrame(frame);
        frame_2->setObjectName(QStringLiteral("frame_2"));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        horizontalLayout = new QHBoxLayout(frame_2);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalSlider = new QSlider(frame_2);
        horizontalSlider->setObjectName(QStringLiteral("horizontalSlider"));
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
        label->setObjectName(QStringLiteral("label"));
        label->setText(QStringLiteral("Speed : 1"));

        horizontalLayout->addWidget(label);


        verticalLayout_2->addWidget(frame_2, 0, Qt::AlignHCenter);


        horizontalLayout_2->addWidget(frame, 0, Qt::AlignVCenter);

        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setEnabled(false);
        QSizePolicy sizePolicy3(QSizePolicy::Ignored, QSizePolicy::Ignored);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(tabWidget->sizePolicy().hasHeightForWidth());
        tabWidget->setSizePolicy(sizePolicy3);

        horizontalLayout_2->addWidget(tabWidget);

        QtGuiClass->setCentralWidget(centralWidget);

        retranslateUi(QtGuiClass);
        QObject::connect(pb_simulation, SIGNAL(clicked()), QtGuiClass, SLOT(setup_simulation()));
        QObject::connect(pb_optimization, SIGNAL(clicked()), QtGuiClass, SLOT(setup_optimization()));
        QObject::connect(pb_start_learning, SIGNAL(clicked()), QtGuiClass, SLOT(start_learning()));
        QObject::connect(pb_force_opti_end, SIGNAL(clicked()), QtGuiClass, SLOT(stop_opti()));
        QObject::connect(horizontalSlider, SIGNAL(valueChanged(int)), QtGuiClass, SLOT(set_speed_value(int)));
        QObject::connect(horizontalSlider, SIGNAL(valueChanged(int)), QtGuiClass, SLOT(set_speed_value(int)));

        tabWidget->setCurrentIndex(-1);


        QMetaObject::connectSlotsByName(QtGuiClass);
    } // setupUi

    void retranslateUi(QMainWindow *QtGuiClass)
    {
        QtGuiClass->setWindowTitle(QApplication::translate("QtGuiClass", "QtGui", nullptr));
        pb_simulation->setText(QApplication::translate("QtGuiClass", "Setup Simulation", nullptr));
        pb_optimization->setText(QApplication::translate("QtGuiClass", "Setup/Start Optimization", nullptr));
        pb_force_opti_end->setText(QApplication::translate("QtGuiClass", "Force Optimization end", nullptr));
        pb_start_learning->setText(QApplication::translate("QtGuiClass", "Start Learning", nullptr));
        pb_start->setText(QApplication::translate("QtGuiClass", "Start", nullptr));
    } // retranslateUi

};

namespace Ui {
    class QtGuiClass: public Ui_QtGuiClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QT_GUI_H
