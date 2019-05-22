/********************************************************************************
** Form generated from reading UI file 'Viewer_widget_form.ui'
**
** Created by: Qt User Interface Compiler version 5.11.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VIEWER_WIDGET_FORM_H
#define UI_VIEWER_WIDGET_FORM_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Viewer_widget_form
{
public:
    QHBoxLayout *horizontalLayout;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout;
    QGridLayout *gridLayout;

    void setupUi(QWidget *Viewer_widget_form)
    {
        if (Viewer_widget_form->objectName().isEmpty())
            Viewer_widget_form->setObjectName(QStringLiteral("Viewer_widget_form"));
        Viewer_widget_form->resize(833, 561);
        horizontalLayout = new QHBoxLayout(Viewer_widget_form);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        scrollArea = new QScrollArea(Viewer_widget_form);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 813, 541));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(scrollAreaWidgetContents->sizePolicy().hasHeightForWidth());
        scrollAreaWidgetContents->setSizePolicy(sizePolicy);
        verticalLayout = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QStringLiteral("gridLayout"));

        verticalLayout->addLayout(gridLayout);

        scrollArea->setWidget(scrollAreaWidgetContents);

        horizontalLayout->addWidget(scrollArea);


        retranslateUi(Viewer_widget_form);

        QMetaObject::connectSlotsByName(Viewer_widget_form);
    } // setupUi

    void retranslateUi(QWidget *Viewer_widget_form)
    {
        Viewer_widget_form->setWindowTitle(QApplication::translate("Viewer_widget_form", "Form", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Viewer_widget_form: public Ui_Viewer_widget_form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VIEWER_WIDGET_FORM_H
