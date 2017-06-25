/********************************************************************************
** Form generated from reading UI file 'meshviewerwidget.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MESHVIEWERWIDGET_H
#define UI_MESHVIEWERWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MeshViewerWidgetClass
{
public:

    void setupUi(QWidget *MeshViewerWidgetClass)
    {
        if (MeshViewerWidgetClass->objectName().isEmpty())
            MeshViewerWidgetClass->setObjectName(QStringLiteral("MeshViewerWidgetClass"));
        MeshViewerWidgetClass->resize(600, 400);

        retranslateUi(MeshViewerWidgetClass);

        QMetaObject::connectSlotsByName(MeshViewerWidgetClass);
    } // setupUi

    void retranslateUi(QWidget *MeshViewerWidgetClass)
    {
        MeshViewerWidgetClass->setWindowTitle(QApplication::translate("MeshViewerWidgetClass", "MeshViewerWidget", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MeshViewerWidgetClass: public Ui_MeshViewerWidgetClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MESHVIEWERWIDGET_H
