/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>
#include <photoview.h>
#include "singlephotoview.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actOpenFile;
    QAction *actSaveFile;
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QPushButton *btnPrev;
    QLabel *lblFilename;
    QPushButton *btnNext;
    QLineEdit *editCur;
    QLabel *lblTotal;
    QTabWidget *tabView;
    QWidget *tabSingleView;
    QGridLayout *gridLayout_3;
    SinglePhotoView *graphicsView;
    QFrame *frame;
    QGridLayout *gridLayout_2;
    QCheckBox *chkColor;
    QCheckBox *chkUse;
    QPushButton *btnSaveNext;
    QWidget *tabCmpView;
    QGridLayout *gridLayout_4;
    PhotoView *viewRef;
    PhotoView *viewEdit;
    QPushButton *btnCmpSaveNext;
    QMenuBar *menuBar;
    QMenu *menu;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(600, 400);
        actOpenFile = new QAction(MainWindow);
        actOpenFile->setObjectName(QString::fromUtf8("actOpenFile"));
        actSaveFile = new QAction(MainWindow);
        actSaveFile->setObjectName(QString::fromUtf8("actSaveFile"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        btnPrev = new QPushButton(centralWidget);
        btnPrev->setObjectName(QString::fromUtf8("btnPrev"));

        gridLayout->addWidget(btnPrev, 0, 0, 1, 1);

        lblFilename = new QLabel(centralWidget);
        lblFilename->setObjectName(QString::fromUtf8("lblFilename"));

        gridLayout->addWidget(lblFilename, 0, 1, 1, 1);

        btnNext = new QPushButton(centralWidget);
        btnNext->setObjectName(QString::fromUtf8("btnNext"));

        gridLayout->addWidget(btnNext, 0, 4, 1, 1);

        editCur = new QLineEdit(centralWidget);
        editCur->setObjectName(QString::fromUtf8("editCur"));

        gridLayout->addWidget(editCur, 0, 2, 1, 1);

        lblTotal = new QLabel(centralWidget);
        lblTotal->setObjectName(QString::fromUtf8("lblTotal"));

        gridLayout->addWidget(lblTotal, 0, 3, 1, 1);

        tabView = new QTabWidget(centralWidget);
        tabView->setObjectName(QString::fromUtf8("tabView"));
        tabSingleView = new QWidget();
        tabSingleView->setObjectName(QString::fromUtf8("tabSingleView"));
        gridLayout_3 = new QGridLayout(tabSingleView);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        graphicsView = new SinglePhotoView(tabSingleView);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));

        gridLayout_3->addWidget(graphicsView, 0, 0, 1, 1);

        frame = new QFrame(tabSingleView);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        gridLayout_2 = new QGridLayout(frame);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        chkColor = new QCheckBox(frame);
        chkColor->setObjectName(QString::fromUtf8("chkColor"));

        gridLayout_2->addWidget(chkColor, 0, 0, 1, 1);

        chkUse = new QCheckBox(frame);
        chkUse->setObjectName(QString::fromUtf8("chkUse"));

        gridLayout_2->addWidget(chkUse, 1, 0, 1, 1);

        btnSaveNext = new QPushButton(frame);
        btnSaveNext->setObjectName(QString::fromUtf8("btnSaveNext"));

        gridLayout_2->addWidget(btnSaveNext, 2, 0, 1, 1);


        gridLayout_3->addWidget(frame, 0, 1, 1, 1);

        tabView->addTab(tabSingleView, QString());
        tabCmpView = new QWidget();
        tabCmpView->setObjectName(QString::fromUtf8("tabCmpView"));
        gridLayout_4 = new QGridLayout(tabCmpView);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        viewRef = new PhotoView(tabCmpView);
        viewRef->setObjectName(QString::fromUtf8("viewRef"));

        gridLayout_4->addWidget(viewRef, 0, 0, 1, 1);

        viewEdit = new PhotoView(tabCmpView);
        viewEdit->setObjectName(QString::fromUtf8("viewEdit"));

        gridLayout_4->addWidget(viewEdit, 0, 1, 1, 1);

        btnCmpSaveNext = new QPushButton(tabCmpView);
        btnCmpSaveNext->setObjectName(QString::fromUtf8("btnCmpSaveNext"));

        gridLayout_4->addWidget(btnCmpSaveNext, 1, 1, 1, 1);

        tabView->addTab(tabCmpView, QString());

        gridLayout->addWidget(tabView, 1, 0, 1, 5);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 25));
        menu = new QMenu(menuBar);
        menu->setObjectName(QString::fromUtf8("menu"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menu->menuAction());
        menu->addAction(actOpenFile);
        menu->addAction(actSaveFile);

        retranslateUi(MainWindow);
        QObject::connect(viewRef, SIGNAL(selectedPointUpdated(int)), viewEdit, SLOT(setMirrorPoint(int)));
        QObject::connect(viewEdit, SIGNAL(selectedPointUpdated(int)), viewRef, SLOT(setMirrorPoint(int)));

        tabView->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        actOpenFile->setText(QApplication::translate("MainWindow", "\346\211\223\345\274\200", 0, QApplication::UnicodeUTF8));
        actSaveFile->setText(QApplication::translate("MainWindow", "Save", 0, QApplication::UnicodeUTF8));
        btnPrev->setText(QApplication::translate("MainWindow", "\344\270\212\344\270\200\344\270\252", 0, QApplication::UnicodeUTF8));
        lblFilename->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        btnNext->setText(QApplication::translate("MainWindow", "\344\270\213\344\270\200\344\270\252", 0, QApplication::UnicodeUTF8));
        lblTotal->setText(QApplication::translate("MainWindow", "TextLabel", 0, QApplication::UnicodeUTF8));
        chkColor->setText(QApplication::translate("MainWindow", "Color Image", 0, QApplication::UnicodeUTF8));
        chkUse->setText(QApplication::translate("MainWindow", "Use this PTS", 0, QApplication::UnicodeUTF8));
        btnSaveNext->setText(QApplication::translate("MainWindow", "Save and Next", 0, QApplication::UnicodeUTF8));
        tabView->setTabText(tabView->indexOf(tabSingleView), QApplication::translate("MainWindow", "Single", 0, QApplication::UnicodeUTF8));
        btnCmpSaveNext->setText(QApplication::translate("MainWindow", "Save and Next", 0, QApplication::UnicodeUTF8));
        tabView->setTabText(tabView->indexOf(tabCmpView), QApplication::translate("MainWindow", "Compare", 0, QApplication::UnicodeUTF8));
        menu->setTitle(QApplication::translate("MainWindow", "\346\226\207\344\273\266", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
