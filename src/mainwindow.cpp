#include "Picking_Studio/mainwindow.hpp"
#include <QMouseEvent>
#include <QGraphicsView>
#include <QHBoxLayout>
#include <QTimer>

#include "ElaWindow.h"
#include "ElaContentDialog.h"
#include "ElaDockWidget.h"
#include "ElaEventBus.h"
#include "ElaLog.h"
#include "ElaMenu.h"
#include "ElaMenuBar.h"
#include "ElaProgressBar.h"
#include "ElaStatusBar.h"
#include "ElaText.h"
#include "ElaTheme.h"
#include "ElaToolBar.h"
#include "ElaToolButton.h"

MainWindow::MainWindow(QWidget *parent)
    :ElaWindow(parent), m_isDragging(false)
{
    initWindow();
    initContent();
    initTimer();
}

MainWindow::~MainWindow()
{
}

void MainWindow::initWindow()
{
    setWindowTitle("Picking Studio");
    setIsStayTop(false);
    setFixedSize(1200, 720);
    setUserInfoCardVisible(false);
    setAttribute(Qt::WA_Hover, true);
}

void MainWindow::initContent()
{
    // Initialize your UI components here
}

void MainWindow::initTimer()
{
    // Initialize your timer here
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        m_isDragging = true;
        m_startPos = event->pos();
    }
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
    if (m_isDragging) {
        QPoint delta = event->pos() - m_startPos;
        // Handle the dragging motion
    }
}

void MainWindow::setIsStayTop(bool stayOnTop) {
    Qt::WindowFlags flags = windowFlags();
    if (stayOnTop) {
        flags |= Qt::WindowStaysOnTopHint;
    } else {
        flags &= ~Qt::WindowStaysOnTopHint;
    }
    setWindowFlags(flags);
    show();  // 重新显示窗口以应用新的flags
}