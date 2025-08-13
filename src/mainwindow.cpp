#include "Picking_Studio/mainwindow.hpp"
#include <QGraphicsView>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QTimer>

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
#include "ElaWindow.h"

MainWindow::MainWindow(QWidget *parent)
    : ElaWindow(parent), m_isDragging(false) {
  initWindow();
  initContent();
  initTimer();
}

MainWindow::~MainWindow() {}

void MainWindow::initWindow() {
  setWindowTitle("Picking Studio");
  setIsStayTop(false);
  // setFixedSize(1200, 720);
  setMinimumSize(640, 400); // 允许缩放的最小尺寸
  setUserInfoCardVisible(false);
  setAttribute(Qt::WA_Hover, true);
}

void MainWindow::initContent() {
  // Initialize your UI components here
}

void MainWindow::initTimer() {
  // Initialize your timer here
}

void MainWindow::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    // 命中测试：优先进入缩放，其次才是拖动窗口
    m_resizeRegion = hitTest(event->pos());
    if (m_resizeRegion != ResizeRegion::None) {
      m_isResizing = true;
      m_pressGlobalPos = event->globalPos();
      m_pressGeom = geometry();
      event->accept();
      return;
    }

    // 非边缘则执行原拖动窗口逻辑
    m_isDragging = true;
    m_startPos = event->globalPos() - frameGeometry().topLeft();
    event->accept();
  }
}

void MainWindow::mouseMoveEvent(QMouseEvent *event) {
  if (m_isResizing && (event->buttons() & Qt::LeftButton)) {
    const QPoint delta = event->globalPos() - m_pressGlobalPos;

    QRect newRect = m_pressGeom;
    const int minW = minimumWidth();
    const int minH = minimumHeight();

    auto applyLeft = [&](int dx) {
      int right = m_pressGeom.right();
      int newLeft = m_pressGeom.left() + dx;
      int maxLeft = right - minW;
      if (newLeft > maxLeft)
        newLeft = maxLeft;
      newRect.setLeft(newLeft);
    };
    auto applyRight = [&](int dx) {
      int w = std::max(minW, m_pressGeom.width() + dx);
      newRect.setWidth(w);
    };
    auto applyTop = [&](int dy) {
      int bottom = m_pressGeom.bottom();
      int newTop = m_pressGeom.top() + dy;
      int maxTop = bottom - minH;
      if (newTop > maxTop)
        newTop = maxTop;
      newRect.setTop(newTop);
    };
    auto applyBottom = [&](int dy) {
      int h = std::max(minH, m_pressGeom.height() + dy);
      newRect.setHeight(h);
    };

    switch (m_resizeRegion) {
    case ResizeRegion::Left:
      applyLeft(delta.x());
      break;
    case ResizeRegion::Right:
      applyRight(delta.x());
      break;
    case ResizeRegion::Top:
      applyTop(delta.y());
      break;
    case ResizeRegion::Bottom:
      applyBottom(delta.y());
      break;
    case ResizeRegion::TopLeft:
      applyLeft(delta.x());
      applyTop(delta.y());
      break;
    case ResizeRegion::TopRight:
      applyRight(delta.x());
      applyTop(delta.y());
      break;
    case ResizeRegion::BottomLeft:
      applyLeft(delta.x());
      applyBottom(delta.y());
      break;
    case ResizeRegion::BottomRight:
      applyRight(delta.x());
      applyBottom(delta.y());
      break;
    case ResizeRegion::None:
      break;
    }

    setGeometry(newRect);
    event->accept();
    return;
  }

  if (m_isDragging && (event->buttons() & Qt::LeftButton)) {
    move(event->globalPos() - m_startPos);
    event->accept();
    return;
  }

  // 未按下：根据命中区域更新光标
  updateCursorByRegion(hitTest(event->pos()));
}

void MainWindow::setIsStayTop(bool stayOnTop) {
  Qt::WindowFlags flags = windowFlags();
  if (stayOnTop) {
    flags |= Qt::WindowStaysOnTopHint;
  } else {
    flags &= ~Qt::WindowStaysOnTopHint;
  }
  setWindowFlags(flags);
  show(); // 重新显示窗口以应用新的flags
}

static constexpr int kResizeMargin = 8;

MainWindow::ResizeRegion MainWindow::hitTest(const QPoint &pos) const {
  const QRect r = rect();
  const bool left = pos.x() <= kResizeMargin;
  const bool right = pos.x() >= r.width() - kResizeMargin;
  const bool top = pos.y() <= kResizeMargin;
  const bool bottom = pos.y() >= r.height() - kResizeMargin;

  if (top && left)
    return ResizeRegion::TopLeft;
  if (top && right)
    return ResizeRegion::TopRight;
  if (bottom && left)
    return ResizeRegion::BottomLeft;
  if (bottom && right)
    return ResizeRegion::BottomRight;
  if (left)
    return ResizeRegion::Left;
  if (right)
    return ResizeRegion::Right;
  if (top)
    return ResizeRegion::Top;
  if (bottom)
    return ResizeRegion::Bottom;
  return ResizeRegion::None;
}

void MainWindow::updateCursorByRegion(ResizeRegion r) {
  switch (r) {
  case ResizeRegion::Top:
  case ResizeRegion::Bottom:
    setCursor(Qt::SizeVerCursor);
    break;
  case ResizeRegion::Left:
  case ResizeRegion::Right:
    setCursor(Qt::SizeHorCursor);
    break;
  case ResizeRegion::TopLeft:
  case ResizeRegion::BottomRight:
    setCursor(Qt::SizeFDiagCursor);
    break;
  case ResizeRegion::TopRight:
  case ResizeRegion::BottomLeft:
    setCursor(Qt::SizeBDiagCursor);
    break;
  case ResizeRegion::None:
  default:
    unsetCursor();
    break;
  }
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    m_isDragging = false;
    if (m_isResizing) {
      m_isResizing = false;
    }
    updateCursorByRegion(hitTest(event->pos()));
    event->accept();
  }
}

void MainWindow::leaveEvent(QEvent *event) {
  if (!m_isDragging && !m_isResizing) {
    unsetCursor();
  }
  QWidget::leaveEvent(event);
}