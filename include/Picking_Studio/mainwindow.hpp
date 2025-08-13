#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include "ElaWindow.h"
#include <QMutex>
#include <QTimer>
#include <ros/ros.h>
#include <std_msgs/String.h>

class MainWindow : public ElaWindow {
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
  void initWindow();
  void initContent();
  void initTimer();
  //拖动重写
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  // Resize重写
  void mouseReleaseEvent(QMouseEvent *event) override;
  void leaveEvent(QEvent *event) override;
  //窗口是否置顶重写
  void setIsStayTop(bool stayOnTop);

private:
  enum class ResizeRegion {
    None,
    Left,
    Right,
    Top,
    Bottom,
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight
  };
  ResizeRegion hitTest(const QPoint &pos) const;
  void updateCursorByRegion(ResizeRegion r);

private:
  //重写窗口拖动用到的变量
  bool m_isDragging;
  QPoint m_startPos;
  //重写窗口大小调整用到的变量
  bool m_isResizing{false};
  ResizeRegion m_resizeRegion{ResizeRegion::None};
  QPoint m_pressGlobalPos;
  QRect m_pressGeom;
};

#endif // MAINWINDOW_HPP