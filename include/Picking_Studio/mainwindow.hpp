#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include "ElaWindow.h"
#include <QTimer>
#include <QMutex>
#include <ros/ros.h>
#include <std_msgs/String.h>

class MainWindow : public ElaWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void initWindow();
    void initContent();
    void initTimer();
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void setIsStayTop(bool stayOnTop);  

private slots:


private:
    bool m_isDragging;
    QPoint m_startPos;
};

#endif // MAINWINDOW_HPP