#ifndef MAINWINDOW2_H
#define MAINWINDOW2_H

#include <QMainWindow>
#include "ros_ui/main_window.hpp"
#include "ui_mainwindow2.h"
#include <QMessageBox>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
namespace qdude {


class MainWindow2 : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow2(int argc, char **argv,QWidget *parent = 0);
    ~MainWindow2();
    static void signalHandler(int signum);
public Q_SLOTS:
    void show_login();
    void show_passwd();
    void exit_login();
    void paintfaceCam(QImage map);
    void face_login();

private:
    Ui::MainWindow2 *ui;
    bool flag,face_flag;
    QGraphicsScene *face_scene;
    QGraphicsPixmapItem* face_imageItem;
    QImage face_imageMap;
    MainWindow w;
    QNode qnode;
};

}
#endif // MAINWINDOW2_H
