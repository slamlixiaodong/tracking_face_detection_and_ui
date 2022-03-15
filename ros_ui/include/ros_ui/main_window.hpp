/**
 * @file /include/qdude/main_window.hpp
 *
 * @brief Qt based gui for qdude.
 *
 * @date November 2010
 **/
#ifndef qdude_MAIN_WINDOW_H
#include <iostream>
#include <vector>
#include <map>
#define qdude_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QPushButton>
#include <QImage>
#include <QLabel>
#include <QObject>
#include <QGraphicsItem>
#include <QGraphicsSceneWheelEvent>
#include <QMainWindow>
#include <QComboBox>



/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qdude {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    void scaleImage(float factor);
    bool load_map(std::string file_path);
    bool load_target_file(std::string file_path);
    void target_pose_pub(map_process::taregt_pose& msg);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
//	void on_button_connect_clicked(bool check );
//	void on_checkbox_use_environment_stateChanged(int state);
    bool eventFilter(QObject* watched, QEvent* event);
    /******************************************
    ** Manual connections
    *******************************************/
//    void updateLoggingView(); // no idea why this can't connect automatically

    void paintMaps(QImage map,float m_mapResolution);
    void paintCam(QImage map);

    void get_target_pose();
    void cancel_target_pose();
    bool target_list();
    bool target_list_pub();
    bool map_load_pub();
    bool cancel_navigation();
    void exit_login();
    void image_control();

private:
	Ui::MainWindowDesign ui;
    QNode qnode;
    QImage m_imageMap,c_imageMap;
    QGraphicsPixmapItem* m_imageItem;
    QGraphicsPixmapItem* g_imageItem;
    QGraphicsPixmapItem* c_imageItem;
    QGraphicsScene *m_scene;
    QGraphicsScene *c_scene;
    double m_scalefactor;
    float map_resolution;
    bool get_target_pose_flag;
    std::string common_file_path;
    QComboBox *comboBox;
    QStringList map_list,target_file_list,mode_state;
    std::map<std::string,std::vector<float>> move_base_goal;
    bool first_control_target_pub, image_control_flag;
    std::string last_target_pose_flag;
    QPointF p_new=QPointF(0,0);
};

}  // namespace qdude

#endif // qdude_MAIN_WINDOW_H
