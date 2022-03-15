/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include "../include/ros_ui/main_window.hpp"
#include "map_process/taregt_pose.h"
#include "map_process/task.h"
#include <ostream>
#include <dirent.h>
#include <ros/ros.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qdude {

using namespace Qt;
using namespace std;
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
    ,first_control_target_pub(false)
    ,image_control_flag(false)

{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
//    this->setWindowFlags(Qt::FramelessWindowHint);


    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
    setWindowTitle("smart car");
	setWindowIcon(QIcon(":/images/icon.png"));
    move(0,0);
    setFixedSize(960,540);
    // setWindowFlags(Qt::WindowTitleHint);
//	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this alre ady hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    QObject::connect(ui.get_target_pose,SIGNAL(clicked()),this,SLOT(get_target_pose()));    QObject::connect(ui.cancel_load_target_pose,SIGNAL(clicked()),this,SLOT(cancel_target_pose()));

    QObject::connect(ui.cancel_navigate,SIGNAL(clicked()),this,SLOT(cancel_navigation()));
    QObject::connect(ui.exit,SIGNAL(clicked()),this,SLOT(exit_login()));
	/*********************
	** Logging
	**********************/
//	ui.view_logging->setModel(qnode.loggingModel());
//    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    //map single
    QObject::connect(&qnode, SIGNAL(updateMap(QImage,float)), this, SLOT(paintMaps(QImage,float)));
    m_scalefactor=1;
    get_target_pose_flag = false;
    m_scene = new QGraphicsScene();
    m_imageItem = new QGraphicsPixmapItem();
    g_imageItem = new QGraphicsPixmapItem();
    m_scene->addItem(m_imageItem);

    ui.map_image->setScene(m_scene);
    ui.map_image->installEventFilter(this);
    ui.map_image->setMouseTracking(true);
    ui.map_image->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui.map_image->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui.map_image->setMaximumSize(640,480);
    ui.map_image->setMinimumSize(640,480);

    //cam_image single
    QObject::connect(&qnode, SIGNAL(updateCam(QImage)), this, SLOT(paintCam(QImage)));
    c_scene = new QGraphicsScene();
    c_imageItem = new QGraphicsPixmapItem();
    c_scene->addItem(c_imageItem);
    ui.cam_image->setScene(c_scene);
    ui.cam_image->installEventFilter(this);
    ui.cam_image->setMouseTracking(true);
    ui.cam_image->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui.cam_image->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui.cam_image->setMaximumSize(320,320);
    ui.cam_image->setMinimumSize(320,240);

    /*********************
    ** Auto Start
    **********************/
//    if ( ui.checkbox_remember_settings->isChecked() ) {
//        on_button_connect_clicked(true);
//    }

    while(!qnode.init(false))
        continue;
    common_file_path = qnode.update_file_path();
    load_map(common_file_path);
    load_target_file(common_file_path);
    connect(ui.target_pose_file_list_comboBox,SIGNAL(currentIndexChanged(int)),this, SLOT(target_list()));

    mode_state<<"navigate_mode"<<"mapping_mode"<<"charge_mode";
    ui.mode_state->addItems(mode_state);

    connect(ui.target_pose_list_comboBox,SIGNAL(currentIndexChanged(int)),this, SLOT(target_list_pub()));

    connect(ui.map_list_comboBox,SIGNAL(currentIndexChanged(int)),this, SLOT(map_load_pub()));

    connect(ui.radioButton_open,SIGNAL(clicked()),this,SLOT(image_control()));
    connect(ui.radioButton_close,SIGNAL(clicked()),this,SLOT(image_control()));
}

MainWindow::~MainWindow() {}

void MainWindow::exit_login()
{
    this->close();
}

void MainWindow::image_control()
{
    if(sender() == ui.radioButton_open)
            image_control_flag = true;
    else if(sender() == ui.radioButton_close)
            image_control_flag = false;
}

bool MainWindow::target_list()
{
   string tmp = string(common_file_path + "target_pose_file/" + ui.target_pose_file_list_comboBox->currentText().toStdString()+".csv");
   string s_pose,position_name;
   bool flag = true;
   ifstream infile(tmp,ios::in | ios::binary);
   if(!infile)
     {cerr <<"file open failed"<<endl; return FALSE;}
   move_base_goal.clear();
   QStringList target_pose_list;
   while(getline(infile,s_pose))
   {
     size_t s = s_pose.find_first_not_of(' ');
     size_t e = s_pose.find_last_not_of(' ');
     std::stringstream ss(s_pose.substr(s, e - s + 1));
     float data_pose[3];
     int i_pose = 0;
     while (ss.good())
     {
         if(flag)
         {
           ss >> position_name;
           flag = false;
         }
         ss >> data_pose[i_pose];
         i_pose++;
     }
     flag = true;
//     cout<<position_name.c_str() << " "<< data_pose[0] << " "<< data_pose[1] << " "<< data_pose[2]<<endl;
     vector<float> goal = {data_pose[0],data_pose[1],data_pose[2]};
     target_pose_list<<position_name.c_str();
     move_base_goal.insert(pair<string,vector<float>>(position_name,goal));
   }
   infile.close();
   ui.target_pose_list_comboBox->clear();
   ui.target_pose_list_comboBox->addItems(target_pose_list);
   last_target_pose_flag = ui.target_pose_file_list_comboBox->currentText().toStdString();
   return true;
}

bool MainWindow::cancel_navigation()
{
    // map_process::task msg;
    // msg.task = "cancel_navigation";
    // qnode.task_pub(msg);

    bool flag = false;
    qnode.control_smart_car(flag);
}

bool MainWindow::map_load_pub()
{
    map_process::task msg;
    msg.task = "load_map";
    msg.map_name = ui.map_list_comboBox->currentText().toStdString();
    qnode.task_pub(msg);
//    cout<<ui.map_list_comboBox->currentText().toStdString()<<endl;
}

bool MainWindow::target_list_pub()
{
    if(first_control_target_pub && !(ui.target_pose_list_comboBox->currentText().toStdString() == "") && last_target_pose_flag == ui.target_pose_file_list_comboBox->currentText().toStdString())
    {
        map_process::taregt_pose target_msg;

        const auto tmp_iterator = move_base_goal.find(ui.target_pose_list_comboBox->currentText().toStdString());
        if(tmp_iterator!= move_base_goal.end())
        {
            target_msg.map_name = ui.target_pose_list_comboBox->currentText().toStdString();
            target_msg.target_pose_x = tmp_iterator->second[0];
            target_msg.target_pose_y = tmp_iterator->second[1];
            target_msg.target_pose_theta = tmp_iterator->second[2];
        }
        qnode.target_pose_pub(target_msg);
     }
//        cout<<"test:"<<ui.target_pose_list_comboBox->currentText().toStdString()<<endl;
    first_control_target_pub = true;
    return true;
}

bool MainWindow::load_map(std::string file_path)
{
    std::string map_file_path = file_path + "map_file/";

    DIR * dir;
    struct dirent * ptr;
    std::vector<std::string> map_file_name;
    dir = opendir(map_file_path.c_str()); //打开一个目录
    while((ptr = readdir(dir)) != NULL) //循环读取目录数据
    {
        std::string s(ptr->d_name);
        if(s == "." || s == ".." || s.substr(s.length()-4) == ".pgm")
            continue;
        else
        {
//            cout<<s<<endl;
            map_file_name.push_back(s);
        }
    }
    closedir(dir);//关闭目录指针

    if(!map_file_name.empty())
    {
//        std::cout<<"there are "<<map_file_name.size()<<" map, map_name are respectively: ";
        for(auto &map_name : map_file_name)
        {
//            cout<<map_name.substr(0,map_name.length()-4)<<",";
            map_list << QString::fromStdString(map_name.substr(0,map_name.length()-5));
        }
    }
    else
        return FALSE;
    ui.map_list_comboBox->addItems(map_list);
    return TRUE;

}

bool MainWindow::load_target_file(std::string file_path)
{
    std::string target_file_path = file_path + "target_pose_file/";
    DIR * dir;
    struct dirent * ptr;
    std::vector<std::string> target_file_name;
    dir = opendir(target_file_path.c_str()); //打开一个目录
    while((ptr = readdir(dir)) != NULL) //循环读取目录数据
    {
        std::string s(ptr->d_name);
        if(s == "." || s == "..")
            continue;
        else
        {
//            cout<<s<<endl;
            target_file_name.push_back(s);
        }
    }
    closedir(dir);//关闭目录指针

    if(!target_file_name.empty())
    {
//        std::cout<<"there are "<<target_file_name.size()<<" map, map_name are respectively: ";
        for(auto &target_name : target_file_name)
        {
//            cout<<target_name.substr(0,target_name.length()-4)<<",";
            target_file_list << QString::fromStdString(target_name.substr(0,target_name.length()-4));
        }
    }
    else
        return FALSE;
    ui.target_pose_file_list_comboBox->addItems(target_file_list);
    return TRUE;
}
void MainWindow::get_target_pose()
{
    get_target_pose_flag = true;
}
void MainWindow::cancel_target_pose()
{
    get_target_pose_flag = false;
    qnode.save_map();
}

void MainWindow::paintCam(QImage map)
{
    if(image_control_flag)
        c_imageMap = map;
    else {
        c_imageMap.fill(QColor(Qt::white));
    }

    c_imageItem->setPixmap(QPixmap::fromImage(c_imageMap));
    ui.cam_image->show();
    QMatrix matrix;
    if (c_imageMap.height()>480||c_imageMap.width()>640)
//    matrix.scale(0.75,0.9);
    ui.cam_image->setMatrix(matrix);
    update();
}
void MainWindow::paintMaps(QImage map,float m_mapResolution) {
  m_imageMap = map;
  map_resolution = m_mapResolution;
  m_imageItem->setPixmap(QPixmap::fromImage(m_imageMap));
  g_imageItem->setPixmap(QPixmap(":/images/get_pos.png").scaled(20,20));

  ui.map_image->show();

  update();
}

void MainWindow::scaleImage(float factor)
{
    m_scalefactor *= factor;
    QMatrix matrix;
//    if(m_scalefactor > 2)
//        m_scalefactor = 2;
//    else if (m_scalefactor <= 0.2) {
//        m_scalefactor = 0.2;
//    }
    matrix.scale(m_scalefactor,m_scalefactor);
    ui.map_image->setMatrix(matrix);
}

void MainWindow::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    p_new = event->pos();
    cout<<p_new.x()<<","<<p_new.y()<<endl;
    update();
}

bool MainWindow::eventFilter(QObject* watched, QEvent* event)
{
    //如果信号不是来自于QGraphicsView，返回。
    if (watched != ui.map_image || !get_target_pose_flag)
        return false;

    switch (event->type()) {

        case QEvent::MouseButtonPress:
        {
            QMouseEvent* mEvent = (QMouseEvent*)event;
            if(mEvent->button() != Qt::LeftButton)
                break;
            QPoint p = mEvent->pos();
            QSize size = ui.map_image->viewport()->size();
            QPoint target_pix( p.x() - (size.width() - m_imageMap.width() )/2,p.y() - (size.height() - m_imageMap.height() )/2);
            if((target_pix.x()<m_imageMap.width()  && target_pix.x() > 0) && (target_pix.y()<m_imageMap.height() && target_pix.y()>0))
            {      
//                cout<<"111"<<endl;
                // double theta = qnode.getAngle(target_pix.x(),target_pix.y(),p_new.x(),p_new.y());
                // g_imageItem->setPos(target_pix.x(),target_pix.y());
                // cout<<p_new.x()<<","<<p_new.y()<<","<<theta<<endl;
                // g_imageItem->setRotation(theta);
                // m_scene->addItem(g_imageItem);

               auto target_x = -( target_pix.x() - m_imageMap.width()/ 2.0 ) * map_resolution;
               auto target_y = (target_pix.y() - m_imageMap.height()/ 2.0 ) * map_resolution;
               std::cout<<size.width()<<" "<<size.height()<<" "<<target_pix.x()
                   <<" "<<target_pix.y()<<" "<< target_x  <<" "<<target_y  <<std::endl;
            //    move_base(target_x,target_y);
            map_process::taregt_pose target_msg;
            target_msg.map_name = "";
            target_msg.target_pose_x = target_x;
            target_msg.target_pose_y = target_y;
            target_msg.target_pose_theta = 0;
            qnode.target_pose_pub(target_msg);


            }
            // target_pix = QPoint(0, 0);
            // p_new = QPointF(0, 0);
            break;
        }
        case QEvent::Wheel:
        {
            ui.map_image->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
            QWheelEvent* mEvent = (QWheelEvent*)event;
            QPoint p = mEvent->pos();
            QSize size = ui.map_image->viewport()->size();
            QPoint target_pix( p.x() - (size.width() - m_imageMap.width() )/2,p.y() - (size.height() - m_imageMap.height() )/2);
            if((target_pix.x()<m_imageMap.width()  && target_pix.x() > 0) && (target_pix.y()<m_imageMap.height() && target_pix.y()>0))
            {
                if((mEvent->delta()>0) && (m_scalefactor >= 2))
                    break;
                if((mEvent->delta()<0) && (m_scalefactor <= 0.8))
                    break;
                else {
                    int wheelDeltaValue = mEvent->delta();
                    if(wheelDeltaValue > 0)
                        scaleImage(1.1);
                    else {
                        scaleImage(0.9);
                    }
                }
            }
            break;
        }
        default:
            break;
    }
    return false;
}





/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}



/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

//void MainWindow::on_button_connect_clicked(bool check ) {
//	if ( ui.checkbox_use_environment->isChecked() ) {
//		if ( !qnode.init() ) {
//			showNoMasterMessage();
//		} else {
//			ui.button_connect->setEnabled(false);
//		}
//	} else {
//		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
//				   ui.line_edit_host->text().toStdString()) ) {
//			showNoMasterMessage();
//		} else {
//			ui.button_connect->setEnabled(false);
//			ui.line_edit_master->setReadOnly(true);
//			ui.line_edit_host->setReadOnly(true);
//		//	ui.line_edit_topic->setReadOnly(true);
//		}
//	}
//}


//void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
//	bool enabled;
//	if ( state == 0 ) {
//		enabled = true;
//	} else {
//		enabled = false;
//	}
//	ui.line_edit_master->setEnabled(enabled);
//	ui.line_edit_host->setEnabled(enabled);
//	ui.line_edit_topic->setEnabled(enabled);
//}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
//void MainWindow::updateLoggingView() {
//        ui.view_logging->scrollToBottom();
//}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "qdude");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.0.3:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.0.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
//    ui.line_edit_master->setText(master_url);
//    ui.line_edit_host->setText(host_url);
//    //ui.line_edit_topic->setText(topic_name);
//    bool remember = settings.value("remember_settings", false).toBool();
//    ui.checkbox_remember_settings->setChecked(remember);
//    bool checked = settings.value("use_environment_variables", false).toBool();
//    ui.checkbox_use_environment->setChecked(checked);
//    if ( checked ) {
//    	ui.line_edit_master->setEnabled(false);
//    	ui.line_edit_host->setEnabled(false);
//    	ui.line_edit_topic->setEnabled(false);
//    }
}

void MainWindow::WriteSettings() {
//    QSettings settings("Qt-Ros Package", "qdude");
//    settings.setValue("master_url",ui.line_edit_master->text());
//    settings.setValue("host_url",ui.line_edit_host->text());
//    //settings.setValue("topic_name",ui.line_edit_topic->text());
//    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
//    settings.setValue("geometry", saveGeometry());
//    settings.setValue("windowState", saveState());
//    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace qdude

