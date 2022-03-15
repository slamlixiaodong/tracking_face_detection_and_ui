/**
 * @file /include/qdude/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qdude_QNODE_HPP_
#define qdude_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include "map_process/task.h"
#include "map_process/taregt_pose.h"
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Bool.h"
#include <opencv2/opencv.hpp>



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qdude {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    std::string mode;
    QNode(int argc, char** argv );
    virtual ~QNode();
	bool init(bool _flag);
	bool init(const std::string &master_url, const std::string &host_url);
    void run();
    double getAngle(float x1, float y1, float x2, float y2);
	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
    void imagecallback(const sensor_msgs::ImageConstPtr& map_msg);
    void face_flag_call(const std_msgs::BoolConstPtr& q_face_flag);
    cv::Mat RotaMap(cv::Mat const& map);
    QImage Mat2QImage(cv::Mat const& src);
    void control_smart_car(bool tmp);
    struct QPointF{
        int x;
        int y;
        void setX(int x_)
        {
            x = x_;
        }
        void setY(int y_)
        {
            y = y_;
        }
    };
    std::string update_file_path();
    void target_pose_pub(map_process::taregt_pose& msg);
    void task_pub(map_process::task& msg);
    void save_map();
    bool face_flag=false;
Q_SIGNALS:
//	void loggingUpdated();
    void rosShutdown();
    void updateMap(QImage map, float m_mapResolution);
    void updateCam(QImage cam_image);

private:
	int init_argc;
	char** init_argv;
//	ros::Publisher chatter_publisher;
    ros::Subscriber map_sub,face_sub;
    QPointF m_mapCenterPoint,m_sceneCenterPoint;
    double mapWidth,mapHeight,m_mapOriginX,m_mapOriginY,m_mapResolution;
    QStringListModel logging_model;
    cv::Mat rotaedMap;
    image_transport::Subscriber cam_sub;
    std::string load_file_path;
    ros::Publisher target_pub,task_publisher,smart_control;
};

}  // namespace qdude

#endif /* qdude_QNODE_HPP_ */
