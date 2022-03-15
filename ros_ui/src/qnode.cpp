/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ros_ui/qnode.hpp"



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qdude {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
    {}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

double QNode::getAngle(float x1, float y1, float x2, float y2)
{
    float angle_temp;
    float xx,yy;
    xx = x2 - x1;
    yy = y2 - y1;
    if (xx == 0.0)
        angle_temp = M_PI / 2.0;
    else
        angle_temp = atan(fabs(yy / xx));
    if ((xx < 0.0) && (yy >= 0.0))
        angle_temp = M_PI - angle_temp;
    else if ((xx < 0.0) && (yy < 0.0))
        angle_temp = M_PI + angle_temp;
    else if ((xx >= 0.0) && (yy < 0.0))
        angle_temp = M_PI * 2.0 - angle_temp;
    return (angle_temp);
}

bool QNode::init(bool _flag) {
    ros::init(init_argc,init_argv,"qdude");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.

	// Add your ros communications here.
//	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    ros::NodeHandle n("~");
    n.param<std::string>("load_file_path",load_file_path,"/home/lxd");
    n.param<std::string>("mode",mode,"trace_detection");
    image_transport::ImageTransport it(n);
    
    if(_flag)
        cam_sub = it.subscribe("/cv_bridge_image",10,&QNode::imagecallback,this);
    else
        cam_sub = it.subscribe("/camera/rgb",10,&QNode::imagecallback,this);
    face_sub = n.subscribe("/face_detection_result",1,&QNode::face_flag_call,this);
    map_sub = n.subscribe("/map",10,&QNode::map_callback,this);
    
    
    target_pub = n.advertise<map_process::taregt_pose>("target_pose_execu",10,this);

    smart_control = n.advertise<std_msgs::Bool>("/smart_control_result",10,this);
    task_publisher = n.advertise<map_process::task>("task_execu",10,this);
    start();
	return true;
}

//bool QNode::init(const std::string &master_url, const std::string &host_url) {
//	std::map<std::string,std::string> remappings;
//	remappings["__master"] = master_url;
//	remappings["__hostname"] = host_url;
//	ros::init(remappings,"qdude");
//	if ( ! ros::master::check() ) {
//		return false;
//	}
//	ros::start(); // explicitly needed since our nodehandle is going out of scope.
//	// Add your ros communications here.
// //	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
//    ros::NodeHandle n("~");
//    image_transport::ImageTransport it(n);
//    n.param<std::string>("load_file_path",load_file_path,"/home/lxd");
//    cam_sub = it.subscribe("/camera/rgb/image_raw",10,&QNode::imagecallback,this);
//    map_sub = n.subscribe<nav_msgs::OccupancyGrid>("/map",10,&QNode::map_callback,this);
//	start();
//	return true;
//}

std::string QNode::update_file_path()
{
    return load_file_path;
}

void QNode::save_map()
{
    map_process::task msg;
    msg.task = "save_map";
    msg.map_name = "map";
    task_publisher.publish(msg);
}
void QNode::control_smart_car(bool tmp)
{
  std_msgs::Bool msg;
  msg.data = tmp;
  smart_control.publish(msg);
}

void QNode::target_pose_pub(map_process::taregt_pose& msg)
{
    target_pub.publish(msg);
}

void QNode::task_pub(map_process::task& msg)
{
    task_publisher.publish(msg);
}

cv::Mat QNode::RotaMap(cv::Mat const& map) {
  cv::Mat result;
  result.create(map.size(), map.type());
  int height = map.rows;
  int width = map.cols;
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      result.at<uchar>(height - i - 1, j) = map.at<uchar>(i, j);
    }
  }
  return result;
}





QImage QNode::Mat2QImage(cv::Mat const& src) {
  QImage dest(src.cols, src.rows, QImage::Format_ARGB32);

  const float scale = 255.0;

  if (src.depth() == CV_8U) {
    if (src.channels() == 1) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          int level = src.at<quint8>(i, j);
          dest.setPixel(j, i, qRgb(level, level, level));
        }
      }
    } else if (src.channels() == 3) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
          dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
        }
      }
    }
  } else if (src.depth() == CV_32F) {
    if (src.channels() == 1) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          int level = scale * src.at<float>(i, j);
          dest.setPixel(j, i, qRgb(level, level, level));
        }
      }
    } else if (src.channels() == 3) {
      for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
          cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
          dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
        }
      }
    }
  }

  return dest;
}

void QNode::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
      mapWidth = map_msg->info.width;
      mapHeight = map_msg->info.height;
      m_mapOriginX = map_msg->info.origin.position.x;
      m_mapOriginY = map_msg->info.origin.position.y;
      m_mapResolution = map_msg->info.resolution;
      int row, col, value;
      cv::Mat image(map_msg->info.height, map_msg->info.width, CV_8UC1);
      for (int array_index = 0; array_index < map_msg->data.size(); array_index++) {
        //计算当前所在行
        row = (int)array_index / image.cols;
        //计算当前所在列
        col = array_index % image.cols;
        //获取当前位置的像素值
        int curr_data = map_msg->data[array_index];
        //计算值
        if (curr_data == -1) {
          value = 125;  // grey
        } else if (curr_data == 100) {
          value = 0;  // black
        } else if (curr_data == 0) {
          value = 255;  // white
        } else {
          ROS_WARN("Unsupported value in Occupancy Grid");
          value = 125;
        }
        image.at<uchar>(row, col) = (uchar)value;
      }
      //沿x轴翻转地图
      // flip(image,rotaedMap,1);
      rotaedMap = RotaMap(image);
      cv::flip(rotaedMap,rotaedMap,-1);

      //计算map坐标系地图中心点坐标
      // scene(0,0) ^
      //           **********|************
      //           **********|************
      //           ----------o-map(0,0)---
      //           **********|************
      //           **********|************
      // origin(x,y)^
      //地图中心点map坐标系坐标

      QImage imageMap = Mat2QImage(rotaedMap);

      Q_EMIT updateMap(imageMap,m_mapResolution);

      m_mapCenterPoint.setX(m_mapOriginX + m_mapResolution * mapWidth * 0.5);
      m_mapCenterPoint.setY(m_mapOriginY + m_mapResolution * mapHeight * 0.5);
      //地图中心点图元坐标系坐标
      m_sceneCenterPoint.setX(mapWidth / 2.0);
      m_sceneCenterPoint.setY(mapHeight / 2.0) ;
}

void QNode::imagecallback(const sensor_msgs::ImageConstPtr& map_msg)
{
    QImage cam_image = Mat2QImage(cv_bridge::toCvShare(map_msg,"bgr8")->image);
    Q_EMIT updateCam(cam_image);
}

void QNode::face_flag_call(const std_msgs::BoolConstPtr& q_face_flag)
{
    face_flag = q_face_flag->data;
  //  ROS_INFO("%d",face_flag);
}

void QNode::run() {

    ros::Rate loop_rate(1);
    int count = 0;
    while ( ros::ok() ) {

//        std_msgs::String msg;
//        std::stringstream ss;
//        ss << "hello world " << count;
//        msg.data = ss.str();
//        chatter_publisher.publish(msg);
//        log(Info,std::string("I sent: ")+msg.data);
        ros::spinOnce();
//        loop_rate.sleep();
//        ++count;
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;

    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


//void QNode::log( const LogLevel &level, const std::string &msg) {
//	logging_model.insertRows(logging_model.rowCount(),1);
//	std::stringstream logging_model_msg;
//	switch ( level ) {
//		case(Debug) : {
//				ROS_DEBUG_STREAM(msg);
//				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
//				break;
//		}
//		case(Info) : {
//				ROS_INFO_STREAM(msg);
//				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
//				break;
//		}
//		case(Warn) : {
//				ROS_WARN_STREAM(msg);
//				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
//				break;
//		}
//		case(Error) : {
//				ROS_ERROR_STREAM(msg);
//				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
//				break;
//		}
//		case(Fatal) : {
//				ROS_FATAL_STREAM(msg);
//				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
//				break;
//		}
//	}
//	QVariant new_row(QString(logging_model_msg.str().c_str()));
//	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
//	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
//}

}  // namespace qdude
