#include "ros_ui/mainwindow2.hpp"
#include "csignal"
namespace qdude {
    using namespace Qt;
    using namespace std;
    MainWindow2::MainWindow2(int argc, char **argv,QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow2)
      ,w(argc,argv,parent)
      ,flag(true)
      ,face_flag(false)
      ,qnode(argc,argv)
    {
        ui->setupUi(this);
        qnode.init(true);
        this->setStyleSheet("#MainWindow2{border-image:url(:/images/background.gif)}");
        setWindowIcon(QIcon(":/images/icon.png"));
        setFixedSize(640,480);
        setWindowTitle("smart car");
        // setWindowFlags(Qt::WindowTitleHint);
        QObject::connect(&qnode, SIGNAL(updateCam(QImage)), this, SLOT(paintfaceCam(QImage)));
        face_scene = new QGraphicsScene();
        face_imageItem = new QGraphicsPixmapItem();
        face_scene->addItem(face_imageItem);
        ui->face_show->setScene(face_scene);
        ui->face_show->installEventFilter(this);
        ui->face_show->setMouseTracking(true);
        ui->face_show->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        ui->face_show->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);


        ui->face_show->setMaximumSize(320,240);
        ui->face_show->setMinimumSize(320,240);

        ui->password->setEchoMode(QLineEdit::Password);
        connect(ui->login_in,SIGNAL(clicked()),this,SLOT(show_login()));
        connect(ui->face_login_in,SIGNAL(clicked()),this,SLOT(face_login()));
        connect(ui->show_password,SIGNAL(clicked()),this,SLOT(show_passwd()));
        
        connect(ui->exit,SIGNAL(clicked()),this,SLOT(exit_login()));

        // w.show();
    }

    void MainWindow2::paintfaceCam(QImage map)
    {
        face_imageMap = map;
        face_imageItem->setPixmap(QPixmap::fromImage(face_imageMap));
        ui->face_show->show();
        QMatrix matrix;
        // ROS_INFO("test:%d",qnode.face_flag);
        if(face_flag && qnode.face_flag)
        {
            face_flag = false;
            QString strTemp(" face detection successful");
            QMessageBox msg;
            msg.setText(strTemp);
            msg.exec();
            if(qnode.mode == "track_detection")
                qnode.control_smart_car(true);
            else
                qnode.control_smart_car(false);
            w.show();
        }

        // if (face_imageMap.height()>280||face_imageMap.width()>320)
        //       matrix.scale(0.75,0.75);
        // ui->face_show->setMatrix(matrix);
        update();
    }
    
    void MainWindow2::face_login()
    {
        face_flag = true;
    }

    void MainWindow2::show_login()
    {
        QString user = ui->user->text();
        QString password = ui->password->text();
        if(user=="admin" && password=="123456")
        {
            w.show();
        }
        else if(user=="" && password=="")
        {
            QString strTemp("please input user name and password!!!");
            QMessageBox msg;
            msg.setText(strTemp);
            msg.exec();
        }
        else {
            QString strTemp("user name or password is error, please log in again!!!");
            QMessageBox msg;
            msg.setText(strTemp);
            msg.exec();
        }
    }
    void MainWindow2::signalHandler(int signum)
    {
      ROS_INFO("%s is received, Terminating the node...",strsignal(signum));
      ros::shutdown();
      exit(signum);
    }
    void MainWindow2::exit_login()
    {
        signal(SIGINT,signalHandler);
        this->close();
    }
    void MainWindow2::show_passwd()
    {
        if (flag)
        {
            ui->password->setEchoMode(QLineEdit::Normal);
            flag=false;
        }
        else
        {
            ui->password->setEchoMode(QLineEdit::Password);
            flag=true;
        }
    }

    MainWindow2::~MainWindow2()
    {
    //    delete ui;
    }
}

