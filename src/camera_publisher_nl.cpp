#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <opencv2/opencv.hpp>

//Nodeletクラスの場合必ず名前空間をつける
namespace scamper_vision
{
//Nodeletクラスを継承したクラスを作成
class CameraPublisherNl : public nodelet::Nodelet
{
public:
	CameraPublisherNl();
        ~CameraPublisherNl();
	virtual void onInit();
private:
	void onTimerElapsed(const ros::TimerEvent &e);
	ros::NodeHandle nh_;
	ros::Timer timer_;
	ros::Publisher pub_img_;
	cv::VideoCapture camera_;
	cv_bridge::CvImage img_;
};

CameraPublisherNl::CameraPublisherNl()
{

}

CameraPublisherNl::~CameraPublisherNl()
{

}
//モジュールの初期化を行う関数
void CameraPublisherNl::onInit()
{
	//ノードハンドラの初期化
	nh_= this->getNodeHandle();
	//Parameterの読み込み
	std::string camera_path;
	nh_.param("camera_path", camera_path, std::string("/dev/video0"));
	//カメラの初期化
	camera_.open(camera_path, cv::CAP_V4L2);
	//カメラの初期化に失敗したらノード終了
	if(!camera_.isOpened()){
		ROS_ERROR("failed to initialized camera");
		ros::shutdown();
	}
	//Publisherの登録
	pub_img_ = nh_.advertise<sensor_msgs::Image>("image", 10);
	//33msecごににTime割り込み
	timer_ = nh_.createTimer(ros::Duration(0.033), &CameraPublisherNl::onTimerElapsed, this);
}

//Timer割り込み発生時に呼び出される関数
void CameraPublisherNl::onTimerElapsed(const ros::TimerEvent &e)
{
	camera_ >> img_.image;
	img_.encoding = "bgr8";
	img_.header.stamp = ros::Time::now();
	pub_img_.publish(img_.toImageMsg());
}

}

PLUGINLIB_EXPORT_CLASS(scamper_vision::CameraPublisherNl, nodelet::Nodelet)


