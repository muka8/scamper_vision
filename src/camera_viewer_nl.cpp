#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <opencv2/opencv.hpp>

using namespace sensor_msgs;
using namespace cv_bridge;

namespace scamper_vision
{

class CameraViewerNl : public nodelet::Nodelet
{
public:
	CameraViewerNl();
	~CameraViewerNl();
	virtual void onInit();
private:
	void onImgSubscribed(const ImageConstPtr &img);
	ros::NodeHandle nh_;
	ros::Subscriber sub_img_;
};

CameraViewerNl::CameraViewerNl()
{

}

CameraViewerNl::~CameraViewerNl()
{

}

//モジュールが読み込まれると自動的に呼び出される関数
void CameraViewerNl::onInit()
{
	//ノードハンドラの初期化
	nh_ = this->getNodeHandle();
	//Subscirberの初期化
	sub_img_ = nh_.subscribe("Image", 10, &CameraViewerNl::onImgSubscribed, this);
}

//画像を受信すると呼ばれるコールバック関数
void CameraViewerNl::onImgSubscribed(const ImageConstPtr &img)
{
	//受信した画を変換
	CvImageConstPtr cv_img = toCvShare(img, img->encoding);
	if(cv_img->image.empty()) return;
	//画像の表示
	cv::imshow("image", cv_img->image);
	cv::waitKey(10);
}

}

PLUGINLIB_EXPORT_CLASS(scamper_vision::CameraViewerNl, nodelet::Nodelet)

