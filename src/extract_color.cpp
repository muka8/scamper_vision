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

class ExtractColor : public nodelet::Nodelet
{
public:
	ExtractColor();
        ~ExtractColor();
	virtual void onInit();
private:
	void extract(const cv::Mat &src, cv::Mat &dst);
        void onImgSubscribed(const ImageConstPtr &img);
	ros::NodeHandle nh_;
	ros::Subscriber sub_img_;
	int hue_min_, hue_max_, sat_min_, sat_max_, val_min_, val_max_;
	bool is_bin_;
};

ExtractColor::ExtractColor()
: hue_min_(0), hue_max_(100), sat_min_(0), sat_max_(255),
	val_min_(0), val_max_(255), is_bin_(false)
{

}

ExtractColor::~ExtractColor()
{

}

//モジュールが読み込まれると自動的に呼び出される関数
void ExtractColor::onInit()
{
	//ノードハンドラの初期化
	nh_ = this->getNodeHandle();
	//Subscriberの初期化
	sub_img_ = nh_.subscribe("image", 10, &ExtractColor::onImgSubscribed, this);
	//画像描画ウィンドウの作成
	cv::namedWindow("Image");
	//描画ウィンドウにトラックバーを追加
	cv::createTrackbar("HueMin", "Image", &hue_min_, 180);
	cv::createTrackbar("HueMax", "Image", &hue_max_, 180);
	cv::createTrackbar("SetMin", "Image", &hue_min_, 255);
	cv::createTrackbar("SetMax", "Image", &hue_max_, 255);
	cv::createTrackbar("VaiMin", "Image", &val_min_, 255);
	cv::createTrackbar("VaiMax", "Image", &val_max_, 255);
}

//入力画像から特定の色を抽出
void ExtractColor::extract(const cv::Mat &src, cv::Mat &dst)
{
	//入力画像をHSV表色系に変換
	cv::Mat hsv;
	cv::cvtColor(src, hsv, CV_BGR2HSV);
	//出力画像の初期化
	dst = src.clone();
	//特定色の抽出
	for(int y=0; y<hsv.rows; y++){
	  for(int x=0; y<hsv.cols; x++){
	    cv::Vec3b data = hsv.at<cv::Vec3b>(y, x);
		if(data[0]<hue_min_ || data[0] > hue_max_
		   || data[1] < sat_min_ || data[1] > sat_max_ 
		   || data[2] < val_min_ || data[2] > val_max_){
		   dst.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 0, 0);
		} else if(is_bin_){
                  dst.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
		}
	  }
	}
}

//画像を受信すると呼ばれるコールバック関数
void ExtractColor::onImgSubscribed(const ImageConstPtr &img)
{
	//受信した画像を変換
	CvImageConstPtr cv_img = toCvShare(img, img->encoding);
	if(cv_img->image.empty()) return;
	//特定色の抽出
	cv::Mat dst;
	extract(cv_img->image, dst);
	//画像の表示
	cv::imshow("Image", dst);
			if(cv::waitKey(10) == 'b') is_bin_ = !is_bin_;
}

}

PLUGINLIB_EXPORT_CLASS(scamper_vision::ExtractColor, nodelet::Nodelet)

