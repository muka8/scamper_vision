#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace sensor_msgs;
using namespace cv_bridge;

//画像を受信すると呼ばれるコールバック関数
void onImgSubscribed(const Image &img)
{
	//受信した画像を変換
	CvImagePtr cv_img = toCvCopy(img, img.encoding);
	//画像の表示
	cv::imshow("Image", cv_img->image);
	cv::waitKey(10);
}

int main(int argc, char **argv)
{
	//ROSノードの初期化
	ros::init(argc, argv, "camera_viwer");
	ros::NodeHandle nh;
	//Subscriberの登録
	ros::Subscriber sub_img = nh.subscribe("image", 10, onImgSubscribed);
	//コールバックの待機
	ros::spin();
	return 0;
}

