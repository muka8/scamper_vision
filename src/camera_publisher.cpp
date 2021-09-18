#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
	//ROSの初期化
	ros::init(argc, argv, "streo_publisher");
	ros::NodeHandle nh;
	//Parameterの読み込み
	std::string camera_path;
	nh.param("camera_path", camera_path, std::string("/dev/video0"));
	//カメラの初期化
	cv::VideoCapture camera(camera_path, cv::CAP_V4L2);
	//カメラの初期化に失敗したらノード終了
	if(!camera.isOpened()){
		ROS_ERROR("Failed to initialize camera");
		return -1;
	}
	//Publisherの登録
	ros::Publisher pub_img = nh.advertise<sensor_msgs::Image>("Image", 10);
	//ループ周期の設定
	ros::Rate loop_rate(30);
	//像用用変数の定義
	cv_bridge::CvImage img;
	img.encoding = "bgr8";
	//カメラ画像を取得してPublish
	while(ros::ok()){
		camera >> img.image;
		img.header.stamp = ros::Time::now();
		pub_img.publish(img.toImageMsg());
		loop_rate.sleep();
	}
	return 0;
}

