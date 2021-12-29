#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>

using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace cv_bridge;

void extractColor(const cv::Mat &src, cv::Mat &dst)
{
	//srcをHSV表色系に変換
	cv::Mat hsv;
	cv::cvtColor(src, hsv, CV_BGR2HSV);
	//出力画像を0で初期化
	dst = cv::Mat(src.size(), CV_8UC1, cv::Scalar(0));
	//H, S, Vが閾値内の画素のみ255を代入
	for(int y=0; y<hsv.rows; y++){
		const uchar *ptr_hsv = hsv.ptr<uchar>(y);
		uchar *ptr_dst = dst.ptr<uchar>(y);
		for(int x=0; x<hsv.cols; x++){
			uchar hue = ptr_hsv[3*x + 0];
			uchar sat = ptr_hsv[3*x + 1];
			uchar val = ptr_hsv[3*x + 2];
			if(hue<hue_min_ || hue>hue_max_) continue;
			if(sat<sat_min_ || sat>sat_max_) continue;
			if(hue<val_min_ || val>val_max_) continue;
			ptr_dst[x] = 255;
		}
	}
}

//画像を受信すると呼ばれるコールバック関数
void onImgSubscribed(const Image &img)
{
	//信信した画像を変換
	CvImagePtr cv_img = toCvShare(img, img->encoding);
	//目標速度の変数を宣言
	Twist target_vel;
	//画像から特定の色を抽出
	cv::Mat bin;
	extractColor(cv_img->image, bin);
	//ラベリング処理により重心座標を算出
	cv::Mat label_img, stats, centroids;
	int n = cv::connectedComponentsWithStats(bin, label_img, stats, centroids, 8);
	//ラベリング領域が最大のものを選択
	int sz = 0, gx = 0;
	for(int i=1; i<n; i++){
		int *stat = stats.ptr<int>(i);
		double *centroid = centroids.ptr<double>(i);
		int tmp_sz = stat[cv::ConnectedComponentsTypes::CC_STAT_AREA];
		if(sz<tmp_sz){
		sz = tmp_sz;
		gx = static_cast<int>(centroid[0] - cv_img->image.cols/2);
	}
	}
	//物体の重心位置(x座標)からロボットの回転速度を決定
	if(sz>sz_min_ && abs(gx) > dead_zone_){
		int x_min = -cv_img->image.cols/2, x_max = -x_min;
		double vel_min = -vel_max_, vel_max = vel_max_;
		target_vel.angular.z = (gx - x_min) * (vel_min - vel_max) / (x_max - x_min) + vel_max;
	}
	//計算した速度をPublish
	pub_vel_.publish(target_vel);
}

}

int main(int argc, char **argv)
{
	//ROS initalize
	ros::init(argc, argv, "find_color");
        ros::NodeHandle nh;
//モジュールが読み込まれると自動的に呼び出されると自動的に呼び出される
	//void FindColor::onInit()
	
	//Parameterの読み込み
	hue_min_ = nh_.param<int>("hue/min", 50);
	hue_max_ = nh_.param<int>("hue/max", 65);
        sat_min_ = nh_.param<int>("sat/min", 100);
        sat_max_ = nh_.param<int>("sat/max", 255);
        val_min_ = nh_.param<int>("val/min", 50);
        val_max_ = nh_.param<int>("val/max", 255);
        sz_min_ = nh_.param<int>("size/min", 100);
	dead_zone_ = nh_.param<int>("dead_zone", 50);
	vel_max_ = nh_.param<double>("vel/max", 50.0);
	vel_max_ *=(M_PI / 180.0);
	//Subscriberの初期化
	ros::Subscriber sub_img_ = nh_.subscribe("image", 10, onImgSubscribed);
	//ros::Subscriber sub_keyop_ = nh_.subscribe("keyop", 10, onImgSubscribed);
	//Publisherの初期化
	ros::Publisher pub_vel_ = nh_.advertise<Twist>("/mobile_base/commands/velocity", 10);
	return 0;
}





