#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>


using namespace cv;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global variable
boost::mutex mutex;
sensor_msgs::LaserScan g_scan;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// A template method to check 'nan'
template<typename T>
inline bool isnan(T value)
{
    return value != value;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// callback function
void
scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
    // receive a '/odom' message with the mutex
    mutex.lock(); {
        g_scan = msg;
    } mutex.unlock();
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<Vec3d> &XYZs)
{
    int nRangeSize = (int)lrfScan.ranges.size();
    XYZs.clear();
    XYZs.resize(nRangeSize);

    for(int i=0; i<nRangeSize; i++) {
        double dRange = lrfScan.ranges[i];

        if(isnan(dRange)) {
            XYZs[i] = Vec3d(0., 0., 0.);
        } else {
            double dAngle = lrfScan.angle_min + i*lrfScan.angle_increment;
            XYZs[i] = Vec3d(dRange*cos(dAngle), dRange*sin(dAngle), 0.);
        }
    }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void
initGrid(Mat &display, int nImageSize)
{
    const int nImageHalfSize = nImageSize/2;
    const int nAxisSize = nImageSize/16;
    const Vec2i imageCenterCooord = Vec2i(nImageHalfSize, nImageHalfSize);
    display = Mat::zeros(nImageSize, nImageSize, CV_8UC3);
    line(display, Point(imageCenterCooord[0], imageCenterCooord[1]), Point(imageCenterCooord[0]+nAxisSize, imageCenterCooord[1]), Scalar(0, 0, 255), 2);
    line(display, Point(imageCenterCooord[0], imageCenterCooord[1]), Point(imageCenterCooord[0], imageCenterCooord[1]+nAxisSize), Scalar(0, 255, 0), 2);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// A callback function. Executed eack time a new pose message arrives.
void
drawLRFScan(Mat &display, vector<Vec3d> &laserScanXY, double dMaxDist)
{
    //??????????????????????????????

}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int main(int argc, char **argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "turtle_kinect_lrf_view");
    ros::NodeHandle nh;

    // Create subscriber objects
    ros::Subscriber subScan = nh.subscribe("/scan", 10, &scanMsgCallback);

    // Display buffer
    Mat display;
    initGrid(display, 801);

    // Scan buffer
    sensor_msgs::LaserScan scan;

    // LRF scan 정보
    vector<Vec3d> laserScanXY;

    // Mat distance for grid
    const double dGridMaxDist = 4.5;

    // main loop
    while(ros::ok()) {
        // callback 함수을 call!
        ros::spinOnce();

        // receive the global '/scan' message with the mutex
        mutex.lock(); {
           scan = g_scan;
        } mutex.unlock();

        // scan으로부터 Cartesian X-Y scan 획득
        convertScan2XYZs(scan, laserScanXY);

        // 현재 상황을 draw할 display 이미지를 생성
        initGrid(display, 801);
        drawLRFScan(display, laserScanXY, dGridMaxDist);

        // 2D 영상좌표계에서 top-view 방식의 3차원 월드좌표계로 변환
        //transpose(display, display);  // X-Y축 교환
        //flip(display, display, 0);  // 수평방향 반전
        //flip(display, display, 1);  // 수직방향 반전

        // 영상 출력!
        imshow("KNU ROS Lecture >> turtle_kinect_lrf_view", display);

        // 사용자의 키보드 입력을 받음!
        int nKey = waitKey(30) % 255;

        if(nKey == 27) {
            // 종료
            break;
        }
    }

    return 0;
}
