#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>


#define USE_X_EAST_Y_NORTH      (1)

using namespace cv;


// A template method to check 'nan'
template<typename T>
inline bool isnan(T value)
{
    return value != value;
}


// A callback function. Executed eack time a new pose message arrives.
void poseMessageReceivedLRF(const sensor_msgs::LaserScan& msg)
{
    //ROS_INFO("angle_min = %f, angle_max = %f, angle_increment = %f", msg.angle_min, msg.angle_max, msg.angle_increment);
    //ROS_INFO("time_increment = %f, scan_time = %f", msg.time_increment, msg.scan_time);
    //ROS_INFO("range_min = %f, range_max = %f", msg.range_min, msg.range_max);
    //ROS_INFO("range_count = %d, intensities_count = %d", (int)msg.ranges.size(), (int)msg.intensities.size());

    // convert polar to Cartesian coordinate
    vector<Vec2d> coord;
    int nRangeSize = (int)msg.ranges.size();

    for(int i=0; i<nRangeSize; i++) {
        double dRange = msg.ranges[i];

        if(isnan(dRange)) {
            coord.push_back(Vec2d(0., 0.));
        } else {
             if(USE_X_EAST_Y_NORTH) {
                // for Y-heading in world coordinate system (YX)
                double dAngle = msg.angle_max - i*msg.angle_increment;
                coord.push_back(Vec2d(dRange*sin(dAngle), dRange*cos(dAngle)));
             } else {
                // for X-heading in world coordinate system (XY)
                double dAngle = msg.angle_min + i*msg.angle_increment;
                coord.push_back(Vec2d(dRange*cos(dAngle), dRange*sin(dAngle)));
             }
        }
    }


    // draw the 'coord' in image plane
    const int nImageSize = 801;
    const int nImageHalfSize = nImageSize/2;
    const int nAxisSize = nImageSize/16;

    const Vec2i imageCenterCooord = Vec2i(nImageHalfSize, nImageHalfSize);
    Mat image = Mat::zeros(nImageSize, nImageSize, CV_8UC3);
    line(image, Point(imageCenterCooord[0], imageCenterCooord[1]), Point(imageCenterCooord[0]+nAxisSize, imageCenterCooord[1]), Scalar(0, 0, 255), 2);
    line(image, Point(imageCenterCooord[0], imageCenterCooord[1]), Point(imageCenterCooord[0], imageCenterCooord[1]+nAxisSize), Scalar(0, 255, 0), 2);

    for(int i=0; i<nRangeSize; i++) {
        int x_image = imageCenterCooord[0] + cvRound((coord[i][0]/msg.range_max)*nImageHalfSize);
        int y_image = imageCenterCooord[1] + cvRound((coord[i][1]/msg.range_max)*nImageHalfSize);

        if(x_image >= 0 && x_image < nImageSize && y_image >= 0 && y_image < nImageSize) {
            image.at<Vec3b>(y_image, x_image) = Vec3b(255, 255, 0);
            //circle(image, Point(x_image, y_image), -1, Scalar(255, 255, 0), 2, CV_AA);
        }
    }

    // image coordinate transformation
    flip(image, image, 0);

    imshow("Kinect LRF Preview", image);
    waitKey(30);
}


int main(int argc, char **argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "turtle_kinect_lrf_view");
    ros::NodeHandle nh;

    // Create a subscriber object
    ros::Subscriber sub = nh.subscribe("/scan", 10, &poseMessageReceivedLRF);

    // Let ROS take over
    ros::spin();

    return 0;
}
