#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>

using namespace cv;




// make a pseudo color depth
void
makeDepthImage(Mat &inXyz, Mat &outImage, unsigned short minDepth, unsigned short maxDepth)
{
    if(outImage.rows != inXyz.rows || outImage.cols != inXyz.cols || outImage.dims != CV_8UC3) {
        outImage = Mat::zeros(inXyz.rows, inXyz.cols, CV_8UC3);
    }

    unsigned short len = maxDepth-minDepth;
    int nTotal = inXyz.rows*inXyz.cols;
    unsigned short *pSrc = (unsigned short*) inXyz.data;
    unsigned char *pDst = (unsigned char*) outImage.data;

    for(int i=0; i<nTotal; i++, pSrc++, pDst+=3) {
        if(pSrc[2] == 0.) {
            pDst[0] = 0;
            pDst[1] = 0;
            pDst[2] = 0;
            //pDst[3] = 0;
        } else {
            int v = (int) (255*6*(1. - ((double) (maxDepth-pSrc[2]))/len));

            if (v < 0)
                v = 0;

            int lb = v & 0xff;

            switch (v / 256) {
            case 0:
                //pDst[3] = 0;
                pDst[2] = 255;
                pDst[1] = 255-lb;
                pDst[0] = 255-lb;
                break;
            case 1:
                //pDst[3] = 0;
                pDst[2] = 255;
                pDst[1] = lb;
                pDst[0] = 0;
                break;
            case 2:
                //pDst[3] = 0;
                pDst[2] = 255-lb;
                pDst[1] = 255;
                pDst[0] = 0;
                break;
            case 3:
                //pDst[3] = 0;
                pDst[2] = 0;
                pDst[1] = 255;
                pDst[0] = lb;
                break;
            case 4:
                //pDst[3] = 0;
                pDst[2] = 0;
                pDst[1] = 255-lb;
                pDst[0] = 255;
                break;
            case 5:
                //pDst[3] = 0;
                pDst[2] = 0;
                pDst[1] = 0;
                pDst[0] = 255-lb;
                break;
            default:
                //pDst[3] = 0;
                pDst[2] = 0;
                pDst[1] = 0;
                pDst[0] = 0;
                break;
            }

            if (v == 0) {
                //pDst[3] = 0;
                pDst[2] = 0;
                pDst[1] = 0;
                pDst[0] = 0;
            }
        }
    }


}



// A callback function. Executed eack time a new pose message arrives.
void poseMessageReceivedRGB(const sensor_msgs::Image& msg) {
    ROS_INFO("seq = %d / width = %d / height = %d / step = %d", msg.header.seq, msg.width, msg.height, msg.step);
    ROS_INFO("encoding = %s", msg.encoding.c_str());
    //vector<unsigned char> data = msg.data;

    Mat image = Mat(msg.height, msg.width, CV_8UC3);
    memcpy(image.data, &msg.data[0], sizeof(unsigned char)*msg.data.size());
    imshow("RGB Preview", image);
    waitKey(30);
}


// A callback function. Executed eack time a new pose message arrives.
void poseMessageReceivedDepthRaw(const sensor_msgs::Image& msg) {
    ROS_INFO("seq = %d / width = %d / height = %d / step = %d", msg.header.seq, msg.width, msg.height, msg.step);
    ROS_INFO("encoding = %s", msg.encoding.c_str());
    //vector<unsigned char> data = msg.data;

    Mat rawDepth = Mat(msg.height, msg.width, CV_16UC1);
    memcpy(rawDepth.data, &msg.data[0], sizeof(unsigned char)*msg.data.size());

    Mat pseduoColorDepth;
    makeDepthImage(rawDepth, pseduoColorDepth, 0, 4096);

    imshow("Depth Preview", pseduoColorDepth);
    waitKey(30);
}


int main(int argc, char **argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "turtle_kinect_image_view");
    ros::NodeHandle nh;

    // Create a subscriber object
    ros::Subscriber subRgb = nh.subscribe("/camera/rgb/image_color", 10, &poseMessageReceivedRGB);
    ros::Subscriber subDetphRaw = nh.subscribe("/camera/depth_registered/image_raw", 10, &poseMessageReceivedDepthRaw);

    // Let ROS take over
    ros::spin();

    return 0;
}
