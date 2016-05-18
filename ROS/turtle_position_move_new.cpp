#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <tf/tf.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global variable
boost::mutex mutex;
nav_msgs::Odometry g_odom;
float pre_dAngleTurned;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// callback function
void
odomMsgCallback(const nav_msgs::Odometry &msg)
{
    // receive a '/odom' message with the mutex
    mutex.lock(); {
        g_odom = msg;
    } mutex.unlock();
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// odomÀž·ÎºÎÅÍ ÇöÀçÀÇ º¯È¯Çà·Ä Á€ºžžŠ ž®ÅÏ!
tf::Transform
getCurrentTransformation(void)
{
    // transformation ¹öÆÛ
    tf::Transform transformation;

    // odom ¹öÆÛ
    nav_msgs::Odometry odom;

    // copy a global '/odom' message with the mutex
    mutex.lock(); {
        odom = g_odom;
    } mutex.unlock();

    // À§Ä¡ ÀúÀå
    transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

    // ÈžÁ€ ÀúÀå
    transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

    // ž®ÅÏ
    return transformation;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ·Îº¿ÀÌ žØÃçÀÖŽÂ »óÅÂ(Ã³Àœ »óÅÂ)ÀÇ À§Ä¡žŠ ÀúÀå!
tf::Transform
getInitialTransformation(void)
{
    // tf º¯È¯Çà·Ä
    tf::Transform transformation;

    // Ã³ÀœÀ§Ä¡¿¡ ŽëÇÑ odometry žÞœÃÁö ¹Þ±â
    ros::Rate loopRate(1000.0);

    while(ros::ok()) {
        // ÀÏŽÜ callback žÞœÃÁöžŠ ¹Þ°í!
        ros::spinOnce();

        // get current transformationreturn;
        transformation = getCurrentTransformation();

        // žÞœÃÁöžŠ ¹ÞŸÒÀžžé break!
        if(transformation.getOrigin().getX() != 0. || transformation.getOrigin().getY() != 0. && transformation.getOrigin().getZ() != 0.) {
            break;
        } else {
            loopRate.sleep();
        }
    }

    // ž®ÅÏ
    return transformation;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ÈžÀü
bool
doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
{
    //the command will be to turn at 'rotationSpeed' rad/s
    geometry_msgs::Twist baseCmd;
    baseCmd.linear.x = 0.0;
    baseCmd.linear.y = 0.0;

    if(dRotation < 0.) {
        baseCmd.angular.z = -dRotationSpeed;
    } else {
        baseCmd.angular.z = dRotationSpeed;
    }

    // ÀÌµ¿ÇÏžéŒ­ ÇöÀçÀ§Ä¡¿¡ ŽëÇÑ odometry žÞœÃÁö ¹Þ±â
    bool bDone = false;
    ros::Rate loopRate(1000.0);



    while(ros::ok() && !bDone) {
        // ÀÏŽÜ callback žÞœÃÁöžŠ ¹Þ°í!
        ros::spinOnce();

        // get current transformation
        tf::Transform currentTransformation = getCurrentTransformation();

        //see how far we've traveled
        tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
        tf::Quaternion rotationQuat = relativeTransformation.getRotation();


        
	double dAngleTurned = atan2((2 * rotationQuat[2] * rotationQuat[3]) , (1-(2 * (rotationQuat[2] * rotationQuat[2]) ) ));

    if( fabs(dAngleTurned) > fabs(dRotation) || (abs(pre_dAngleTurned - dRotation) <  abs(dAngleTurned - dRotation)) || (dRotation == 0)) 
	{
            bDone = true;
            break;
        } else {
	    pre_dAngleTurned = dAngleTurned;
            //send the drive command
            pubTeleop.publish(baseCmd);

            // sleep!
            loopRate.sleep();
        }
    }

    // ÃÊ±âÈ­
    baseCmd.linear.x = 0.0;
    baseCmd.angular.z = 0.0;
    pubTeleop.publish(baseCmd);

    return bDone;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ÀÌµ¿
bool
doTranslation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dTranslation, double dTranslationSpeed)
{
    //the command will be to go forward at 'translationSpeed' m/s
    geometry_msgs::Twist baseCmd;

    if(dTranslation < 0) {
        baseCmd.linear.x = -dTranslationSpeed;
    } else {
        baseCmd.linear.x = dTranslationSpeed;
    }

    baseCmd.linear.y = 0;
    baseCmd.angular.z = 0;

    // ÀÌµ¿ÇÏžéŒ­ ÇöÀçÀ§Ä¡¿¡ ŽëÇÑ odometry žÞœÃÁö ¹Þ±â
    bool bDone = false;
    ros::Rate loopRate(1000.0);

    while(ros::ok() && !bDone) {
        // ÀÏŽÜ callback žÞœÃÁöžŠ ¹Þ°í!
        ros::spinOnce();

        // get current transformation
        tf::Transform currentTransformation = getCurrentTransformation();

        //see how far we've traveled
        tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
        double dDistMoved = relativeTransformation.getOrigin().length();

        // ÁŸ·áÁ¶°Ç ÃŒÅ©
        if(fabs(dDistMoved) >= fabs(dTranslation)) {
            bDone = true;
            break;
        } else {
            //send the drive command
            pubTeleop.publish(baseCmd);

            // sleep!
            loopRate.sleep();
        }
    }

    // ÃÊ±âÈ­
    baseCmd.linear.x = 0.0;
    baseCmd.angular.z = 0.0;
    pubTeleop.publish(baseCmd);

    return bDone;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int main(int argc, char **argv)
{
    // ROSžŠ ÃÊ±âÈ­
    ros::init(argc, argv, "turtle_position_move");

    // Ros initialization
    ros::NodeHandle nhp, nhs;

    // Decleation of subscriber
    ros::Subscriber sub = nhs.subscribe("/odom", 100, &odomMsgCallback);

    // Create a publisher object
    ros::Publisher pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 100);

    // exception
    if(argc != 3) {
        printf(">> rosrun knu_ros_lecture turtle_position_move [rot_degree] [trans_meter]\n");
        return 1;
    }

    // ÆÄ¶ó¹ÌÅÍ ¹ÞŸÆ¿À±â
    double dRotation = atof(argv[1]);

    float _dRatation = (float)((int)dRotation % 360);
    
  
    double dTranslation = atof(argv[2]);


    if(abs(_dRatation) > 180){
	if(dRotation > 0) dRotation = -(360-_dRatation);
        else dRotation = (360+_dRatation); }
    else
        dRotation = _dRatation;


    // ·Îº¿ÀÌ žØÃçÀÖŽÂ »óÅÂ(Ã³Àœ »óÅÂ)ÀÇ º¯È¯Çà·Ä °¡Á®¿À±â
    tf::Transform initialTransformation = getInitialTransformation();

    // ÈžÀü!


    doRotation(pub, initialTransformation, toRadian(dRotation), 0.75);
//doRotation(pub, initialTransformation, 3.14*2, 0.75);

    // ÀÌµ¿!
    doTranslation(pub, initialTransformation, dTranslation, 0.25);

    return 0;
}
