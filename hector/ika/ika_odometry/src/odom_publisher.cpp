#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>


std::string robot_name;
geometry_msgs::Pose2D pose2d_data;
geometry_msgs::Pose pose_data;
ros::Time current_time, last_time;

void pose2dCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
    pose2d_data.x = msg->x;
    pose2d_data.y = msg->y;
    pose2d_data.theta = msg->theta;
}

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    pose_data = *msg;
}


int main(int argc, char** argv){

    ros::init(argc, argv, ",my_odometry_publisher");
    if (argc != 2){
        ROS_ERROR("need robot name as argument");
        return -1;
    };
    robot_name = argv[1];

    ros::NodeHandle n;
    ros::Rate r(100); // Hz

    ros::Subscriber pose2d_sub = n.subscribe( "pose2d", 10, pose2dCallback);
    ros::Subscriber pose_sub = n.subscribe( "pose", 10, poseCallback);
    ros::Publisher  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    pose2d_data.x = 0;
    pose2d_data.y = 0;
    pose2d_data.theta = 0;
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;

    pose_data.position.x = 0;
    pose_data.position.y = 0;

    while(n.ok()){
        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();
        x = pose2d_data.x / 4.49;
        y = pose2d_data.y / 4.49; 
        th = pose2d_data.theta;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = robot_name + "/odom";
        odom_trans.child_frame_id = robot_name + "/base_footprint";

         odom_trans.transform.translation.x = x;
         odom_trans.transform.translation.y = y;
         odom_trans.transform.translation.z = 0.0;
         odom_trans.transform.rotation = odom_quat;
//        odom_trans.transform.translation.x = pose_data.position.x;
//        odom_trans.transform.translation.y = pose_data.position.y;
//        odom_trans.transform.translation.z = pose_data.position.z;
//        odom_trans.transform.rotation = pose_data.orientation;



        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = robot_name + "/odom";
        odom.child_frame_id = robot_name + "/base_link";

        //set the position
        // odom.pose.pose.position.x = x;
        // odom.pose.pose.position.y = y;
        // odom.pose.pose.position.z = 0.0;
        // odom.pose.pose.orientation = odom_quat;

        //set the velocity

        // odom.twist.twist.linear.x = vx;
        // odom.twist.twist.linear.y = vy;
        // odom.twist.twist.angular.z = vth;

        //publish the message

//        odom.pose.pose = pose_data;
        odom.pose.pose.position.x = x;//pose2d_data.x;
        odom.pose.pose.position.y = y;//pose2d_data.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }
    ros::spin();
}
