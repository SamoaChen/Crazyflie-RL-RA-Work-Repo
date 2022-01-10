#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"
#include <geometry_msgs/Twist.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "tf_test_node");
    ros::NodeHandle n;
    ros::Publisher tf_pub = n.advertise<std_msgs::Float64>("tf_z", 1000);
    //ros::Publisher tf_pub = n.advertise<geometry_msgs::Twist>("tf_z", 1000);
    std::string m_world = "world";
    std::string m_frame = "crazy_mpc/base_link";
    std_msgs::Float64 z_value;

    tf::TransformListener listener;
    listener.waitForTransform(m_world, m_frame, ros::Time(0), ros::Duration(10.0));

    ros::Rate loop_rate(140);
    while(ros::ok) {
        tf::StampedTransform transform;
        listener.lookupTransform(m_world, m_frame, ros::Time(0), transform);
        //std::cout << (transform.getOrigin().z()) << std::endl;
        z_value.data = transform.getOrigin().z();
        tf_pub.publish(z_value);

        loop_rate.sleep();
    }

}