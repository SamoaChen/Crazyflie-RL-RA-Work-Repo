#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "test_tf_listener");

    ros::NodeHandle node;

    ros::Publisher test_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    tf::TransformListener listener;

    ros::Rate rate(250.0);
    ros::Time old_time, current_time;
    float diff = 0.0;
    current_time = ros::Time::now();
    old_time = ros::Time::now();
    while (node.ok()){
        tf::StampedTransform transform;
        try{
            listener.waitForTransform("/crazy_mpc/base_link",  "/world", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/crazy_mpc/base_link",  "/world", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::Twist vel;
        current_time = ros::Time::now();
        diff = current_time.toSec() - old_time.toSec();
        std::cout << 1/diff << std::endl; 
        vel.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
        vel.linear.y = diff;
        vel.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
        test_pub.publish(vel);
        old_time = current_time;

        rate.sleep();
    }
    return 0;
};