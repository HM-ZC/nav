#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "send_goal_node");
    ros::NodeHandle nh;
    
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    
    // 确保连接建立
    while (goal_pub.getNumSubscribers() == 0) {
        ros::Duration(1.0).sleep();
    }

    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";

    // 目标位置
    goal.pose.position.x = 9.1;
    goal.pose.position.y = -1.7;
    goal.pose.position.z = 0.0;

    // 目标方向
    tf::Quaternion q = tf::createQuaternionFromYaw(0.0);
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    // 发布目标
    goal_pub.publish(goal);
    ROS_INFO("Goal published!");

    ros::spinOnce();

    return 0;
}