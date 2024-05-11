#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <tf/tf.h>

// 函数用于发送单个目标
void send_goal(ros::Publisher &pub, double x, double y, double yaw) {
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0;

    tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    pub.publish(goal);
    ROS_INFO("Sending goal: x=%f, y=%f, yaw=%f", x, y, yaw);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "send_goals_node");
    ros::NodeHandle nh;
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    // 等待发布者建立
    while (goal_pub.getNumSubscribers() == 0 && ros::ok()) {
        ros::Duration(1.0).sleep();
    }

    std::vector<std::pair<std::pair<double, double>, double>> goals = {
        {{5.7, 0.1}, 0.0},
        {{5.7, -3.5}, 0.0},
        {{7.7, -3.6}, 0.0},
        {{9.2, -1.9}, 0.0},
        // 在这里添加更多目标点
    };

    // 发送每个目标点
    for (auto &goal : goals) {
        send_goal(goal_pub, goal.first.first, goal.first.second, goal.second);
        ros::Duration(5.0).sleep(); // 等待一段时间或者直到目标达成
        // 你可以添加检查目标是否达成的代码
    }

    ros::spin();
    return 0;
}