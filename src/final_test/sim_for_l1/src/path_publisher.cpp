// C++ 示例代码
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_publisher");
    ros::NodeHandle nh;

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/move_base_node/NavfnROS/plan", 10);

    ros::Rate loop_rate(1); // 1 Hz

    while (ros::ok()) {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "odom";

        // 创建一条直线路径
        for (int i = 0; i < 10; ++i) {
            geometry_msgs::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = i * 0.5;
            pose.pose.position.y = 0.0;
            pose.pose.orientation.w = 1.0;

            path_msg.poses.push_back(pose);
        }

        path_pub.publish(path_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}