#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h> // 确保包含了必要的头文件

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_publisher");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odometry/filtered", 10);
    tf::TransformBroadcaster odom_broadcaster; // 创建 TF 广播器
    static tf::TransformBroadcaster static_broadcaster; // 创建静态 TF 广播器

    ros::Rate loop_rate(10); // 10 Hz

    double x = 0.0;
    double y = 0.0;
    double th = 0.0; // 假设机器人朝向不变

    // 在循环外声明静态变换
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.frame_id = "map";
    static_transformStamped.child_frame_id = "odom";
    static_transformStamped.transform.translation.x = 0.0;
    static_transformStamped.transform.translation.y = 0.0;
    static_transformStamped.transform.translation.z = 0.0;
    static_transformStamped.transform.rotation.x = 0.0;
    static_transformStamped.transform.rotation.y = 0.0;
    static_transformStamped.transform.rotation.z = 0.0;
    static_transformStamped.transform.rotation.w = 1.0;

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();

        // 创建里程计消息
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // 设定位置信息
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);

        // 发布里程计消息
        odom_pub.publish(odom_msg);

        // 发布 TF 变换
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_msg.pose.pose.orientation;

        odom_broadcaster.sendTransform(odom_trans);

        // 发布 map 到 odom 的静态变换
        static_transformStamped.header.stamp = current_time;
        static_broadcaster.sendTransform(static_transformStamped);

        // 模拟前进
        x += 0.005;
        y += 0.005

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}