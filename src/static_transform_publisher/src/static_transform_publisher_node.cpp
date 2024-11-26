#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <rosgraph_msgs/Clock.h>

class StaticTransformPublisher
{
public:
  StaticTransformPublisher() {
    ros::NodeHandle nh;

    // Initialize the static transform broadcaster
    static_broadcaster = new tf2_ros::StaticTransformBroadcaster();

    // Subscribe to the /clock topic
    clock_sub = nh.subscribe("/clock", 20, &StaticTransformPublisher::clock_callback, this);

    // Set up the static transforms
    setupTransforms();
  }

private:
  tf2_ros::StaticTransformBroadcaster* static_broadcaster;
  ros::Subscriber clock_sub;
  std::vector<geometry_msgs::TransformStamped> transforms;

void setupTransforms() {
  geometry_msgs::TransformStamped transform;

  // base_footprint -> base_link
  // transform.header.frame_id = "base_footprint";
  // transform.child_frame_id = "base_link";
  // transform.transform.translation.x = 0.0;
  // transform.transform.translation.y = 0.0;
  // transform.transform.translation.z = 0.0;
  // transform.transform.rotation.x = 0.0;
  // transform.transform.rotation.y = 0.0;
  // transform.transform.rotation.z = 0.0;
  // transform.transform.rotation.w = 1.0;
  // transforms.push_back(transform);

  // base_link -> laser_link
  // transform.header.frame_id = "base_link";
  // transform.child_frame_id = "laser_link";
  // transform.transform.translation.x = 0.1;
  // transform.transform.translation.y = 0.0;
  // transform.transform.translation.z = 0.2;
  // transform.transform.rotation.x = 0.0;
  // transform.transform.rotation.y = 0.0;

  // map -> odom
  // transform.header.frame_id = "map";
  // transform.child_frame_id = "odom";
  // transform.transform.translation.x = 0.0;
  // transform.transform.translation.y = 0.0;
  // transform.transform.translation.z = 0.0;
  // transform.transform.rotation.x = 0.0;
  // transform.transform.rotation.y = 0.0;
  // transform.transform.rotation.z = 0.0;
  // transform.transform.rotation.w = 1.0;
  // transforms.push_back(transform);

  // odom -> base_footprint
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_footprint";
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  transforms.push_back(transform);

  // base_footprint -> base_link
  transform.header.frame_id = "base_footprint";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  transforms.push_back(transform);

  // base_link -> laser_frame
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "laser_link";
  transform.transform.translation.x = 0.2245;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.2;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  transforms.push_back(transform);
}

  void clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg)
  {
    // Update the timestamp from the /clock topic
    for (auto& transform : transforms) {
      transform.header.stamp = msg->clock;
      static_broadcaster->sendTransform(transform);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_transform_publisher_node");
  StaticTransformPublisher node;
  ros::spin();
  return 0;
}