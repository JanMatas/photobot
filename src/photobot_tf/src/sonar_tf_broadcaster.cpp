/*
 * Node that publishes a sonar -> base_link transform to tf
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "photobot_sonar_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(
            tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.2, 0.0, 1.0)),
            ros::Time::now(), "base_link", "sonar"));
    r.sleep();
  }
}