/*
 * Node that will use published transforms to transform a point
 * in sonar to a point in base_link.
 */

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the sonar frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped sonar_point;
  sonar_point.header.frame_id = "sonar";

  //we'll just use the most recent transform available for our simple example
  sonar_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  sonar_point.point.x = 1.0;
  sonar_point.point.y = 0.2;
  sonar_point.point.z = 0.0;

  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", sonar_point, base_point);

    ROS_INFO("sonar: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        sonar_point.point.x, sonar_point.point.y, sonar_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"sonar\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "photobot_sonar_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}
