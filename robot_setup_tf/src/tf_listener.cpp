#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
//在后边的代码中，我们会使用到tf::TransformListener对象，该对象会自动订阅ROS中的tf消息，并且管理所有的变换关系数据。所以需要先包含相关的头文件。
#include <tf/transform_listener.h>

//我们创建一个回调函数，每次收到tf消息时，都会自动调用该函数，上一节我们设置了发布tf消息的频率是1Hz，所以回调函数执行的频率也是1Hz。在回调函数中，我们需要完成数据从base_laser到base_link参考系的坐标变换。 
void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";
 
  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();
 
  //just an arbitrary point in space
  laser_point.point.x = 1.0;
  laser_point.point.y = 0.2;
  laser_point.point.z = 0.0;
 
  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", laser_point, base_point);
 
    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
}
 
int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;
 
  tf::TransformListener listener(ros::Duration(10));
 
  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
 
  ros::spin();
 
}
