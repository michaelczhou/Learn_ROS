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
  // 我们创建了一个geometry_msgs::PointStamped类型的虚拟点，该点的坐标为（1.0，0.2，0.0）。该类型包含标准的header消息结构，这样，我们可以就可以在消息中加入发布数据的时间戳和参考系的id。
  laser_point.point.x = 1.0;
  laser_point.point.y = 0.2;
  laser_point.point.z = 0.0;
 
  // 这里是代码的关键位置。我们已经在base_laser参考系下虚拟了一个数据点，那么怎样将该点的数据转换到base_base参考系下呢？使用TransformListener 对象中的transformPoint（）函数即可，该函数包含三个参数：第一个参数是需要转换到的参考系id，当然是base_link了；第二个参数是需要转换的原始数据；第三个参数用来存储转换完成的数据。该函数执行完毕后，base_point就是我们转换完成的点坐标了！
  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", laser_point, base_point);
 
    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }

  //为了保证代码的稳定性，我们也需要应对出错处理，例如当tf并没有发布需要的变换关系时，在执行transformPoint时就会出现错误。
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
