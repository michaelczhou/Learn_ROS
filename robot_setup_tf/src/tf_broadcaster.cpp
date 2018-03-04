#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
//因为后边会使用到tf::TransformBroadcaster类的实例，来完成tf树的广播，所以需要先包含相关的头文件。

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;
 
  ros::Rate r(100);

  //创建一个tf::TransformBroadcaster类的实例，用来广播 base_link → base_laser的变换关系。
  tf::TransformBroadcaster broadcaster;
  
  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
        ros::Time::now(),"base_link", "base_laser"));
    //这部分是代码的关键所在。通过TransformBroadcaster 类来发布变换关系的接口，需要五个参数。首先是两个参考系之间的旋转变换，通过btQuaternion四元数来存储旋转变换的参数，因为我们用到的两个参考系没有发生旋转变换，所以倾斜角、滚动角、偏航角都是0。第二个参数是坐标的位移变换，我们用到的两个参考系在X轴和Z轴发生了位置，根据位移值填入到btVector3 向量中。第三个参数是时间戳，直接太难过ROS的API完成。第四个参数是母节点存储的参考系，即base_link，最后一个参数是子节点存储的参考系，即base_laser。
    r.sleep();
  }
}
