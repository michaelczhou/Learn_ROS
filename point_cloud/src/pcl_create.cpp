#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_create");
    //初始化ROS.允许ROS通过命令进行名称重映射.我们可以指定节点的名称.运行过程中,节点的名称必须唯一.
    //这里的名称必须是一个base name,名称内不能包含 / 等符号.
    ros::NodeHandle nh;
    //为这个进程的节点创建一个句柄.第一个创建的NodeHandle会为节点进行初始化.最后一个销毁的NodeHandle则会释放该节点所占用的所有资源.
    //
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    /***
    告诉master我们将要在pcl_output（话题名）上发布sensor_msgs::PointCloud2消息类型的消息。这样master就会告诉所有订阅了pcl_output话题的节点，将要有数据发布。
      第二个参数是发布序列的大小。如果我们发布的消息频率太高，缓冲区中的消息在大于1个的时候就会开始丢弃先前发布的消息。
      NodeHandle::advertise()返回一个ROS::Publisher对象，他有两个作用：
       1）他有一个publish（）成员函数可以让你在topic上发布消息；2）如果消息类型不对，他会拒绝
    ***/
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;

    // Fill in the cloud data
    cloud.width  = 100;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    //Convert the cloud to ROS message
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "odom";

    ros::Rate loop_rate(1);
    // 1 Hz
    //ros::Rate对象可以允许你指定自循环的频率。他会追踪记录上一次调用Rate::sleep()后时间的流逝，并休眠直到一个频率周期的时间。
    while (ros::ok())
    {
        pcl_pub.publish(output);
        //向所有订阅"pcl_output"话题的节点发送消息.
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
