#include "ros/ros.h"
//ros/ros.h是一个实用的头文件，它引用了ROS系统中大部分常用的头文件。
#include "std_msgs/String.h"
//它引用了std_msg/String消息，他存放在std_msgs package里，是由String.msg文件自动生成的头文件。
#include <sstream>
#include<stdlib.h>
#include<string>

using namespace std;
   /**
   * This tutorial demonstrates simple sending of messages over the ROS system.
    */
int main(int argc, char **argv)
   {
     /**
     * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line. For programmatic
     * remappings you can use a different version of init() which takes remappings
    * directly, but for most command-line programs, passing argc and argv is the easiest
    * way to do it.  The third argument to init() is the name of the node.
      *
      * You must call one of the versions of ros::init() before using any other
      * part of the ROS system.
      */
     ros::init(argc, argv, "talker");
     //初始化ROS。允许ROS通过命令进行名称重映射。我们可以指定节点的名称-运行过程中，节点的名称必须唯一。
     //这里的名称必须是一个base name,也就是说，名称内不能包含 / 等符号。
     /**
      * NodeHandle is the main access point to communications with the ROS system.
      句柄是连接ROS系统的主要获取点。
      * The first NodeHandle constructed will fully initialize this node, and the last
      * NodeHandle destructed will close down the node.
      */
     ros::NodeHandle n;
     //为这个进程的节点创建一个句柄。第一个创建的NodeHandle会为节点进行初始化，最后一个销毁的NodeHandle则会释放该节点所占用的所有资源。
     /**
      * The advertise() function is how you tell ROS that you want to
      * publish on a given topic name. This invokes a call to the ROS
      * master node, which keeps a registry of who is publishing and who
      * is subscribing. After this advertise() call is made, the master
      * node will notify anyone who is trying to subscribe to this topic name,
      * and they will in turn negotiate a peer-to-peer connection with this
      * node.  advertise() returns a Publisher object which allows you to
      * publish messages on that topic through a call to publish().  Once
      * all copies of the returned Publisher object are destroyed, the topic
      * will be automatically unadvertised.
      *
      * The second parameter to advertise() is the size of the message queue
      * used for publishing messages.  If messages are published more quickly
      * than we can send them, the number here specifies how many messages to
      * buffer up before throwing some away.
      *告诉master我们将要在chatter（话题名）上发布std_msgs/String消息类型的消息。这样master就会告诉所有订阅了chatter话题的节点，将要有数据发布。
      第二个参数是发布序列的大小。如果我们发布的消息频率太高，缓冲区中的消息在大于1000个的时候就会开始丢弃先前发布的消息。
      NodeHandle::advertise()返回一个ROS::Publisher对象，他有两个作用：
       1）他有一个publish（）成员函数可以让你在topic上发布消息；2）如果消息类型不对，他会拒绝
      */
     ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

     ros::Rate loop_rate(10);
     //ros::Rate对象可以允许你指定自循环的频率。他会追踪记录上一次调用Rate::sleep()后时间的流逝，并休眠直到一个频率周期的时间。

     /**
      * A count of how many messages we have sent. This is used to create
      * a unique string for each message.
      */
     int count = 0;
     bool begin = false;
     while (!begin)
     {
     /*
      roscpp 会默认生成一个 SIGINT 句柄，它负责处理 Ctrl-C 键盘操作——使得 ros::ok() 返回 false。
      如果下列条件之一发生，ros::ok() 返回false：
      列表项SIGINT 被触发 (Ctrl-C)
      列表项被另一同名节点踢出 ROS 网络
      列表项ros::shutdown() 被程序的另一部分调用
      节点中的所有 ros::NodeHandles 都已经被销毁 一旦 ros::ok() 返回 false, 所有的 ROS 调用都会失效。
     */
       /**
        * This is a message object. You stuff it with data, and then publish it.
        */
       std_msgs::String msg;
       ///home/csc105/Project/myrobot/build
       ///home/csc105/Project/myrobot/build/build_orbslam2_map home/csc105/Project/myrobot/config/top_setting.yaml
       std::stringstream ss;
       ss << "hello world " << count;

       msg.data = ss.str();
       //我们使用一个由msg file文件产生的消息自适应类在ROS网络中广播消息。现在我们使用标准的string消息，他只有一个数据成员“data".
       //当然，你也可以发布更为复杂的消息类型。
       ROS_INFO("%s", msg.data.c_str());
       begin = true;
       string dir_ = "/home/csc105/Project/myrobot/build/build_orbslam2_map /home/csc105/Project/myrobot/config/top_setting.yaml";
       char* dir = new char[dir_.length() + 1];
       strcpy(dir,dir_.c_str());
       system (dir);
       //ROS_INFO和其他类似的函数可以用来代替print/cout等函数。
       /**
        * The publish() function is how you send messages. The parameter
        * is the message object. The type of this object must agree with the type
        * given as a template parameter to the advertise<>() call, as was done
        * in the constructor above.
       */
      chatter_pub.publish(msg);
      //向所有订阅chatter话题的节点发送消息。
      ros::spinOnce();
      //这个例子不一定调用ros::spinOnce()，因为我们不接受回调。然而，如果你的程序里包含其他回调函数，最好在这里加上ros::spinOnce()这一句，
      //否则你的回调函数就永远也不会被调用了。
      loop_rate.sleep();
      //调用ros::Rate对象来休眠一段时间使得发布频率为10Hz/
      ++count;
    }


    return 0;
  }
/*
对上边的内容进行一下总结：

    @列表项初始化 ROS 系统
    @列表项在 ROS 网络内广播我们将要在 chatter 话题上发布 std_msgs/String 类型的消息
    @列表项以每秒 10 次的频率在 chatter 上发布消息

*/
