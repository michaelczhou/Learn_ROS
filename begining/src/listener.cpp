#include "ros/ros.h"
#include "std_msgs/String.h"

   /**
    * This tutorial demonstrates simple receipt of messages over the ROS system.
    */
   void chatterCallback(const std_msgs::String::ConstPtr& msg)
   //&表引用，即std_msgs::String::ConstPtr的别名为 msg
   {
     ROS_INFO("I heard: [%s]", msg->data.c_str());
   }
   //这是一个回调函数，当接收到chatter话题的时候就会被调用。消息是以boost shared_ptr指针的形式传输，这就意味着你可以存储它而又不需要复制数据。
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
     ros::init(argc, argv, "listener");
     //初始化ROS
     /**
      * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
      * NodeHandle destructed will close down the node.
      */
     ros::NodeHandle n;
     //创建句柄。
     /**
      * The subscribe() call is how you tell ROS that you want to receive messages
      * on a given topic.  This invokes a call to the ROS
      * master node, which keeps a registry of who is publishing and who
      * is subscribing.  Messages are passed to a callback function, here
      * called chatterCallback.  subscribe() returns a Subscriber object that you
      * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
      * object go out of scope, this callback will automatically be unsubscribed from
      * this topic.
      *告诉master我们要订阅chatter话题上的消息。当有消息发布到这个话题上时，ROS就会调用chatterCallback()函数。
     第二个参数是队列大小，以防我们处理消息的速度不够快，当缓存达到1000条消息后，再有新的消息到来就将开始丢弃先前接收的消息，
     NodeHandle::subscribe()返回ros::sunscriber对象，你必须让它处于活动状态直到你不想订阅该消息。当这个对象销毁时，它将自动退订chatter话题的消息
      * The second parameter to the subscribe() function is the size of the message
      * queue.  If messages are arriving faster than they are being processed, this
      * is the number of messages that will be buffered up before beginning to throw
      * away the oldest ones.
    有各种不同的NodeHandle::subcribe()函数，允许你指定类的成员函数，甚至Boost.Function对象可以调用任何数据类型。roscpp overview have more details
      */
     ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

     /**
      * ros::spin() will enter a loop, pumping callbacks.  With this version, all
      * callbacks will be called from within this thread (the main one).  ros::spin()
      * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
      ros::spin()进入自循环，可以尽可能的调用消息回调函数。如果没有消息到达，他不会占用很多CPU，所以不用担心。
     一旦ros::ok()返回false，ros::spin()就会立刻跳出自循环。这有可能是ros::shutdown()被调用，也可能是按下ctrl+c,似的master告诉节点要终止运行。人为关闭
     可参考roscpp_tutorials package里的一些demo应用。 
     */
     ros::spin();

     return 0;
   }
/*
下边，我们来总结一下:

    @列表项初始化ROS系统
    @列表项订阅 chatter 话题
    @列表项进入自循环，等待消息的到达
    @列表项当消息到达，调用 chatterCallback() 函数

*/
