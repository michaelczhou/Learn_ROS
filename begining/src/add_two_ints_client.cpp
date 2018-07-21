#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
    {
      ros::init(argc, argv, "add_two_ints_client");
     //初始化ROS。定义客户端名称。
      if (argc != 3)
      {
        ROS_INFO("usage: add_two_ints_client X Y");
        return 1;
      }

      ros::NodeHandle n;
      ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
      //为add_two_ints service创建一个client。ros::ServiceClient对象会用来调用service。
       beginner_tutorials::AddTwoInts srv;
       srv.request.a = atoll(argv[1]);
       srv.request.b = atoll(argv[2]);
       //实例化一个由ROS编译系统自动生成的service类，并给其request成员赋值。一个service类包含两个成员request和response。
	//同时也包括两个类定义Request和Response。
       if (client.call(srv))
	//调用service。由于service的调用是模态过程（调用的时候占用进程组织其他代码的执行），一旦调用完成，将返回调用结果。
        //如果调用成功，call()函数将返回true，srv.reponse里面的值将是合法的值。反之。
       {
         ROS_INFO("Sum: %ld", (long int)srv.response.sum);
       }
       else
       {
        ROS_ERROR("Failed to call service add_two_ints");
         return 1;
       }

      return 0;
     }
