#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

ros::Publisher pub;

void chatterCallback(sensor_msgs::PointCloud2 pc)
{
  int width =  pc.width;
  int height = pc.height;

  std::list<int> list;

  for(int i = 0; i < pc.width; i++) 
  {
    int array_position = i * pc.point_step;

    float x = 0.0;
    float y = 0.0;
    float z = 0.0;

    memcpy(&x, &pc.data[array_position + pc.fields[0].offset], sizeof(float));
    memcpy(&y, &pc.data[array_position + pc.fields[1].offset], sizeof(float));
    memcpy(&z, &pc.data[array_position + pc.fields[2].offset], sizeof(float));

    if(std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2)) < 1) 
    {
      //list.insert(list.end(), pc.data + array_position, pc.data + array_position + pc.point_step)
    }
  }

  ROS_INFO("The original size was: %i. The new size will be: %i", pc.data.size(), list.size());

  std::string s = std::to_string(pc.point_step);
  //ROS_INFO("Step: %i. Bigend: %d. Fields: %i. Offset of fields[0]: %i. Datatype: %i. H*W: %i*%i. Row size: %i. Data size: %i", pc->point_step, pc->is_bigendian, pc->fields.size(), pc->fields[3].offset, pc->fields[3].datatype, pc->height, pc->width, pc->row_step, pc->data.size());
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/camera/depth/color/filtered_points", 1, chatterCallback);
                  pub = n.advertise<sensor_msgs::PointCloud2>("camera/depth/color/filters", 1);
  //ros::Rate loop_rate(10); //we will test without at first, just publishing something whenever we receive it on subscription

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}