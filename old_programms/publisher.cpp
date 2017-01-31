#include "ros/ros.h"
#include <data_kinect/BodyArray.h>

#include <sstream>


int main(int argc, char **argv)
{
  //KinectPoints kpoints;
  //  kinect_msgs::BodyArray kpoints;

  ros::init(argc, argv, "n_publisher_points");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher chatter_pub  = nh.advertise<kinect_msgs::BodyArray>("t_kinectdata", 1000);
  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    
    
    kpoints.body.resize(2);

    kpoints.body[0].x=0.8;
    kpoints.body[0].y=-0.4;
    kpoints.body[0].z=0.1;

    kpoints.left_hand.data=0;
    kpoints.right_hand.data=0;

   
    kpoints.body[1].x=0.8;
    kpoints.body[1].y=0.4;
    kpoints.body[1].z=0.1;

    chatter_pub.publish(kpoints);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
