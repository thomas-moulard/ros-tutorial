#include <ros/ros.h>

#include <turtlesim/Velocity.h>

int
main (int argc, char* argv[])
{
  ros::init (argc, argv, "my_controller");

  ros::NodeHandle nodeHandle;

  ros::Rate rate (1000);
  
  ros::Publisher pub = nodeHandle.advertise<turtlesim::Velocity>
    ("/turtle1/command_velocity", 1);

  turtlesim::Velocity vel;
  
  while (ros::ok ())
    {
      vel.linear = 1.;
      vel.angular = 1.;
      pub.publish (vel);
      ros::spinOnce ();
      rate.sleep ();
    }
  
  return 0;
}
