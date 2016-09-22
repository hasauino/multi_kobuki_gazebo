#include <ros/ros.h>
int main( int argc, char** argv )
{
ros::init( argc, argv, “test1” );
ros::NodeHandle node;
ros::Rate rate(10);
while ( ros::ok() )
{
ROS_INFO( “Hello World!” );
rate.sleep();
}
return 0;
}
