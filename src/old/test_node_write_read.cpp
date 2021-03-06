/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;



void enumerate_ports()
{
	vector<serial::PortInfo> devices_found = serial::list_ports();

	vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while( iter != devices_found.end() )
	{
		serial::PortInfo device = *iter++;

		printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );
	}
}

void print_usage()
{
  cerr << "Usage: test_serial {-e|<serial port address>} ";
  cerr << "<baudrate>" << endl;
}

int run(int argc, char **argv)
{

  if(argc < 2) {
	  print_usage();
    return 0;
  }

  // Argument 1 is the serial port or enumerate flag
  string port(argv[1]);

  if( port == "-e" ) {
	  enumerate_ports();
	  return 0;
  }
  else if( argc < 3 ) {
	  print_usage();
	  return 1;
  }

  // Argument 2 is the baudrate
  unsigned long baud = 0;
// #if defined(WIN32) && !defined(__MINGW32__)
//  sscanf_s(argv[2], "%lu", &baud);
//#else
  sscanf(argv[2], "%lu", &baud);
//#endif

  ros::NodeHandle n;
  tf::TransformBroadcaster transform_broadcaster;
  tf::TransformListener listener;
  ros::Publisher odom_pub;
  ros::Rate rate(1);
  odom_pub = n.advertise<nav_msgs::Odometry>("/odometry/indoor_rtk", 1);

  // port, baudrate, timeout in milliseconds
  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;


  // declare some stuff
  int dot_count = 0;
  int current_coordinate = 0;
  string letter; // = my_serial.read(1);
  string result = "";


  // enter loop
  while (ros::ok()) {
	ros::Time start_time = ros::Time::now();
	double coordinates[3];
	bool writing = false;
	my_serial.flushInput();
	my_serial.write("new\n");

	//while(my_serial.available()<1)
	//{
	//	cout << "waiting" << endl;
	//}

	// read next byte
	letter = my_serial.read(1);

	//debug
	ros::Time timeout = ros::Time::now();


	//do until new line
	while(letter != "\n" && letter != "" && ros::ok())
	{

		letter = my_serial.read(1);

		if (letter == ".")
		{
			dot_count ++;
			//writing = false;
		}
		else if (letter == " ")
		{
			if (writing)
			{

				std::stringstream convert(result);
				convert >> coordinates[current_coordinate];
				//cout << "coordinate " << current_coordinate << ": " << coordinates[current_coordinate] << endl;
				result = "";
				current_coordinate++;
			}

			writing = false;

		}
		else
		{
			if (dot_count == 2)
			{
				writing = true;
				//cout << current_coordinate << endl;
			}

			if (writing)
			{
				result.append(letter);

			}

			dot_count = 0;
		}

		// if we have all three coordinates, publish them to ROS and reset counter
		if (current_coordinate == 3)
		{
		  cout << "coordinates received: [" << coordinates[0] << "," << coordinates[1] << "," << coordinates[2] << "]" << endl;
		  tf::Transform transform_prism_in_ENU;
		  transform_prism_in_ENU.setOrigin( tf::Vector3(coordinates[0]/1000, coordinates[1]/1000, coordinates[2]/1000));
		  tf::Quaternion quat;
		  quat.setRPY(0, 0, 0);
		  transform_prism_in_ENU.setRotation(quat);
		  //tf::StampedTransform transform_stamped(transform, ros::Time::now(), "prism", "ENU");

		  tf::StampedTransform transform_base_link_in_prism;
		  ros::Time now = ros::Time::now();
		  try{

			listener.waitForTransform("base_link", "prism", now, ros::Duration(3.0));
		    listener.lookupTransform("base_link", "prism",
		  						     now, transform_base_link_in_prism);
		  }
		  catch (tf::TransformException ex){
		    ROS_ERROR("%s",ex.what());
		    ros::Duration(1.0).sleep();
		  }
		  tf::Transform transform_ENU_base_link;
		  transform_ENU_base_link =   transform_prism_in_ENU * transform_base_link_in_prism.inverse();

		  geometry_msgs::Vector3Stamped prism_position;
		  //geometry_msgs::Vector3Stamped base_link_position;
		  prism_position.header.stamp = ros::Time::now();
		  prism_position.header.frame_id = "ENU";
		  prism_position.vector.x = coordinates[0]/1000;
		  prism_position.vector.y = coordinates[1]/1000;
		  prism_position.vector.z = coordinates[2]/1000;



		  //geometry_msgs::Vector3Stamped base_link_position =  prism_position * base_link_position; //transform_prism_base_link

		  //listener.waitForTransform("ENU", "prism", ros::Time::now(), ros::Duration(3.0));
		  //listener.transformVector("base_link", prism_position, base_link_position);


		  nav_msgs::Odometry odom_msg;
		  odom_msg.header.stamp = now;
		  odom_msg.header.frame_id = "ENU";
		  odom_msg.child_frame_id = "base_link";
		  odom_msg.pose.pose.position.x =   transform_ENU_base_link.getOrigin().getX(); //coordinates[0]/1000;
		  odom_msg.pose.pose.position.y = transform_ENU_base_link.getOrigin().getY();//coordinates[1]/1000;
		  odom_msg.pose.pose.position.z = transform_ENU_base_link.getOrigin().getZ();//coordinates[2]/1000;
		  boost::array<double, 36ul> cov = {   0.2,0,0,0,0,0,
		                                       0,0.2,0,0,0,0,
		                                       0,0,0.2,0,0,0,
		                                       0,0,0,0.2,0,0,
		                                       0,0,0,0,0.2,0,
		                                       0,0,0,0,0,0.2};
		  odom_msg.pose.covariance = cov;
		  odom_pub.publish(odom_msg);



		  current_coordinate = 0;
		}



	}
	rate.sleep();
	ros::Duration duration_timeout = timeout - start_time;
	ros::Duration duration = ros::Time::now() - start_time;
	cout << "timeout: " << duration_timeout.toSec() << endl;
	cout << "time: " << duration.toSec() << endl;


    
  }


  return 0;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_test_node");


  try {
    return run(argc, argv);
  } catch (exception &e) {
    cerr << "Unhandled Exception: " << e.what() << endl;
  }
}
