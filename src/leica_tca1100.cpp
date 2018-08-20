/***
 * This node requests positions from a Leica tca1100 connected to a serial port.
 * It requires the port name and baudrate as arguments. Author: Lars Grimstad,
 * NMBU (lars.grimstad@nmbu.no)
 */

// ros includes
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "serial/serial.h"

class LeicaTCA1100 {
 public:
  LeicaTCA1100(std::string dev, unsigned long baud);
  void publishPosition();

 private:
  int setupSerial(std::string port, unsigned long baud);
  serial::Serial serial_;
  ros::NodeHandle n_;
  tf::TransformBroadcaster transform_broadcaster_;
  tf::TransformListener tf_listener_;
  ros::Publisher odom_prism_pub_;
  ros::Publisher odom_base_link_pub_, groundtruth_pub;

  // int update_rate_;
};

LeicaTCA1100::LeicaTCA1100(std::string port, unsigned long baud) {
  // update_rate_ = 1;
  /// ros::Rate rate(update_rate_);
  odom_prism_pub_ =
      n_.advertise<nav_msgs::Odometry>("/odometry/leica_prism", 1);
  odom_base_link_pub_ =
      n_.advertise<nav_msgs::Odometry>("/odometry/leica_base_link", 1);
  groundtruth_pub = n_.advertise<nav_msgs::Odometry>("/groundtruth", 1);
  setupSerial(port, baud);
}

int LeicaTCA1100::setupSerial(std::string port, unsigned long baud) {
  serial_.setPort(port);
  serial_.setBaudrate(baud);
  serial_.setTimeout(2000, 2000, 2000, 2000, 2000);
  serial_.open();

  std::cout << "Is the serial port open?";
  if (serial_.isOpen())
    std::cout << " Yes." << std::endl;
  else
    std::cout << " No." << std::endl;

  return 0;
}

void LeicaTCA1100::publishPosition() {
  // declare some stuff
  std::string delimiter_1 = ":";
  std::string delimiter_2 = ",";
  size_t pos = 0;

  // for storing coordinates
  double coordinates[8];

  serial_.flush();
  serial_.write("%R1Q,2082:500,1\r\n");

  if (serial_.waitReadable()) {
    // read response from Leica
    std::string input = serial_.readline(1000, "\r\n");

    // debug
    // string input =
    // "%R1P,0,0:1285,4.171101398787006,-3.298025825649888,-0.294882080940466,95182,4.171097966262274,-3.298030166851005,-0.294924192611223,98360\r\n";

    // get time to be used in time stamp
    ros::Time input_time = ros::Time::now();

    // remove carriage return and newline
    input.erase(input.length() - 2, input.length());

    // Get response code from input
    std::string response_msg;
    pos = input.find(delimiter_1);
    response_msg = input.substr(0, pos);
    input.erase(0, pos + delimiter_1.length());
    pos = input.find(delimiter_2);
    response_msg.append(input.substr(0, pos));
    input.erase(0, pos + delimiter_2.length());
    ROS_INFO("Leica responded: %s", response_msg.c_str());

    // Get coordinates from input
    std::stringstream input_stream(input);
    std::string token;

    int counter = 0;
    while (std::getline(input_stream, token, ',')) {
      std::stringstream convert(token);
      convert >> coordinates[counter];
      counter++;
    }

    /*
        // Get coordinates from input
        while (pos != std::string::npos)
        {
          pos = input.find(delimiter_2);
          std::stringstream convert(input.substr(0,pos));
          convert >> coordinates[counter];
          input.erase(0, pos + delimiter_2.length());
          counter++;
        }

    */

    // debug
    std::cout.precision(17);
    for (int i = 0; i < 8; i++) std::cout << coordinates[i] << ", ";
    std::cout << std::endl;

    // create prism in ENU transform
    tf::Transform transform_ENU_prism;
    transform_ENU_prism.setOrigin(
        tf::Vector3(coordinates[0], coordinates[1], coordinates[2]));
    tf::Quaternion quat;
    quat.setRPY(0, 0, 0);
    transform_ENU_prism.setRotation(quat);

    // ask ros for prism in base_link transform
    tf::StampedTransform transform_base_link_prism;

    try {
      tf_listener_.waitForTransform("base_link", "prism", ros::Time(0),
                                    ros::Duration(3.0));
      tf_listener_.lookupTransform("base_link", "prism", ros::Time(0),
                                   transform_base_link_prism);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    // calculate ENU in base_link transform
    tf::Transform transform_base_link_ENU;
    transform_base_link_ENU =
        transform_base_link_prism * transform_ENU_prism.inverse();

    tf::Transform transform_ENU_base_link = transform_base_link_ENU.inverse();
    tf::StampedTransform transform_ENU_base_link_stamped(
        transform_ENU_base_link, input_time, "ENU", "base_link");

    // broadcast tf directly
    transform_broadcaster_.sendTransform(transform_ENU_base_link_stamped);

    // Convert Leica's Northing/Easting coordinate to ENU REP-105
    tf::Quaternion Rotation_q;
    // with current septup, need to rotate +pi/2 around yaw
    Rotation_q.setRPY(0, 0, 1.57079632679);
    tf::Transform tf_Leica_map;
    tf_Leica_map.setOrigin(transform_ENU_prism.getOrigin());
    tf_Leica_map.setRotation(transform_ENU_prism.getRotation() * Rotation_q);
    tf::StampedTransform tf_Leica_map_stamped(tf_Leica_map, input_time, "ENU",
                                              "groundtruth");
    transform_broadcaster_.sendTransform(tf_Leica_map_stamped);

    nav_msgs::Odometry groundtruth_msg;
    groundtruth_msg.header.stamp = input_time;
    groundtruth_msg.header.frame_id = "map";
    groundtruth_msg.child_frame_id = "prism";
    groundtruth_msg.pose.pose.position.x = tf_Leica_map.getOrigin().getY();
    groundtruth_msg.pose.pose.position.y = -tf_Leica_map.getOrigin().getX();
    groundtruth_msg.pose.pose.position.z = tf_Leica_map.getOrigin().getZ();
    groundtruth_pub.publish(groundtruth_msg);

    // create and populate prism odometry_msg
    nav_msgs::Odometry prism_odom_msg;
    prism_odom_msg.header.stamp = input_time;
    prism_odom_msg.header.frame_id = "ENU";
    prism_odom_msg.child_frame_id = "base_link";
    prism_odom_msg.pose.pose.position.x =
        transform_ENU_prism.getOrigin().getX();
    prism_odom_msg.pose.pose.position.y =
        transform_ENU_prism.getOrigin().getY();
    prism_odom_msg.pose.pose.position.z =
        transform_ENU_prism.getOrigin().getZ();
    boost::array<double, 36ul> cov_prism = {
        0.2, 0, 0, 0,   0, 0, 0, 0.2, 0, 0, 0,   0, 0, 0, 0.2, 0, 0, 0,
        0,   0, 0, 0.2, 0, 0, 0, 0,   0, 0, 0.2, 0, 0, 0, 0,   0, 0, 0.2};
    prism_odom_msg.pose.covariance = cov_prism;

    // publish odometry
    odom_prism_pub_.publish(prism_odom_msg);

    // create and populate base_link odometry_msg
    nav_msgs::Odometry base_link_odom_msg;
    base_link_odom_msg.header.stamp = input_time;
    base_link_odom_msg.header.frame_id = "ENU";
    base_link_odom_msg.child_frame_id = "base_link";
    base_link_odom_msg.pose.pose.position.x =
        transform_ENU_base_link.getOrigin().getX();
    base_link_odom_msg.pose.pose.position.y =
        transform_ENU_base_link.getOrigin().getY();
    base_link_odom_msg.pose.pose.position.z =
        transform_ENU_base_link.getOrigin().getZ();
    boost::array<double, 36ul> cov_base_link = {
        0.2, 0, 0, 0,   0, 0, 0, 0.2, 0, 0, 0,   0, 0, 0, 0.2, 0, 0, 0,
        0,   0, 0, 0.2, 0, 0, 0, 0,   0, 0, 0.2, 0, 0, 0, 0,   0, 0, 0.2};
    base_link_odom_msg.pose.covariance = cov_base_link;

    // publish odometry
    odom_base_link_pub_.publish(base_link_odom_msg);

  } else {
    ROS_WARN("Leica is not responding");
  }
}

void enumeratePorts() {
  std::vector<serial::PortInfo> devices_found = serial::list_ports();

  std::vector<serial::PortInfo>::iterator iter = devices_found.begin();

  while (iter != devices_found.end()) {
    serial::PortInfo device = *iter++;

    printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
           device.hardware_id.c_str());
  }
}

void printUsage() {
  ROS_WARN("Usage: test_serial {-e|<serial port address>} <baudrate>");
}

int main(int argc, char **argv) {
  // initiate ros node
  ros::init(argc, argv, "serial_test_node");

  // read arguments
  if (argc < 2) {
    printUsage();
    return 1;
  }

  // Argument 1 is the serial port or enumerate flag
  std::string port(argv[1]);

  if (port == "-e") {
    enumeratePorts();
    return 0;
  } else if (argc < 3) {
    printUsage();
    return 1;
  }

  // Argument 2 is the baudrate
  unsigned long baud = 0;
#if defined(WIN32) && !defined(__MINGW32__)
  sscanf_s(argv[2], "%lu", &baud);
#else
  sscanf(argv[2], "%lu", &baud);
#endif

  // create LeicaTCA1100 object
  LeicaTCA1100 leica_tca1100(port, baud);

  // set rate for position query
  ros::Rate rate(1);

  // loop forever
  while (ros::ok()) {
    leica_tca1100.publishPosition();
    rate.sleep();
  }

  return 0;
}
