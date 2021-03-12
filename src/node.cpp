#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_pkg/Laser3d.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <gazebo/msgs/laserscan_stamped.pb.h>

#include <iostream>

void convertGazeboToROS (ConstLaserScanStampedPtr& _msg, laser_pkg::Laser3d& ros_msg) {
    std_msgs::String frame;
    frame.data = _msg->scan ().frame ();
    ros_msg.frame = frame;
    geometry_msgs::Pose world_pose;
    world_pose.position.x = _msg->scan ().world_pose().position().x();
    world_pose.position.y = _msg->scan ().world_pose().position().y();
    world_pose.position.z = _msg->scan ().world_pose().position().z();

    world_pose.orientation.x = _msg->scan ().world_pose().orientation().x();
    world_pose.orientation.y = _msg->scan ().world_pose().orientation().y();
    world_pose.orientation.z = _msg->scan ().world_pose().orientation().z();
    world_pose.orientation.w = _msg->scan ().world_pose().orientation().w();

    ros_msg.world_pose = world_pose;

    ros_msg.angle_min = _msg->scan ().angle_min ();
    ros_msg.angle_max = _msg->scan ().angle_max ();
    ros_msg.angle_step = _msg->scan ().angle_step ();
    ros_msg.range_min = _msg->scan ().range_min ();
    ros_msg.range_max = _msg->scan ().range_max ();
    ros_msg.count = _msg->scan ().count ();
    ros_msg.vertical_angle_min = _msg->scan ().vertical_angle_min ();
    ros_msg.vertical_angle_max = _msg->scan ().vertical_angle_max ();
    ros_msg.vertical_angle_step = _msg->scan ().vertical_angle_step ();
    ros_msg.vertical_count = _msg->scan ().vertical_count ();

    int ranges_size = _msg->scan ().ranges_size ();
    int intensities_size = _msg->scan ().intensities_size ();

    ros_msg.ranges.clear ();
    ros_msg.intensities.clear ();

    for (int i = 0; i < ranges_size; i++) {
        ros_msg.ranges.push_back (_msg->scan ().ranges (i));
    }

    for (int i = 0; i < intensities_size; i++) {
        ros_msg.intensities.push_back (_msg->scan ().intensities (i));
    }
}
laser_pkg::Laser3d ros_msg;
void laser_cb (ConstLaserScanStampedPtr &_msg) {
    convertGazeboToROS (_msg, ros_msg);
}

int main(int _argc, char **_argv) {

    ros::init(_argc, _argv, "laser_node");
    ros::NodeHandle nh;

    // Load gazebo
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    gazebo::transport::SubscriberPtr laser_sub = node->Subscribe("/gazebo/default/hokuyo/link/laser/scan", laser_cb);
    ros::Publisher pub = nh.advertise<laser_pkg::Laser3d>("/ros/laser/scan", 10, true);

    ros::Rate rate (10);

    // Busy wait loop...replace with your own code as needed.
    while (ros::ok ()) {
        pub.publish (ros_msg);
        rate.sleep ();
        ros::spinOnce ();
    }

    // Make sure to shut everything down.
    gazebo::client::shutdown();
    ros::spin ();
    return 0;
}
