#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>

#include <math.h>

#define CELL_SIZE	0.08
#define SIZE		100

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  tf::TransformBroadcaster broadcaster;
  tf::Transform transform_1;
  tf::Quaternion q;
  
  sensor_msgs::JointState joint_state;
  
  transform_1.setOrigin( tf::Vector3( 0.0, 0.0, 0.0) );
  q.setRPY(0, 0, 0); 		  
  transform_1.setRotation(q);transform_1.setRotation(q);

  ros::Publisher joint_state_publisher = n.advertise<sensor_msgs::JointState>("joint_states",1000);

  ros::Rate loop_rate(50);

  std::cout << "Map to Odom Transformation set!" << std::endl;    

  while(n.ok()){

    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(4); //Pioneer has 4 joint state definitions. 
    joint_state.position.resize(4);

    joint_state.name[0] = "p3dx_back_right_wheel_joint";
    joint_state.position[0] = 0;

    joint_state.name[1] = "p3dx_back_left_wheel_joint";
    joint_state.position[1] = 0;

    joint_state.name[2] = "p3dx_front_left_wheel_joint";
    joint_state.position[2] = 0;

    joint_state.name[3] = "p3dx_front_right_wheel_joint";
    joint_state.position[3] = 0; 

    joint_state_publisher.publish(joint_state);

    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(transform_1), ros::Time::now(), "map", "odom"));
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(transform_1), ros::Time::now(), "base_link", "left_hub"));
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(transform_1), ros::Time::now(), "base_link", "left_wheel"));
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(transform_1), ros::Time::now(), "base_link", "right_hub"));
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(transform_1), ros::Time::now(), "base_link", "right_wheel"));
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(transform_1), ros::Time::now(), "base_link", "back_sonar"));
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(transform_1), ros::Time::now(), "base_link", "front_sonar"));
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(transform_1), ros::Time::now(), "base_link", "top_plate"));
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(transform_1), ros::Time::now(), "base_link", "caster_wheel"));
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(transform_1), ros::Time::now(), "base_link", "caster_hubcap"));
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(transform_1), ros::Time::now(), "base_link", "caster_swivel"));
    loop_rate.sleep();
  }
}
