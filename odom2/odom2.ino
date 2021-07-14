#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

double x = 1.0;
double y = 0.0;
double theta = 1.57;

void setup(){
	nh.initNode();
	broadcaster.init(nh);
}

void loop(){	// drive in a circle
	double dx = 0.2;
	double dtheta = 0.18;

	x += cos(theta)*dx*0.1;
	y += sin(theta)*dx*0.1;
	theta += dtheta*0.1;
	
	if(theta > 3.14){
		theta=-3.14;
		// tf odom->base_link
	}
	
	t.header.frame_id = "odom";
	t.child_frame_id = "base_link";
	t.transform.translation.x = x;
	t.transform.translation.y = y;
	t.transform.rotation = tf::createQuaternionFromYaw(theta);
	t.header.stamp = nh.now();
	broadcaster.sendTransform(t);
	
	odom.header.stamp = nh.now();
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = tf::createQuaternionFromYaw(theta);

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x =0.0;// vx;
	odom.twist.twist.linear.y = 0.0;//vy;
	odom.twist.twist.angular.z = 0.0;//vth;

	//publish the message
	odom_pub.publish(&odom);

	//last_time = current_time;

	nh.spinOnce();	
	delay(10);
}
