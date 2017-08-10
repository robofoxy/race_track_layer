#ifndef ROBOT_TYPE_H_
#define ROBOT_TYPE_H_

#include <race_track_layer/race_track_layer.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class RobotType {
	public:
		RobotType(std::string nm);
		geometry_msgs::PolygonStamped getFootprint() const;
		void footprint_collector(const geometry_msgs::PolygonStamped::ConstPtr&);
		std::string getName() const;
		double getYaw() const;
		double getSpeed() const;
		geometry_msgs::Point getMidPoint();
		double getMaxWeight() const;
		
	private:
		std::string name;
		ros::NodeHandle n;
		ros::Subscriber sub, sub_yaw, sub_speed;
		geometry_msgs::PolygonStamped footprint;
		float x, y, z, w;
		double weight;
		double angle, speed;
		void setSpeed(const geometry_msgs::Twist::ConstPtr& msg);
		void setYaw(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
		void setMaxWeight();
};



RobotType::RobotType(std::string nm){
	speed = 0;
	angle = 0;
	x = 0;
	y = 0;
	z = 0;
	w = 0;
	name = nm;
	footprint.header.frame_id = name;
	std::string subscriber_link = "/" + name + "/move_base_node/global_costmap/footprint";
	sub = n.subscribe(subscriber_link, 1000, &RobotType::footprint_collector, this);
	sub_yaw = n.subscribe("/" + name + "/amcl_pose", 1000, &RobotType::setYaw, this);
	sub_speed = n.subscribe("/" + name + "/cmd_vel", 1000, &RobotType::setSpeed, this);
}

void RobotType::footprint_collector(const geometry_msgs::PolygonStamped::ConstPtr& msg){
	footprint = *msg;
}

geometry_msgs::PolygonStamped RobotType::getFootprint() const{
	return footprint;
}

std::string RobotType::getName() const{
	return name;
}

void RobotType::setSpeed(const geometry_msgs::Twist::ConstPtr& msg){
	speed = msg->linear.x;
}

void RobotType::setYaw(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	x = msg->pose.pose.orientation.x;
	y = msg->pose.pose.orientation.y;
	z = msg->pose.pose.orientation.z;
	w = msg->pose.pose.orientation.w;
	
	tf::Quaternion q(x, y, z, w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	
	angle = yaw;
}

double RobotType::getYaw() const{
	return angle;
}

double RobotType::getSpeed() const{
	return speed;
}

geometry_msgs::Point RobotType::getMidPoint() {
	int size = footprint.polygon.points.size();
	geometry_msgs::Point mid;
	float x, y;
	
	for(int i = 0; i < size; i++){
		x += footprint.polygon.points[i].x;
		y += footprint.polygon.points[i].y;
	}
	
	mid.x = x / size;
	mid.y = y / size;
	
	setMaxWeight();
	
	return mid;
}


void RobotType::setMaxWeight(){
	int size = footprint.polygon.points.size();
	float max = -1;
	
	for(int i = 0; i < size; i++){
		float tempX = footprint.polygon.points[i].x;
		float tempY = footprint.polygon.points[i].y;
		 
		for(int j = i+1; j < size; j++){
			float temp2X = footprint.polygon.points[j].x;
			float temp2Y = footprint.polygon.points[j].y;
			
			float res = sqrt(pow(tempX - temp2X, 2) + pow(tempY - temp2Y, 2));
			if(res > max) max = res; 
		}
	}
	
	weight = max;
}


double RobotType::getMaxWeight() const{
	return weight;
}



#endif
