#include <signal.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// Global variables
float _fps = 10.0f; // Hz
float _transVelocity = 0.2f; // m/s
float _rotVelocity = 0.5f; // rad/s
int _angleMax = 30; // deg
int _nbRangeValues = 360;
float _distanceThreshold = 0.3f; // m
bool _stop = false;
ros::Publisher _cmdvelPublisher;

void customSigIntHandler(int sig)
{
	ROS_INFO("Autopilot stopped...");
			
	// stop the robot by sending a zero velocity command
	geometry_msgs::Twist msg;
	msg.linear.x = 0.0;
	msg.angular.z = 0.0;
	_cmdvelPublisher.publish(msg);
	
	// shutdown ROS
	ros::shutdown();
}

bool checkForCollision(float sensedDistance, float distanceThreshold)
{
	// check if the value is valid (i.e. != 0)
	if (sensedDistance != 0.0f)
	{
		if (sensedDistance < distanceThreshold)
			_stop = true;
		else
			_stop = false;
	}
	return _stop; 	
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidarData)
{
	// loop over the angle range [0 _angleMax] and [(_nbRangeValues-1)-_angleMax (_nbRangeValues-1)-]
	for (int l_angle = 0; l_angle < _angleMax; l_angle++)
	{
		if (checkForCollision(lidarData->ranges[l_angle] , _distanceThreshold))
			return;
		
		if (checkForCollision(lidarData->ranges[(_nbRangeValues-1) - l_angle]  , _distanceThreshold))
			return;		
	}
}


int main(int argc, char** argv)
{
	// create a node called autopilot
	ros::init(argc, argv, "autopilot", ros::init_options::NoSigintHandler);
	//ros::init(argc, argv, "autopilot");
	
	// create a node handle
	ros::NodeHandle nh;
	
	// override the default sigint handler (must be set after the first node handler is created)
	signal(SIGINT, customSigIntHandler);
	
	// create a publisher to cmd_vel topic
	_cmdvelPublisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	// create a subscriber to scan topic
	ros::Subscriber lidarSubscriber = nh.subscribe("scan", 1, lidarCallback);
	
	// create a loop rate
	ros::Rate loopRate(_fps);
	
	// create a Twist message
	geometry_msgs::Twist msg;
	
	ROS_INFO("Autopilot launched...");
	
	// loop until Ctrl+C is pressed or ROS connectivity issues
	while(ros::ok())
	{
		// fill the message depending on the condition (stop or go)
		if (_stop)
		{
			// spin around z
			msg.linear.x = 0.0;
			msg.angular.z = _rotVelocity;
		}
		else
		{
			// move forward along x
			msg.linear.x = _transVelocity;
			msg.angular.z = 0.0;
		}
		
		// publish the Twist message to the cmd_vel topic
		_cmdvelPublisher.publish(msg);
		
		// spin once to let the process handle callback ad key stroke
		ros::spinOnce();
		
		// sleep the right amout of time to comply with _fps 
		loopRate.sleep();
	}
	
	return 0;
}