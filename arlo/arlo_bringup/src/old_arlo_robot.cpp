#include <signal.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>


// Global variables
ros::Subscriber _cmdvelSubscriber;
ros::Publisher _encoderPublisher;

int serial_port;

void customSigIntHandler(int sig)
{
	ROS_INFO("===Stopping Arlo robot===");
			
	// stop the robot by sending a zero velocity command
	// TODO ? on the serial com side?
	close(serial_port);	
	// shutdown ROS
	ros::shutdown();
}

void cmdCallback(const geometry_msgs::Twist::ConstPtr& velocityCmd)
{
	char* msg;
	// velocityCmd->linear.x contains velocity translation along x
	// velocityCmd->angular.z contains velocity translation around z
	std::cout << "speed= (" << velocityCmd->linear.x << ", " << velocityCmd->angular.z << ")" << std::endl;	
	// TODO
	//convert these 2 values and send them to the serial portYour local changes to the following files would be overwritten by merge:
	asprintf(&msg, "GO_,%.03lf,%.03lf\r",velocityCmd->linear.x,velocityCmd->angular.z);
	
	write(serial_port,msg,strlen(msg));
	
	// then receive sensor info back from the encoders via serail port
	// publish them on ros
	// <type of msg> encoderMsg;
	// fill with values
	// publish it
	//_encoderSubscriber.publish(encoderMsg);
	
}


int main(int argc, char** argv)
{

	//initiate serial port
	
	serial_port = open("/dev/ttyS0", O_RDWR);
	

	if (serial_port < 0)
	{
		printf("Error %i from open: %s\n", errno, strerror(errno));
	}
	else
		std::cout << "Serial port opened successfully!" <<std::endl;
	
	struct termios tty;
	
	if(tcgetattr(serial_port, &tty) !=  0) {
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	}
	else
		std::cout << "Structure termios created successfully!" <<std::endl;

	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_cflag |= CREAD;

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO;
	tty.c_lflag &= ~ECHOE;
	tty.c_lflag &= ~ECHONL;
	tty.c_lflag &= ~ISIG;

	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

	tty.c_oflag &= ~OPOST;
	tty.c_oflag &= ~ONLCR;

	tty.c_cc[VTIME] =10;
	tty.c_cc[VMIN] =0;

	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	if (tcsetattr(serial_port, TCSANOW, &tty) != 0){
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	}
	else
		std::cout << "Structure saved successfully!" << std::endl;
	
	// create a node called arlo_control
	ros::init(argc, argv, "arlo_robot", ros::init_options::NoSigintHandler);
	//ros::init(argc, argv, "autopilot");
	
	// create a node handle
	ros::NodeHandle nh;
	
	// override the default sigint handler (must be set after the first node handler is created)
	//signal(SIGINT, customSigIntHandler);
	
	// create a subscriber to cmd_vel topic
	_cmdvelSubscriber = nh.subscribe("cmd_vel", 1, cmdCallback);
	
	// create a publisher to broadcast LiDAR data
	_encoderPublisher = nh.advertise<std_msgs::String>("encoder", 1);
	
	ros::Rate loopRate(10);
	
	std::string read_buffer;
	char serial_buf;
	bool stringComplete = false;	
	bool endCom = false;
	
	ROS_INFO("Serial com launched...");
		
	while(ros::ok()){
				
		while (!endCom){
				
			int num_bytes = read(serial_port, &serial_buf, sizeof (serial_buf));
			if (num_bytes != 0)
			{
				read_buffer.push_back(serial_buf);
				//std::cout << "read_buffer = " << read_buffer << std::endl;
			
				if (serial_buf == '\r')
				{
					stringComplete = true;
					//std::cout << "stringComplete: " << read_buffer << std::endl;
				}
			}
			
			if (stringComplete)
			{
				//std::cout << "inside stringComplete : " << read_buffer << std::endl;
				if (read_buffer.find("SPD")!=std::string::npos){
					//std::cout << "SPD found: " <<read_buffer << std::endl;
					std::size_t posFirstComma = read_buffer.find(",");
					if (posFirstComma != std::string::npos)
					{
						std::size_t posSecondComma = read_buffer.find(",", posFirstComma+1);
						if (posSecondComma != std::string::npos)
						{
							//std::cout << "firstCommaPos = " << posFirstComma << std::endl;
							//std::cout << "secondCommaPos = " << posSecondComma << std::endl;
							
							std::string l_sLeftWheelSpeed = read_buffer.substr(posFirstComma+1, posSecondComma - posFirstComma-1);
							std::string l_sRightWheelSpeed = read_buffer.substr(posSecondComma+1, read_buffer.size()-posSecondComma-1);
							//std::cout << "left = " << l_sLeftWheelSpeed << std::endl;
							//std::cout << "right = " << l_sRightWheelSpeed << std::endl;
							int l_i32LeftWheelSpeed = stoi(l_sLeftWheelSpeed);
							int l_i32RightWheelSpeed = stoi(l_sRightWheelSpeed);
							double l_dLeftWheelSpeedKmh=l_i32LeftWheelSpeed;
							double l_dRightWheelSpeedKmh=l_i32RightWheelSpeed;
							std::cout << "(L, R) = (" << l_dLeftWheelSpeedKmh << ", " << l_dRightWheelSpeedKmh << ")" << std::endl;
						}
					}
										
				}

				if (read_buffer.find("END")!=std::string::npos){
					std::cout << "END: " << read_buffer << std::endl;
					endCom = true;
				}
				
				stringComplete = false;
				std_msgs::String encoderMsg;
				encoderMsg.data=read_buffer;
				_encoderPublisher.publish(encoderMsg);
				read_buffer.erase();
				
			}
			ros::spinOnce();
			loopRate.sleep();
		}
	}
		
	return 0;
}
