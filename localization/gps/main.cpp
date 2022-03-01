#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nmea_msgs/Sentence.h>

#include <sstream>
#include <fstream>
#include <sys/time.h>
#include <unistd.h>
#include "serial_wrap.h"
#include <time.h>

using namespace std;


long getTimestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec*1000 + tv.tv_usec/1000);
}

string makeFileName()
{
	char buff[128] = {0};
	time_t tt;
	struct tm tr = {0};
	time(&tt);
	localtime_r(&tt, &tr);
	snprintf(buff, sizeof(buff),
		"GPS_DATA_%02d%02d%"
		"02d%02d%02d.txt", 
		tr.tm_mon + 1, tr.tm_mday, 
		tr.tm_hour, tr.tm_min, tr.tm_sec);
	
	string filename = buff;
	return filename;
}

static void generateMsg(nmea_msgs::Sentence& msg, string& sentence)
{
	static int seq = 1;
	
    msg.header.seq = seq++;
    msg.header.stamp = ros::Time::now();
    msg.sentence = sentence;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "gps_pub");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle nh;
	ros::NodeHandle private_nh_("~");
	/**
	 * The advertise() function is how you tell ROS that you want to
	 * publish on a given topic name. This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing. After this advertise() call is made, the master
	 * node will notify anyone who is trying to subscribe to this topic name,
	 * and they will in turn negotiate a peer-to-peer connection with this
	 * node.  advertise() returns a Publisher object which allows you to
	 * publish messages on that topic through a call to publish().  Once
	 * all copies of the returned Publisher object are destroyed, the topic
	 * will be automatically unadvertised.
	 *
	 * The second parameter to advertise() is the size of the message queue
	 * used for publishing messages.  If messages are published more quickly
	 * than we can send them, the number here specifies how many messages to
	 * buffer up before throwing some away.
	 */
	ros::Publisher pub = nh.advertise<nmea_msgs::Sentence>("nmea_sentence", 1000);
	
	string port = "/dev/ttyS0";
	private_nh_.getParam("port", port);
	ROS_INFO_STREAM("Configured Serial Port:" << port);

	if ( port.size() == 0)
	{
		ROS_FATAL_STREAM("Serial Port not configured.");
		return -1;
	}

        SerialWrap serial(port, 115200, 8, 1, 0);
        if (false == serial.openSerial())
	{
		ROS_FATAL_STREAM("Open GPS COM Failed! Maybe need to to sudo chmod 666 /dev/ttyS*...");
		return -1;
	}	
	else
	{
		ROS_INFO("open serial successful...");
	}
	//ofstream outFile;
	//string fileName = makeFileName();	
	//outFile.open(fileName.c_str(), ios::out);
	
	string gps_msg;
	while (ros::ok())
	{
		gps_msg.clear();
		if (serial.readMsg(gps_msg) < 0)
			continue;

		if (gps_msg.size() < 32)
		{
			printf("msg too short...");
			continue;
		}	
		string msgtype = gps_msg.substr(0,6);
		if ( msgtype == "$GPFPD" || msgtype == "$GTIMU")
		{
			nmea_msgs::Sentence pub_msg;
			generateMsg(pub_msg, gps_msg);
			pub.publish(pub_msg);
	//		printf("gps:%s\n", gps_msg.c_str());
			//outFile << gps_msg << endl;
		}
		else
		{
			printf("unexpected msg!!!");
		}
	}

	return 0;
}
