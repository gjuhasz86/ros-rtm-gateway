#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <stdio.h>
#include <stdlib.h>

#include <sstream>

int main(int argc, char **argv) {
	ros::init(argc, argv, "talkerString");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatterString", 1000);
	int count = 0;
	while (ros::ok()) {
		std_msgs::String msg;

		std::string str;
		std::string pre = "Msg from ros #";

		char numstr[21];
		sprintf(numstr,"%d",count);
		str = pre + numstr;

		msg.data=str;
		std::cout << str << std::endl;

		ros::Rate loop_rate(1);
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();

		++count;
	}

	return 0;
}
