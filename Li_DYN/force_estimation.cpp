#include <ros/ros.h>
#include <std_msgs/String.h>
#include <raven_2/raven_state.h>


#include "two_arm_dyn.h"
#include <time.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <string>

#define MAX_BUF 1024


//#define LOGGING
#define EULER_INT
//#define UPDATE_MODEL
//#define UPDATE_FREQ 20
//Integration steps in msec
double RK_STEP = 0.001;
double EULER_STEP = 0.001;

double DACs[3]; // will be used in two_arm_dyn.cpp
static state_type r_state = {0,0,0,0,0,0,0,0,0,0,0,0};

void ravenstate_callback(raven_2::raven_state msg){
	ROS_INFO("Raven State: [%f %f %f %f %f %f]", msg.jpos[0], msg.jpos[1], msg.jpos[2],
												msg.jpos[3], msg.jpos[4], msg.jpos[5]);
	ROS_INFO("joint encoder position: %f, %f, %f: ", msg.j_enc_pos[0], msg.j_enc_pos[1], msg.j_enc_pos[2]);
	ROS_INFO("Joint position: %f, %f, %f, %f, %f, %f: ", msg.jvel[0], msg.jvel[1], msg.jvel[2], msg.jvel[3], msg.jvel[4], msg.jvel[5]);
	for (int i=0; i<3; ++i) {
		DACs[i] = msg.dac_val[i];
		r_state[i] = msg.jpos[i];
		r_state[3+i] = msg.jvel[i];
		r_state[6+i] = msg.mpos[i];
		r_state[9+i] = msg.mvel[i];
	}
	//sys_dyn_gold_euler(r_state, EULER_STEP*1000);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "force_estimation");
	ros::NodeHandle n;
	ros::Subscriber ravenstate_sub = n.subscribe<raven_2::raven_state>("ravenstate",10, ravenstate_callback); 
	ros::spin();
	return 0;
}
