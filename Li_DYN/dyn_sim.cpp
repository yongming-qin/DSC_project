#include <ros/ros.h>
#include <std_msgs/String.h>
//#include <raven_2/raven_state.h>
#include <raven_2/input_dyn_sim.h>
#include <raven_2/output_dyn_sim.h>


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

bool Initialized = false;
long iter_gold = 0; // number of iteration for each arm
long iter_green = 0;
double DACs[3]; // will be used in two_arm_dyn.cpp
static state_type r_state = {0,0,0,0,0,0,0,0,0,0,0,0};

void input_callback(raven_2::input_dyn_sim msg) {
	//ROS_INFO("Raven State: [%f %f %f %f %f %f]", msg.jpos[0], msg.jpos[1], msg.jpos[2],msg.jpos[3], msg.jpos[4], msg.jpos[5]);
	//ROS_INFO("joint encoder position: %f, %f, %f: ", msg.j_enc_pos[0], msg.j_enc_pos[1], msg.j_enc_pos[2]);
	ROS_INFO("Joint position: %d, %d, %d, %d, %d, %d: ", msg.dac_val[0], msg.dac_val[1], msg.dac_val[2], msg.dac_val[3], msg.dac_val[4], msg.dac_val[5]);
	ROS_INFO("Joint position: %f, %f, %f, %f, %f, %f: ", msg.mpos[0], msg.mpos[1], msg.mpos[2], msg.mpos[3], msg.mpos[4], msg.mpos[5]);
	ROS_INFO("Joint position: %f, %f, %f, %f, %f, %f: ", msg.mvel[0], msg.mvel[1], msg.mvel[2], msg.mvel[3], msg.mvel[4], msg.mvel[5]);
	//
	static double mpos[6],mvel[6], jpos, jvel;
	if Initialized { // only update DAC
		for (int i=0; i<3; ++i) {DACs[i] = msg.dac_val[i];}
	} else { // update DAC and position (be the initial position)
		for (int i=0; i<6; ++i) {
			DACs[i] = msg.dac_val[i];
			mpos[i] = msg.mpos[i];
			mvel[i] = msg.mvel[i];
		}
		Initialized = true;
	}
	if (iter_gold == 0) {
		for (int i = 0; i < 3; i++) {
			switch (i)
			{
				case 0:
					jpos = 0.007342345766264*mpos[0]-PI;
					jvel = 0.007342345766264*mvel[0];
				break;
				case 1:
					jpos = 0*-0.001067944191703*mpos[0]+0.008228805750159*mpos[1]-PI;
					jvel = 0*-0.001067944191703*mvel[0]+0.008228805750159*mvel[1];
				break;
				case 2:
					jpos=0*-0.000048622484703*mpos[0]-0*0.000066464064044*mpos[1]
							+0.000463265306122*mpos[2];
					jvel=0*-0.000048622484703*mvel[0]-0*0.000066464064044*mvel[1]
							+0.000463265306122*mvel[2];
				break;
			}
			r_state[i] = jpos;
			r_state[3+i] = jvel;
			r_state[6+i] = mpos[i];
			r_state[9+i] = mvel[i];
		}
	}
	sys_dyn_gold_euler(r_state, EULER_STEP*1000);
	iter_gold++;
}

bool output_srv(void) {raven_2::output_dyn_sim::Request &req,
					   raven_2::output_dyn_sim::Response &res)
{
	for (int i=0; i<3; ++i) {
		res.mpos[i] = r_state[6+i];
		res.mvel[i] = r_state[9+i];
	}
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "dynamics_simulator");
	ros::NodeHandle n;
	ros::Subscriber sub_input = n.subscribe<raven_2::input_dyn_sim>("input_dyn_sim",1, input_callback);
	//ros::ServiceServer service = n.advertiseService("output_dyn_sim", output_srv)
	ROS_INFO("I'm in dyn_sim process.")
	ros::spin();
	return 0;
}
