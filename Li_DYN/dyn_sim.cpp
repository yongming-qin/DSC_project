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
// Individual data structure for two arms so that we only change 
double DACs[3]; // this global variable is not used here. I use arguments to replace that.
static state_type gold_state = {0,0,0,0,0,0,0,0,0,0,0,0};
static state_type green_state = {0,0,0,0,0,0,0,0,0,0,0,0};
static float gold_DACs[3];
static float green_DACs[3];

void input_callback(raven_2::input_dyn_sim msg) {
	//ROS_INFO("Raven State: [%f %f %f %f %f %f]", msg.jpos[0], msg.jpos[1], msg.jpos[2],msg.jpos[3], msg.jpos[4], msg.jpos[5]);
	//ROS_INFO("joint encoder position: %f, %f, %f: ", msg.j_enc_pos[0], msg.j_enc_pos[1], msg.j_enc_pos[2]);
	ROS_INFO("Joint position: %d, %d, %d, %d, %d, %d: ", msg.dac_val[0], msg.dac_val[1], msg.dac_val[2], msg.dac_val[3], msg.dac_val[4], msg.dac_val[5]);
	ROS_INFO("Joint position: %f, %f, %f, %f, %f, %f: ", msg.mpos[0], msg.mpos[1], msg.mpos[2], msg.mpos[3], msg.mpos[4], msg.mpos[5]);
	ROS_INFO("Joint position: %f, %f, %f, %f, %f, %f: ", msg.mvel[0], msg.mvel[1], msg.mvel[2], msg.mvel[3], msg.mvel[4], msg.mvel[5]);
	//
	static double mpos[6],mvel[6], jpos, jvel;
	// update DAC every time
	for (int i=0; i<3; ++i) { // for the three joints
		gold_DACs[i] = msg.dac_val[i];
		green_DACs[i] = msg.dac_val[i+3];
	}
	if !Initialized { // update DAC and position (be the initial position)
		for (int i=0; i<6; ++i) { // position for two arms
			mpos[i] = msg.mpos[i];
			mvel[i] = msg.mvel[i];
		}
		for (int i=0; i<2; ++i) { // for two arms
			for (int j = 0; j < 3; j++) { // for the three joints
				switch (j)
				{
					case 0:
						jpos = 0.007342345766264*mpos[0+i*3]-PI;
						jvel = 0.007342345766264*mvel[0+i*3];
					break;
					case 1:
						jpos = 0*-0.001067944191703*mpos[0+i*3]+0.008228805750159*mpos[1+i*3]-PI;
						jvel = 0*-0.001067944191703*mvel[0+i*3]+0.008228805750159*mvel[1+i*3];
					break;
					case 2:
						jpos=0*-0.000048622484703*mpos[0+i*3]-0*0.000066464064044*mpos[1+i*3]
								+0.000463265306122*mpos[2+i*3];
						jvel=0*-0.000048622484703*mvel[0+i*3]-0*0.000066464064044*mvel[1+i*3]
								+0.000463265306122*mvel[2+i*3];
					break;
				}
				if (i == 0) { // gold arm
					gold_state[i] = jpos;
					gold_state[3+i] = jvel;
					gold_state[6+i] = mpos[i];
					gold_state[9+i] = mvel[i];
				} else { // green arm
					green_state[i] = jpos;
					green_state[3+i] = jvel;
					green_state[6+i] = mpos[i];
					green_state[9+i] = mvel[i];
				}
			}
		}
		Initialized = true;
	}
	sys_dyn_euler(gold_state, EULER_STEP*1000, gold_state);
	iter_gold++;
	sys_dyn_euler(green_state, EULER_STEP*1000, gold_state);
	iter_green++;
}

bool output_srv(void) {raven_2::output_dyn_sim::Request &req,
					   raven_2::output_dyn_sim::Response &res)
{
	for (int i=0; i<3; ++i) {
		res.mpos[i] = gold_state[6+i];
		res.mvel[i] = gold_state[9+i];
		res.mpos[i+3] = green_state[6+i];
		res.mvel[i+3] = green_state[9+i];
	}
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "dynamics_simulator");
	ros::NodeHandle n;
	ros::Subscriber sub_input = n.subscribe<raven_2::input_dyn_sim>("input_dyn_sim",1, input_callback);
	ros::ServiceServer service = n.advertiseService("output_dyn_sim", output_srv)
	ROS_INFO("I'm in dyn_sim process.")
	ros::spin();
	return 0;
}
