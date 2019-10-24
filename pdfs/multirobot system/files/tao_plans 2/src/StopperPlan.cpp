/*
 * BasicPlan.cpp
 *
 *  Created on: May 31, 2014
 *      Author: roiyeho
 */

#include <iostream>
#include <ros/ros.h>

#include <decision_making/TAO.h>
#include <decision_making/TAOStdProtocols.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace std;
using namespace decision_making;

TAO(Stopper)
{
	TAO_PLANS {
		MoveForward
	}
	TAO_START_PLAN(MoveForward);
	TAO_BGN {
		TAO_PLAN(MoveForward) {
			TAO_START_CONDITION(true);
			cout << "Moving forward" << endl;
			TAO_ALLOCATE_EMPTY
			TAO_STOP_CONDITION(true);
			TAO_NEXT_EMPTY
		}
	}
	TAO_END
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stopper_plan");
	ros::NodeHandle nh;

	ros_decision_making_init(argc, argv);
	RosEventQueue eventQueue;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	eventQueue.async_spin();
	ROS_INFO("Starting stopper plan...");
	//CallContext call_ctx;
	//TaoStopper(&call_ctx, &eventQueue);
	TaoStopper(NULL, &eventQueue);

	eventQueue.close();
	ROS_INFO("Tao finished.");

	return 0;
}



