/*
 * BasicPlanWithWorldModel.cpp
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

struct WorldModel : public CallContextParameters {
	int counter;

	string str() const {
		stringstream s;
		s << "counter=" << counter;
		return s.str();
	}
};
#define WM TAO_CONTEXT.parameters<WorldModel>()

TAO(Incrementer)
{
	TAO_PLANS {
		Increment
	}
	TAO_START_PLAN(Increment);
	TAO_BGN {
		TAO_PLAN(Increment) {
			TAO_START_CONDITION(WM.counter < 100);

			WM.counter++;
			cout << "counter: " << WM.counter << endl;
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));

			TAO_ALLOCATE_EMPTY
			TAO_STOP_CONDITION(true);

			TAO_NEXT(NextFirstReady){
				TAO_NEXT_PLAN(Increment);
			}
		}
	}
	TAO_END
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "basic_plan_wm");
	ros::NodeHandle nh;

	ros_decision_making_init(argc, argv);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	RosEventQueue eventQueue;
	CallContext context;
	context.createParameters(new WorldModel());

	// CallContext must define a team (otherwise there will be a runtime exception)
	teamwork::SharedMemory db;
	teamwork::Teammates teammates;
	teamwork::Team main_team = teamwork::createMainTeam(db, "main", teammates);
	context.team(TAO_CURRENT_TEAM_NAME, main_team.ptr());

	eventQueue.async_spin();
	ROS_INFO("Starting incrementer plan...");

	TaoIncrementer(&context, &eventQueue);

	eventQueue.close();
	ROS_INFO("TAO finished.");

	return 0;
}





