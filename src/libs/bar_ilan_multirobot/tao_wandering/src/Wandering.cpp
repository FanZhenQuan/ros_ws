/*
 * Wandering.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: roiyeho
 */

#include <iostream>

#include <ros/ros.h>
#include <random_numbers/random_numbers.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <decision_making/TAO.h>
#include <decision_making/TAOStdProtocols.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace std;
using namespace decision_making;

/*************************************************************************************************
*** Constants
**************************************************************************************************/
#define MIN_SCAN_ANGLE_RAD -45.0/180*M_PI
#define MAX_SCAN_ANGLE_RAD +45.0/180*M_PI
#define MIN_DIST_TO_OBSTACLE 0.8 // in meters

/*************************************************************************************************
*** Variables
**************************************************************************************************/
random_numbers::RandomNumberGenerator _randomizer;
ros::Publisher _velocityPublisher;

/*************************************************************************************************
*** World model
**************************************************************************************************/
struct WorldModel : public CallContextParameters {
	bool obstacleDetected;
	bool driveFinished;
	bool turnFinished;

	string str() const {
		stringstream s;
		s << "obstacleDetected = " << obstacleDetected;
		return s.str();
	}
};
#define WM TAO_CONTEXT.parameters<WorldModel>()

/*************************************************************************************************
*** TAO machine
**************************************************************************************************/
TAO_HEADER(Drive)
TAO_HEADER(Turn)

TAO(Wandering)
{
	TAO_PLANS {
		Wandering
	}
	TAO_START_PLAN(Wandering);
	TAO_BGN {
		TAO_PLAN(Wandering) {
			TAO_START_CONDITION(true);

			WM.driveFinished = false;
			WM.turnFinished = false;

			TAO_ALLOCATE(AllocFirstReady) {
				TAO_SUBPLAN(Drive);
				TAO_SUBPLAN(Turn);
			}

			TAO_STOP_CONDITION(WM.driveFinished || WM.turnFinished);

			TAO_NEXT(NextFirstReady) {
				TAO_NEXT_PLAN(Wandering);
			}
		}
	}
	TAO_END
}

TAO(Drive)
{
    TAO_PLANS {
    	Drive,
    }
    TAO_START_PLAN(Drive);
    TAO_BGN {
        TAO_PLAN(Drive) {
            TAO_START_CONDITION(!WM.obstacleDetected);
            TAO_CALL_TASK(driveTask);

            TAO_ALLOCATE_EMPTY
            TAO_STOP_CONDITION(WM.obstacleDetected);
            TAO_NEXT_EMPTY

            WM.driveFinished = true;
        }
    }
    TAO_END
}

TAO(Turn)
{
    TAO_PLANS{
    	Turn,
    }
    TAO_START_PLAN(Turn);
    TAO_BGN {
        TAO_PLAN(Turn) {
        	TAO_START_CONDITION(WM.obstacleDetected);
        	TAO_CALL_TASK(turnTask);

            TAO_ALLOCATE_EMPTY
            TAO_STOP_CONDITION(true);
            TAO_NEXT_EMPTY

            WM.turnFinished = true;
        }
    }
    TAO_END
}

/*************************************************************************************************
*** Task implementations
**************************************************************************************************/
TaskResult driveTask(string name, const CallContext& context, EventQueue& eventQueue) {
	ROS_INFO("Driving...");

	geometry_msgs::Twist forwardMessage;
	forwardMessage.linear.x = 1.0;

	// Preemptive wait
	while (!eventQueue.isTerminated()) {
		_velocityPublisher.publish(forwardMessage);
		boost::this_thread::sleep(boost::posix_time::milliseconds(100.0));
	}

	ROS_INFO("Obstacle detected!");
	return TaskResult::SUCCESS();
}

TaskResult turnTask(string name, const CallContext& context, EventQueue& eventQueue) {
	ROS_INFO("Turning...");

	bool turnRight = _randomizer.uniformInteger(0, 1);

	geometry_msgs::Twist turnMessage;
	turnMessage.angular.z = 2 * (turnRight ? 1 : -1);

	int timeToTurnMs = _randomizer.uniformInteger(2000, 4000);
	int turnLoops = timeToTurnMs / 100;

	for (int i = 0; i < turnLoops; i++) {
		_velocityPublisher.publish(turnMessage);
		boost::this_thread::sleep(boost::posix_time::milliseconds(100.0));
	}

	return TaskResult::SUCCESS();
}

/*************************************************************************************************
*** ROS Subscriptions
**************************************************************************************************/
void onLaserScanMessage(const sensor_msgs::LaserScan::Ptr laserScanMessage, CallContext* context) {

	bool obstacleFound = false;

	int minIndex = ceil((MIN_SCAN_ANGLE_RAD - laserScanMessage->angle_min) / laserScanMessage->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE_RAD - laserScanMessage->angle_min) / laserScanMessage->angle_increment);

	for (int i = minIndex; i <= maxIndex; i++) {
		if (laserScanMessage->ranges[i] < MIN_DIST_TO_OBSTACLE) {
			obstacleFound = true;
		}
	}

	context->parameters<WorldModel>().obstacleDetected = obstacleFound;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wandering_node");
	ros::NodeHandle nh;

	ros_decision_making_init(argc, argv);

	// ROS spinner for topic subscriptions
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Tasks registration
	LocalTasks::registrate("driveTask", driveTask);
	LocalTasks::registrate("turnTask", turnTask);

	RosEventQueue eventQueue;
	CallContext context;
	context.createParameters(new WorldModel());

	// CallContext must define a team
	teamwork::SharedMemory db;
	teamwork::Teammates teammates;
	teamwork::Team main_team = teamwork::createMainTeam(db, "main", teammates);
	context.team(TAO_CURRENT_TEAM_NAME, main_team.ptr());

	// Subscription for the laser topic and velocity publisher creation
	ros::Subscriber laserSubscriber = nh.subscribe<void>("base_scan", 1,
					boost::function<void(const sensor_msgs::LaserScan::Ptr)>(boost::bind(onLaserScanMessage, _1, &context)));
	_velocityPublisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

	eventQueue.async_spin();

	ROS_INFO("Starting wandering machine...");
	TaoWandering(&context, &eventQueue);

	eventQueue.close();
	ROS_INFO("TAO finished.");

	return 0;
}


