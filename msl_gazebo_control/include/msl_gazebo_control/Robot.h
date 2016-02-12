/*
 * ControlledRobot.h
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#ifndef SUPPLEMENTARY_RQT_ROBOT_CONTROL_SRC_RQT_ROBOT_CONTROL_CONTROLLEDROBOT_H_
#define SUPPLEMENTARY_RQT_ROBOT_CONTROL_SRC_RQT_ROBOT_CONTROL_CONTROLLEDROBOT_H_

#include <chrono>
#include <process_manager/RobotMetaData.h>
#include <QFrame>
#include <ros/ros.h>
#include "alica_ros_proxy/AlicaEngineInfo.h"

namespace Ui {
	class ControlledRobotWidget;
}

namespace supplementary{
	class RobotExecutableRegistry;
}

namespace ros{
	class Publisher;
}

namespace msl_gazebo_control
{
	class RobotsControl;

	class Robot : public QObject, public supplementary::RobotMetaData
	{

		Q_OBJECT

	public:
		Robot(string robotName, int robotId, RobotsControl* parentRobotsControl);

		virtual ~Robot();

		// Message processing
		chrono::time_point<chrono::steady_clock> timeLastMsgReceived; /**< the last time a message was received for this robot */
		void handleAlicaInfo(alica_ros_proxy::AlicaEngineInfoConstPtr aei);

		// GUI Methods and Members
		RobotsControl* parentRobotsControl;
		void updateGUI(chrono::system_clock::time_point now);
		void clearGUI();
		void toggle();
		void show();
		void hide();
		bool shown;

		public Q_SLOTS:
		void sendRobotCommand(bool start);


	private:

		QFrame* widget;
		Ui::ControlledRobotWidget* uiControlledRobot;

		// replace with service stuff
		ros::Publisher robotCommandPub;
		ros::ServiceClient robotSpawnServiceClient;


	};

} /* namespace msl_gazebo_control */

#endif /* SUPPLEMENTARY_RQT_ROBOT_CONTROL_SRC_RQT_ROBOT_CONTROL_CONTROLLEDROBOT_H_ */
