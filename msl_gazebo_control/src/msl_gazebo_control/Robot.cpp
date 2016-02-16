/*
 * ControlledRobot.cpp
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#include <msl_gazebo_control/Robot.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <process_manager/RobotExecutableRegistry.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>

#include "msl_gazebo_control/RobotCommand.h"
#include "msl_gazebo_control/RobotsControl.h"
#include "ui_ControlledRobot.h"
#include "alica/AlicaWidget.h"

#include <chrono>
#include <limits.h>
#include <fstream>
#include <iostream>

namespace msl_gazebo_control
{
	Robot::Robot(string robotName, int robotId, RobotsControl* parentRobotsControl) :
			RobotMetaData(robotName, robotId), parentRobotsControl(parentRobotsControl), widget(new QFrame()), uiControlledRobot(
					new Ui::ControlledRobotWidget()), shown(false)
	{
		this->uiControlledRobot->setupUi(this->widget);

		// manual configuration of widgets
		this->uiControlledRobot->robotStartStopBtn->setText(QString(this->name.c_str()));

		// signals and slots
		QObject::connect(this->uiControlledRobot->robotStartStopBtn, SIGNAL(toggled(bool)), this,
							SLOT(sendRobotCommand(bool)));

		// hide default
		this->widget->hide();

		// add to parent widget
		this->parentRobotsControl->robotControlWidget_.allRobotsFlowLayout->addWidget(this->widget);


		robotSpawnServiceClient = this->parentRobotsControl->rosNode->serviceClient<gazebo_msgs::SpawnModel>(
				"/gazebo/spawn_sdf_model");
		robotDeleteServiceClient = this->parentRobotsControl->rosNode->serviceClient<gazebo_msgs::DeleteModel>(
				"/gazebo/delete_model");

		string path = ros::package::getPath("nubot_description");
		cout << "msl_gazebo_control: Path to nubot_descritpion " << path << endl;
		ifstream in(path + "/models/nubot/model.sdf");
		model_xml = string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
	}

	Robot::~Robot()
	{
	}

	void Robot::updateGUI(chrono::system_clock::time_point now)
	{
		if (chrono::steady_clock::now() - this->timeLastMsgReceived > std::chrono::milliseconds(1000))
		{
			this->clearGUI();
		}
	}

	void Robot::clearGUI()
	{

	}

	void Robot::sendRobotCommand(bool start)
	{
		if (start)
		{ // spawn model
			gazebo_msgs::SpawnModel srv;
			srv.request.robot_namespace = this->name;
			srv.request.initial_pose.position.x = 0;
			srv.request.initial_pose.position.y = 0;
			srv.request.initial_pose.position.z = 0;
			srv.request.model_name = this->name;
			srv.request.reference_frame = "world";
			// TODO read in the whole document
			srv.request.model_xml = this->model_xml;

			if (this->robotSpawnServiceClient.call(srv))
			{
				cout << this->name << ": Spawn Model Response: " << srv.response.status_message << endl;
				cout << this->name << ": Spawn Model Success? - " << (srv.response.success ? "True" : "False") << endl;
			}
			else
			{
				cout << this->name << ": Spawn Model Call didn't work!" << endl;
			}
		}
		else
		{ // destroy model
			gazebo_msgs::DeleteModel srv;
			srv.request.model_name = this->name;
			if (this->robotDeleteServiceClient.call(srv))
			{
				cout << this->name << ": Delete Model Response: " << srv.response.status_message << endl;
				cout << this->name << ": Delete Model Success? - " << (srv.response.success ? "True" : "False") << endl;
			}
			else
			{
				cout << this->name << ": Delete Model Call didn't work!" << endl;
			}
		}

	}

	void Robot::toggle()
	{
		if (shown)
		{
			this->hide();
		}
		else
		{
			this->show();
		}
	}

	void Robot::show()
	{
		this->shown = true;
		this->widget->show();
	}

	void Robot::hide()
	{
		this->shown = false;
		this->widget->hide();
	}

} /* namespace msl_gazebo_control */
