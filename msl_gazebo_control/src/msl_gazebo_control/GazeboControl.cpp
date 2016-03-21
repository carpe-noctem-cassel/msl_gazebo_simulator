#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <ros/package.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelState.h>
#include <msl_gazebo_control/GazeboControl.h>
#include <std_msgs/Bool.h>

#include <SystemConfig.h>

namespace msl_gazebo_control
{
	const string GazeboControl::redBackground = "background-color:#FF4719;";
	const string GazeboControl::greenBackground = "background-color:#66FF66;";
	const string GazeboControl::grayBackground = "background-color:gray;";

	GazeboControl::GazeboControl() :
			rqt_gui_cpp::Plugin(), widget_(0), guiUpdateTimer(nullptr)
	{
		setObjectName("GazeboControl");
		rosNode = new ros::NodeHandle();

		this->sc = supplementary::SystemConfig::getInstance();

		robotSpawnServiceClient = this->rosNode->serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
		setModelPublisher = this->rosNode->advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
		stopSimulatorPublisher = this->rosNode->advertise<std_msgs::Bool>("/gazebo/stopSimulator", 10);

		string path = ros::package::getPath("nubot_description");
		cout << "msl_gazebo_control: Path to nubot_descritpion " << path << endl;
		ifstream in(path + "/models/nubot/model.sdf");
		this->robot_model_xml = string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		this->obsCounter = 0;
	}

	void GazeboControl::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		this->widget_ = new QWidget();
		this->gazeboControlWidget_.setupUi(widget_);

		if (context.serialNumber() > 1)
		{
			widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
		}
		context.addWidget(widget_);

		// connect ball set button
		QObject::connect(this->gazeboControlWidget_.stopSimBtn, SIGNAL(toggled(bool)), this, SLOT(stopSim(bool)));

		// connect ball set button
		QObject::connect(this->gazeboControlWidget_.ballSetBtn, SIGNAL(clicked(bool)), this, SLOT(setBall(bool)));

		// connect add obstacle
		QObject::connect(this->gazeboControlWidget_.obsAddBtn, SIGNAL(clicked(bool)), this, SLOT(addObstacle(bool)));

		// create robot control stuff for every robot in Globals.conf
		auto robotNames = (*this->sc)["Globals"]->getSections("Globals.Team", NULL);
		for (auto robotName : (*robotNames))
		{
			this->activeRobotsMap[robotName] = false;

			QHBoxLayout* curRobotHLayout = new QHBoxLayout();
			curRobotHLayout->setSpacing(0);
			curRobotHLayout->setContentsMargins(0, 0, 0, 0);
			curRobotHLayout->setObjectName(QString::fromUtf8((robotName + "HLayout").c_str()));

			QPushButton* curBtn = new QPushButton(this->gazeboControlWidget_.robotBox);
			curBtn->setObjectName(QString::fromUtf8((robotName + "SetButton").c_str()));
			curBtn->setText(QString(robotName.c_str()));
			curBtn->setStyleSheet(GazeboControl::redBackground.c_str());
			curBtn->setAutoFillBackground(true);
			curBtn->update();
			QObject::connect(curBtn, SIGNAL(clicked(bool)), this, SLOT(setRobot(bool)));

			curRobotHLayout->addWidget(curBtn);

			QLabel* curXLbl = new QLabel(this->gazeboControlWidget_.robotBox);
			curXLbl->setObjectName(QString::fromUtf8((robotName + "XLbl").c_str()));
			curXLbl->setText("X:");

			curRobotHLayout->addWidget(curXLbl);

			QLineEdit* curXEdit = new QLineEdit(this->gazeboControlWidget_.robotBox);
			curXEdit->setObjectName(QString::fromUtf8((robotName + "XEdit").c_str()));

			curRobotHLayout->addWidget(curXEdit);

			QLabel* curYLbl = new QLabel(this->gazeboControlWidget_.robotBox);
			curYLbl->setObjectName(QString::fromUtf8((robotName + "YLbl").c_str()));
			curYLbl->setText("Y:");

			curRobotHLayout->addWidget(curYLbl);

			QLineEdit* curYEdit = new QLineEdit(this->gazeboControlWidget_.robotBox);
			curYEdit->setObjectName(QString::fromUtf8((robotName + "YEdit").c_str()));

			curRobotHLayout->addWidget(curYEdit);
			this->gazeboControlWidget_.robotBoxVerticalLayout->addLayout(curRobotHLayout);
		}
	}

	void GazeboControl::setObstacle(bool checked)
	{
		QPushButton* btn = (QPushButton*)QObject::sender();
		string name = btn->text().toStdString();
		auto activeMapIter = this->activeRobotsMap.find(name);
		if (activeMapIter != this->activeRobotsMap.end())
		{ // ROBOTS
			if (activeMapIter->second)
			{ // robot is already active, so just set its position
				cout << "GC: setModel(" << checked << ") called from button \"" << name << "\"" << endl;
				gazebo_msgs::ModelState ms;
				ms.model_name = name;
				QLineEdit* xEdit = this->gazeboControlWidget_.robotBox->findChild<QLineEdit*>((name + "XEdit").c_str());
				QLineEdit* yEdit = this->gazeboControlWidget_.robotBox->findChild<QLineEdit*>((name + "YEdit").c_str());
				try
				{
					ms.pose.position.x = stod(xEdit->text().toStdString()) / 1000.0;
					ms.pose.position.y = stod(yEdit->text().toStdString()) / 1000.0;
				}
				catch (const std::exception& ex)
				{
					cerr
							<< "GazeboControl: Cannot convert your coordinates. Please enter a correct double coordinates for the ball."
							<< endl;
					ms.pose.position.x = 0;
					ms.pose.position.y = 0;
				}
				ms.pose.position.z = 0;
				ms.reference_frame = "world";
				this->setModelPublisher.publish(ms);
			}
			else
			{ // robot is not spawned yet, so spawn it at the position
				gazebo_msgs::SpawnModel srv;
				srv.request.robot_namespace = name;
				QLineEdit* xEdit = this->gazeboControlWidget_.robotBox->findChild<QLineEdit*>((name + "XEdit").c_str());
				QLineEdit* yEdit = this->gazeboControlWidget_.robotBox->findChild<QLineEdit*>((name + "YEdit").c_str());
				try
				{
					srv.request.initial_pose.position.x = stod(xEdit->text().toStdString()) / 1000.0;
					srv.request.initial_pose.position.y = stod(yEdit->text().toStdString()) / 1000.0;
				}
				catch (const std::exception& ex)
				{
					cerr
							<< "GazeboControl: Cannot convert your coordinates. Please enter a correct double coordinates for \""
							<< name << "\"" << endl;
					srv.request.initial_pose.position.x = 0;
					srv.request.initial_pose.position.y = 0;
				}
				srv.request.initial_pose.position.z = 0;
				srv.request.model_name = name;
				srv.request.reference_frame = "world";
				srv.request.model_xml = this->robot_model_xml;

				if (this->robotSpawnServiceClient.call(srv))
				{
					cout << name << ": Spawn Model Response: " << srv.response.status_message << endl;
					cout << name << ": Spawn Model Success? - " << (srv.response.success ? "True" : "False") << endl;
					if (srv.response.success
							|| srv.response.status_message.find("Failure - model already exists") != string::npos)
					{
						btn->setStyleSheet(greenBackground.c_str());
						this->activeRobotsMap[name] = true;
					}
				}
				else
				{
					cout << name << ": Spawn Model Call didn't work!" << endl;
					btn->setStyleSheet(redBackground.c_str());
					this->activeRobotsMap[name] = false;
				}
			}
		}
	}

	void GazeboControl::setRobot(bool checked)
	{
		QPushButton* btn = (QPushButton*)QObject::sender();
		string name = btn->text().toStdString();
		auto activeMapIter = this->activeRobotsMap.find(name);
		if (activeMapIter != this->activeRobotsMap.end())
		{ // ROBOTS
			if (activeMapIter->second)
			{ // robot is already active, so just set its position
				cout << "GC: setModel(" << checked << ") called from button \"" << name << "\"" << endl;
				gazebo_msgs::ModelState ms;
				ms.model_name = name;
				QLineEdit* xEdit = this->gazeboControlWidget_.robotBox->findChild<QLineEdit*>((name + "XEdit").c_str());
				QLineEdit* yEdit = this->gazeboControlWidget_.robotBox->findChild<QLineEdit*>((name + "YEdit").c_str());
				try
				{
					ms.pose.position.x = stod(xEdit->text().toStdString()) / 1000.0;
					ms.pose.position.y = stod(yEdit->text().toStdString()) / 1000.0;
				}
				catch (const std::exception& ex)
				{
					cerr
							<< "GazeboControl: Cannot convert your coordinates. Please enter a correct double coordinates for the ball."
							<< endl;
					ms.pose.position.x = 0;
					ms.pose.position.y = 0;
				}
				ms.pose.position.z = 0;
				ms.reference_frame = "world";
				this->setModelPublisher.publish(ms);
			}
			else
			{ // robot is not spawned yet, so spawn it at the position
				gazebo_msgs::SpawnModel srv;
				srv.request.robot_namespace = name;
				QLineEdit* xEdit = this->gazeboControlWidget_.robotBox->findChild<QLineEdit*>((name + "XEdit").c_str());
				QLineEdit* yEdit = this->gazeboControlWidget_.robotBox->findChild<QLineEdit*>((name + "YEdit").c_str());
				try
				{
					srv.request.initial_pose.position.x = stod(xEdit->text().toStdString()) / 1000.0;
					srv.request.initial_pose.position.y = stod(yEdit->text().toStdString()) / 1000.0;
				}
				catch (const std::exception& ex)
				{
					cerr
							<< "GazeboControl: Cannot convert your coordinates. Please enter a correct double coordinates for \""
							<< name << "\"" << endl;
					srv.request.initial_pose.position.x = 0;
					srv.request.initial_pose.position.y = 0;
				}

				srv.request.initial_pose.position.z = 0;
				srv.request.model_name = name;
				srv.request.reference_frame = "world";
				srv.request.model_xml = this->robot_model_xml;

				if (this->robotSpawnServiceClient.call(srv))
				{
					cout << name << ": Spawn Model Response: " << srv.response.status_message << endl;
					cout << name << ": Spawn Model Success? - " << (srv.response.success ? "True" : "False") << endl;
					if (srv.response.success
							|| srv.response.status_message.find("Failure - model already exists") != string::npos)
					{
						btn->setStyleSheet(greenBackground.c_str());
						this->activeRobotsMap[name] = true;
					}
				}
				else
				{
					cout << name << ": Spawn Model Call didn't work!" << endl;
					btn->setStyleSheet(redBackground.c_str());
					this->activeRobotsMap[name] = false;
				}
			}
		}
	}


	void GazeboControl::stopSim(bool checked)
	{
		QPushButton* btn = (QPushButton*)QObject::sender();
		string name = btn->text().toStdString();
		if (btn == this->gazeboControlWidget_.stopSimBtn)
		{ // STOP SIM
			cout << "GC: stopSim(" << checked << ") called from button \"Stop Simulator\"" << endl;
			std_msgs::Bool toggleMsg;
			toggleMsg.data = !checked;
			this->stopSimulatorPublisher.publish(toggleMsg);
		}
	}

	void GazeboControl::setBall(bool checked)
	{
		QPushButton* btn = (QPushButton*)QObject::sender();
		string name = btn->text().toStdString();
		if (btn == this->gazeboControlWidget_.ballSetBtn)
		{ // BALL
			cout << "GC: setModel(" << checked << ") called from button \"Ball Set\"" << endl;
			gazebo_msgs::ModelState ms;
			ms.model_name = "football";
			try
			{
				ms.pose.position.x = stod(this->gazeboControlWidget_.ballXEdit->text().toStdString()) / 1000.0;
				ms.pose.position.y = stod(this->gazeboControlWidget_.ballYEdit->text().toStdString()) / 1000.0;
			}
			catch (const std::exception& ex)
			{
				cerr
						<< "GazeboControl: Cannot convert your coordinates. Please enter a correct double coordinates for the ball."
						<< endl;
				ms.pose.position.x = 0;
				ms.pose.position.y = 0;
			}
			ms.pose.position.z = 0;
			ms.reference_frame = "world";
			this->setModelPublisher.publish(ms);
		}
	}

	void GazeboControl::addObstacle(bool checked)
	{
		QPushButton* btn = (QPushButton*)QObject::sender();
		string name = btn->text().toStdString();
		if (btn == this->gazeboControlWidget_.obsAddBtn)
		{
			cout << "GC: setModel(" << checked << ") called from button \"Add Obstacle\"" << endl;

			// TODO
			string obsName = to_string(this->obsCounter);
			QHBoxLayout* curObsHLayout = new QHBoxLayout();
			curObsHLayout->setSpacing(0);
			curObsHLayout->setContentsMargins(0, 0, 0, 0);
			curObsHLayout->setObjectName(QString::fromUtf8((obsName + "HLayout").c_str()));

			QPushButton* curBtn = new QPushButton(this->gazeboControlWidget_.obsBox);
			curBtn->setObjectName(QString::fromUtf8((obsName + "SetButton").c_str()));
			curBtn->setText(QString(obsName.c_str()));
			curBtn->setStyleSheet(GazeboControl::redBackground.c_str());
			curBtn->setAutoFillBackground(true);
			curBtn->update();
			QObject::connect(curBtn, SIGNAL(clicked(bool)), this, SLOT(setObstacle(bool)));

			curObsHLayout->addWidget(curBtn);

			QLabel* curXLbl = new QLabel(this->gazeboControlWidget_.obsBox);
			curXLbl->setObjectName(QString::fromUtf8((obsName + "XLbl").c_str()));
			curXLbl->setText("X:");

			curObsHLayout->addWidget(curXLbl);

			QLineEdit* curXEdit = new QLineEdit(this->gazeboControlWidget_.obsBox);
			curXEdit->setObjectName(QString::fromUtf8((obsName + "XEdit").c_str()));

			curObsHLayout->addWidget(curXEdit);

			QLabel* curYLbl = new QLabel(this->gazeboControlWidget_.obsBox);
			curYLbl->setObjectName(QString::fromUtf8((obsName + "YLbl").c_str()));
			curYLbl->setText("Y:");

			curObsHLayout->addWidget(curYLbl);

			QLineEdit* curYEdit = new QLineEdit(this->gazeboControlWidget_.obsBox);
			curYEdit->setObjectName(QString::fromUtf8((obsName + "YEdit").c_str()));

			curObsHLayout->addWidget(curYEdit);
			this->gazeboControlWidget_.obsBoxVLayout->addLayout(curObsHLayout);
			this->obsCounter++;
		}
	}

	void GazeboControl::shutdownPlugin()
	{
	}

	void GazeboControl::saveSettings(qt_gui_cpp::Settings& plugin_settings,
										qt_gui_cpp::Settings& instance_settings) const
	{

	}

	void GazeboControl::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
										const qt_gui_cpp::Settings& instance_settings)
	{

	}

}

PLUGINLIB_EXPORT_CLASS(msl_gazebo_control::GazeboControl, rqt_gui_cpp::Plugin)
