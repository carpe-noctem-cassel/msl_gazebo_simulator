#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

#include <msl_gazebo_control/RobotsControl.h>

#include <SystemConfig.h>
#include <process_manager/RobotExecutableRegistry.h>

namespace msl_gazebo_control
{

	chrono::duration<double> RobotsControl::msgTimeOut = chrono::duration<double>(0);

	RobotsControl::RobotsControl() :
			rqt_gui_cpp::Plugin(), widget_(0), guiUpdateTimer(nullptr)
	{
		setObjectName("RobotsControl");
		rosNode = new ros::NodeHandle();

		this->sc = supplementary::SystemConfig::getInstance();
		RobotsControl::msgTimeOut = chrono::duration<double>(
				(*this->sc)["PMControl"]->get<unsigned long>("timeLastMsgReceivedTimeOut", NULL));
		this->pmRegistry = new supplementary::RobotExecutableRegistry();

		/* Initialise the registry data structure for better performance
		 * with data from Globals.conf and Processes.conf file. */

		// Register robots from Globals.conf
		int curId;
		auto robotNames = (*this->sc)["Globals"]->getSections("Globals.Team", NULL);
		for (auto robotName : (*robotNames))
		{
			curId = this->pmRegistry->addRobot(robotName);
		}
	}

	void RobotsControl::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		widget_ = new QWidget();
		widget_->setAttribute(Qt::WA_AlwaysShowToolTips, true);
		robotControlWidget_.setupUi(widget_);

		this->robotControlWidget_.scrollAreaWidgetContents->setContextMenuPolicy(
				Qt::ContextMenuPolicy::CustomContextMenu);
		QObject::connect(this->robotControlWidget_.scrollAreaWidgetContents,
							SIGNAL(customContextMenuRequested(const QPoint&)), this,
							SLOT(showContextMenu(const QPoint&)));

		if (context.serialNumber() > 1)
		{
			widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
		}
		context.addWidget(widget_);

		// Initialise the GUI refresh timer
		this->guiUpdateTimer = new QTimer();
		QObject::connect(guiUpdateTimer, SIGNAL(timeout()), this, SLOT(run()));
		this->guiUpdateTimer->start(200);
	}

	void RobotsControl::showContextMenu(const QPoint& pos)
	{
		/* HINT: remember, if there are some problems that way:
		 * For QAbstractScrollArea and derived classes you would use:
		 * QPoint globalPos = myWidget->viewport()->mapToGlobal(pos); */

		QPoint globalPos = this->robotControlWidget_.scrollAreaWidgetContents->mapToGlobal(pos);

		QMenu myMenu;
		for (auto robot : this->pmRegistry->getRobots())
		{
			myMenu.addAction(robot->name.c_str());
		}

		QAction* selectedItem = myMenu.exec(globalPos);
		if (selectedItem)
		{
			cout << "RC: " << selectedItem->iconText().toStdString() << endl;
			int robotId;
			if (this->pmRegistry->getRobotId(selectedItem->iconText().toStdString(), robotId))
			{
				this->checkAndInit(robotId);
				this->controlledRobotsMap[robotId]->toggle();
			}
			else
			{
				cerr << "RC: Chosen robot is not known in the robot registry!" << endl;
			}
		}
		else
		{
			cout << "RC: Nothing chosen!" << endl;
		}
	}

	/**
	 * The worker method of RobotsControl. It processes the received ROS messages and afterwards updates the GUI.
	 */
	void RobotsControl::run()
	{

		updateGUI();
	}

	/**
	 * Updates the GUI, after ROS process stat message have been processed.
	 */
	void RobotsControl::updateGUI()
	{
		// TODO: Not implemented, yet.
		chrono::system_clock::time_point now = chrono::system_clock::now();
		for (auto controlledRobotEntry : this->controlledRobotsMap)
		{
			controlledRobotEntry.second->updateGUI(now);
		}
	}

	/**
	 * If the given robot ID is already known, nothing is done.
	 * Otherwise a new entry in the controlled robot map is created.
	 */
	void RobotsControl::checkAndInit(int robotId)
	{
		auto pmEntry = this->controlledRobotsMap.find(robotId);
		if (pmEntry == this->controlledRobotsMap.end())
		{ // robot is not known, so create a corresponding instance
			string robotName;
			if (this->pmRegistry->getRobotName(robotId, robotName))
			{
				cout << "RC: Create new ControlledRobot with ID " << robotId << " and host name "
						<< robotName << "!" << endl;
				Robot* controlledRobot = new Robot(robotName, robotId, this);
				this->controlledRobotsMap.emplace(robotId, controlledRobot);
			}
			else
			{
				cerr << "RC: Received message from unknown robot with sender id " << robotId << endl;
			}
		}
	}

	void RobotsControl::shutdownPlugin()
	{
	}

	void RobotsControl::saveSettings(qt_gui_cpp::Settings& plugin_settings,
										qt_gui_cpp::Settings& instance_settings) const
	{

	}

	void RobotsControl::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
										const qt_gui_cpp::Settings& instance_settings)
	{

	}

}

PLUGINLIB_EXPORT_CLASS(msl_gazebo_control::RobotsControl, rqt_gui_cpp::Plugin)
