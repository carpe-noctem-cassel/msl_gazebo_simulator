#ifndef msl_gazebo_control__PMControl_H
#define msl_gazebo_control__PMControl_H

#include <rqt_gui_cpp/plugin.h>

#include "ros/ros.h"
#include <ros/macros.h>
#include <msl_gazebo_control/Robot.h>

#include "process_manager/ProcessStats.h"
#include "alica_ros_proxy/AlicaEngineInfo.h"

#include <ui_RobotsControl.h>
#include <QtGui>
#include <QWidget>
#include <QDialog>

#include <queue>
#include <mutex>
#include <utility>
#include <chrono>

using namespace std;

namespace supplementary
{
	class SystemConfig;
	class RobotExecutableRegistry;
}

namespace msl_gazebo_control
{

	class RobotsControl : public rqt_gui_cpp::Plugin
	{

	Q_OBJECT

	public:

		RobotsControl();
		virtual void initPlugin(qt_gui_cpp::PluginContext& context);
		virtual void shutdownPlugin();
		virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
		virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

		void addRobot();
		void removeRobot();

		static chrono::duration<double> msgTimeOut;

		Ui::RobotControlWidget robotControlWidget_;
		QWidget* widget_;

		supplementary::RobotExecutableRegistry* pmRegistry;
		ros::NodeHandle* rosNode;

	private:
		supplementary::SystemConfig* sc;

		map<int, Robot*> controlledRobotsMap;

		void checkAndInit(int robotId);

		QTimer* guiUpdateTimer;

	public Q_SLOTS:
		void run();
		void updateGUI();
		void showContextMenu(const QPoint& pos);
	};

}

#endif // rqt_msl_refbox__RefBox_H
