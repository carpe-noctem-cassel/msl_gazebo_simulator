#ifndef msl_gazebo_control__GazeboControl_H
#define msl_gazebo_control__GazeboControl_H

#include <rqt_gui_cpp/plugin.h>

#include "ros/ros.h"
#include <ros/macros.h>

#include <ui_GazeboControl.h>
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

	class GazeboControl : public rqt_gui_cpp::Plugin
	{

	Q_OBJECT

	public:

		GazeboControl();
		virtual void initPlugin(qt_gui_cpp::PluginContext& context);
		virtual void shutdownPlugin();
		virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
		virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

		Ui::GazeboControlWidget gazeboControlWidget_;
		QWidget* widget_;

		ros::NodeHandle* rosNode;

	private:
		supplementary::SystemConfig* sc;
		static const string redBackground;
		static const string greenBackground;
		static const string grayBackground;
		string robot_model_xml;
		QTimer* guiUpdateTimer;
		ros::ServiceClient robotSpawnServiceClient;
		ros::Publisher setModelPublisher;
		map<string,bool> activeRobotsMap;
		int obsCounter;

	public Q_SLOTS:
		void setRobot(bool checked);
		void setBall(bool checked);
		void setObstacle(bool checked);
		void addObstacle(bool checked);
	};

}

#endif // msl_gazebo_control__GazeboControl_H
