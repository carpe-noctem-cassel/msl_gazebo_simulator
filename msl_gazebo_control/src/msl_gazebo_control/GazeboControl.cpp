#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <ros/package.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <msl_gazebo_control/GazeboControl.h>

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
		robotDeleteServiceClient = this->rosNode->serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

		string path = ros::package::getPath("nubot_description");
		cout << "msl_gazebo_control: Path to nubot_descritpion " << path << endl;
		ifstream in(path + "/models/nubot/model.sdf");
		this->model_xml = string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
	}

	void GazeboControl::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		widget_ = new QWidget();
		gazeboControlWidget_.setupUi(widget_);

		if (context.serialNumber() > 1)
		{
			widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
		}
		context.addWidget(widget_);

		auto robotNames = (*this->sc)["Globals"]->getSections("Globals.Team", NULL);
		for (auto robotName : (*robotNames))
		{
			// TODO: create Button for each robot
			QPushButton* curBtn = new QPushButton();
			curBtn->setCheckable(true);
			curBtn->setText(QString(robotName.c_str()));
			curBtn->setStyleSheet(GazeboControl::redBackground.c_str());
			curBtn->setAutoFillBackground(true);
			curBtn->update();
			this->gazeboControlWidget_.gazeboControlVerticalLayout->addWidget(curBtn);
			QObject::connect(curBtn, SIGNAL(toggled(bool)), this,
										SLOT(toggleModel(bool)));
		}
	}

	void GazeboControl::toggleModel(bool spawn)
	{
		QPushButton* btn = (QPushButton*) QObject::sender();
		string name = btn->text().toStdString();
		cout << "GC: toggleModel(" << spawn << ") called from button " << name << endl;
		if (spawn)
		{ // spawn model
			gazebo_msgs::SpawnModel srv;
			srv.request.robot_namespace = name;
			srv.request.initial_pose.position.x = 0;
			srv.request.initial_pose.position.y = 0;
			srv.request.initial_pose.position.z = 0;
			srv.request.model_name = name;
			srv.request.reference_frame = "world";
			// TODO read in the whole document
			srv.request.model_xml = this->model_xml;

			if (this->robotSpawnServiceClient.call(srv))
			{
				cout << name << ": Spawn Model Response: " << srv.response.status_message << endl;
				cout << name << ": Spawn Model Success? - " << (srv.response.success ? "True" : "False") << endl;
				if (srv.response.success)
				{
					btn->setStyleSheet(greenBackground.c_str());
				}
			}
			else
			{
				cout << name << ": Spawn Model Call didn't work!" << endl;
				btn->setStyleSheet(redBackground.c_str());
			}
		}
		else
		{ // destroy model
			gazebo_msgs::DeleteModel srv;
			srv.request.model_name = name;
			if (this->robotDeleteServiceClient.call(srv))
			{
				cout << name << ": Delete Model Response: " << srv.response.status_message << endl;
				cout << name << ": Delete Model Success? - " << (srv.response.success ? "True" : "False") << endl;
				if (srv.response.success)
				{
					btn->setStyleSheet(redBackground.c_str());
				}
			}
			else
			{
				cout << name << ": Delete Model Call didn't work!" << endl;
				btn->setStyleSheet(greenBackground.c_str());
			}
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
