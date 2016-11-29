#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include "msl_gazebo_sim_link/GazeboSimLink.h"
#include <gazebo-7/gazebo/physics/Model.hh>
#include <gazebo-7/gazebo/physics/World.hh>
#include <iostream>
#include <QString>

namespace gazebo
{


	////////////////////////////////////////////////////////////////////////////////
	// Constructor
	GazeboSimLink::GazeboSimLink()
	{
		this->udpsocket = nullptr;

		this->sc = supplementary::SystemConfig::getInstance();
		this->clientAddress = QString((*sc)["GazeboSimLink"]->get<string>("GazeboSimLink.clientAddress", NULL).c_str());
		this->hostAddress = QString((*sc)["GazeboSimLink"]->get<string>("GazeboSimLink.hostAddress", NULL).c_str());
		this->port = qint16((*sc)["GazeboSimLink"]->get<short>("GazeboSimLink.port", NULL));
		this->iAmHost = qint16((*sc)["GazeboSimLink"]->get<bool>("GazeboSimLink.iAmHost", NULL));
	}

	////////////////////////////////////////////////////////////////////////////////
	// Destructor
	GazeboSimLink::~GazeboSimLink()
	{
		ROS_DEBUG_STREAM_NAMED("GazeboSimLink", "Unloaded");
		this->disconnectUDP();
	}

	void GazeboSimLink::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
		// Make sure the ROS node for Gazebo has already been initialized
		if (!ros::isInitialized())
		{
			ROS_FATAL_STREAM(
					"A ROS node for Gazebo has not been initialized, unable to load plugin. " << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
			return;
		}

		this->world_ = _parent->GetWorld();

		this->football = this->world_->GetModel("football");

		this->connectUDP(this->clientAddress, this->port);

		ROS_INFO("GazeboSimLink PlugIn loaded!");
		std::cout << "GazeboSimLink PlugIn loaded!" << std::endl;
	}

	void GazeboSimLink::connectUDP(QString addressString, qint16 port)
	{
		this->udpsocket = new QUdpSocket();

		const QHostAddress address = QHostAddress(addressString);

		this->udpsocket->bind(address, port);

		connect(this->udpsocket, SIGNAL(readyRead()), this, SLOT(receiveSimLinkMsgUdp()));
	}

	void GazeboSimLink::disconnectUDP()
	{
		this->udpsocket->close();
		disconnect(this->udpsocket, SIGNAL(readyRead()), this, SLOT(receiveSimLinkMsgUdp()));
		delete this->udpsocket;
		this->udpsocket = nullptr;
	}

	void GazeboSimLink::receiveSimLinkMsgUdp(void)
	{
		QByteArray buffer;
		buffer.resize(this->udpsocket->pendingDatagramSize());

		QHostAddress sender;
		quint16 senderPort;

		this->udpsocket->readDatagram(buffer.data(), buffer.size(), &sender, &senderPort);

//struct SimLink
//{
//    char    simLinkMode;               /* [c/h] Simulator link mode [0:host, 1:client, 2:none] */
//    char    blueIsHomeGoal;            /* [c/h] Is blue goal my home goal? */
//    char    teamColor;                 /* [c/h] team color on this pc */
//    char    refboxCommand;             /* [h]   Refbox command from host */
//    char    has_ball[MAX_ACTIVE_TURTLES];               /* [c]   Turtle in ball possession ( 0 or 1 ) */
//    float   ball_xyz[3];               /* [h]   ball position in m */
//    float   ball_xyz_dot[3];           /* [h]   ball position in m */
//    char    turtleIsActive[MAX_ACTIVE_TURTLES];         /* [h/c] Turtle is active (1) and in-field, otherwise 0 */
//    float   turtlePoses[3*MAX_ACTIVE_TURTLES];          /* [h/c] Turtle poses [x, y, phi] */
//    float   turtleVelocities[2*MAX_ACTIVE_TURTLES];     /* [h/c] Turtle velocities [x_dot, y_dot] */
//};

		if (buffer.size() > 0 && buffer.size() < 160)
		{
			// simLinkMode
			this->receivedSimLinkMsg.simLinkMode = buffer.data()[0];

			// blueIsHomeGoal
			this->receivedSimLinkMsg.blueIsHomeGoal = buffer.data()[1];

			// teamColor
			this->receivedSimLinkMsg.teamColor = buffer.data()[2];

			// refboxCommand
			this->receivedSimLinkMsg.refboxCommand = buffer.data()[3];

			// ball_xyz[3]
			// X
			char xBytes[] = {buffer.data()[10], buffer.data()[11], buffer.data()[12], buffer.data()[13]};
			memcpy(&this->receivedSimLinkMsg.ball_xyz[0], &xBytes, sizeof(this->receivedSimLinkMsg.ball_xyz[0]));

			// Y
			char yBytes[] = {buffer.data()[14], buffer.data()[15], buffer.data()[16], buffer.data()[17]};
			memcpy(&this->receivedSimLinkMsg.ball_xyz[0], &yBytes, sizeof(this->receivedSimLinkMsg.ball_xyz[0]));

			// Z
			char zBytes[] = {buffer.data()[18], buffer.data()[19], buffer.data()[20], buffer.data()[21]};
			memcpy(&this->receivedSimLinkMsg.ball_xyz[0], &zBytes, sizeof(this->receivedSimLinkMsg.ball_xyz[0]));

			std::cout << this->receivedSimLinkMsg.to_string() << std::endl;
			this->overwriteSim();
		}
		else
		{
			std::cerr << "GazeboSimLink: Received a SimLink message of wrong size. (Received Size: " << buffer.size() << " )" << std::endl;
		}
	}

	void GazeboSimLink::overwriteSim()
	{
		math::Pose ballPose;

		// pay attention to the conversion between between gazebo coords and standard coords (not ours)
		ballPose.pos.x = this->receivedSimLinkMsg.ball_xyz[1];
		ballPose.pos.y = -this->receivedSimLinkMsg.ball_xyz[0];
		ballPose.pos.z = this->receivedSimLinkMsg.ball_xyz[2];
		this->football->SetWorldPose(ballPose);
	}

	string SimLink::to_string()
	{
		stringstream ss;

		ss << "SimLink: " << this->simLinkMode << std::endl;
		ss << "blueIsHomeGoal: " << this->blueIsHomeGoal << std::endl;
		ss << "teamColor: " << this->teamColor << std::endl;
		ss << "refboxCommand: " << this->refboxCommand << std::endl;

		return ss.str();
	}

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(GazeboSimLink)
}
