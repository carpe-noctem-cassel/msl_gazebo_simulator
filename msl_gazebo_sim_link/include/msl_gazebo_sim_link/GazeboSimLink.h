#ifndef GAZEBO_ROS_DISTANCE_SENSOR_HH
#define GAZEBO_ROS_DISTANCE_SENSOR_HH

#include <string>

// library for processing camera data for gazebo / ros conversions

#include <gazebo/gazebo.hh>
#include <SystemConfig.h>
#include <chrono>
#include <QUdpSocket>
#include <SystemConfig.h>

namespace gazebo
{
	struct SimLink
	{
		char    simLinkMode;               /* [c/h] Simulator link mode [0:host, 1:client, 2:none] */
		char    blueIsHomeGoal;            /* [c/h] Is blue goal my home goal? */
		char    teamColor;                 /* [c/h] team color on this pc */
		char    refboxCommand;             /* [h]   Refbox command from host */
		char    has_ball[6];               /* [c]   Turtle in ball possession ( 0 or 1 ) */
		float   ball_xyz[3];               /* [h]   ball position in m */
		float   ball_xyz_dot[3];           /* [h]   ball position in m */
		char    robotIsActive[6];          /* [h/c] Turtle is active (1) and in-field, otherwise 0 */
		float   robotPoses[3*6];           /* [h/c] Turtle poses [x, y, phi] */
		float   robotVelocities[2*6];      /* [h/c] Turtle velocities [x_dot, y_dot] */

		string to_string();
	};

	class GazeboSimLink : public ModelPlugin, public QObject
	{
	Q_OBJECT
	public:
		GazeboSimLink();
		virtual ~GazeboSimLink();
		virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	public Q_SLOTS:
		void receiveSimLinkMsgUdp(void);
	protected:
		QUdpSocket* udpsocket;
		void connectUDP(QString host, qint16 port);
		void disconnectUDP();
		void overwriteSim();

	private:
        physics::WorldPtr world_;
        physics::ModelPtr football;





        supplementary::SystemConfig* sc;
        QString clientAddress;
        QString hostAddress;
        qint16 port;

        bool iAmHost;

        SimLink receivedSimLinkMsg;
	};


}
#endif
