#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt32.h"

class MyoMotor {
public:
	/*
  * Setup motor in position, velocity or force control mode.
  * position control mode: 0
  * velocity control mode: 1
  * effort / force control mode: 2
  */
	void setupMotorCallback(const std_msgs::UInt32::ConstPtr &controlMode) {
		setupMotor(controlMode->data);
	}
  bool setupMotor(unsigned int controlMode) {
    enum controllerOptions { position, velocity, effort = 2, force = 2 };
    switch (controlMode) {
    case position:
      flexray.initPositionControl((uint)0, (uint)0);
      ROS_INFO(
          "control_me: Set up motor 0 on ganglion 0 in position control mode.");
      break;
    case velocity:
      flexray.initVelocityControl((uint)0, (uint)0);
      ROS_INFO(
          "control_me: Set up motor 0 on ganglion 0 in velocity control mode.");
      break;
    case effort:
      flexray.initForceControl((uint)0, (uint)0);
      ROS_INFO(
          "control_me: Set up motor 0 on ganglion 0 in force control mode.");
      break;
    default:
      ROS_ERROR("control_me: Received an unknown control mode. Check the enum: "
                "position control mode: 0, velocity control mode: 1, effort / "
                "force control mode: 2 ");
      return false;
    }
    flexray.exchangeData();
    return true;
  }
  /*
  * Implements the service to move the motors.
  */
	void moveMotorCallback(const std_msgs::Float64::ConstPtr &setpoint) {
		moveMotor(setpoint->data);
	}
  bool moveMotor(double setpoint) {
    if (flexray.commandframe0[0].sp[0] = setpoint) {
      flexray.updateCommandFrame();
      flexray.exchangeData();
      return true;
    }
    return false;
  }

  double readDisplacementSensor(void) {
    flexray.exchangeData();
    return flexray.GanglionData[0].muscleState[0].tendonDisplacement / 32768.0f;
  }

private:
  FlexRayHardwareInterface flexray;
};

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command
   * line.
   * For programmatic remappings you can use a different version of init() which
   * takes
   * remappings directly, but for most command-line programs, passing argc and
   * argv is
   * the easiest way to do it.  The third argument to init() is the name of the
   * node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "tester");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the
   * last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  MyoMotor motor;
	motor.setupMotor(0);

	//ros::Publisher pub = n.advertise<std_msgs::Float64>("tester", 1000);
	ros::Subscriber setupMotorSub = n.subscribe("/setupMotor", 1000, &MyoMotor::setupMotorCallback, &motor);
	ros::Subscriber moveMotorSub = n.subscribe("/moveMotor", 1000, &MyoMotor::moveMotorCallback, &motor);
	ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::Float64 msg;
    msg.data = motor.readDisplacementSensor();

    ROS_INFO("%f", msg.data);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    //pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
	}

	return 0;
}
