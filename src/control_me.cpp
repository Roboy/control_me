#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"
#include "ros/ros.h"

class MyoMotor {
  /*
  * Setup motor in position, velocity or force control mode.
  * position control mode: 0
  * velocity control mode: 1
  * effort / force control mode: 2
  */
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
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the
   * last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  MyoMotor motor;
}
