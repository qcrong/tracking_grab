/*
 *    Universal Robots UR5 ROS node
 *    Copyright (C) 2012 Wouter Caarls
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ur_arm/ur_arm_node.h>

using namespace ur_arm;

/// Entry point for UR5 arm controller node
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_arm_controller");

  std::string host = std::string("192.168.0.103");
  int port = 30003;

  ros::param::get("host", host);
  ros::param::get("port", port);

  Arm *arm = new Arm(host, port);
  ArmNode arm_node(arm);

/*
  //关节角速度控制
  JointSpeeds speeds;
  double a=0.2;
  double t=3;
  speeds.base=-0.1;
  speeds.shoulder=0;
  speeds.elbow=0;
  speeds.wrist1=0;
  speeds.wrist2=0;
  speeds.wrist3=0;
  arm->setJointSpeeds(speeds,a,t);
  
  ros::Duration(1).sleep();
  speeds.base=0;
  speeds.shoulder=0.1;
  arm->setJointSpeeds(speeds,a,t);
*/
 
/*
  //关节角控制
  JointAngles desirePosition; 
  double v=0.5;
  double a=0.1;
  double p[6]={-45,-90,-75,-105,90,0};
  for(int i=0;i<6;i++)
  {
    p[i]=M_PI/180*p[i];
  }
  desirePosition.base=p[0];
  desirePosition.shoulder=p[1];
  desirePosition.elbow=p[2];
  desirePosition.wrist1=p[3];
  desirePosition.wrist2=p[4];
  desirePosition.wrist3=p[5];
  
  arm->moveJoints(desirePosition,v,a);
*/
/*
  ToolTwist speed;
  double a=0.01;
  double time=6;
  speed.x=0;
  speed.y=0;
  speed.z=0;
  speed.roll=0;
  speed.pitch=0.06;
  speed.yaw=0;
  
  arm->setToolSpeed(speed,a,time);
*/



  //末端位姿控制,不要单独直接运行，容易出错, 与示教器基座、旋转矢量一致
/*
  ToolPosition desirePosition;
  double v=0.1; 
  double a=0.1;
  desirePosition.x=0.4;
  desirePosition.y=-0.2;
  desirePosition.z=0.5;
  desirePosition.roll=3.2;
  desirePosition.pitch=0.14;
  desirePosition.yaw=-0.41;
  
  arm->moveTool(desirePosition,v,a);
*/
  


  arm_node.spin();

  

  return 0;
}
