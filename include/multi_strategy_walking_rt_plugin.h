/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef multi_strategy_walking_PLUGIN_H_
#define multi_strategy_walking_PLUGIN_H_

#include <XCM/XBotControlPlugin.h>

#include <atomic>

// #include "RTControl/XBotRTControlClass.h"
// #include "RTControl/ReactXBotRTControlClass.h"
#include "RTControl/MpcRTControlClass.h"



#ifdef USE_ROS
#include <ros/ros.h>
#include <XBotCore-interfaces/XBotRosUtils.h>
#include <XBotCore-interfaces/XBotHandle.h>
//#include <ADVR_ROS/advr_locomotion.h>
#endif


namespace XBotPlugin {

enum WalkCmd{
    KeyCmd = 0,
    WalkFront = 1,
    WalkBack,
    WalkLeft,
    WalkRight,
    TurnLeft,
    TurnRight,
};

/**
 * @brief multi_strategy_walking XBot RT Plugin
 *
 **/
class multi_strategy_walking : public XBot::XBotControlPlugin
{

public:

    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual bool close();

    virtual void on_start(double time);

    virtual void on_stop(double time);

protected:

    virtual void control_loop(double time, double period);

private:

    XBot::RobotInterface::Ptr _robot;
    // XBot::ModelInterface::Ptr _model;

    double _start_time;

    Eigen::VectorXd _q0;

    XBot::MatLogger::Ptr _logger;


    // ------ homing ---------------------------
    double _time, _homing_time, _first_loop_time;
    Eigen::VectorXd _q_home, _qref;


    // -----------------------------------------
    void updateWBS();
 //   void readWalkingGoal();

    MpcRTControlClass RTControl;

    XBot::JointNameMap _q_name;

    // XBot::ForceTorqueSensor::ConstPtr lft_ft, rft_ft, larm_ft, rarm_ft;

    std::map< std::string, XBot::ForceTorqueSensor::ConstPtr > ft_map;
    std::map< std::string, XBot::ImuSensor::ConstPtr >  imu_map;

    Eigen::VectorXd _q, _q_msr;
    Eigen::VectorXd _q_tmp;

    double dt;
    std::vector<double> FTSensor, HandFTSensor, qall, _q_home_JOINT_NAME;

    Eigen::Matrix3d Rpelvis_abs;
    Eigen::Vector3d LnAcc, AgVel, EulerAng;

    Eigen::Vector6d l_leg_ft, r_leg_ft;



};

}

#endif // multi_strategy_walking_PLUGIN_H_