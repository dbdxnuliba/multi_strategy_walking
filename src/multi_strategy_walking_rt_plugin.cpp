/*
 * Copyright (C) 2017 IIT-ADVR
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

#include <multi_strategy_walking_rt_plugin.h>

/* Specify that the class XBotPlugin::multi_strategy_walking is a XBot RT plugin with name "multi_strategy_walking" */
REGISTER_XBOT_PLUGIN(multi_strategy_walking, XBotPlugin::multi_strategy_walking)

namespace XBotPlugin {

bool multi_strategy_walking::init_control_plugin(XBot::Handle::Ptr handle)
{
	/* This function is called outside the real time loop, so we can
	 * allocate memory on the heap, print stuff, ...
	 * The RT plugin will be executed only if this init function returns true. */


	/* Save robot to a private member. */
	_robot = handle->getRobotInterface();

	_robot->getRobotState("home", _q_home);

	/* Print the homing */
 	std::cout << "_q_home from SRDF : " << _q_home.transpose() << std::endl;

	_robot->sense();
	_robot->getMotorPosition(_q0);
	_q = _q0;
	_qref = _q0;
	_q_msr = _q0;

	_q_tmp = _q0;

	_time = 0;
	_homing_time = RTControl.RobotPara().setParaForSimOrReal(1.0, 5.0);; /// homimg_time =5s

	// qall.resize(_robot->getJointNum());
	// _q_home_JOINT_NAME.resize(_robot->getJointNum());
	qall.resize(RTControl.RobotPara().JOINT_NUM());
	_q_home_JOINT_NAME.resize(RTControl.RobotPara().JOINT_NUM());

	_robot->getPositionReference(_q_name);
	ft_map = _robot->getForceTorque();
	imu_map = _robot->getImu();

	FTSensor.resize(12);
	RTControl.initOpenSoTIK(handle->getPathToConfigFile());

//#ifdef USE_ROS
//	_walking_srv = handle->getRosHandle()->advertiseService("/walking_command", &multi_strategy_walking::walking_srv_callback, this);
//#endif

	/* Initialize a logger which saves to the specified file. Remember that
	 * the current date/time is always appended to the provided filename,
	 * so that logs do not overwrite each other. */

// 	_logger = XBot::MatLogger::getLogger("/tmp/multi_strategy_walking_log");
//         DPRINTF("========= loop_run_init. =============\n");
	return true;


}

void multi_strategy_walking::on_start(double time)
{
	/* This function is called on plugin start, i.e. when the start command
	 * is sent over the plugin switch port (e.g. 'rosservice call /multi_strategy_walking_switch true').
	 * Since this function is called within the real-time loop, you should not perform
	 * operations that are not rt-safe. */

	/* Save the plugin starting time to a class member */
        //DPRINTF("========= loop_on_start. =============\n");
	_first_loop_time = time;
	_robot->sense();
	_robot->getJointPosition(_q0);
	// _robot->getPositionReference(_q0);
//         DPRINTF("========= loop_on_start. =============\n");
	//std::cout << "_q0 from motor on start: " << _q0.transpose() << std::endl;

	RTControl.SortFromXBotToJointName(_q_home, _q_home_JOINT_NAME);
	RTControl.HomingInit(_q_home_JOINT_NAME);

	if (RTControl.RobotPara().name == "walkman") {
		DPRINTF("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
		DPRINTF("\n\n\n==================================================\n");
		DPRINTF(" _       _____    __    __ __ __  ______    _   __ \n");
		DPRINTF("| |     / /   |  / /   / //_//  |/  /   |  / | / / \n");
		DPRINTF("| | /| / / /| | / /   / ,<  / /|_/ / /| | /  |/ /  \n");
		DPRINTF("| |/ |/ / ___ |/ /___/ /| |/ /  / / ___ |/ /|  /   \n");
		DPRINTF("|__/|__/_/  |_/_____/_/ |_/_/  /_/_/  |_/_/ |_/    \n");

		DPRINTF("\n==================================================\n\n\n");
	}
	else if (RTControl.RobotPara().name == "cogimon") {
		DPRINTF("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
		DPRINTF("\n\n\n=====================================================\n");
		DPRINTF("   ______                  ____  __  ___                 \n");
		DPRINTF("  / ____/ ____    ____ _  /  _/ /  |/  / ____    ____    \n");
		DPRINTF(" / /     / __ \\  / __ `/  / /  / /|_/ / / __ \\  / __ \\\n");
		DPRINTF("/ /___  / /_/ / / /_/ /  / /  / /  / / / /_/ / / / / /   \n");
		DPRINTF("\\____/  \\____/  \\__,/  /___/ /_/  /_/  \\____/ /_/ /_/\n");

		DPRINTF("\n=====================================================\n\n\n");
	}
	else if (RTControl.RobotPara().name == "coman") {
		DPRINTF("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
	}	
	/* Save the robot starting config to a class member */
	// _start_time = time;
}

void multi_strategy_walking::on_stop(double time)
{
	/* This function is called on plugin stop, i.e. when the stop command
	 * is sent over the plugin switch port (e.g. 'rosservice call /multi_strategy_walking_switch false').
	 * Since this function is called within the real-time loop, you should not perform
	 * operations that are not rt-safe. */

	_q_home = _q;
}


void multi_strategy_walking::control_loop(double time, double period)
{
	/* This function is called on every control loop from when the plugin is start until
	 * it is stopped.
	 * Since this function is called within the real-time loop, you should not perform
	 * operations that are not rt-safe. */

	updateWBS();

	// Go to homing: original verision
	if ( (time - _first_loop_time) <= _homing_time ) {
		_q = _q0 + 0.5 * (1 - std::cos(M_PI * (time - _first_loop_time) / _homing_time)) * (_q_home - _q0);
		_q_tmp = _q;
		_qref = _q;
//  		DPRINTF("=========  intial pose_control_loop. =============\n");
		
	}
	else {
/* 	        DPRINTF("=========  RUN rt_control_loop. =============\n");*/ 
		RTControl.Run();
// 		RTControl.JointRefToXBot(_q);
		RTControl.JointRefToXBot(_q_tmp);
		//// interpolation of stabilizer angle
		if ( (time - _first_loop_time -_homing_time)  <= 2){
		  _q = pow(time - _first_loop_time -_homing_time,2)/4*(_q_tmp-_qref)+_qref;
//		  _q = _q_tmp;
// 		  if ( (time - _first_loop_time -_homing_time)  <= 0.01)
// 		  {
// 		  std::cout << "_q_home from SRDF : " << _q_home.transpose() << std::endl;
// 		  std::cout << "/////////////////////////: " << std::endl;
// 		  std::cout << "_q_initial : " << _q_tmp.transpose() << std::endl;
// 		  }
		}
		else{
		  _q = _q_tmp;
		}
		
		
//  		_q.head(12) = _q_tmp.head(12); ///joint	angle of lower limb, other angle are upper angles	
//		_q = _q_tmp;	
	}
	
	
/*	// Go to homing: walking_no_arm
	if ( (time - _first_loop_time) <= _homing_time ) {
		_q = _q0;
		_q_tmp = _q0 + 0.5 * (1 - std::cos(M_PI * (time - _first_loop_time) / _homing_time)) * (_q_home - _q0);
		_q.head(12) = _q_tmp.head(12); //only lower body moven
// 		_q = _q_tmp; //whole-body movement		
		_qref = _q;
		
 		DPRINTF("=========  intial pose_control_loop. =============\n");
		
	}
	else {
 	        DPRINTF("=========  RUN rt_control_loop. =============\n"); 
		RTControl.Run();
		// RTControl.JointRefToXBot_LowerBody(_q);
		RTControl.JointRefToXBot(_q_tmp);
// 		_q.head(12) = _q_tmp.head(12); ///joint	angle of lower limb, other angle are upper angles	
		//// interpolation of stabilizer angle
		if ( (time - _first_loop_time -_homing_time)  <= 1){
		  _q.head(12) = pow(time - _first_loop_time -_homing_time,1)/1*(_q_tmp.head(12)-_qref.head(12))+_qref.head(12);
//		  cout<< "transition process"<<endl;
		}
		else{
		  _q.head(12) = _q_tmp.head(12);
// 		  _q.head(4) = -_q_tmp.head(4); 
//		  cout<< "normal process"<<endl;
		}	
	}*/	
	
	
	// _q[_q.size()-1] = _q_home[_q.size()-1];
// 	cout<<"_q:"<< _q.transpose()<<endl; 
  	_robot->setPositionReference(_q);
 	_robot->move();
}

bool multi_strategy_walking::close()
{
	/* This function is called exactly once, at the end of the experiment.
	 * It can be used to do some clean-up, or to save logging data to disk. */

	/* Save logged data to disk */
	// 	_logger->flush();

	// RTControl.savedata();
	return true;
}

void multi_strategy_walking::updateWBS()
{
//  _q_msr =_q;
 	_robot->getJointPosition(_q_msr);

	RTControl.SortFromXBotToJointName(_q_msr, qall);

	ft_map["l_leg_ft"]->getWrench(l_leg_ft);
	ft_map["r_leg_ft"]->getWrench(r_leg_ft);
	for (int i = 0; i < 6; i++) {
#ifdef REAL_ROBOT
// 		FTSensor.at(i) = -r_leg_ft(i);
// 		FTSensor.at(6 + i) = -l_leg_ft(i);
		FTSensor.at(i) = r_leg_ft(i);
		FTSensor.at(6 + i) = l_leg_ft(i);	  
#endif
#ifdef IN_GAZEBO
		FTSensor.at(i) = r_leg_ft(i);
		FTSensor.at(6 + i) = l_leg_ft(i);
#endif
	}
	// std::cout <<"l_leg_ft: "<< l_leg_ft.transpose() << std::endl;
	// std::cout <<"r_leg_ft: "<< r_leg_ft.transpose() << std::endl;
//         DPRINTF("l_leg_ft_z: %.2f\tr_leg_ft_z: %.2f\n", l_leg_ft[2], r_leg_ft[2]);
	// imu_map["imu_link"]->getOrientation(Rpelvis_abs);
	// imu_map["imu_link"]->getLinearAcceleration(LnAcc);
	// imu_map["imu_link"]->getAngularVelocity(AgVel);

	if (!_robot->getImu().empty()) {
		// imu_map["imu_link"]->getOrientation(_quaternion);
		imu_map["imu_link"]->getImuData(Rpelvis_abs, LnAcc, AgVel);
#ifdef REAL_ROBOT
		Eigen::Vector3d EulerAng(0, 0, 0);
// 		EulerAng[0] = std::atan2(Rpelvis_abs(2, 1), Rpelvis_abs(2, 2));
// 		EulerAng[1] = -std::asin(-Rpelvis_abs(2, 0));
// 		EulerAng[2] = -std::atan2(Rpelvis_abs(1, 0), Rpelvis_abs(0, 0));
//              EulerAng[0] -= sign(EulerAng[0]) * M_PI;
// 		Rpelvis_abs = Rz(EulerAng[2]) * Ry(EulerAng[1]) * Rx(EulerAng[0]);
                EulerAng[0] = std::atan2(Rpelvis_abs(2, 1), Rpelvis_abs(2, 2));
                EulerAng[1] = std::asin(-Rpelvis_abs(2, 0));
                EulerAng[2] = std::atan2(Rpelvis_abs(1, 0), Rpelvis_abs(0, 0));
//                 DPRINTF("roll: %.3f,\t pitch: %.3f,\t yaw: %.3f\n", 57.3*EulerAng[0], 57.3*EulerAng[1], 57.3*EulerAng[2]);
//                 DPRINTF("roll: %.2f,\t pitch: %.2f,\t yaw: %.2f,\t ax: %.3f,\t ay: %.3f,\t az: %.3f\n", 57.3*EulerAng[0], 57.3*EulerAng[1], 57.3*EulerAng[2], LnAcc[0], LnAcc[1], LnAcc[2]);
#endif
	}
	else {
		// _quaternion = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
		Rpelvis_abs = Eigen::Matrix3d::Identity();
		AgVel.setZero();
		LnAcc.setZero();
	}
	// Rpelvis_abs = Eigen::Matrix3d::Identity();
	// imu_map["imu_link"]->getImuData(Rpelvis_abs, LnAcc, AgVel);
	RTControl.UpdateWBS(qall, Rpelvis_abs, LnAcc, AgVel, FTSensor);

//         DPRINTF("_q_msr[3]: %.3f,\t qall[LEFT_KNEE_PITCH]: %.3f\n",_q_msr[3], qall[LEFT_KNEE_PITCH]);
}





}
