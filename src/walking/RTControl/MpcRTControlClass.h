/*****************************************************************************
MpcRTControlClass.h
*****************************************************************************/
#pragma once
#include "RTControl/XBotRTControlClass.h"
#include "MPC/MPCClass.h"

#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>

const double dt_mpc = 0.05;   // definition of sampling time of MPC solover
 const double _height_offsetx = 0.05;  
const double _height_offset_time = 2; 


using namespace Eigen;
using namespace std;



class MpcRTControlClass: public XBotRTControlClass
{
public:
	MpcRTControlClass();
	~MpcRTControlClass(){};
	using XBotRTControlClass::Run;

	MPCClass mpc;

private:
	void StartWalking();
	void StopWalking();
	
	/// step parameters reference
	double stepwidthinput,steplengthinput,stepheightinput;

        int _refer_t_max;
	int _t_int;
	double _dtx;
	

	
	int _t_walkdtime_flag, _t_walkdtime_restart_flag;
	bool _stop_walking, _start_walking_again;
	
	double _ppx, _pix, _pdx,  _ppy,_piy, _pdy, _ppz,_piz,  _pdz, _ppthetax, _pdthetax, _pithetax,  _ppthetay, _pithetay,  _pdthetay,  _ppthetaz, _pithetaz,  _pdthetaz; 
	
	Eigen::VectorXd _error_com_position, _error_torso_angle;
	
	
	Eigen::VectorXd _flag_walkdtime;
	
	Eigen::VectorXd _stop_flag_walkdtime;
  
        
	Eigen::Vector3d optCoM;
	
	int _walkdtime_max, _wal_max;	
	int _walkdtime1;	
	
	Eigen::MatrixXd _COM_IN, _COM_est;
	Eigen::MatrixXd _body_IN;	
	Eigen::MatrixXd _FootR_IN;	
	Eigen::MatrixXd _FootL_IN;	

	Eigen::Matrix<double,18,1> _estimated_state;	
	Eigen::MatrixXd _estimated_state_global;
	Eigen::Vector3d _Rfoot_location_feedback,_Lfoot_location_feedback;
	
	
	
	
	Eigen::MatrixXd _state_generate_interpo;

	
// 	Eigen::MatrixXd _coeff_inte;
	

	virtual void StandingReactStepping() final;
	virtual void WalkingReactStepping() final;
	virtual void EnableStandingReact() final;
	virtual void EnableWalkingReact() final;

	virtual void InternalLoggerLoop() final;
protected:
	int t_walkdtime_flag;
	int dt_sample;
	
	Vector3d _torso_angle;
	void torso_angle(int arg1);
	
	double _feedback_lamda;
	
	
	/****KMP based trajectory***********************/
	Vector6d _kmp_leg_traje;
	
	
	Vector3d _F_r_mpc, _F_l_mpc,_M_r_mpc,_M_l_mpc;	
	
	
};

