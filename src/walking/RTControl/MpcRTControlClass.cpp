/*****************************************************************************
MpcRTControlClass.cpp

*****************************************************************************/
#include "RTControl/MpcRTControlClass.h"

#include "MPC/MPCClass.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>



using namespace Eigen;
using namespace std;

MpcRTControlClass::MpcRTControlClass()
    : XBotRTControlClass()
{
#ifdef USE_XBOT_LOGGER
  int interleave = 1;
  int buffer_size = logger_len;
	xbot_logger->createVectorVariable("nlp_hipPos", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("nlp_torso_angle", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("nlp_LeftFootPos", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("nlp_RightFootPos", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("nlp_Fr_ref", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("nlp_Fl_ref", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("nlp_Mr_ref", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("nlp_Ml_ref", 3, interleave, buffer_size);		
#endif
	
  
  mpc._method_flag = 2;//for height opt:strategy: 0: reactive step; 1: reactive step+ body inclination; 2: reactive step+ body inclination+height variation;	

	
  mpc._robot_name = RobotPara().name;
  mpc._robot_mass = RobotPara().totalmass;
  mpc._lift_height = 0.05;
//  mpc._tstep = RobotPara().Tstep;
  
  // initialization
  // input step parameters
  stepwidthinput = RobotParaClass::HALF_HIP_WIDTH()*2; 
  
  if (RobotPara().name == "coman")
  {
      steplengthinput = 0.1;
  }
  else if (RobotPara().name == "bigman")
  {
    steplengthinput = 0.2;
  }
  else if (RobotPara().name == "cogimon")
   {
    steplengthinput = 0.1;
  } 
  else
  {DPRINTF("Errorrrrrrrr for IK\n");}

  stepheightinput = 0.0;  	  
  mpc.FootStepInputs(stepwidthinput, steplengthinput, stepheightinput);
  
  
//   cout<<"next: MPC initialization"<<endl;
  // offline initialization
  mpc.Initialize();
//   cout<<"Finish: MPC initialization"<<endl;  
  
  _refer_t_max = mpc.Get_maximal_number_reference();

 _t_int = 0;
  _t_walkdtime_flag = 0;
  _t_walkdtime_restart_flag = 0;
  _walkdtime1 =0;
 
 _stop_walking = false;
 _start_walking_again = false;
 
  // loop numbern generation
  _dtx = RobotParaClass::dT();    
  _walkdtime_max = mpc.Get_maximal_number(_dtx)+1;
  _wal_max = _walkdtime_max;
  
  _flag_walkdtime.setZero(_walkdtime_max);
  _stop_flag_walkdtime.setZero(_walkdtime_max);
    
  _estimated_state.setZero();
  
  _Rfoot_location_feedback.setZero();
  _Lfoot_location_feedback.setZero(); 
  
  
  _COM_IN.setZero(3,_walkdtime_max);
  _COM_IN(2,0) = RobotParaClass::Z_C()-_height_offsetx;
  _COM_IN(2,1) = RobotParaClass::Z_C()-_height_offsetx;
  _COM_est.setZero(3,_walkdtime_max); 
  _body_IN.setZero(3,_walkdtime_max);
  _FootR_IN.setZero(3,_walkdtime_max);
  _FootR_IN(1,0) = -RobotParaClass::HALF_HIP_WIDTH();
  _FootR_IN(1,1) = -RobotParaClass::HALF_HIP_WIDTH();
  _FootL_IN.setZero(3,_walkdtime_max);	
  _FootL_IN(1,0) = RobotParaClass::HALF_HIP_WIDTH();
  _FootL_IN(1,1) = RobotParaClass::HALF_HIP_WIDTH(); 
   

//// parameters for local coordinate  
  _ppx = 0.01; _pdx = 0.001;     _pix = 0.0001;
  _ppy = 0.02;  _pdy = 0.001;    _piy = 0.00001;
  _ppz = 0.01;  _pdz = 0.001;   _piz = 0.000001; 
  
  _ppthetax= 0.01; _pdthetax =0; _pithetax =0.0001;
  _ppthetay= 0.01; _pdthetay = 0;_pithetax =0.0001; 
  _ppthetaz= 0.1; _pdthetaz = 0.001;_pithetaz =0.0001;     
  
  _error_com_position.setZero(3);
  _error_torso_angle.setZero(3);
  
  _feedback_lamda = 0;
  
  
  /////*************leg***************************//////
  _kmp_leg_traje.setZero();
  
  _F_r_mpc.setZero();
  _F_l_mpc.setZero();
  _M_r_mpc.setZero();
  _M_l_mpc.setZero();    
  
  _F_r_mpc(2) = 9.8/2*RobotPara().totalmass;
  _F_l_mpc(2) = 9.8/2*RobotPara().totalmass;  
  
  
}

void MpcRTControlClass::StandingReactStepping()
{

}

void MpcRTControlClass::WalkingReactStepping()
{
  // this is the loop for normal walking
  _walkdtime1 = walkdtime - _t_walkdtime_restart_flag;

  clock_t t_finish;
	
  if(IsStartWalk)
  {
    if (!_start_walking_again)  ////in this case, robot start first walking process///////
    {
      if (_walkdtime1*_dtx<=_height_offset_time)
      {      
	PelvisPos = mpc.X_CoM_position_squat(_walkdtime1, _dtx);   
//	body_thetax.setZero();	  
	body_thetax(0) = 0;  body_thetax(1) = 0; body_thetax(2) = 0;	  
	LeftFootPosx(0) =0;    LeftFootPosx(1) = RobotParaClass::HALF_HIP_WIDTH();   LeftFootPosx(2) =0;
	RightFootPosx(0) = 0;  RightFootPosx(1) = -RobotParaClass::HALF_HIP_WIDTH(); RightFootPosx(2) = 0;	      


	ZMPxy_realx = zmp_ref = (LeftFootPosx+RightFootPosx)/2;
	F_R.setZero();
	_F_r_mpc.setZero();
	F_L.setZero(); 
	_F_l_mpc.setZero();
	F_R(2) = _F_r_mpc(2) = 9.8/2*RobotPara().totalmass;
	F_L(2) = _F_l_mpc(2) = 9.8/2*RobotPara().totalmass;
	M_R.setZero();
	M_L.setZero();
	
	j_count = 1;
	bjx1 = 1;
	
	tx = 1;
	td = 0;	
	
//         DPRINTF("========= robot squat =============\n");     
	 // // cout<<"PelvisPos_height:"<<PelvisPos(2)<<endl;		    
      }
      else
      {
//	DPRINTF("=========normal walking=============\n"); 
	
	_walkdtime1 -= (int)_height_offset_time/_dtx;	      
	if(_walkdtime1 < _walkdtime_max)
	{
            
	  _t_walkdtime_flag = _walkdtime1;	  		
	  _t_int = floor(_walkdtime1 * _dtx / dt_mpc);
	  
	  
	  
	  if (_t_int >=1)
	  {
	    _flag_walkdtime(_walkdtime1) = _t_int;
	    
	    if (_flag_walkdtime(_walkdtime1 -1) < _t_int)
	    {
 //             DPRINTF("=========Enter NLP optimization=============\n");   
	      mpc.step_timing_opti_loop(_t_int, _estimated_state,_Rfoot_location_feedback,_Lfoot_location_feedback,_feedback_lamda,_stop_walking);
 //             DPRINTF("=========Finish NLP optimization=============\n"); 
	      mpc.Foot_trajectory_solve(_t_int, _stop_walking);
 //             DPRINTF("=========Finish Foot_trajectory trajectory=============\n"); 
// 		    /////////////////////  CoM height generation, when no mpc                                  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	      ///////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	      mpc.CoM_height_solve(_t_int, _stop_walking);
	    }
	  }
// 	  // cout << "generation complete!!"<<endl;
// // get the optimized footsteps: second method: use the generated reference trajectory at each 0.05s and intepolated trajectory : used in real time
	    Eigen::Vector3d COM_in1, COM_in2, COM_in3;
	    Eigen::Vector3d body_in1, body_in2, body_in3;
	    Eigen::Vector3d FootL_in1, FootL_in2, FootL_in3;
	    Eigen::Vector3d FootR_in1, FootR_in2, FootR_in3;
	    
	  if (_walkdtime1>=2){
	    COM_in1 = _COM_IN.col(_walkdtime1-2);
	    COM_in2 = _COM_IN.col(_walkdtime1-1);
	    COM_in3 = _COM_IN.col(_walkdtime1);

	    body_in1 = _body_IN.col(_walkdtime1-2);
	    body_in2 = _body_IN.col(_walkdtime1-1);
	    body_in3 = _body_IN.col(_walkdtime1);

	    FootL_in1 = _FootL_IN.col(_walkdtime1-2);
	    FootL_in2 = _FootL_IN.col(_walkdtime1-1);
	    FootL_in3 = _FootL_IN.col(_walkdtime1);	
	    
	    FootR_in1 = _FootR_IN.col(_walkdtime1-2);
	    FootR_in2 = _FootR_IN.col(_walkdtime1-1);
	    FootR_in3 = _FootR_IN.col(_walkdtime1);		  
	    
	  }
	  else{
	    COM_in1.setZero();
	    COM_in2.setZero();
	    COM_in3 = _COM_IN.col(_walkdtime1);	
	    
	    body_in1.setZero();
	    body_in2.setZero();
	    body_in3 = _body_IN.col(_walkdtime1);	
	    
	    FootL_in1.setZero();
	    FootL_in2.setZero();
	    FootL_in3 = _FootL_IN.col(_walkdtime1);
	    
	    FootR_in1.setZero();
	    FootR_in2.setZero();
	    FootR_in3 = _FootR_IN.col(_walkdtime1);			  
	    
	  }
	  
	  PelvisPos = mpc.XGetSolution_CoM_position(_walkdtime1, _dtx,COM_in1,COM_in2,COM_in3);  
 //         DPRINTF("=========Finish PelvisPos trajectory generation=============\n"); 
	  body_thetax = mpc.XGetSolution_body_inclination(_walkdtime1, _dtx, body_in1,body_in2,body_in3);
 //         DPRINTF("=========Finish body inclination generation=============\n"); 
 		LeftFootPosx = mpc.XGetSolution_Foot_positionL(_walkdtime1, _dtx, FootL_in1,FootL_in2,FootL_in3);
 		RightFootPosx = mpc.XGetSolution_Foot_positionR(_walkdtime1, _dtx, FootR_in1,FootR_in2,FootR_in3);
  
// 	  /////*****************leg trajectory generated by KMP********************************/////
// 	  _kmp_leg_traje = mpc.XGetSolution_Foot_position_KMP(_walkdtime1,_dtx,_t_int,_stop_walking);
// 	  DPRINTF("=========Finish KMP foot trajectory generation=============\n"); 
//           
// 	  RightFootPosx(0) = _kmp_leg_traje(0);  RightFootPosx(1) = _kmp_leg_traje(1); RightFootPosx(2) = _kmp_leg_traje(2);	
// 	  LeftFootPosx(0) = _kmp_leg_traje(3);   LeftFootPosx(1) = _kmp_leg_traje(4);  LeftFootPosx(2) = _kmp_leg_traje(5);			  

          
	  HipO_Turn = Rz(body_thetax[2])*Ry(body_thetax[1])*Rx(body_thetax[0]);	
	  
	  mpc.Zmp_distributor(_walkdtime1,_dtx);
	  
	  ZMPxy_realx = zmp_ref = mpc._ZMPxy_realx;
	  F_R = _F_r_mpc = mpc._F_R;
	  F_L = _F_l_mpc = mpc._F_L;
	  M_R = _M_r_mpc = mpc._M_R;
	  M_L = _M_l_mpc = mpc._M_L;
	  
	  j_count = mpc._j_count;
	  bjx1 = mpc._bjx1;
	  if (bjx1>=1)
	  {		    
	    tx = mpc._tx(bjx1-1);
	    td = mpc._td(bjx1-1);			  
	  }
	  else
	  {		    
	    tx = 0;
	    td = 0;			  
	  }
	  
	  // store
	  _COM_IN(0,_walkdtime1) = PelvisPos(0);
	  _COM_IN(1,_walkdtime1) = PelvisPos(1);
	  _COM_IN(2,_walkdtime1) = PelvisPos(2);
	  
// 	  // cout << "com state"<<endl;
	  
	  _body_IN(0,_walkdtime1) = body_thetax(0);
	  _body_IN(1,_walkdtime1) = body_thetax(1);
	  _body_IN(2,_walkdtime1) = body_thetax(2);	  
	  
	  _FootL_IN(0,_walkdtime1) = LeftFootPosx(0);
	  _FootL_IN(1,_walkdtime1) = LeftFootPosx(1);
	  _FootL_IN(2,_walkdtime1) = LeftFootPosx(2);	

	  _FootR_IN(0,_walkdtime1) = RightFootPosx(0);
	  _FootR_IN(1,_walkdtime1) = RightFootPosx(1);
	  _FootR_IN(2,_walkdtime1) = RightFootPosx(2);
	      
		  
	  
	  ////// simulator: CoM pelvis_position && velocity:
	  /////////// can be replaced by HipPos;:
	  const RobotStateClass& irobot = _WBS.getRobotState();


	  Eigen::Vector3d hip_l = irobot.glft - irobot.IMU_abs * irobot.Lft;
	  Eigen::Vector3d hip_r = irobot.grft - irobot.IMU_abs * irobot.Rft;
	  double Fz_ratio_l = irobot.Fzl / (irobot.Fzl + irobot.Fzr);
	  double Fz_ratio_r = irobot.Fzr / (irobot.Fzl + irobot.Fzr);
	  Eigen::Vector3d hip_pos = Fz_ratio_l * hip_l + Fz_ratio_r * hip_r;
	  
	  _Rfoot_location_feedback = irobot.grft;
	  _Lfoot_location_feedback = irobot.glft;
	  
	      
	  
	  Eigen::Vector3d IMU_Euler = irobot.IMU_Euler;
	  Eigen::Vector3d IMU_AngularVel = irobot.IMU_AngularVel;
	  Eigen::Vector3d IMU_AngularAcc = irobot.IMU_AngularAcc;

	  _estimated_state(0,0) = irobot.gcom[0];  
	  _estimated_state(1,0) = irobot.gdcom[0];
	  _estimated_state(2,0) = irobot.gddcom[0];	    
	  
	  _estimated_state(3,0) = irobot.gcom[1];
	  _estimated_state(4,0) = irobot.gdcom[1];
	  _estimated_state(5,0) = irobot.gddcom[1];	    
	  _estimated_state(6,0) = irobot.gcom[2];
	  _estimated_state(7,0) = irobot.gdcom[2];
	  _estimated_state(8,0) = irobot.gddcom[2];
	  
	  _estimated_state(9,0) = IMU_Euler[0];
	  _estimated_state(10,0) = IMU_AngularVel[0];
	  _estimated_state(11,0) = IMU_AngularAcc[0];
	  
	  _estimated_state(12,0) = IMU_Euler[1];
	  _estimated_state(13,0) = IMU_AngularVel[1];	   
	  _estimated_state(14,0) = IMU_AngularAcc[1];	    
	  
	  _estimated_state(15,0) = IMU_Euler[2];
	  _estimated_state(16,0) = IMU_AngularVel[2];	   
	  _estimated_state(17,0) = IMU_AngularAcc[2];		    
	  
	  
	  /// CoM position: using the hip position  
// 	    _estimated_state(3,0) = hip_pos[1]; 
	  _estimated_state(6,0) = hip_pos[2];	    	   
	  
	  
// 	      /// PD control of the reference trajectory
//	  if (_walkdtime1 >1)
// 	  {
// /*		  _error_com_position(0) += (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0));
// 	    _error_com_position(1) += (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0));
// 	    _error_com_position(2) += (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0));
// 	    
// 	    _error_torso_angle(0) += (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0));
// 	    _error_torso_angle(1) += (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0));
// 	    _error_torso_angle(2) += (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0));
// 	    
//   */	  
// //		  PelvisPos(0) += (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0)) * _ppx + (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0))/_dtx * _pdx + _error_com_position(0)*_pix;  
// //		  PelvisPos(1) += (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0)) * _ppy + (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0))/_dtx * _pdy + _error_com_position(1)*_piy;  
// //		  PelvisPos(2) += (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0)) * _ppz + (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0))/_dtx * _pdz + _error_com_position(2)*_piz;
// 	      
// /*            PelvisPos(0) += (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0)) * _ppx + (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0))/_dtx * _pdx *10 + _error_com_position(0)*_pix *10;  
// 	    PelvisPos(1) += (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0)) * _ppy + (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0))/_dtx * _pdy + _error_com_position(1)*_piy;  
// 	    PelvisPos(2) += (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0)) * _ppz + (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0))/_dtx * _pdz + _error_com_position(2)*_piz;*/	// hip_pos_test     
// 
// /*		  body_thetax(0) += (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0))*_ppthetax + (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0))/_dtx*_pdthetax  + _error_torso_angle(0)*_pithetax;  
// 	    body_thetax(1) += (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0))*_ppthetay + (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0))/_dtx*_pdthetay+ _error_torso_angle(1)*_pithetay;  
// 	    body_thetax(2) += (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0))*_ppthetaz + (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0))/_dtx*_pdthetaz+ _error_torso_angle(2)*_pithetaz;  
// 	    
// 	    HipO_Turn = Rz(body_thetax[1])*Ry(body_thetax[1])*Rx(body_thetax[0]);	*/	    
// 	  }


	
	}
	else  //walking beyond time counter
	{
	  
// 	  DPRINTF("=========Finish normal walking=============\n"); 
//	  DPRINTF("=========!!!!!!!!!!!!!!!!1=============\n"); 
	  _t_walkdtime_restart_flag = walkdtime;	  
	  IsStartWalk = false;	  
	  _t_walkdtime_restart_flag = walkdtime;	  
	}      
      }      
    
      
    }
    else   /////// stop walking =====> start walking again/// this time: the stop motion should be redesigned
    {
	//////// stop walking: firstly, judge if the stop walking is enables
	if (_stop_walking)
	{

	  if (_t_int>=10)  ////stop
	  {
	/// the last four step      
	    _stop_flag_walkdtime(_t_int) = 1;
	    if (_stop_flag_walkdtime(_t_int)>_stop_flag_walkdtime(_t_int-1))
	    {
	      mpc.Indexfind((_t_int-1)*dt_mpc,0);	      	       
	      _wal_max = round(mpc._tx(mpc._j_period)/_dtx)+ 4 * round(RobotPara().Tstep/_dtx);		       
	      mpc._j_period = 0;	       
	    }
			      
	    if (_wal_max > _walkdtime_max){
	      _wal_max = _walkdtime_max; 	      
	    }
	    
	    /// stop walking///////////////////// 	      
	    if(_walkdtime1 < _wal_max)
	    {	        

	    
	      _t_int = floor(_walkdtime1 * _dtx / dt_mpc);
	      
	      if (_t_int >=1)
	      {
		_flag_walkdtime(_walkdtime1) = _t_int;
		
		if (_flag_walkdtime(_walkdtime1 -1) < _t_int)
		{		
		  mpc.step_timing_opti_loop(_t_int, _estimated_state,_Rfoot_location_feedback,_Lfoot_location_feedback,_feedback_lamda,_stop_walking);	
    /*		    mpc.CoM_foot_trajection_generation_local(_t_int, _estimated_state,_Rfoot_location_feedback,_Lfoot_location_feedback,_feedback_lamda,_stop_walking);	*/	
		  mpc.Foot_trajectory_solve(_t_int, _stop_walking);	
    // 	      // cout << "walking ref generation"<<endl;	  
    // 		    /////////////////////  CoM height generation, when no mpc                                  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		  ///////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		  mpc.CoM_height_solve(_t_int, _stop_walking);		    
		}
	      }
    // 	  // cout << "generation complete!!"<<endl;
    // // get the optimized footsteps: second method: use the generated reference trajectory at each 0.05s and intepolated trajectory : used in real time
		Eigen::Vector3d COM_in1, COM_in2, COM_in3;
		Eigen::Vector3d body_in1, body_in2, body_in3;
		Eigen::Vector3d FootL_in1, FootL_in2, FootL_in3;
		Eigen::Vector3d FootR_in1, FootR_in2, FootR_in3;
		
	      if (_walkdtime1>=2){
		COM_in1 = _COM_IN.col(_walkdtime1-2);
		COM_in2 = _COM_IN.col(_walkdtime1-1);
		COM_in3 = _COM_IN.col(_walkdtime1);

		body_in1 = _body_IN.col(_walkdtime1-2);
		body_in2 = _body_IN.col(_walkdtime1-1);
		body_in3 = _body_IN.col(_walkdtime1);

		FootL_in1 = _FootL_IN.col(_walkdtime1-2);
		FootL_in2 = _FootL_IN.col(_walkdtime1-1);
		FootL_in3 = _FootL_IN.col(_walkdtime1);	
		
		FootR_in1 = _FootR_IN.col(_walkdtime1-2);
		FootR_in2 = _FootR_IN.col(_walkdtime1-1);
		FootR_in3 = _FootR_IN.col(_walkdtime1);		  
		
	      }
	      else{
		COM_in1.setZero();
		COM_in2.setZero();
		COM_in3 = _COM_IN.col(_walkdtime1);	
		
		body_in1.setZero();
		body_in2.setZero();
		body_in3 = _body_IN.col(_walkdtime1);	
		
		FootL_in1.setZero();
		FootL_in2.setZero();
		FootL_in3 = _FootL_IN.col(_walkdtime1);
		
		FootR_in1.setZero();
		FootR_in2.setZero();
		FootR_in3 = _FootR_IN.col(_walkdtime1);			  
		
	      }
	      
	      PelvisPos = mpc.XGetSolution_CoM_position(_walkdtime1, _dtx,COM_in1,COM_in2,COM_in3);     
	      body_thetax = mpc.XGetSolution_body_inclination(_walkdtime1, _dtx, body_in1,body_in2,body_in3);	  
    		LeftFootPosx = mpc.XGetSolution_Foot_positionL(_walkdtime1, _dtx, FootL_in1,FootL_in2,FootL_in3);
    		RightFootPosx = mpc.XGetSolution_Foot_positionR(_walkdtime1, _dtx, FootR_in1,FootR_in2,FootR_in3);
	      
	      
	      
	      /////*****************leg trajectory generated by KMP********************************/////
// 	      _kmp_leg_traje = mpc.XGetSolution_Foot_position_KMP(_walkdtime1,_dtx,_t_int,_stop_walking);
// 	      
// 	      RightFootPosx(0) = _kmp_leg_traje(0);  RightFootPosx(1) = _kmp_leg_traje(1); RightFootPosx(2) = _kmp_leg_traje(2);	
// 	      LeftFootPosx(0) = _kmp_leg_traje(3);   LeftFootPosx(1) = _kmp_leg_traje(4);  LeftFootPosx(2) = _kmp_leg_traje(5);
	      
	      
	      
	      // store
	      _COM_IN(0,_walkdtime1) = PelvisPos(0);
	      _COM_IN(1,_walkdtime1) = PelvisPos(1);
	      _COM_IN(2,_walkdtime1) = PelvisPos(2);
	      
    // 	  // cout << "com state"<<endl;
	      
	      _body_IN(0,_walkdtime1) = body_thetax(0);
	      _body_IN(1,_walkdtime1) = body_thetax(1);
	      _body_IN(2,_walkdtime1) = body_thetax(2);	  
	      
	      _FootL_IN(0,_walkdtime1) = LeftFootPosx(0);
	      _FootL_IN(1,_walkdtime1) = LeftFootPosx(1);
	      _FootL_IN(2,_walkdtime1) = LeftFootPosx(2);	

	      _FootR_IN(0,_walkdtime1) = RightFootPosx(0);
	      _FootR_IN(1,_walkdtime1) = RightFootPosx(1);
	      _FootR_IN(2,_walkdtime1) = RightFootPosx(2);
	          
		      
	      
	      ////// simulator: CoM pelvis_position && velocity:
	      /////////// can be replaced by HipPos;:
	      const RobotStateClass& irobot = _WBS.getRobotState();


	      Eigen::Vector3d hip_l = irobot.glft - irobot.IMU_abs * irobot.Lft;
	      Eigen::Vector3d hip_r = irobot.grft - irobot.IMU_abs * irobot.Rft;
	      double Fz_ratio_l = irobot.Fzl / (irobot.Fzl + irobot.Fzr);
	      double Fz_ratio_r = irobot.Fzr / (irobot.Fzl + irobot.Fzr);
	      Eigen::Vector3d hip_pos = Fz_ratio_l * hip_l + Fz_ratio_r * hip_r;
	      
	      _Rfoot_location_feedback = irobot.grft;
	      _Lfoot_location_feedback = irobot.glft;
	      
		  
	      
	      Eigen::Vector3d IMU_Euler = irobot.IMU_Euler;
	      Eigen::Vector3d IMU_AngularVel = irobot.IMU_AngularVel;
	      Eigen::Vector3d IMU_AngularAcc = irobot.IMU_AngularAcc;

	      _estimated_state(0,0) = irobot.gcom[0];  
	      _estimated_state(1,0) = irobot.gdcom[0];
	      _estimated_state(2,0) = irobot.gddcom[0];	    
	      
	      _estimated_state(3,0) = irobot.gcom[1];
	      _estimated_state(4,0) = irobot.gdcom[1];
	      _estimated_state(5,0) = irobot.gddcom[1];	    
	      _estimated_state(6,0) = irobot.gcom[2];
	      _estimated_state(7,0) = irobot.gdcom[2];
	      _estimated_state(8,0) = irobot.gddcom[2];
	      
	      _estimated_state(9,0) = IMU_Euler[0];
	      _estimated_state(10,0) = IMU_AngularVel[0];
	      _estimated_state(11,0) = IMU_AngularAcc[0];
	      
	      _estimated_state(12,0) = IMU_Euler[1];
	      _estimated_state(13,0) = IMU_AngularVel[1];	   
	      _estimated_state(14,0) = IMU_AngularAcc[1];	    
	      
	      _estimated_state(15,0) = IMU_Euler[2];
	      _estimated_state(16,0) = IMU_AngularVel[2];	   
	      _estimated_state(17,0) = IMU_AngularAcc[2];		    
	      
	      
	      /// CoM position: using the hip position  
    // 	    _estimated_state(3,0) = hip_pos[1];	    
	      _estimated_state(6,0) = hip_pos[2];	    	   
	      
	      
    // 	      /// PD control of the reference trajectory
    // 		if (_walkdtime1 >1)
    // 		{
    // 		  _error_com_position(0) += (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0));
    // 		  _error_com_position(1) += (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0));
    // 		  _error_com_position(2) += (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0));
    // 		  
    // 		  _error_torso_angle(0) += (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0));
    // 		  _error_torso_angle(1) += (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0));
    // 		  _error_torso_angle(2) += (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0));
    // 		  
    // 		  
    // 		  PelvisPos(0) += (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0)) * _ppx + (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0))/_dtx * _pdx + _error_com_position(0)*_pix;  
    // 		  PelvisPos(1) += (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0)) * _ppy + (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0))/_dtx * _pdy + _error_com_position(1)*_piy;  
    // 		  PelvisPos(2) += (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0)) * _ppz + (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0))/_dtx * _pdz + _error_com_position(2)*_piz;
    // 		    
    //       /*            PelvisPos(0) += (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0)) * _ppx + (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0))/_dtx * _pdx *10 + _error_com_position(0)*_pix *10;  
    // 		  PelvisPos(1) += (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0)) * _ppy + (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0))/_dtx * _pdy + _error_com_position(1)*_piy;  
    // 		  PelvisPos(2) += (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0)) * _ppz + (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0))/_dtx * _pdz + _error_com_position(2)*_piz;*/	// hip_pos_test     
    // 
    // 		  body_thetax(0) += (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0))*_ppthetax + (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0))/_dtx*_pdthetax  + _error_torso_angle(0)*_pithetax;  
    // 		  body_thetax(1) += (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0))*_ppthetay + (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0))/_dtx*_pdthetay+ _error_torso_angle(1)*_pithetay;  
    // 		  body_thetax(2) += (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0))*_ppthetaz + (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0))/_dtx*_pdthetaz+ _error_torso_angle(2)*_pithetaz;  
    // 		  
    // 		  HipO_Turn = Rz(body_thetax[1])*Ry(body_thetax[1])*Rx(body_thetax[0]);		    
    // 		}
    // 		


	      
	    
	    }
	    else
	    {
	      _t_walkdtime_restart_flag = walkdtime;
	      
	
	    }		      
	  } 
	  else
	  {
		_t_walkdtime_restart_flag = walkdtime;    
	  }

	  
	}  
	else   ///////normal walking ===============================
	{	      
	    if(_walkdtime1 < _walkdtime_max)
	    {
	      _t_walkdtime_flag = _walkdtime1;	  		
	      _t_int = floor(_walkdtime1 * _dtx / dt_mpc);

	      
	      if (_t_int >=1)
	      {
		_flag_walkdtime(_walkdtime1) = _t_int;
		
		if (_flag_walkdtime(_walkdtime1 -1) < _t_int)
		{	
		  mpc.step_timing_opti_loop(_t_int, _estimated_state,_Rfoot_location_feedback,_Lfoot_location_feedback,_feedback_lamda,_stop_walking);			    	
		  mpc.Foot_trajectory_solve(_t_int, _stop_walking);
    // 		    /////////////////////  CoM height generation, when no mpc                                  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		  ///////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		  mpc.CoM_height_solve(_t_int, _stop_walking);
		}
	      }
    // 	  // cout << "generation complete!!"<<endl;
    // // get the optimized footsteps: second method: use the generated reference trajectory at each 0.05s and intepolated trajectory : used in real time
		Eigen::Vector3d COM_in1, COM_in2, COM_in3;
		Eigen::Vector3d body_in1, body_in2, body_in3;
		Eigen::Vector3d FootL_in1, FootL_in2, FootL_in3;
		Eigen::Vector3d FootR_in1, FootR_in2, FootR_in3;
		
	      if (_walkdtime1>=2){
		COM_in1 = _COM_IN.col(_walkdtime1-2);
		COM_in2 = _COM_IN.col(_walkdtime1-1);
		COM_in3 = _COM_IN.col(_walkdtime1);

		body_in1 = _body_IN.col(_walkdtime1-2);
		body_in2 = _body_IN.col(_walkdtime1-1);
		body_in3 = _body_IN.col(_walkdtime1);

		FootL_in1 = _FootL_IN.col(_walkdtime1-2);
		FootL_in2 = _FootL_IN.col(_walkdtime1-1);
		FootL_in3 = _FootL_IN.col(_walkdtime1);	
		
		FootR_in1 = _FootR_IN.col(_walkdtime1-2);
		FootR_in2 = _FootR_IN.col(_walkdtime1-1);
		FootR_in3 = _FootR_IN.col(_walkdtime1);		  
		
	      }
	      else{
		COM_in1.setZero();
		COM_in2.setZero();
		COM_in3 = _COM_IN.col(_walkdtime1);	
		
		body_in1.setZero();
		body_in2.setZero();
		body_in3 = _body_IN.col(_walkdtime1);	
		
		FootL_in1.setZero();
		FootL_in2.setZero();
		FootL_in3 = _FootL_IN.col(_walkdtime1);
		
		FootR_in1.setZero();
		FootR_in2.setZero();
		FootR_in3 = _FootR_IN.col(_walkdtime1);			  
		
	      }
	      
	      PelvisPos = mpc.XGetSolution_CoM_position(_walkdtime1, _dtx,COM_in1,COM_in2,COM_in3);     	      	
	      body_thetax = mpc.XGetSolution_body_inclination(_walkdtime1, _dtx, body_in1,body_in2,body_in3);	  
/*    		LeftFootPosx = mpc.XGetSolution_Foot_positionL(_walkdtime1, _dtx, FootL_in1,FootL_in2,FootL_in3);
    		RightFootPosx = mpc.XGetSolution_Foot_positionR(_walkdtime1, _dtx, FootR_in1,FootR_in2,FootR_in3);
   */   
// 	      /////*****************leg trajectory generated by KMP********************************/////
	      _kmp_leg_traje = mpc.XGetSolution_Foot_position_KMP(_walkdtime1,_dtx,_t_int,_stop_walking);
	      
	      RightFootPosx(0) = _kmp_leg_traje(0);  RightFootPosx(1) = _kmp_leg_traje(1); RightFootPosx(2) = _kmp_leg_traje(2);	
	      LeftFootPosx(0) = _kmp_leg_traje(3);   LeftFootPosx(1) = _kmp_leg_traje(4);  LeftFootPosx(2) = _kmp_leg_traje(5);		
	      
    // 		// cout<<LeftFootPosx(1)<<endl;
    // 		// cout<<RightFootPosx(1)<<endl;
	      
	      // store
	      _COM_IN(0,_walkdtime1) = PelvisPos(0);
	      _COM_IN(1,_walkdtime1) = PelvisPos(1);
	      _COM_IN(2,_walkdtime1) = PelvisPos(2);
	      
    // 	  // cout << "com state"<<endl;
	      
	      _body_IN(0,_walkdtime1) = body_thetax(0);
	      _body_IN(1,_walkdtime1) = body_thetax(1);
	      _body_IN(2,_walkdtime1) = body_thetax(2);	  
	      
	      _FootL_IN(0,_walkdtime1) = LeftFootPosx(0);
	      _FootL_IN(1,_walkdtime1) = LeftFootPosx(1);
	      _FootL_IN(2,_walkdtime1) = LeftFootPosx(2);	

	      _FootR_IN(0,_walkdtime1) = RightFootPosx(0);
	      _FootR_IN(1,_walkdtime1) = RightFootPosx(1);
	      _FootR_IN(2,_walkdtime1) = RightFootPosx(2);

	      
	      ////// simulator: CoM pelvis_position && velocity:
	      /////////// can be replaced by HipPos;:
	      const RobotStateClass& irobot = _WBS.getRobotState();


	      Eigen::Vector3d hip_l = irobot.glft - irobot.IMU_abs * irobot.Lft;
	      Eigen::Vector3d hip_r = irobot.grft - irobot.IMU_abs * irobot.Rft;
	      double Fz_ratio_l = irobot.Fzl / (irobot.Fzl + irobot.Fzr);
	      double Fz_ratio_r = irobot.Fzr / (irobot.Fzl + irobot.Fzr);
	      Eigen::Vector3d hip_pos = Fz_ratio_l * hip_l + Fz_ratio_r * hip_r;
	      
	      _Rfoot_location_feedback = irobot.grft;
	      _Lfoot_location_feedback = irobot.glft;
	      
		  
	      
	      Eigen::Vector3d IMU_Euler = irobot.IMU_Euler;
	      Eigen::Vector3d IMU_AngularVel = irobot.IMU_AngularVel;
	      Eigen::Vector3d IMU_AngularAcc = irobot.IMU_AngularAcc;

	      _estimated_state(0,0) = irobot.gcom[0];  
	      _estimated_state(1,0) = irobot.gdcom[0];
	      _estimated_state(2,0) = irobot.gddcom[0];	    
	      
	      _estimated_state(3,0) = irobot.gcom[1];
	      _estimated_state(4,0) = irobot.gdcom[1];
	      _estimated_state(5,0) = irobot.gddcom[1];	    
	      _estimated_state(6,0) = irobot.gcom[2];
	      _estimated_state(7,0) = irobot.gdcom[2];
	      _estimated_state(8,0) = irobot.gddcom[2];
	      
	      _estimated_state(9,0) = IMU_Euler[0];
	      _estimated_state(10,0) = IMU_AngularVel[0];
	      _estimated_state(11,0) = IMU_AngularAcc[0];
	      
	      _estimated_state(12,0) = IMU_Euler[1];
	      _estimated_state(13,0) = IMU_AngularVel[1];	   
	      _estimated_state(14,0) = IMU_AngularAcc[1];	    
	      
	      _estimated_state(15,0) = IMU_Euler[2];
	      _estimated_state(16,0) = IMU_AngularVel[2];	   
	      _estimated_state(17,0) = IMU_AngularAcc[2];		    
	      
	      
	      /// CoM position: using the hip position  
    // 	    _estimated_state(3,0) = hip_pos[1];
    // discussion	    
	      _estimated_state(6,0) = hip_pos[2];	    	   
	      
	      
//     // 	      /// PD control of the reference trajectory
// 	      if (_walkdtime1 >1)
// 	      {
//     /*		  _error_com_position(0) += (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0));
// 		_error_com_position(1) += (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0));
// 		_error_com_position(2) += (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0));
// 		
// 		_error_torso_angle(0) += (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0));
// 		_error_torso_angle(1) += (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0));
// 		_error_torso_angle(2) += (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0));
// 		
//       */	  
//     //		  PelvisPos(0) += (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0)) * _ppx + (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0))/_dtx * _pdx + _error_com_position(0)*_pix;  
//     //		  PelvisPos(1) += (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0)) * _ppy + (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0))/_dtx * _pdy + _error_com_position(1)*_piy;  
//     //		  PelvisPos(2) += (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0)) * _ppz + (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0))/_dtx * _pdz + _error_com_position(2)*_piz;
// 		  
//     /*            PelvisPos(0) += (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0)) * _ppx + (_COM_IN(0,_walkdtime1-1)-_estimated_state(0,0))/_dtx * _pdx *10 + _error_com_position(0)*_pix *10;  
// 		PelvisPos(1) += (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0)) * _ppy + (_COM_IN(1,_walkdtime1-1)-_estimated_state(3,0))/_dtx * _pdy + _error_com_position(1)*_piy;  
// 		PelvisPos(2) += (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0)) * _ppz + (_COM_IN(2,_walkdtime1-1)-_estimated_state(6,0))/_dtx * _pdz + _error_com_position(2)*_piz;*/	// hip_pos_test     
// 
//     /*		  body_thetax(0) += (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0))*_ppthetax + (_body_IN(0,_walkdtime1-1)-_estimated_state(9,0))/_dtx*_pdthetax  + _error_torso_angle(0)*_pithetax;  
// 		body_thetax(1) += (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0))*_ppthetay + (_body_IN(1,_walkdtime1-1)-_estimated_state(12,0))/_dtx*_pdthetay+ _error_torso_angle(1)*_pithetay;  
// 		body_thetax(2) += (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0))*_ppthetaz + (_body_IN(2,_walkdtime1-1)-_estimated_state(15,0))/_dtx*_pdthetaz+ _error_torso_angle(2)*_pithetaz;  
// 		
// 		HipO_Turn = Rz(body_thetax[1])*Ry(body_thetax[1])*Rx(body_thetax[0]);	*/	    
// 	      }

	    
	    }
	    else  //walking beyond time counter
	    {
	      _t_walkdtime_restart_flag = walkdtime;
	      
	      IsStartWalk = false;
    // 		_t_walkdtime_restart_flag = walkdtime	      
	      _t_walkdtime_restart_flag = walkdtime;
	    }      
	      
	}	 
  
        
    }
      
  } 
  else
  { 
    /////keep the current state:double support phase
	  
    ZMPxy_realx = zmp_ref =(LeftFootPosx+RightFootPosx)/2;
    F_R.setZero();
    _F_r_mpc.setZero();
    F_L.setZero(); 
    _F_l_mpc.setZero();
    F_R(2) = _F_r_mpc(2) = 9.8/2*RobotPara().totalmass;
    F_L(2) = _F_l_mpc(2) = 9.8/2*RobotPara().totalmass;
    M_R.setZero();
    M_L.setZero();
    
    j_count = 1;
    bjx1 = 1;
    
    tx = 1;
    td = 0;    
  }


  
}



void MpcRTControlClass::StartWalking()
{
  
  IsStartWalk = true;
//  DPRINTF(" start  Walking\n");
  if (_stop_walking)
  {
    _start_walking_again = true;
 //   DPRINTF("========= start walking again. =============\n");     
  }
  else
  {
//    DPRINTF("========= start walking-first time =============\n");   
  }
   _stop_walking = false;
  
}

void MpcRTControlClass::StopWalking()
{

    if (_t_int <10)
    {

     IsStartWalk = false;  
    }
    else
    {
      
      IsStartWalk = true;
      _stop_walking = true;  
      DPRINTF("========= motion for stop walking =============\n");          
    }
      
  
}


void MpcRTControlClass::EnableStandingReact()
{

}


void MpcRTControlClass::EnableWalkingReact()
{

}

void MpcRTControlClass::InternalLoggerLoop()
{
#ifdef USE_XBOT_LOGGER
	xbot_logger->add("nlp_hipPos", PelvisPos);
	xbot_logger->add("nlp_torso_angle", body_thetax);
	xbot_logger->add("nlp_LeftFootPos", LeftFootPosx);
	xbot_logger->add("nlp_RightFootPos", RightFootPosx);
	xbot_logger->add("nlp_Fr_ref", _F_r_mpc);
	xbot_logger->add("nlp_Fl_ref", _F_l_mpc);
	xbot_logger->add("nlp_Mr_ref", _M_r_mpc);
	xbot_logger->add("nlp_Ml_ref", _M_l_mpc);			
	
	
#endif
}

