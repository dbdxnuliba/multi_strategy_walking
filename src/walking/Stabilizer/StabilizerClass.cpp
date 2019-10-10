/*****************************************************************************
StabilizerClass.cpp
*****************************************************************************/
#include "StabilizerClass.h"
#include "WBS/RobotStateClass.h"

#include "Filters/FilterClass.h"
#include "Filters/MeanFilterClass.h"

StabilizerClass::StabilizerClass()
	: FilterCutOff(30.0)
	, N_ButWth(4)
	, mVerticalScale(0.95)
	, mZc(0.6)
	, tx_old(0.0)
	, ty_old(0.0)
	, xd_old(0.0)
	, yd_old(0.0)
	, zd_old(0.0)
	, mKd(3, 0.0)
	, mBd(3, 0.0)
	, A(2, 0.0)
	, mEqui0(2, 0.0)
	, mEnable(3, 0.0)
	, deltaFtZ_old(2, 0.0)
	, deltaFtZ(2, 0.0)
	, dT(RobotParaClass::dT())
	, torque_x(0.0)
	, torque_y(0.0)
	, td_x(0.0)
	, td_y(0.0)
	, _name("Stab")
{
	txy_Filter.reset(new FilterClass);
	FzFT_Filter.reset(new FilterClass);
	tx_Filter.reset(new FilterClass);
	ty_Filter.reset(new FilterClass);
	FextZ_hip_Filter.reset(new FilterClass);
	deltaHip_Filter.reset(new FilterClass);
	txy_MeanFilter.reset(new MeanFilterClass);
	Fz_MeanFilter.reset(new MeanFilterClass);
	FzFT_MeanFilter.reset(new MeanFilterClass);

	FootPosZ_ctrl.reset(new DampingCtrClass);
	LFootPosZ_ctrl.reset(new DampingCtrClass);
	RFootPosZ_ctrl.reset(new DampingCtrClass);
	LFootOri_ctrl.reset(new DampingCtrClass);
	RFootOri_ctrl.reset(new DampingCtrClass);

	deltaHip = deltaFtAng_l = deltaFtAng_r = deltaFtPos = Eigen::Vector3d::Zero();
	Ft_force_diff_msr = Ft_force_diff_ref = Eigen::Vector3d::Zero();
	
        _det_COM_position.setZero(); _det_COM_position_old.setZero();
        _det_COM_pose.setZero();  _det_COM_pose_old.setZero();
        _det_foot_r.setZero();    _det_foot_l.setZero();     _det_foot_r_old.setZero(); _det_foot_l_old.setZero();
        _det_footz_r=0;            _det_footz_l=0;             _det_footz_r_old=0;         _det_footz_l_old=0;	
	

};






////// CoM damping control: (admittance control) ZMP trajectory difference ===> CoM postion det
Eigen::Vector3d StabilizerClass::COMdampingCtrl(int bjx1,Eigen::Vector3d Lfootxyzx,Eigen::Vector3d Rfootxyzx,Eigen::Vector3d zmp_ref,const RobotStateClass &irobot)
{  
  Eigen::Matrix<double, 3,3> c_angle_singe;
  c_angle_singe.setZero();
// //   c_angle_singe(0,0) = 0.0000001; 
// //   c_angle_singe(1,1) = 0.0000005; 
// //   c_angle_singe(2,2) = 0.00000005;
  
  c_angle_singe(0,0) = 1; 
  c_angle_singe(1,1) = 1; 
  c_angle_singe(2,2) = 0.5;
  
//   Eigen::Vector3d det_COM_position = c_angle_singe * (zmp_ref - irobot.gcop);
  



  
  if (bjx1 % 2 == 0)  //left support
  {
    _det_COM_position= -c_angle_singe * ((zmp_ref-Lfootxyzx) - (irobot.gcop-irobot.glft));  
  }
  else
  {
    _det_COM_position= -c_angle_singe * ((zmp_ref-Rfootxyzx) - (irobot.gcop-irobot.grft));  
  }
 
  
  
  
  Eigen::Vector3d det_COM_positionx = 0.5 *(_det_COM_position+0.025*(_det_COM_position-_det_COM_position_old)/(RobotParaClass::dT()))*RobotParaClass::dT()*RobotParaClass::dT();
  
  _det_COM_position_old = _det_COM_position;
  
  clamp(det_COM_positionx[0], -0.03, 0.03);   ///3 cm
  clamp(det_COM_positionx[1], -0.01, 0.01);   ///1 cm
  clamp(det_COM_positionx[2], -0.05, 0.005);  ///
  
  
  return det_COM_positionx;

}


////// CoM angle control (admittance control) ===> CoM RPY angle det
Eigen::Vector3d StabilizerClass::COMangleCtrl(int bjx1, Eigen::Vector3d thetaxyx,Eigen::Vector3d comxyzx,Eigen::Vector3d Lfootxyzx,Eigen::Vector3d Rfootxyzx,const RobotStateClass &irobot)
{  
  Eigen::Vector3d det_COM_pose(0,0,0);
  
  double theatx_p,footx_p;
  theatx_p = 0.5;
  footx_p = 0.5;
  
//   if (bjx1 % 2 == 0)  //left support
//   {
//     
//     det_COM_pose(0) = 0.0000005 * (thetaxyx(0) - irobot.IMU_Euler(0)) + 0.0000005*(comxyzx(1)-Lfootxyzx(1) - (irobot.gcom(1)-irobot.glft(1))); 
//   
//     det_COM_pose(1) = 0.0000005 * (thetaxyx(1) - irobot.IMU_Euler(1)) + 0.0000005*(comxyzx(0)-Lfootxyzx(0) - (irobot.gcom(0)-irobot.glft(0)));  
//     
//     det_COM_pose(2) = 0.0000005 * (thetaxyx(2) - irobot.IMU_Euler(2)) - 0.0000005*(irobot.gcom(0) - irobot.glft(0));
//   }
//   else
//   {
//     det_COM_pose(0) = 0.0000005 * (thetaxyx(0) - irobot.IMU_Euler(0)) + 0.0000005*(comxyzx(1)-Rfootxyzx(1) - (irobot.gcom(1)-irobot.grft(1))); 
//   
//     det_COM_pose(1) = 0.0000005 * (thetaxyx(1) - irobot.IMU_Euler(1)) + 0.0000005*(comxyzx(0)-Rfootxyzx(0) - (irobot.gcom(0)-irobot.grft(0)));      
//     det_COM_pose(2) = 0.0000005 * (thetaxyx(2) - irobot.IMU_Euler(2)) + 0.0000005*(irobot.gcom(0) - irobot.grft(0));
//   }
  
  
  if (bjx1 % 2 == 0)  //left support
  {
    
    _det_COM_pose(0) = theatx_p * (thetaxyx(0) - irobot.IMU_Euler(0)) + footx_p*(comxyzx(1)-Lfootxyzx(1) - (irobot.gcom(1)-irobot.glft(1))); 
  
    _det_COM_pose(1) = theatx_p * (thetaxyx(1) - irobot.IMU_Euler(1)) + footx_p*(comxyzx(0)-Lfootxyzx(0) - (irobot.gcom(0)-irobot.glft(0)));  
    
    _det_COM_pose(2) = theatx_p * (thetaxyx(2) - irobot.IMU_Euler(2)) - footx_p*(irobot.gcom(0) - irobot.glft(0));
  }
  else
  {
    _det_COM_pose(0) = theatx_p * (thetaxyx(0) - irobot.IMU_Euler(0)) + footx_p*(comxyzx(1)-Rfootxyzx(1) - (irobot.gcom(1)-irobot.grft(1))); 
  
    _det_COM_pose(1) = theatx_p * (thetaxyx(1) - irobot.IMU_Euler(1)) + footx_p*(comxyzx(0)-Rfootxyzx(0) - (irobot.gcom(0)-irobot.grft(0)));      
    _det_COM_pose(2) = theatx_p * (thetaxyx(2) - irobot.IMU_Euler(2)) + footx_p*(irobot.gcom(0) - irobot.grft(0));
  }
  
  
 
  det_COM_pose = _det_COM_pose + 0.025*(_det_COM_pose-_det_COM_pose_old)/(RobotParaClass::dT());
  
  _det_COM_pose_old = _det_COM_pose;
  
  clamp(det_COM_pose[0], -0.052, 0.052); // =- 3 degree
  clamp(det_COM_pose[1], -0.052, 0.052); // =- 3 degree
  clamp(det_COM_pose[2], -0.087, 0.87); // =- 5 degree   
  
  
  return det_COM_pose;

}




////// Foot admittance control: ankle torque difference ===> foot RPY angle det
Eigen::Vector6d StabilizerClass::FootdampiingCtrol_LR(int bjx1, int j_count, double tx, double td, Eigen::Vector3d M_L, Eigen::Vector3d M_R,const RobotStateClass &irobot)
{
/*  Eigen::Matrix<double, 3,3> F_single;
  F_single.setZero();
  F_single(0,0) = 0.000001; 
  F_single(1,1) = 0.000001; 
  F_single(2,2) = 0.0000001;  
 
  Eigen::Matrix<double, 3,3> F_double;
  F_double.setZero();
  F_double(0,0) = 0.0000001; 
  F_double(1,1) = 0.0000001; 
  F_double(2,2) = 0.00000001; */



  Eigen::Matrix<double, 3,3> F_single;
  F_single.setZero();
  F_single(0,0) = 0.0001; 
  F_single(1,1) = 0.0001; 
  F_single(2,2) = 0.00001;  
 
  Eigen::Matrix<double, 3,3> F_double;
  F_double.setZero();
  F_double(0,0) = 0.0001; 
  F_double(1,1) = 0.0001; 
  F_double(2,2) = 0.000001; 


  
  Eigen::Vector3d torque_r(0,0,0), torque_l(0,0,0);
  torque_r(0) = irobot.FT_fr_filter(3);
  torque_r(1) = irobot.FT_fr_filter(4);
  torque_r(2) = irobot.FT_fr_filter(5); 
  
  torque_l(0) = irobot.FT_fl_filter(3);
  torque_l(1) = irobot.FT_fl_filter(4);
  torque_l(2) = irobot.FT_fl_filter(5);  

   
  if (bjx1 % 2 == 0)  //left support
  {
//    if ((mpc_ref._j_count +1 - round(mpc_ref._tx(mpc_ref._bjx1-1)/_dt))*_dt < mpc_ref._td(mpc_ref._bjx1-1))
    if ((j_count +1 - round(tx/_dt))*_dt < td)
    {
      _det_foot_l = F_double*(M_L - torque_l); 
      _det_foot_r = F_double*(M_R - torque_r);
             
    }
    else
    {
      _det_foot_l = F_single*(M_L - torque_l);   
    }
  }
  else
  {
    if ((j_count +1 - round(tx/_dt))*_dt < td)
    {
      _det_foot_r = F_double*(M_R - torque_r);      
      _det_foot_l = F_double*(M_L - torque_l);             
    }
    else
    {
      _det_foot_r = F_single*(M_R - torque_r);  
//      cout <<"asd"<<endl;
     
    }
  }
  

  
  Eigen::Vector6d det_foot_rpy_lr;
  det_foot_rpy_lr(0) = _det_foot_l(0) + 0.025*(_det_foot_l(0)-_det_foot_l_old(0))/(RobotParaClass::dT());
  det_foot_rpy_lr(1) = _det_foot_l(1) + 0.025*(_det_foot_l(1)-_det_foot_l_old(1))/(RobotParaClass::dT());
  det_foot_rpy_lr(2) = _det_foot_l(2) + 0.025*(_det_foot_l(2)-_det_foot_l_old(2))/(RobotParaClass::dT());
  det_foot_rpy_lr(3) = _det_foot_r(0) + 0.025*(_det_foot_r(0)-_det_foot_r_old(0))/(RobotParaClass::dT());
  det_foot_rpy_lr(4) = _det_foot_r(1) + 0.025*(_det_foot_r(1)-_det_foot_r_old(1))/(RobotParaClass::dT());
  det_foot_rpy_lr(5) = _det_foot_r(2) + 0.025*(_det_foot_r(2)-_det_foot_r_old(2))/(RobotParaClass::dT());  
  
  
  _det_foot_r_old = _det_foot_r;
  _det_foot_l_old = _det_foot_l;  
  
  
  clamp(det_foot_rpy_lr[0], -0.052, 0.052); // =- 3 degree
  clamp(det_foot_rpy_lr[1], -0.052, 0.052); // =- 3 degree
  clamp(det_foot_rpy_lr[2], -0.087, 0.087); // =- 5 degree    
  clamp(det_foot_rpy_lr[3], -0.052, 0.052); // =- 3 degree
  clamp(det_foot_rpy_lr[4], -0.052, 0.052); // =- 3 degree
  clamp(det_foot_rpy_lr[5], -0.0587, 0.087); // =- 5 degree 

  
  
  return det_foot_rpy_lr;
   
   
}


//////foot_focre_diffence_control: (admittance control) foot reactive force different ===> foot height det
Eigen::Vector2d StabilizerClass::ForcediffCtrol_LR(int bjx1, Eigen::Vector3d F_L,Eigen::Vector3d F_R,const RobotStateClass &irobot)
{
  double foot_damp_co = 0.0000001;

  
  Eigen::Vector3d f_r(0,0,0), f_l(0,0,0);
  f_r(0) = irobot.FT_fr_filter(0);
  f_r(1) = irobot.FT_fr_filter(1);
  f_r(2) = irobot.FT_fr_filter(2); 
  
  f_l(0) = irobot.FT_fl_filter(0);
  f_l(1) = irobot.FT_fl_filter(1);
  f_l(2) = irobot.FT_fl_filter(2);  
   
  if (bjx1 % 2 == 0)  //left support
  {
    _det_footz_l = 0.5 * foot_damp_co*((F_R(2) - F_L(2))- (f_r(2) - f_l(2))); 
    _det_footz_r = -_det_footz_l;
  }
  else
  {
    _det_footz_r = 0.5 * foot_damp_co*((F_L(2) - F_R(2))- (f_l(2) - f_r(2)));      
    _det_footz_l = -_det_footz_r;             
  }
  

  
  Eigen::Vector2d det_footz_lr(0,0);
  det_footz_lr(0) = _det_footz_l + 0.000001*(_det_footz_l-_det_footz_l_old)/(RobotParaClass::dT());
  det_footz_lr(1) = _det_footz_r + 0.000001*(_det_footz_r-_det_footz_r_old)/(RobotParaClass::dT());
  
  _det_footz_l_old = _det_footz_l;
  _det_footz_r_old = _det_footz_r;    

  clamp(det_footz_lr[0], -0.001, 0.001); // =- 5cm
  clamp(det_footz_lr[1], -0.001, 0.001); // =- 5 cm 
  
  return det_footz_lr;
}




















#ifdef USE_XBOT_LOGGER
void StabilizerClass::initLogger(XBot::MatLogger::Ptr xbot_logger, int buffer_size, int interleave)
{
	xbot_logger->createVectorVariable(_name + "_" + "Ft_force_diff_ref", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable(_name + "_" + "Ft_force_diff_msr", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable(_name + "_" + "deltaFtPos", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable(_name + "_" + "deltaFtAng_l", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable(_name + "_" + "deltaFtAng_r", 3, interleave, buffer_size);
}

void StabilizerClass::addToLog(XBot::MatLogger::Ptr xbot_logger)
{
	xbot_logger->add(_name + "_" + "Ft_force_diff_ref", Ft_force_diff_ref);
	xbot_logger->add(_name + "_" + "Ft_force_diff_msr", Ft_force_diff_msr);
	xbot_logger->add(_name + "_" + "deltaFtPos", deltaFtPos);
	xbot_logger->add(_name + "_" + "deltaFtAng_l", deltaFtAng_l);
	xbot_logger->add(_name + "_" + "deltaFtAng_r", deltaFtAng_r);

}
#endif
