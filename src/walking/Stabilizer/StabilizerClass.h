/*****************************************************************************
StabilizerClass.h
*****************************************************************************/
#ifndef STABILIZER_CLASS_H
#define STABILIZER_CLASS_H

#include <vector>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>


#include "MPC/MPCClass.h"



#ifdef USE_XBOT_LOGGER
#include "XBotInterface/Logger.hpp"
#endif

class DampingCtrClass;
class FilterClass;
class MeanFilterClass;
class RobotStateClass;
class StabilizerClass
{
public:
	StabilizerClass();




#ifdef USE_XBOT_LOGGER
	void initLogger(XBot::MatLogger::Ptr xbot_logger, int buffer_size = -1, int interleave = 1);
	void addToLog(XBot::MatLogger::Ptr xbot_logger);
#endif
	
	
	//// whole body admittance controller: COM position&pose control, foot position and pose control;
	Eigen::Vector3d COMdampingCtrl(Eigen::Vector3d zmp_ref,const RobotStateClass &irobot);
	
	Eigen::Vector3d COMangleCtrl(int bjx1, Eigen::Vector3d thetaxyx,Eigen::Vector3d comxyzx,Eigen::Vector3d Lfootxyzx,Eigen::Vector3d Rfootxyzx,const RobotStateClass &irobot);
	
	Eigen::Vector6d FootdampiingCtrol_LR(int bjx1, int j_count, double tx, double td, Eigen::Vector3d M_L, Eigen::Vector3d M_R,const RobotStateClass &irobot);

 	Eigen::Vector2d ForcediffCtrol_LR(int bjx1, Eigen::Vector3d F_L,Eigen::Vector3d F_R,const RobotStateClass &irobot);
		

protected:

	std::vector<double> mKd, mBd, A, mEqui0, mEnable, deltaFtZ_old, deltaFtZ;

	Eigen::Vector3d deltaHip;

	double FilterCutOff;
	int N_ButWth;
	boost::shared_ptr<FilterClass> txy_Filter, FzFT_Filter, FextZ_hip_Filter, deltaHip_Filter;
	double tx_old, ty_old, xd_old, yd_old, zd_old;

	boost::shared_ptr<MeanFilterClass> txy_MeanFilter, Fz_MeanFilter, FzFT_MeanFilter;
	double mVerticalScale;
	double mZc;


	boost::shared_ptr<FilterClass> tx_Filter, ty_Filter;

	boost::shared_ptr<DampingCtrClass> FootPosZ_ctrl, LFootPosZ_ctrl, RFootPosZ_ctrl;
	boost::shared_ptr<DampingCtrClass> LFootOri_ctrl, RFootOri_ctrl;

	double dT;
	double torque_x;
	double torque_y;
	double td_x;
	double td_y;

	Eigen::Vector3d deltaFtPos, deltaFtAng_l, deltaFtAng_r;
	Eigen::Vector3d Ft_force_diff_ref, Ft_force_diff_msr;
	
	std::string _name;

private:

};


#endif

