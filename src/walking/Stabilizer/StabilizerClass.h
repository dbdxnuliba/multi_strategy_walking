/*****************************************************************************
StabilizerClass.h
*****************************************************************************/
#ifndef STABILIZER_CLASS_H
#define STABILIZER_CLASS_H

#include <vector>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

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

	// set parameters
	void Kd(double x_stiff, double y_stiff, double z_stiff);
	void Bd(double x_damp, double y_damp, double z_damp);
	void Enable(int x_enable, int y_enable, int z_enable); // enable Stabilizer in (x ,y, z), default (0,0,0) disable all
	void Equi0(double x_equi, double y_equi); // equilibruim position, default (0,0)
	void VerticalScale(double input); // normalize weight scale for Z compliance, should be <1, default 0.92
	inline void zc(double input) {mZc = input;}; // COM height, default 0.6m
	inline void setFilterPara(double cutoff, int N) {FilterCutOff = cutoff; N_ButWth = N;}; //set cutoff frequency and N of butterworth filter, default (30.0, 4)

	// read parameters
	inline double VerticalScale() {return mVerticalScale;};
	inline double zc() {return mZc;};

	// return deltaHip,(x,y,z), LftRef is left foot reference traj, RftRef is right foot reference traj,///////////////irobot generated//////////////////////
	const Eigen::Vector3d& StabilizerCart(const RobotStateClass &irobot, const std::vector<double> &LftRef, const std::vector<double> &RftRef);
	const Eigen::Vector3d& StabilizerCart(const RobotStateClass &irobot, const Eigen::Vector3d &LftRef, const Eigen::Vector3d &RftRef);
	const Eigen::Vector3d& StabilizerCart(const RobotStateClass &irobot);
	const std::vector<double>& StabilizerCartZ(const bool enable, const double K, const double B, const RobotStateClass &irobot);

	void ReactStepping(const double& tq_x, const double& tq_y);
	Eigen::Vector3d mdeltaPos;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	void LandingFtZctrl(const bool& enable, const RobotStateClass& irobot, Eigen::Vector3d& lft_pos_delta, Eigen::Vector3d& rft_pos_delta);

	double LandingFtZctrl(double damping, double Fz_ref_diff, double Fzl_msr, double Fzr_msr, double settle_time);

	void LandingFtOctrl(const bool& enable, const RobotStateClass& irobot, Eigen::Matrix3d& deltaFtOri_left, Eigen::Matrix3d& deltaFtOri_right);

	void LandingFtOctrl(const RobotStateClass& irobot, const double & tq_x, const double & tq_y, double damping, double settle_time, Eigen::Matrix3d& deltaFtOri_left, Eigen::Matrix3d& deltaFtOri_right);


#ifdef USE_XBOT_LOGGER
	void initLogger(XBot::MatLogger::Ptr xbot_logger, int buffer_size = -1, int interleave = 1);
	void addToLog(XBot::MatLogger::Ptr xbot_logger);
#endif

protected:

	std::vector<double> mKd, mBd, A, Ks, mEqui0, mEnable, deltaFtZ_old, deltaFtZ;

	Eigen::Vector3d deltaHip;

	double FilterCutOff;
	int N_ButWth;
	boost::shared_ptr<FilterClass> txy_Filter, FzFT_Filter, FextZ_hip_Filter, deltaHip_Filter;
	double tx_old, ty_old, xd_old, yd_old, zd_old;

	boost::shared_ptr<MeanFilterClass> txy_MeanFilter, Fz_MeanFilter, FzFT_MeanFilter;
	double mVerticalScale;
	double mZc;
	double mass;

	bool IsInitF;

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
	template <class T>
	const Eigen::Vector3d& StabilizerCartTemp(const RobotStateClass &irobot, const T &LftRef, const T &RftRef);

};

#endif

