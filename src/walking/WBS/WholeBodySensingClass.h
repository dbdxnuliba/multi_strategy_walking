/*****************************************************************************
WholeBodySensingClass.h

Description:	Header file of WholeBodySensingClass

*****************************************************************************/
#ifndef WBS_CLASS_H
#define WBS_CLASS_H

#include <math.h>
#define _USE_MATH_DEFINES // this is for using <cmath>
#include <cmath>
#include <deque>
#include "RobotStateClass.h"

class FilterClass;
class MeanFilterClass;
class KalmanFilterClass;

class WholeBodySensingClass
{
public:
	WholeBodySensingClass();
	void UpdateIMU(double dt, const Eigen::Matrix3d &Rpelvis_abs, const Eigen::Vector3d &LnAcc, const Eigen::Vector3d &AgVel);
	void UpdateIMU(double dt, const Eigen::Matrix3d &Rpelvis_abs, const Eigen::Vector3d &LnAcc, const Eigen::Vector3d &AgVel, const Eigen::Vector3d &EulerAng);
	void UpdateRobotState(const std::vector<double> &qall, double dt, const std::vector<double> &FTSensor);
	void UpdateRobotState(const std::vector<double> &qall, double dt, const std::vector<double> &FTSensor, const std::vector<double> &tall);
	void UpdateJointTorqueFB(const std::vector<double> &tall);
	void UpdateRobotState(const std::vector<double> &qL, const std::vector<double> &qR, const std::vector<double> &qW, const std::vector<double> &qaL, const std::vector<double> &qaR, double dt, const std::vector<double> &FTSensor);
	void UpdateRobotState(const double (&qL)[6], const double (&qR)[6], const double (&qW)[3], const double (&qaL)[4], const double (&qaR)[4], double dt, const std::vector<double> &FTSensor);
	void UpdateRobotState(const double (&qL)[6], const double (&qR)[6], const double (&qW)[3], const double (&qaL)[4], const double (&qaR)[4], double dt, const Eigen::Matrix3d &Rpelvis_abs, const std::vector<double> &FTSensor);
	// template <class T>
	void UpdateHandFT(const std::vector<double> &HandFTSensor);
	const RobotStateClass& getRobotState();
	inline const RobotParaClass& RobotPara() const {return irobot.RobotPara();};
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//inline Eigen::Vector3d com(){return irobot.com;};
	//inline Eigen::Vector3d gcom(){return irobot.gcom;};
	void UpdateRef(const std::vector<double> &ComRef, const std::vector<double> &LftRef, const std::vector<double> &RftRef);
	void UpdateRef(const std::vector<double> &ComRef, const std::vector<double> &LftRef, const std::vector<double> &RftRef, const std::vector<double> &ZmpRef);
	void UpdateRef(const Eigen::Vector3d &ComRef, const Eigen::Vector3d &LftRef, const Eigen::Vector3d &RftRef, const Eigen::Vector3d &ZmpRef, const Eigen::Vector3d &FzRef);
	void UpdateRef(const double &delay, const std::vector<double> &ComRef, const std::vector<double> &LftRef, const std::vector<double> &RftRef, const std::vector<double> &ZmpRef);
	void UpdateRef(const double &delay, const std::vector<double> &ComRef, const std::vector<double> &LftRef, const std::vector<double> &RftRef, const std::vector<double> &ZmpRef, const std::vector<double> &FzRef);
	void UpdateRef(const double &delay, const Eigen::Vector3d &ComRef, const Eigen::Vector3d &LftRef, const Eigen::Vector3d &RftRef, const Eigen::Vector3d &ZmpRef);
	void UpdateRef(const double &delay, const Eigen::Vector3d &ComRef, const Eigen::Vector3d &LftRef, const Eigen::Vector3d &RftRef, const Eigen::Vector3d &ZmpRef, const Eigen::Vector3d &FzRef);
	void SaveData(const bool &IsSave, const std::vector<double> &rawdata);
	inline void IsWholeBodyRobot(const bool &flag) {irobot.IsWholeBodyRobot = flag;};
	inline void setFilterPara(double cutoff, int N) {FilterCutOff = cutoff; N_ButWth = N;}; //set cutoff frequency and N of butterworth filter, default (30.0, 4)
	inline void WhichFootRef(const SupportState &phase) {if (irobot.WhichFootRef != phase) {irobot.WhichFootRef_old = irobot.WhichFootRef; irobot.WhichFootRef = phase;}};
	inline void StepTimeRef(const double &t) {irobot.StepTimeRef = t;};
	inline void UpdateFallState(const FallState &state) {irobot.fall_state = state;};

	void UpdateAngRef(const Eigen::Vector3d &OriComRef, const Eigen::Vector3d &OriLftRef, const Eigen::Vector3d &OriRftRef);
	void UpdateAngRef(const std::vector<double> &OriComRef, const std::vector<double> &OriLftRef, const std::vector<double> &OriRftRef);

	void UpdateRelAngRef(const Eigen::Matrix3d &OriComRefL, const Eigen::Matrix3d &OriComRefR, const Eigen::Matrix3d &OriLftRef, const Eigen::Matrix3d &OriRftRef);
	void UpdateRelAngRef(const std::vector<double> &OriLftToComRef, const std::vector<double> &OriRftToComRef);

	void UpdateHandPosRef(const Eigen::Vector3d &LhdRef, const Eigen::Vector3d &RhdRef);
	void UpdateHandPosRef(const std::vector<double> &LhdRef, const std::vector<double> &RhdRef);


	void UpdateRBDL();

	template <typename T>
	inline void getRequiredTorques(T& torque_all) {irobot.getRequiredTorques(torque_all);};

	void CalcDesiredFootVelocity(const Eigen::Vector3d& HipPosRef, const Eigen::Matrix3d& HipOriLftRef, const Eigen::Matrix3d& HipOriRftRef, const Eigen::Vector3d& LftPosRef, const Eigen::Matrix3d& LftOriRef, const Eigen::Vector3d& RftPosRef, const Eigen::Matrix3d& RftOriRef) {irobot.CalcDesiredFootVelocity(HipPosRef, HipOriLftRef, HipOriRftRef, LftPosRef, LftOriRef, RftPosRef, RftOriRef);};

	void CalcDesiredArmVelocity(const Eigen::Vector3d& LhdPosRef, const Eigen::Matrix3d& LhdOriRef, const Eigen::Vector3d& RhdPosRef, const Eigen::Matrix3d& RhdOriRef) {irobot.CalcDesiredArmVelocity(LhdPosRef, LhdOriRef, RhdPosRef, RhdOriRef);};

	void EmergencyStop(const bool& flag)
	{
		irobot.SendToRobot->IsEmergencyStop = flag;
		if (flag) {
			std::cout << "\n\n\n" << std::endl;
			std::cout << "===================================" << std::endl;
			std::cout << "  Robot fails, emergency stop!!!" << std::endl;
			std::cout << "===================================" << std::endl;
			std::cout << "\n\n\n" << std::endl;
		}
	};

	void EnableGravityCompensation(const bool& flag)
	{
		irobot.SendToRobot->EnableGravityCompensation = flag;
		if (flag) {
			std::cout << "\n" << std::endl;
			std::cout << "===================================" << std::endl;
			std::cout << "  Enable Gravity Compensation." << std::endl;
			std::cout << "===================================" << std::endl;
			std::cout << "\n" << std::endl;
		}
	};
        
        Eigen::VectorXd q_off;
private:
	RobotStateClass irobot;
        double yaw_off;

	SupportState WhichFoot_old;
	boost::shared_ptr<MeanFilterClass> FT_AlexFilter;
	boost::shared_ptr<MeanFilterClass> gcop_AlexFilter;
	boost::shared_ptr<KalmanFilterClass> xKFC, yKFC, zKFC;

	double FilterCutOff;
	int N_ButWth;
	boost::shared_ptr<FilterClass> glft_k_Filter, grft_k_Filter, gftOrigin_Filter, gftcom_Filter, gdftcom_Filter, gddftcom_Filter, gftcop_Filter, gftlcop_Filter, gftrcop_Filter;
	boost::shared_ptr<FilterClass> glft_Filter, grft_Filter, fOrigin_Filter, gOrigin_Filter, gcom_Filter, gdcom_Filter, gddcom_Filter, gcop_Filter, glcop_Filter, grcop_Filter, gGeomCenSP_Filter;
	boost::shared_ptr<FilterClass> lcop_Filter, rcop_Filter, dlcop_Filter, drcop_Filter, cop_in_lft_Filter, cop_in_rft_Filter, cop_Filter, fcop_Filter, ccop_local_Filter;
	boost::shared_ptr<FilterClass> IMU_LinearAcc_Filter, IMU_LinearVel_Filter, IMU_AngularVel_Filter, IMU_AngularAcc_Filter, IMU_odometry_Filter, FT_Filter, FT_hand_Filter;
	Eigen::Vector3d IMU_odometry_raw, IMU_dodometry_raw, IMU_ddodometry_raw;
	double Fz_ratio_l, Fz_ratio_r, Fzmin;

	std::deque<std::vector<double> > FZqueue;

	int dtimeFT, ContactIndex, UpdateFtYawContactIndex_old;
	double gftYawTemp, gftYawTemp1;
	bool IsRefDelayInit, IsUpdateFtYaw;
	std::vector<Eigen::Vector3d > olddRef, olddHandRef;
	std::deque<std::vector<std::vector<double> > > RefDelayQueue;
	std::deque<std::vector<std::vector<double> > > HandRefDelayQueue;
	std::deque<std::vector<std::vector<double> > > AngRefDelayQueue;
	std::deque<std::vector<std::vector<double> > > RelAngRefDelayQueue;
	std::vector<std::vector<double> > oldRef, oldAngRef, oldRelAngRef, oldHandRef;
	Eigen::Matrix3d LftCom_old, RftCom_old;

	void InitWBS(const double &dt);
	void InitKFC(const double &dt);
	void UpdateKFC();
	void UpdateFK2Global();
	void Waist2Global();
	void CheckSupportState();
	void Update2Global();
	void CalcCop();
	void ForwardKinematics();
	void RobotModelFK();
	void UpdateUsefulVariableAfterFK();
	inline double RADTODEG(double x) { return x * 180.0 / M_PI;};
	inline double DEGTORAD(double x) { return x * M_PI / 180.0;};

	std::vector<double> qall_offset;
	bool IsRemoveOffset;

	template <class T1, class T2, class T3, class T4>
	void UpdateRobotStateTemp(const T1 &qL, const T1 &qR, const T2 &qW, const T3 &qaL, const T3 &qaR, double dt, const T4 &FTSensor);

	std::vector<double> qall_temp;
	std::vector<double> FT_filterDS;
	std::vector<std::vector<double> > tempRef;
};

#endif
