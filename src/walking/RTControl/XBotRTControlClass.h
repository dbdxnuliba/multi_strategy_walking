/*****************************************************************************
XBotRTControlClass.h
*****************************************************************************/
#ifndef XBOT_RTCONTROL_CLASS_H
#define XBOT_RTCONTROL_CLASS_H
#include <iostream>

#include "WBS/WholeBodySensingClass.h"
#include "Stabilizer/StabilizerClass.h"
#include "IK/IKClass.h"

#include "utils/utils.h"

#include <sys/time.h>

#ifdef USE_XBOT_LOGGER
#include "XBotInterface/Logger.hpp"
#endif

#if defined( __XENO__ ) || defined( __COBALT__ )
#include <XBotCore-interfaces/XBotRT_ipc.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif

#if defined( __XENO__ ) || defined( __COBALT__ )
static const std::string pipe_prefix1("/proc/xenomai/registry/rtipc/xddp/");
#else
static const std::string pipe_prefix1("/tmp/");
#endif


class XDDP_pipe {

public:
	XDDP_pipe(const std::string pipe_name, int pool_size):
		pipe_name(pipe_name),
		pool_size(pool_size)
	{
#if defined( __XENO__ ) || defined( __COBALT__ )
		fd = xddp_bind(pipe_name.c_str(), pool_size);
#else
		std::string pipe = pipe_prefix1 + pipe_name;
		mkfifo(pipe.c_str(), S_IRWXU | S_IRWXG);
		fd = open(pipe.c_str(), O_RDWR | O_NONBLOCK);
#endif
		assert(fd);
	}

	virtual ~XDDP_pipe()
	{
		close(fd);
#if defined( __XENO__ ) || defined( __COBALT__ )
		std::string pipe = pipe_prefix1 + pipe_name;
		unlink(pipe.c_str());
#endif

	}

protected:
	int fd;
	int pool_size;
	std::string pipe_name;
};


class Write_XDDP_pipe : public XDDP_pipe {
public:
	Write_XDDP_pipe(std::string pipe_name, int pool_size):
		XDDP_pipe(pipe_name, pool_size) {}

	int write(void *buffer, int nbytes)
	{
		return ::write(fd, buffer, nbytes);
	}
};

class Read_XDDP_pipe : public XDDP_pipe {
public:
	Read_XDDP_pipe(std::string pipe_name, int pool_size):
		XDDP_pipe(pipe_name, pool_size) {}

	int read(void *buffer, ssize_t buff_size)
	{
		/////////////////////////////////////////////////////////
		// NON-BLOCKING, read buff_size byte from pipe or cross domain socket
#if defined( __XENO__ ) || defined( __COBALT__ )
		return recvfrom(fd, buffer, buff_size, MSG_DONTWAIT, NULL, 0);
#else
		// NON-BLOCKING
		return ::read(fd, buffer, buff_size);
#endif
	}
};























class IKClass_OpenSoT;
class XBotRTControlClass
{
public:
	XBotRTControlClass();
	virtual ~XBotRTControlClass();

	virtual void Run(const double &dT, std::vector<double> &pos, const std::vector<float> &OffsetAng);
	virtual void Run();
	virtual void KeyBoardControl(char cmd);

	WholeBodySensingClass _WBS;

	inline const RobotStateClass& getRobotState() {return _WBS.getRobotState();};

	virtual inline const RobotParaClass& RobotPara() const {return _WBS.RobotPara();};

	void UpdateJointTorqueFB(const std::vector<double> &tall){_WBS.UpdateJointTorqueFB(tall);};
	virtual void UpdateWBS(const double & dT, const std::vector<double> &qall, const Eigen::Vector3d &EulerAng, const Eigen::Vector3d &Acc, const Eigen::Vector3d &AngleRate, const std::vector<double> &FTSensor, const std::vector<double> &HandFTSensor = std::vector<double>(12, 0.0));

	virtual void UpdateWBS(const std::vector<double> &qall, const Eigen::Matrix3d &Rpelvis_abs, const Eigen::Vector3d &Acc, const Eigen::Vector3d &AngleRate, const std::vector<double> &FTSensor, const std::vector<double> &HandFTSensor = std::vector<double>(12, 0.0));

	virtual void savedata();

	void SortFromXBotToJointName(const Eigen::VectorXd &from, std::vector<double> &to);
	void SortFromXBotToJointName(const Eigen::VectorXd &from, Eigen::VectorXd &to);

	template <class T>
	void SortFromJointNameToXBot(const T &from, std::unordered_map<std::string, double> &to);

	void JointRefToXBot(Eigen::VectorXd &to);
	void JointRefToXBot_LowerBody(Eigen::VectorXd &to);

	void HomingInit(const std::vector<double> &homing_angles);
        
    int KeyBoardType(){return KeyBoardSwitch;};

	bool IsInit;

// 	DCMGeneratorClass ZMPwalk;
	// ZMPGeneratorClass ZMPwalk;

	inline const bool& IsFixedWalk() {return IsFixed;};

	void initOpenSoTIK(std::string path_to_config_file);

	boost::shared_ptr<IKClass_OpenSoT> ik_sot;

protected:

	bool IsStartWalk;
	virtual void StopWalking(){};
	virtual void StartWalking(){};

	Read_XDDP_pipe  * console;

	int user_input(void *buffer, ssize_t buff_size);
	int user_input(char &cmd);

	std::vector<double> srdf_homing_angles;

	double dt;

	Eigen::Vector3d LEFT_HAND_HOME_POS, RIGHT_HAND_HOME_POS;
	Eigen::MatrixXd LEFT_HAND_HOME_ORI, RIGHT_HAND_HOME_ORI;
	Eigen::Vector3d LEFT_FOOT_HOME_POS, RIGHT_FOOT_HOME_POS;
	Eigen::MatrixXd LEFT_FOOT_HOME_ORI, RIGHT_FOOT_HOME_ORI;

	bool IsLoggerSaved;

#ifdef USE_XBOT_LOGGER
	XBot::MatLogger::Ptr xbot_logger;
	void initLogger(int buffer_size = -1, int interleave = 1);
	void addToLog();
	int logger_len = 100000;
#endif

	Eigen::VectorXd q_tmp;

	Eigen::Vector3d com_ref, lft_ref, rft_ref, zmp_ref;
	std::vector<double> Fz_ref, deltaFtZ;
	Eigen::Vector3d deltaHip;
	double TurnYaw;
	double LftYaw, RftYaw;
	virtual void UpdateWbsRef();

	// ------ BodyAttitudeControl -----------
	Eigen::Vector3d theta_pelvis, w_pelvis;
	double body_pitch_des;
	Eigen::Matrix3d deltaHipO;
// 	virtual void BodyAttitudeControl();
	// --------------------------------------


//	virtual void ImpStabilizer();
	StabilizerClass Sta;
	IKClass IK;

	std::vector<double> traj_ref_old;

	double realtime;

	unsigned int dtime, walkdtime;
	double zc, zh;
	double Ksway;
	double xcop, ycop;
	double HipO_comL, HipO_comR;
	double walkstarttime;

	double steptime, steplength, stepwidth, stepheight, turnangle;
	bool IsStart, IsFixed;
	bool StaEnableX, StaEnableY, StaEnableZ;
	bool HandStaEnableX, HandStaEnableY, HandStaEnableZ;

	int KeyBoardSwitch, JointNUM;


	Eigen::Vector3d xstate, ystate;
	Eigen::Vector3d LeftFootPos, RightFootPos, PelvisPos;
	Eigen::Matrix3d LeftFootO, RightFootO, HipO_Left, HipO_Right, HipO_Left_comp, HipO_Right_comp, HipO_Turn;

	Eigen::Vector3d LeftHandPos, RightHandPos;
	Eigen::Matrix3d LeftHandO, RightHandO;

	Eigen::Vector3d vFullLeg;
	Eigen::Vector3d LeftHandPosGlobal, RightHandPosGlobal;
	Eigen::Matrix3d LeftHandOGlobal, RightHandOGlobal;
	Eigen::Matrix3d WaistO;

	bool IsSetHandRefToWorld;
	bool IsMoveHand;
	double hand_startT;
	int hand_startT_d;
	Eigen::Vector3d LeftHandPosGlobal_start, RightHandPosGlobal_start;
	Eigen::Matrix3d LeftHandOGlobal_start, RightHandOGlobal_start;
	Eigen::Vector3d LeftHandPos_start, RightHandPos_start;
	Eigen::Matrix3d LeftHandO_start, RightHandO_start;
	Eigen::Vector3d WaistComp;

	virtual void Init(const double &dT);

	virtual void MoveToInitialPosition();
	virtual void RTControlKeyBoardControl(char cmd);

	double jointestime;
	int cycN;
	bool Is_testJoint, En_testJoint;
	std::vector<double> jointmax, jointmin;

	Eigen::Vector3d HipPos;
	void SolveIK();

	double initHipx, initHipy;
	bool Is_xCOPinCenter, Is_yCOPinCenter;
	int dcounter, dbuffertime;

	virtual void COMTrajGen();
	Eigen::VectorXd zmpx_window_old, zmpy_window_old;
	Eigen::VectorXd zmpx_window, zmpy_window, zmpx_window_new, zmpy_window_new;
	bool IsInitZMPold;

	Eigen::Vector3d deltaXYZ;

	double n_RBDL_floating_jnt;
	Eigen::VectorXd q_rbdl;

	Eigen::VectorXd e_i;

	virtual void StandingReactStepping() {};
	virtual void WalkingReactStepping() {};
	virtual void EnableStandingReact() {};
	virtual void EnableWalkingReact() {};

	Eigen::Vector3d walkingVel;

	virtual void InternalLoggerInit() {};
	virtual void InternalLoggerLoop() {};

	bool EnableFtPos, EnableFtOri;
	Eigen::Vector3d deltaFtPos_l, deltaFtPos_r;
	Eigen::Matrix3d deltaFtOri_left, deltaFtOri_right;
	
	
	/////for new StabilizerClass
	Eigen::Vector3d body_thetax;
	void Admittance_controller();
	
	Eigen::Vector3d det_hip_posotion, det_hip_pose;
	Eigen::Vector6d det_foot_rpy_lr;
	Eigen::Vector2d det_footz_lr;
	
	
	
	Eigen::Vector3d ZMPxy_realx, thetaxyx,comxyzx,Lfootxyzx,Rfootxyzx;
	Eigen::Vector3d M_L, M_R, F_L,F_R;
	
        int j_count,bjx1;
	double tx, td;
	
	Eigen::Matrix3d ankle_Ori_left, ankle_Ori_right;
	Eigen::Vector3d det_ank_foot;
	
	
	Eigen::Vector3d LeftFootPosx,RightFootPosx;		
	
	
	
	

};

#endif
