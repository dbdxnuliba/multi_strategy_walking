/*****************************************************************************
RobotStateClass.h

*****************************************************************************/
#pragma once

#include "RobotModel/RobotModelClass.h"

class RobotCommandClass
{
public:
	RobotCommandClass()
		: IsEmergencyStop(false)
		, EnableGravityCompensation(false)
	{};
	virtual ~RobotCommandClass() {};

	/**
	 * @brief      intialize all the member variables
	 */
	void Init()
	{
		joint_num = RobotParaClass::JOINT_NUM();
		// joint_num = 30; // Wed 03 Jan 2018 06:58, for walkman with centauro upperbody, need to change in the future.
		nDoF_floating = joint_num + 6;
		dt = RobotParaClass::dT();
		q = Eigen::VectorXd::Zero(joint_num);
		q_ref = q;
		qdot = q;
		des_rft_vel = Eigen::VectorXd::Zero(6);
		des_lft_vel = Eigen::VectorXd::Zero(6);
		des_rhd_vel = Eigen::VectorXd::Zero(6);
		des_lhd_vel = Eigen::VectorXd::Zero(6);
		fin_lhd_pose = Eigen::VectorXd::Zero(6);
		fin_rhd_pose = Eigen::VectorXd::Zero(6);

		qdd = q;
		jtau = q;
		jtau_grav_comp = q;
		des_rft_acc = Eigen::VectorXd::Zero(6);
		des_lft_acc = Eigen::VectorXd::Zero(6);
		des_rhd_acc = Eigen::VectorXd::Zero(6);
		des_lhd_acc = Eigen::VectorXd::Zero(6);
		torsodd = Eigen::VectorXd::Zero(3);
		pelvisdd = Eigen::VectorXd::Zero(6);
		com = torsodd;

		HipPos = LeftAnklePos = RightAnklePos = Eigen::Vector3d::Zero();
		HipO_Left = HipO_Right = LeftFootO = RightFootO = Eigen::Matrix3d::Zero();

		gLftRef = gRftRef = gComRef = gZmpRef = Eigen::Vector3d::Zero();

		stabilizer_torque = Eigen::Vector3d::Zero();
		applyDeltaPosX = applyDeltaPosY = Eigen::Vector3d::Zero();
	};

	int nDoF_floating;
	double dt;
	int joint_num;

	// for IK
	Eigen::VectorXd q; //!< desired joints angle
	Eigen::VectorXd q_ref; //!< joints angle reference
	Eigen::VectorXd qdot; //!< desired joints velocity
	Eigen::Vector6d des_rft_vel, des_lft_vel; //!< desired feet pos/ori vel
	Eigen::Vector6d des_rhd_vel, des_lhd_vel; //!< desired hands pos/ori vel
	Eigen::Vector6d fin_lhd_pose, fin_rhd_pose; //!< desired hands pos/ori pos

	// for ID
	Eigen::VectorXd qdd; //!< desired joints acc
	Eigen::VectorXd jtau; //!< desired joints torque
	Eigen::VectorXd jtau_grav_comp; //!< joints gravity compensation torque
	Eigen::Vector6d des_rft_acc, des_lft_acc; //!< desired feet pos/ori acc
	Eigen::Vector6d des_rhd_acc, des_lhd_acc; //!< desired hands pos/ori acc
	Eigen::Vector3d torsodd; //!< desired torso orientation acc
	Eigen::Vector6d pelvisdd; //!< desired pelvis acc/dw
	Eigen::Vector3d com;

	Eigen::MatrixXd P;

	Eigen::Vector3d HipPos, LeftAnklePos, RightAnklePos;
	Eigen::Matrix3d HipO_Left, HipO_Right, LeftFootO, RightFootO;

	Eigen::Vector3d gLftRef, gRftRef, gComRef, gZmpRef;

	Eigen::Vector3d stabilizer_torque;
	Eigen::Vector3d applyDeltaPosX, applyDeltaPosY;

	bool IsEmergencyStop; //!< flag for emergency stop
	bool EnableGravityCompensation; //!< flag for enable gravity compensation

private:

};


class CRobotSeg;
class CRobotLink {
public:
	double m;
	Eigen::Vector3d q;  //!< refer (x,y,z) rotation angle
	Eigen::Vector3d dq; //!< refer angular velocity
	Eigen::Vector3d ddq;//!< refer angular acceleration
	Eigen::Vector3d q_old;
	Eigen::Vector3d dq_old;
	Eigen::Vector3d ddq_old;

	//Eigen::Vector3d qmotor;  //!< motor (x,y,z) rotation angle
	//Eigen::Vector3d dqmotor; //!< motor angular velocity
	//Eigen::Vector3d ddqmotor;//!< motor angular acceleration
	//Eigen::Vector3d qlink_abs;  //!< link_abs (x,y,z) rotation angle
	//Eigen::Vector3d dqlink_abs; //!< link_abs angular velocity
	//Eigen::Vector3d ddqlink_abs;//!< link_abs angular acceleration
	//Eigen::Vector3d qdef;  //!< deflection (x,y,z) rotation angle
	//Eigen::Vector3d dqdef; //!< deflection angular velocity
	//Eigen::Vector3d ddqdef;//!< deflection angular acceleration
	//Eigen::Vector3d link_torque;
	//Eigen::Vector3d k_spring;

	Eigen::Vector3d grcom; //!< com in global coordinate's frame

	Eigen::Vector3d ft_com; //!< com in support foot coordinate's frame
	Eigen::Vector3d ft_dcom;
	Eigen::Vector3d ft_ddcom;
	Eigen::Vector3d ft_joint;

	Eigen::Vector3d pcom; //!< link's com position in TotalCOM frame
	Eigen::Vector3d dpcom;
	Eigen::Vector3d pcom_old;

	Eigen::Vector3d qcom; //!< link's com position in GeomCenDS frame
	// Eigen::Vector3d dqcom;
	// Eigen::Vector3d qcom_old;

	Eigen::Vector3d dcom; //!< link's com in the link local frame
	Eigen::Vector3d limb; //!< link dimension
	Eigen::Vector3d rcom; //!< link's com in the waist frame
	Eigen::Vector3d drcom; //!< link's com velocity in the waist frame
	Eigen::Vector3d ddrcom; //!< link's com acceleration in the waist frame
	Eigen::Vector3d rcom_old;
	Eigen::Vector3d drcom_old;
	Eigen::Vector3d ddrcom_old;
	Eigen::Vector3d Lcom; //!< angular momentum around COM
	Eigen::Vector3d joint; //!< position vector of joint
	Eigen::Vector3d joint_old;
	Eigen::Matrix3d R; //!< rotational matrix of link
	std::string name; //!< name of the CRobotSeg

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** @file */  // for enum in doxygen
enum  SupportState {
	Nowhere,	//!< 0
	InAir,		//!< 1
	OnRFoot,	//!< 2
	OnBoth,		//!< 3. Standing on both feet
	OnLFoot,	//!< 4
};

/** @file */
enum  FallState {
	Stable = -1,	 //!< -1
	FallForward = 1, //!< 1
	FallBackward,	 //!< 2
	FallLeft,		 //!< 3
	FallRight,	     //!< 4
};

class RobotStateClass {

public:
	RobotStateClass();
	virtual ~RobotStateClass() {};

	inline const RobotParaClass& RobotPara() const {return robotpara;};


	void Init();

	boost::shared_ptr<RobotCommandClass> SendToRobot;
	boost::shared_ptr<RobotCommandClass> ReadFromRobot;

	boost::shared_ptr<RobotModelClass> _model;

	std::vector<double> tau_all;
	std::vector<double> q_all_mesure, q_all_mesure_old;
	std::vector<double> qdot_all_mesure;

	template <typename T>
	void getRequiredTorques(T& torque_all) const {_model->getRequiredTorques(torque_all);};

	void InverseDynamics() {_model->InverseDynamics();};
	inline const int& getRBDLJointID(const int& joint_name) {return _model->getRBDLJointID(joint_name);};
	Eigen::MatrixXd getJacobian(const int& body_id) const {return _model->getJacobian(body_id);};

	std::string name;
	bool IsInitial;
	bool IsReady;
	bool IsWholeBodyRobot;
	CRobotLink LINK[3][5];
	double m;
	int idno;
	int segno;
	double dt;
	double t;

	double Fzl;
	double Fzr;

	Eigen::Vector3d com;  //!< overall COM
	Eigen::Vector3d dcom; //!< COM velocity
	Eigen::Vector3d ddcom;//!< COM acceleration
	Eigen::Vector3d com_old;
	Eigen::Vector3d dcom_old;
	Eigen::Vector3d ddcom_old;
	Eigen::Vector3d Lcom; //!< overall angular momentum around COM
	Eigen::Vector3d dLcom;
	Eigen::Vector3d Lcom_old;
	Eigen::Vector3d dLcom_old;
	Eigen::Vector3d Lft;
	Eigen::Vector3d Rft;
	Eigen::Vector3d Larm;
	Eigen::Vector3d Rarm;
	Eigen::Vector3d lankle; //!< left ankle position in hip frame
	Eigen::Vector3d rankle; //!< right ankle position in hip frame
	Eigen::Matrix3d Rot_lft2hip; //!< left foot orientation in hip frame
	Eigen::Matrix3d Rot_rft2hip; //!< right foot orientation in hip frame

	Eigen::Vector3d htot; //!< overall angular momentum around COM caculated by RBDL

	//*********** ref gait para **********
	double StepTimeRef;
	SupportState WhichFootRef;
	SupportState WhichFootRef_old;
	//************************************

	FallState fall_state;
	SupportState WhichFoot;
	SupportState WhichFoot_old;
	std::vector<double> Fz_zerocut;
	std::vector<double> Fz_condition;

	bool IsSavedata;
	std::vector<std::vector<double> > savedata;

	Eigen::Vector3d fOrigin; //!< origin in support foot coordinate's frame
	Eigen::Vector3d ft_com; //!< com in support foot coordinate's frame
	Eigen::Vector3d ft_com_old;
	Eigen::Vector3d ft_dcom;
	Eigen::Vector3d ft_ddcom;
	Eigen::Vector3d gOrigin; //!< fOorigin in global frame
	Eigen::Vector3d gcom; //!< com in global frame
	Eigen::Vector3d gcom_old;
	Eigen::Vector3d gdcom;
	Eigen::Vector3d gdcom_old;
	Eigen::Vector3d gddcom;
	Eigen::Vector3d ghip;
	Eigen::Vector3d glft;
	Eigen::Vector3d grft;
	Eigen::Vector3d glft_old;
	Eigen::Vector3d grft_old;
	Eigen::Vector3d leftstance;
	Eigen::Vector3d rightstance;
	Eigen::Vector3d leftstance_old;
	Eigen::Vector3d rightstance_old;
	Eigen::Vector3d gCapturePoint;

	Eigen::Vector3d dGeomCenDS;
	Eigen::Vector3d GeomCenDS_old;
	Eigen::Vector3d GeomCenDS; //!< geometric centre of two feet in Hip frame during double support
	Eigen::Vector3d gGeomCenDS;
	Eigen::Vector3d gGeomCenDS_raw;

	Eigen::Vector3d cLft; //!< left foot in com frame
	Eigen::Vector3d cRft; //!< right foot in com frame
	Eigen::Vector3d cGeomCenDS; //!< GeomCenDS in com frame
	Eigen::Vector3d gcLft; //!< left foot in com frame, orientaion in Global
	Eigen::Vector3d gcRft; //!< right foot in com frame, orientaion in Global
	Eigen::Vector3d gcLft_old;
	Eigen::Vector3d gcRft_old;
	Eigen::Vector3d dgcLft;
	Eigen::Vector3d dgcRft;
	Eigen::Vector3d gcGeomCenDS; //!< COM in com frame, orientaion in Global
        Eigen::Vector3d dgcGeomCenDS;
        Eigen::Vector3d gcGeomCenDS_old;
        
	Eigen::Vector3d cop_local; //!< overall cop in GeomCen frame
	//Eigen::Vector3d GeomCenLft; //!< left foot in GeomCen frame
	//Eigen::Vector3d GeomCenRft; //!< right foot in GeomCen frame
	//Eigen::Vector3d GeomCenCOM; //!< COM in GeomCen frame
	//Eigen::Vector3d gGeomCenLft; //!< left foot in GeomCen frame, orientaion in Global
	//Eigen::Vector3d gGeomCenRft; //!< right foot in GeomCen frame, orientaion in Global
	//Eigen::Vector3d gGeomCenCOM; //!< COM in GeomCen frame, orientaion in Global

	//********** only for ExtForceEst **************
	Eigen::Vector3d GeomCenLft; //!< left foot in the frame based in the center of two ankles
	Eigen::Vector3d GeomCenRft; //!< right foot in the frame based in the center of two ankles
	Eigen::Vector3d GeomCenCOM; //!< COM in the frame based in the center of two ankles
	Eigen::Vector3d gGeomCenLft; //!< left foot in GeomCen frame, orientaion in Global
	Eigen::Vector3d gGeomCenRft; //!< right foot in GeomCen frame, orientaion in Global
	Eigen::Vector3d gGeomCenCOM; //!< COM in GeomCen frame, orientaion in Global
	//********** end ********************************

	Eigen::Vector6d FT_foot_left; //!< foot F/T sensor feedback
	Eigen::Vector6d FT_foot_right;
	Eigen::Vector6d FT_fl_filter; //!< filtered F/T sensor feedback
	Eigen::Vector6d FT_fr_filter;
	Eigen::Vector6d FT_hand_left;  //!< hand F/T sensor feedback
	Eigen::Vector6d FT_hand_right;
	Eigen::Vector6d FT_hl_filter; //!< filtered F/T sensor feedback
	Eigen::Vector6d FT_hr_filter;

	Eigen::Vector6d FT_hand_left_in_hip;  //!< hand F/T sensor feedback in hip frame
	Eigen::Vector6d FT_hand_right_in_hip;
	Eigen::Vector6d FT_hl_filter_in_hip; //!< filtered F/T sensor feedback in hip frame
	Eigen::Vector6d FT_hr_filter_in_hip;

	double	 gRoll;
	double	 gPitch;
	double	 gYaw;
	Eigen::Matrix3d IMU_abs;
	Eigen::Matrix3d IMU_rel;
	Eigen::Vector3d IMU_LinearVel;
	Eigen::Vector3d IMU_AngularVel;
	Eigen::Vector3d IMU_LinearAcc;
	Eigen::Vector3d IMU_AngularAcc;
	Eigen::Vector3d IMU_Euler;

	Eigen::Vector3d IMU_AngularVel_old;
	Eigen::Vector3d IMU_LinearVel_raw;
	Eigen::Vector3d IMU_AngularVel_raw;
	Eigen::Vector3d IMU_LinearAcc_raw;
	Eigen::Vector3d IMU_AngularAcc_raw;

	Eigen::Vector3d IMU_odometry_raw;
	Eigen::Vector3d IMU_odometry;
	Eigen::Vector3d IMU_dodometry_raw;
	Eigen::Vector3d IMU_dodometry;
	Eigen::Vector3d IMU_ddodometry_raw;
	Eigen::Vector3d IMU_ddodometry;

	Eigen::Vector3d KFC_gcom;
	Eigen::Vector3d KFC_gdcom;
	Eigen::Vector3d KFC_gddcom;

	Eigen::Matrix3d Icom; //!< inertia tensor around COM in hip frame
	Eigen::Matrix3d gIcom; //!< inertia tensor around COM in global frame
	Eigen::Matrix3d Ift; //!< inertia tensor around centre of two feet in hip frame
	Eigen::Matrix3d gIft; //!< inertia tensor around centre of two feet in global frame

	Eigen::Vector3d lcop; //!< left foot COP in left foot frame
	Eigen::Vector3d rcop; //!< right foot COP in right foot frame
	Eigen::Vector3d dlcop; //!< left foot COP velocity
	Eigen::Vector3d drcop; //!< right foot COP velocity
	Eigen::Vector3d lcop_old;
	Eigen::Vector3d rcop_old;
	Eigen::Vector3d lcop_raw;
	Eigen::Vector3d rcop_raw;
	Eigen::Vector3d dlcop_raw;
	Eigen::Vector3d drcop_raw;
	Eigen::Vector3d fcop; //!< COP in GLOBAL frame, origin is the middle of the projections of ankles while standing
	Eigen::Vector3d glcop; //!< left foot COP in GLOBAL frame
	Eigen::Vector3d grcop; //!< right foot COP in GLOBAL frame
	Eigen::Vector3d gcop; //!< overall COP in GLOBAL frame, origin is the GLOBAL origin
	Eigen::Vector3d cop; //!< overall COP in hip frame
	Eigen::Vector3d dcop; //!< overall COP velocity in hip fram
	Eigen::Vector3d cop_in_lft; //!< overall COP in hip frame start from left foot
	Eigen::Vector3d cop_in_rft; //!< overall COP in hip frame start from right foot
	//Eigen::Vector3d lcop_delta;
	//Eigen::Vector3d rcop_delta;
	//Eigen::Vector3d cop_delta;

	Eigen::Vector3d ft_com_rel; //!< com in support foot frame, gloable orientation
	Eigen::Vector3d ft_dcom_rel;
	Eigen::Vector3d ft_com_rel_old;

	/********* For WBS in Kinematics ******************/
	double gftyaw;
	Eigen::Matrix3d gftO;
	Eigen::Matrix3d glftO;
	Eigen::Matrix3d grftO;
	Eigen::Vector3d glft_k;
	Eigen::Vector3d grft_k;
	Eigen::Vector3d glft_k_old;
	Eigen::Vector3d grft_k_old;

	Eigen::Vector3d gftOrigin;
	Eigen::Vector3d gftcom;
	Eigen::Vector3d gftcom_old;
	Eigen::Vector3d gdftcom;
	Eigen::Vector3d gdftcom_old;
	Eigen::Vector3d gddftcom;

	Eigen::Vector3d gftlcop;
	Eigen::Vector3d gftrcop;
	Eigen::Vector3d gftcop;

	Eigen::Vector3d gftOrigin_raw, glft_k_raw, grft_k_raw;
	Eigen::Vector3d gftcom_raw, gdftcom_raw, gddftcom_raw, gftlcop_raw, gftrcop_raw, gftcop_raw;
	/**************************************************/

	Eigen::Vector3d glft_raw, grft_raw, fOrigin_raw, gOrigin_raw;
	Eigen::Vector3d gcom_raw, gdcom_raw, gddcom_raw, glcop_raw, grcop_raw, gcop_raw;
	Eigen::Vector3d cop_in_lft_raw, cop_in_rft_raw, cop_raw, fcop_raw, cop_local_raw;

	double scale;
	double LF_scale;
	double RF_scale;

	Eigen::Vector3d LftRef;
	Eigen::Vector3d RftRef;
	Eigen::Vector3d ComRef;
	Eigen::Vector3d ZmpRef;
	Eigen::Vector3d FzRef; //!< [0]:left, [1]:right
	Eigen::Vector3d dLftRef;
	Eigen::Vector3d dRftRef;
	Eigen::Vector3d dComRef;
	Eigen::Vector3d dZmpRef;
	Eigen::Vector3d dFzRef;
	Eigen::Vector3d ddLftRef;
	Eigen::Vector3d ddRftRef;
	Eigen::Vector3d ddComRef;
	Eigen::Vector3d ddZmpRef;

	Eigen::Vector3d LhdRef;
	Eigen::Vector3d RhdRef;
	Eigen::Vector3d dLhdRef;
	Eigen::Vector3d dRhdRef;
	Eigen::Vector3d ddLhdRef;
	Eigen::Vector3d ddRhdRef;

	Eigen::Vector3d OriLftRef;
	Eigen::Vector3d OriRftRef;
	Eigen::Vector3d OriComRef;
	Eigen::Vector3d dOriLftRef;
	Eigen::Vector3d dOriRftRef;
	Eigen::Vector3d dOriComRef;

	Eigen::Vector3d OriLftToComRef;
	Eigen::Vector3d OriRftToComRef;
	Eigen::Vector3d dOriLftToComRef;
	Eigen::Vector3d dOriRftToComRef;

	Eigen::Vector3d dOriLftToComRef2;
	Eigen::Vector3d dOriRftToComRef2;

	std::vector<double> q_all;

	Eigen::Vector3d f1, f2, f3, f4, hipwidth, shoulder, waist, chest, neck, ankle_offset;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	void UpdateRef(const std::vector<double> &comRef, const std::vector<double> &lftRef, const std::vector<double> &rftRef);



	unsigned int JointNUM;
	Eigen::VectorXd q_min, q_max;

	void CalcDesiredFootVelocity(const Eigen::Vector3d& HipPosRef, const Eigen::Matrix3d& HipOriLftRef, const Eigen::Matrix3d& HipOriRftRef, const Eigen::Vector3d& LftPosRef, const Eigen::Matrix3d& LftOriRef, const Eigen::Vector3d& RftPosRef, const Eigen::Matrix3d& RftOriRef);

	/**
	 * @brief      Calculates the desired arm velocity.
	 *             the hand refs are local ones in pelvis frame
	 *
	 * @param[in]  LhdPosRef  The lhd position reference
	 * @param[in]  LhdOriRef  The lhd ori reference
	 * @param[in]  RhdPosRef  The rhd position reference
	 * @param[in]  RhdOriRef  The rhd ori reference
	 */
	void CalcDesiredArmVelocity(const Eigen::Vector3d& LhdPosRef, const Eigen::Matrix3d& LhdOriRef, const Eigen::Vector3d& RhdPosRef, const Eigen::Matrix3d& RhdOriRef);

private:
	void InitRobotLink(CRobotLink &ulink, CRobotSeg &usegment);
	
	///////////*******************use the robot para class/////
	RobotParaClass robotpara;

	void setJointLimit();

	Eigen::Vector3d LftPosRef_old, RftPosRef_old, HipPosRef_old;
	Eigen::Vector3d dLftRef_old, dRftRef_old, dHipRef_old;
	Eigen::Matrix3d LftCom_old, RftCom_old;

	Eigen::Vector3d RFootFrame_r_old, LFootFrame_r_old;
	Eigen::Matrix3d RFootFrame_E_old, LFootFrame_E_old;
	Eigen::Vector3d dOriLftToComRef_old, dOriRftToComRef_old;

	Eigen::Vector3d LhdPosRef_old, RhdPosRef_old;
	Eigen::Vector3d dLhdRef_old, dRhdRef_old;
	Eigen::Matrix3d LhdOriRef_local_old, RhdOriRef_local_old;

	Eigen::Vector3d RHandFrame_r_old, LHandFrame_r_old;
	Eigen::Matrix3d RHandFrame_E_old, LHandFrame_E_old;
	Eigen::Vector3d dOriLhdToComRef_old, dOriRhdToComRef_old;


};


