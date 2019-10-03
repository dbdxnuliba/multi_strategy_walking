/*****************************************************************************
RobotStateClass.cpp
*****************************************************************************/
#include "WBS/RobotStateClass.h"


class CRobotSeg {
public:
	double m;
	Eigen::Vector3d dcom;
	Eigen::Vector3d limb;
	//Eigen::Vector3d rcom;
	//Eigen::Matrix3d R;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

RobotStateClass::RobotStateClass()
	: IsInitial(false)
	, IsReady(false)
	, IsWholeBodyRobot(true)
	, IsSavedata(false)
	, m(0)
	, dt(0)
	, idno(0)
	, segno(0)
	, t(0)
	, Fzl(0)
	, Fzr(0)
	, gRoll(0)
	, gPitch(0)
	, gYaw(0)
	, scale(0)
	, LF_scale(0)
	, RF_scale(0)
	, gftyaw(0.0)
	, JointNUM(RobotParaClass::JOINT_NUM())
{
	std::cout << "\n\n\n========= Constructing RobotStateClass =================" << std::endl;
	name = robotpara.name;

	tau_all.resize(JointNUM);
	q_all.resize(JointNUM);
	q_all_mesure.resize(JointNUM);
	q_all_mesure_old.resize(JointNUM);
	qdot_all_mesure.resize(JointNUM);

	_model.reset(new RobotModelClass());
	_model->Init(robotpara);

	q_min = Eigen::VectorXd::Zero(JointNUM);
	q_max = Eigen::VectorXd::Zero(JointNUM);
	setJointLimit();

	SendToRobot.reset(new RobotCommandClass());
	SendToRobot->Init();

	ReadFromRobot.reset(new RobotCommandClass());
	ReadFromRobot->Init();

	// update hands local positions according to the homing angles
	std::vector<double> homing_angles(robotpara.HOMING_POS());
	std::transform(homing_angles.begin(), homing_angles.end(), homing_angles.begin(), std::bind1st(std::multiplies<double>(), M_PI / 180.0)); // from degree to radian

	Eigen::VectorXd homing = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(homing_angles.data(), homing_angles.size());

	// _model->Update(homing);
	_model->Update(homing_angles);
	RBDL::Math::SpatialTransform LHandFrame = _model->getLocalBodyFrame(LEFT_HAND);
	RBDL::Math::SpatialTransform RHandFrame = _model->getLocalBodyFrame(RIGHT_HAND);
	RBDL::Math::SpatialTransform LFootFrame = _model->getLocalBodyFrame(LEFT_FOOT);
	RBDL::Math::SpatialTransform RFootFrame = _model->getLocalBodyFrame(RIGHT_FOOT);

	// LftPosRef_old << 0.0, RobotParaClass::HALF_FOOT_DIS(), RobotParaClass::ANKLE_HEIGHT();
	// RftPosRef_old << 0.0, -RobotParaClass::HALF_FOOT_DIS(), RobotParaClass::ANKLE_HEIGHT();
	HipPosRef_old << 0, 0, RobotParaClass::FULL_LEG();
	// LftCom_old << Eigen::Matrix3d::Identity();
	// RftCom_old << Eigen::Matrix3d::Identity();

	LftPosRef_old = robotpara.LEFT_FOOT_HOME_POS = LFootFrame.r;
	RftPosRef_old = robotpara.RIGHT_FOOT_HOME_POS = RFootFrame.r;
	LftCom_old = robotpara.LEFT_FOOT_HOME_ORI = LFootFrame.E;
	RftCom_old = robotpara.RIGHT_FOOT_HOME_ORI = RFootFrame.E;

	LhdPosRef_old = robotpara.LEFT_HAND_HOME_POS = LHandFrame.r;
	RhdPosRef_old = robotpara.RIGHT_HAND_HOME_POS = RHandFrame.r;
	robotpara.LEFT_HAND_HOME_ORI = LHandFrame.E;
	robotpara.RIGHT_HAND_HOME_ORI = RHandFrame.E;

	LhdOriRef_local_old << Eigen::Matrix3d::Identity();
	RhdOriRef_local_old << Eigen::Matrix3d::Identity();

	// COUT("rbdl qall", _model->q_all_floating.transpose());
	// COUT("lhd:", robotpara.LEFT_HAND_HOME_POS.transpose(), "rhd:", robotpara.RIGHT_HAND_HOME_POS.transpose());

	//-----------------------------------------------

	neck << 0, 0, 0.1;
	shoulder << 0, robotpara.HALF_SHOULDER_WIDTH, 0;
	hipwidth << 0, RobotParaClass::HALF_HIP_WIDTH(), 0;
	// chest << -0.015, 0, 0.2029; //(chest, center of 2 shoulders) with respect to frame 22
	chest << -0.0, 0, robotpara.TORSO_HEIGHT; //(chest, center of 2 shoulders) with respect to frame 22
	waist << robotpara.HIP_BODY_OFFSET_X, 0, robotpara.PELVIS_HEIGHT; //frame 22 in pelvis local frame
	ankle_offset << 0, 0, -RobotParaClass::ANKLE_HEIGHT();

	com << Eigen::Vector3d::Zero(3); // overall COM;initialization of vectors
	dcom << Eigen::Vector3d::Zero(3);
	ddcom << Eigen::Vector3d::Zero(3);
	com_old << Eigen::Vector3d::Zero(3);
	dcom_old << Eigen::Vector3d::Zero(3);
	ddcom_old << Eigen::Vector3d::Zero(3);
	Lcom  << Eigen::Vector3d::Zero(3);	// in pelvis local frame
	dLcom << Eigen::Vector3d::Zero(3);
	Lcom_old << Eigen::Vector3d::Zero(3);
	dLcom_old << Eigen::Vector3d::Zero(3);
	Lft  << Eigen::Vector3d::Zero(3);	// in pelvis local frame
	Rft  << Eigen::Vector3d::Zero(3);	// in pelvis local frame
	Larm << Eigen::Vector3d::Zero(3);	// in pelvis local frame
	Rarm << Eigen::Vector3d::Zero(3);	// in pelvis local frame
	lankle << Eigen::Vector3d::Zero(3);	// in pelvis local frame
	rankle << Eigen::Vector3d::Zero(3);	// in pelvis local frame
	Rot_lft2hip << Eigen::Matrix3d::Identity();	// in pelvis local frame
	Rot_rft2hip << Eigen::Matrix3d::Identity();	// in pelvis local frame

	StepTimeRef = 1.0;

	fall_state = Stable;
	WhichFootRef = OnBoth;
	WhichFootRef_old = OnBoth;
	WhichFoot = Nowhere;  // initial position: support on both feet
	WhichFoot_old = Nowhere;  // initial position: support on both feet
	Fz_condition.resize(2, 0);
	Fz_zerocut.resize(2, 0);

	fOrigin << Eigen::Vector3d::Zero(3);
	ft_com << Eigen::Vector3d::Zero(3);		// COM in fOrigin frame with eye(3) orientation align with pelvis frame
	ft_com_old << Eigen::Vector3d::Zero(3);
	ft_dcom << Eigen::Vector3d::Zero(3);
	ft_ddcom << Eigen::Vector3d::Zero(3);

	ft_com_rel << Eigen::Vector3d::Zero(3);
	ft_com_rel_old << Eigen::Vector3d::Zero(3);
	ft_dcom_rel << Eigen::Vector3d::Zero(3);

	Eigen::Vector3d HALF_FOOT_DIS(0.0, RobotParaClass::HALF_FOOT_DIS(), 0);

	gOrigin << Eigen::Vector3d::Zero(3);
	gcom = _model->com_ft;		// COM in world frame
	gcom_old = gcom;
	gdcom << Eigen::Vector3d::Zero(3);
	gdcom_old << Eigen::Vector3d::Zero(3);
	gddcom << Eigen::Vector3d::Zero(3);
	ghip << 0.0, 0.0, RobotParaClass::FULL_LEG();
	glft << HALF_FOOT_DIS;		// left foot center in global/ world coordinate
	grft << -HALF_FOOT_DIS;		// right foot center in global/ world coordinate
	glft_old << HALF_FOOT_DIS;
	grft_old << -HALF_FOOT_DIS;
	leftstance = glft;
	rightstance = grft;
	leftstance_old << Eigen::Vector3d::Zero(3);
	rightstance_old << Eigen::Vector3d::Zero(3);
	gCapturePoint << Eigen::Vector3d::Zero(3);


	FT_foot_left << Eigen::Vector6d::Zero(6);
	FT_foot_right << Eigen::Vector6d::Zero(6);
	FT_fl_filter << Eigen::Vector6d::Zero(6);
	FT_fr_filter << Eigen::Vector6d::Zero(6);

	FT_hand_left << Eigen::Vector6d::Zero(6);
	FT_hand_right << Eigen::Vector6d::Zero(6);
	FT_hl_filter << Eigen::Vector6d::Zero(6);
	FT_hr_filter << Eigen::Vector6d::Zero(6);

	FT_hand_left_in_hip << Eigen::Vector6d::Zero(6);
	FT_hand_right_in_hip << Eigen::Vector6d::Zero(6);
	FT_hl_filter_in_hip << Eigen::Vector6d::Zero(6);
	FT_hr_filter_in_hip << Eigen::Vector6d::Zero(6);

	IMU_AngularVel_old << Eigen::Vector3d::Zero(3);
	IMU_LinearVel_raw << Eigen::Vector3d::Zero(3);
	IMU_AngularVel_raw << Eigen::Vector3d::Zero(3);
	IMU_LinearAcc_raw << Eigen::Vector3d::Zero(3);
	IMU_AngularAcc_raw << Eigen::Vector3d::Zero(3);
	IMU_Euler << Eigen::Vector3d::Zero(3);

	IMU_abs << Eigen::Matrix3d::Identity(3, 3);
	IMU_rel << Eigen::Matrix3d::Identity(3, 3);
	IMU_LinearVel << Eigen::Vector3d::Zero(3);
	IMU_AngularVel << Eigen::Vector3d::Zero(3);
	IMU_LinearAcc << Eigen::Vector3d::Zero(3);
	IMU_AngularAcc << Eigen::Vector3d::Zero(3);

	IMU_odometry_raw << Eigen::Vector3d::Zero(3);
	IMU_odometry << Eigen::Vector3d::Zero(3);
	IMU_dodometry_raw << Eigen::Vector3d::Zero(3);
	IMU_dodometry << Eigen::Vector3d::Zero(3);
	IMU_ddodometry_raw << Eigen::Vector3d::Zero(3);
	IMU_ddodometry << Eigen::Vector3d::Zero(3);

	KFC_gcom = gcom;
	KFC_gdcom << Eigen::Vector3d::Zero(3);
	KFC_gddcom << Eigen::Vector3d::Zero(3);

	Icom << Eigen::Matrix3d::Zero();
	gIcom << Eigen::Matrix3d::Zero();
	Ift << Eigen::Matrix3d::Zero();
	gIft << Eigen::Matrix3d::Zero();

	lcop_old << Eigen::Vector3d::Zero(3);
	rcop_old << Eigen::Vector3d::Zero(3);
	lcop_raw << Eigen::Vector3d::Zero(3);
	rcop_raw << Eigen::Vector3d::Zero(3);
	dlcop_raw << Eigen::Vector3d::Zero(3);
	drcop_raw << Eigen::Vector3d::Zero(3);

	lcop << Eigen::Vector3d::Zero(3);
	rcop << Eigen::Vector3d::Zero(3);
	cop << Eigen::Vector3d::Zero(3);
	gcop << Eigen::Vector3d::Zero(3);
	fcop << Eigen::Vector3d::Zero(3);
	glcop << Eigen::Vector3d::Zero(3);
	grcop << Eigen::Vector3d::Zero(3);
	dlcop << Eigen::Vector3d::Zero(3);
	drcop << Eigen::Vector3d::Zero(3);
	dcop << Eigen::Vector3d::Zero(3);
	cop_in_lft << Eigen::Vector3d::Zero(3);
	cop_in_rft << Eigen::Vector3d::Zero(3);
	//lcop_delta << Eigen::Vector3d::Zero(3);
	//rcop_delta << Eigen::Vector3d::Zero(3);
	//cop_delta << Eigen::Vector3d::Zero(3);

	fOrigin_raw << Eigen::Vector3d::Zero(3);
	gOrigin_raw << Eigen::Vector3d::Zero(3);
	gcom_raw = gcom;
	gdcom_raw << Eigen::Vector3d::Zero(3);
	gddcom_raw << Eigen::Vector3d::Zero(3);
	glcop_raw << Eigen::Vector3d::Zero(3);
	grcop_raw << Eigen::Vector3d::Zero(3);
	gcop_raw << Eigen::Vector3d::Zero(3);
	cop_in_lft_raw << Eigen::Vector3d::Zero(3);
	cop_in_rft_raw << Eigen::Vector3d::Zero(3);
	cop_raw << Eigen::Vector3d::Zero(3);
	fcop_raw << Eigen::Vector3d::Zero(3);
	cop_local_raw << Eigen::Vector3d::Zero(3);
	glft_raw << HALF_FOOT_DIS;		// left foot center in global/ world coordinate
	grft_raw << -HALF_FOOT_DIS;		// right foot center in global/ world coordinate

	dGeomCenDS << Eigen::Vector3d::Zero(3);
	gGeomCenDS << Eigen::Vector3d::Zero(3);
	gGeomCenDS_raw << Eigen::Vector3d::Zero(3);
	GeomCenDS << Eigen::Vector3d::Zero(3);
	GeomCenDS_old << Eigen::Vector3d::Zero(3);
	cop_local << Eigen::Vector3d::Zero(3);
	GeomCenLft << Eigen::Vector3d::Zero(3);
	GeomCenRft << Eigen::Vector3d::Zero(3);
	GeomCenCOM << Eigen::Vector3d::Zero(3);
	gGeomCenLft << Eigen::Vector3d::Zero(3);
	gGeomCenRft << Eigen::Vector3d::Zero(3);
	gGeomCenCOM << Eigen::Vector3d::Zero(3);

	cLft << Eigen::Vector3d::Zero(3);
	cRft << Eigen::Vector3d::Zero(3);
	cGeomCenDS << Eigen::Vector3d::Zero(3);
	gcLft << Eigen::Vector3d::Zero(3);
	gcRft << Eigen::Vector3d::Zero(3);
	gcGeomCenDS << Eigen::Vector3d::Zero(3);
        gcGeomCenDS_old << Eigen::Vector3d::Zero(3);
        dgcGeomCenDS << Eigen::Vector3d::Zero(3);

	gcLft_old = gcRft_old = dgcLft = dgcRft = Eigen::Vector3d::Zero(3);

	LftRef << LftPosRef_old;
	RftRef << RftPosRef_old;
	ComRef << Eigen::Vector3d::Zero(3);
	ZmpRef << Eigen::Vector3d::Zero(3);

	dLftRef << Eigen::Vector3d::Zero(3);
	dRftRef << Eigen::Vector3d::Zero(3);
	dLftRef_old << Eigen::Vector3d::Zero(3);
	dRftRef_old << Eigen::Vector3d::Zero(3);
	dComRef << Eigen::Vector3d::Zero(3);
	dZmpRef << Eigen::Vector3d::Zero(3);
	ddLftRef << Eigen::Vector3d::Zero(3);
	ddRftRef << Eigen::Vector3d::Zero(3);
	ddComRef << Eigen::Vector3d::Zero(3);
	ddZmpRef << Eigen::Vector3d::Zero(3);

	LhdRef << LhdPosRef_old;
	RhdRef << RhdPosRef_old;

	dLhdRef << Eigen::Vector3d::Zero(3);
	dRhdRef << Eigen::Vector3d::Zero(3);
	dLhdRef_old << Eigen::Vector3d::Zero(3);
	dRhdRef_old << Eigen::Vector3d::Zero(3);
	ddLhdRef << Eigen::Vector3d::Zero(3);
	ddRhdRef << Eigen::Vector3d::Zero(3);

	LFootFrame_r_old << LftPosRef_old;
	RFootFrame_r_old << RftPosRef_old;
	LHandFrame_r_old << LhdPosRef_old;
	RHandFrame_r_old << RhdPosRef_old;

	LFootFrame_E_old << Eigen::Matrix3d::Identity(3, 3);
	RFootFrame_E_old << Eigen::Matrix3d::Identity(3, 3);
	LHandFrame_E_old << Eigen::Matrix3d::Identity(3, 3);
	RHandFrame_E_old << Eigen::Matrix3d::Identity(3, 3);

	dOriLftToComRef_old << Eigen::Vector3d::Zero(3);
	dOriRftToComRef_old << Eigen::Vector3d::Zero(3);
	dOriLhdToComRef_old << Eigen::Vector3d::Zero(3);
	dOriRhdToComRef_old << Eigen::Vector3d::Zero(3);

	FzRef << 0.5, 0.5, 0;
	dFzRef << Eigen::Vector3d::Zero(3);

	OriLftRef << Eigen::Vector3d::Zero(3);
	OriRftRef << Eigen::Vector3d::Zero(3);
	OriComRef << Eigen::Vector3d::Zero(3);

	dOriLftRef << Eigen::Vector3d::Zero(3);
	dOriRftRef << Eigen::Vector3d::Zero(3);
	dOriComRef << Eigen::Vector3d::Zero(3);

	OriLftToComRef << Eigen::Vector3d::Zero(3);
	OriRftToComRef << Eigen::Vector3d::Zero(3);

	dOriLftToComRef << Eigen::Vector3d::Zero(3);
	dOriRftToComRef << Eigen::Vector3d::Zero(3);

	dOriLftToComRef2 << Eigen::Vector3d::Zero(3);
	dOriRftToComRef2 << Eigen::Vector3d::Zero(3);

	gftO << Eigen::Matrix3d::Identity(3, 3);
	glftO << Eigen::Matrix3d::Identity(3, 3);
	grftO << Eigen::Matrix3d::Identity(3, 3);
	glft_k << HALF_FOOT_DIS;		// left foot center in global/ world coordinate
	grft_k << -HALF_FOOT_DIS;		// right foot center in global/ world coordinate
	glft_k_old << HALF_FOOT_DIS;
	grft_k_old << -HALF_FOOT_DIS;
	glft_k_raw << HALF_FOOT_DIS;		// left foot center in global/ world coordinate
	grft_k_raw << -HALF_FOOT_DIS;		// right foot center in global/ world coordinate

	gftOrigin << Eigen::Vector3d::Zero(3);
	gftcom << Eigen::Vector3d::Zero(3);		// COM in world frame
	gftcom_old << Eigen::Vector3d::Zero(3);
	gdftcom << Eigen::Vector3d::Zero(3);
	gdftcom_old << Eigen::Vector3d::Zero(3);
	gddftcom << Eigen::Vector3d::Zero(3);

	gftcop << Eigen::Vector3d::Zero(3);
	gftlcop << Eigen::Vector3d::Zero(3);
	gftrcop << Eigen::Vector3d::Zero(3);

	gftOrigin_raw << Eigen::Vector3d::Zero(3);
	gftcom_raw << Eigen::Vector3d::Zero(3);
	gdftcom_raw << Eigen::Vector3d::Zero(3);
	gddftcom_raw << Eigen::Vector3d::Zero(3);
	gftlcop_raw << Eigen::Vector3d::Zero(3);
	gftrcop_raw << Eigen::Vector3d::Zero(3);
	gftcop_raw << Eigen::Vector3d::Zero(3);

	// Init();

	DPRINTF("=============== RobotStateClass is constructed... =============\n");
}


void RobotStateClass::InitRobotLink(CRobotLink & ulink, CRobotSeg & usegment)
{
	// below is the robot parameter
	//CRobotLink ulink;
	ulink.m = usegment.m; // limb mass
	ulink.dcom << usegment.dcom; // limb com
	ulink.limb << usegment.limb; // limb dimension
	// below is the computed data
	ulink.q << Eigen::Vector3d::Zero(3);
	ulink.dq << Eigen::Vector3d::Zero(3);
	ulink.ddq << Eigen::Vector3d::Zero(3);
	ulink.q_old << Eigen::Vector3d::Zero(3);
	ulink.dq_old << Eigen::Vector3d::Zero(3);
	ulink.ddq_old << Eigen::Vector3d::Zero(3);
	ulink.rcom << Eigen::Vector3d::Zero(3);
	ulink.drcom << Eigen::Vector3d::Zero(3);
	ulink.ddrcom << Eigen::Vector3d::Zero(3);
	ulink.rcom_old << Eigen::Vector3d::Zero(3);
	ulink.drcom_old << Eigen::Vector3d::Zero(3);
	ulink.ddrcom_old << Eigen::Vector3d::Zero(3);
	ulink.Lcom << Eigen::Vector3d::Zero(3);
	ulink.joint << Eigen::Vector3d::Zero(3);
	ulink.R << Eigen::Matrix3d::Identity(3, 3);

	//ulink.qmotor << Eigen::Vector3d::Zero(3);
	//ulink.dqmotor << Eigen::Vector3d::Zero(3);
	//ulink.ddqmotor << Eigen::Vector3d::Zero(3);
	//ulink.qlink_abs << Eigen::Vector3d::Zero(3);
	//ulink.dqlink_abs << Eigen::Vector3d::Zero(3);
	//ulink.ddqlink_abs << Eigen::Vector3d::Zero(3);
	//ulink.qdef << Eigen::Vector3d::Zero(3);
	//ulink.dqdef << Eigen::Vector3d::Zero(3);
	//ulink.ddqdef << Eigen::Vector3d::Zero(3);
	//ulink.link_torque << Eigen::Vector3d::Zero(3);
	//ulink.k_spring << Eigen::Vector3d::Zero(3);
	ulink.pcom << Eigen::Vector3d::Zero(3);
	ulink.pcom_old << Eigen::Vector3d::Zero(3);
	ulink.dpcom << Eigen::Vector3d::Zero(3);

	ulink.qcom << Eigen::Vector3d::Zero(3);
	// ulink.qcom_old << Eigen::Vector3d::Zero(3);
	// ulink.dqcom << Eigen::Vector3d::Zero(3);

	ulink.ft_com << Eigen::Vector3d::Zero(3);
	ulink.ft_dcom << Eigen::Vector3d::Zero(3);
	ulink.ft_ddcom << Eigen::Vector3d::Zero(3);
	ulink.ft_joint << Eigen::Vector3d::Zero(3);


	//return ulink;
};

void RobotStateClass::setJointLimit()
{
	// Note that 24 25 boards are not used
	// maximum joint angle positive
	double joint_max[31] = {
		// lower body #15
		80,  50,  30,  45, 45, 25, 50,  110,  50, 35, 60, 50, 110, 50, 35,
		//1,  2,   3,   4,  5,  6,  7,    8,   9, 10, 11, 12,  13, 14, 15
		// upper body #10 right arm to left arm,
		90, 85,  80,  0,  90, 30, 45, 90, 100, 80, 0 , 90, 30, 80, 0,  0
		//16,17, 18,  19, 20, 21, 22, 23,  24, 25
	};
	// maximum joint angle negative
	double joint_min[31] = {
		// lower body #15
		-80, -20, -30, -110, -110, -60, -50,  0 + 3, -70, -35, -25, -50,  0 + 3, -70, -35,
		// 1,  2,  3,     4,   5,  6,   7,        8,   9,  10,  11,  12,     13, 14, 15
		// upper body #10 right arm to left arm,
		-190, -100, -80, -120, -90, -30, -80, -190, -85,  -80, -120, -90, -30, -45,  0,  0
		//16,  17,   18,   19,  20,  21,   22, 23,  24,    25
	};
	//  need to add some degrees to knee joints for EiQuadProg, no need for qpOases

	// for (int i = 0; i < JointNUM; i++) {
	// 	q_max[i] = DEGTORAD(joint_max[i]);
	// 	q_min[i] = DEGTORAD(joint_min[i]);
	// }
	// q_max[4] = DEGTORAD(joint_max[5]); //RIGHT_HIP_ROLL,   //  5
	// q_max[5] = DEGTORAD(joint_max[6]); //RIGHT_HIP_YAW,    //  6
	// q_max[6] = DEGTORAD(joint_max[7]); //RIGHT_KNEE_PITCH,   //  7
	// q_max[8] = DEGTORAD(joint_max[8]); //RIGHT_FOOT_PITCH,   //  9
	// q_max[7] = DEGTORAD(joint_max[9]); //RIGHT_FOOT_ROLL,    //  8
	// q_max[9] = DEGTORAD(joint_max[4]); //LEFT_HIP_PITCH,   //  4
	// q_max[14] = DEGTORAD(joint_max[13]); //LEFT_FOOT_PITCH,    // 14
	// q_max[13] = DEGTORAD(joint_max[14]); //LEFT_FOOT_ROLL,   // 13

	// q_min[4] = DEGTORAD(joint_min[5]); //RIGHT_HIP_ROLL,   //  5
	// q_min[5] = DEGTORAD(joint_min[6]); //RIGHT_HIP_YAW,    //  6
	// q_min[6] = DEGTORAD(joint_min[7]); //RIGHT_KNEE_PITCH,   //  7
	// q_min[8] = DEGTORAD(joint_min[8]); //RIGHT_FOOT_PITCH,   //  9
	// q_min[7] = DEGTORAD(joint_min[9]); //RIGHT_FOOT_ROLL,    //  8
	// q_min[9] = DEGTORAD(joint_min[4]); //LEFT_HIP_PITCH,   //  4
	// q_min[14] = DEGTORAD(joint_min[13]); //LEFT_FOOT_PITCH,    // 14
	// q_min[13] = DEGTORAD(joint_min[14]); //LEFT_FOOT_ROLL,   // 13

	if (robotpara.TRUE_SIM_FALSE_REALROBOT) {
		joint_max[RIGHT_SHOULDER_ROLL] = (0);
		joint_min[RIGHT_SHOULDER_ROLL] = (-180);
		joint_max[LEFT_SHOULDER_ROLL] = (180);
		joint_min[LEFT_SHOULDER_ROLL] = (0);
	}

	joint_max[RIGHT_ELBOW_PITCH] = (-5);
	joint_max[LEFT_ELBOW_PITCH] = (-5);
	joint_min[RIGHT_ELBOW_PITCH] = (-120);
	joint_min[LEFT_ELBOW_PITCH] = (-120);

	// joint_max[LEFT_SHOULDER_PITCH] = (0);
	// joint_min[LEFT_SHOULDER_PITCH] = (0);

	for (int i = 0; i < JointNUM; i++) {
		q_max[i] = DEGTORAD(joint_max[i]);
		q_min[i] = DEGTORAD(joint_min[i]);
	}
	q_max[RIGHT_HIP_ROLL  ] = DEGTORAD(joint_max[5]); //RIGHT_HIP_ROLL,   //  5
	q_max[RIGHT_HIP_YAW   ] = DEGTORAD(joint_max[6]); //RIGHT_HIP_YAW,    //  6
	q_max[RIGHT_KNEE_PITCH] = DEGTORAD(joint_max[7]); //RIGHT_KNEE_PITCH,   //  7
	q_max[RIGHT_FOOT_PITCH] = DEGTORAD(joint_max[8]); //RIGHT_FOOT_PITCH,   //  9
	q_max[RIGHT_FOOT_ROLL ] = DEGTORAD(joint_max[9]); //RIGHT_FOOT_ROLL,    //  8
	q_max[LEFT_HIP_PITCH  ] = DEGTORAD(joint_max[4]); //LEFT_HIP_PITCH,   //  4
	q_max[LEFT_FOOT_PITCH ] = DEGTORAD(joint_max[13]); //LEFT_FOOT_PITCH,    // 14
	q_max[LEFT_FOOT_ROLL  ] = DEGTORAD(joint_max[14]); //LEFT_FOOT_ROLL,   // 13

	q_min[RIGHT_HIP_ROLL  ] = DEGTORAD(joint_min[5]); //RIGHT_HIP_ROLL,   //  5
	q_min[RIGHT_HIP_YAW   ] = DEGTORAD(joint_min[6]); //RIGHT_HIP_YAW,    //  6
	q_min[RIGHT_KNEE_PITCH] = DEGTORAD(joint_min[7]); //RIGHT_KNEE_PITCH,   //  7
	q_min[RIGHT_FOOT_PITCH] = DEGTORAD(joint_min[8]); //RIGHT_FOOT_PITCH,   //  9
	q_min[RIGHT_FOOT_ROLL ] = DEGTORAD(joint_min[9]); //RIGHT_FOOT_ROLL,    //  8
	q_min[LEFT_HIP_PITCH  ] = DEGTORAD(joint_min[4]); //LEFT_HIP_PITCH,   //  4
	q_min[LEFT_FOOT_PITCH ] = DEGTORAD(joint_min[13]); //LEFT_FOOT_PITCH,    // 14
	q_min[LEFT_FOOT_ROLL  ] = DEGTORAD(joint_min[14]); //LEFT_FOOT_ROLL,   // 13
}

void RobotStateClass::Init()
{
	if (!IsInitial) {

		// f1 << 0.13,-0.05,0;
		// f2 << 0.13,0.05,0;
		// f3 << -0.05,0.05,0;
		// f4 << -0.05,-0.05,0;

		// fcontact.col(0)<<f1;
		// fcontact.col(1)<<f2;
		// fcontact.col(2)<<f3;
		// fcontact.col(3)<<f4;

		// if lower body k=0; else k=1
		//int k=1;
		//
		CRobotSeg thigh, calf, foot, pelvis, body, head, upperarm, lowerarm, hand;

		double k;
		if (IsWholeBodyRobot) {
			k = 1;
		}
		else {
			k = 0;
		}

		// thigh.m = robotpara.THIGH_MASS;
		// thigh.dcom << 0, 0, -0.1007;
		// thigh.limb << 0, 0, -robotpara.THIGH_HEIGHT;

		// calf.m = robotpara.CALF_MASS;
		// calf.dcom << 0.0013, 0, -0.1267;
		// calf.limb << 0, 0, -robotpara.CALF_HEIGHT;

		// foot.m = robotpara.FOOT_MASS;
		// foot.dcom << 0.018, 0, -0.0559;
		// foot.limb << 0, 0, -RobotParaClass::ANKLE_HEIGHT();

		// pelvis.m = robotpara.PELVIS_MASS;
		// pelvis.dcom << -0.00108, 0, 0.08115;
		// pelvis.limb << robotpara.HIP_BODY_OFFSET_X, 0, robotpara.PELVIS_HEIGHT;

		// body.m = k * robotpara.TORSO_MASS;
		// body.dcom << 0.0089 + 0.0, 0, 0.1725;
		// body.limb << -0.015, 0, robotpara.TORSO_HEIGHT;

		// head.m = k * robotpara.HEAD_MASS;
		// head.dcom << 0, 0, 0.15;
		// head.limb << 0, 0, 0.30; // i assume a pseudo head size

		// upperarm.m = k * robotpara.UPPER_ARM_MASS;
		// upperarm.dcom << 0, 0, -0.06536;
		// upperarm.limb << 0, 0, -robotpara.UPPER_ARM_HEIGHT;

		// lowerarm.m = k * robotpara.FORE_ARM_MASS;
		// lowerarm.dcom << 0, 0, -0.12;
		// lowerarm.limb << 0, 0, -robotpara.FORE_ARM_HEIGHT;

		// hand.m = k * robotpara.HAND_MASS;
		// hand.dcom << 0, 0, -0.05;
		// hand.limb << 0, 0, 0;

		thigh.m = robotpara.THIGH_MASS;
		thigh.dcom << 0, 0, -0.5 * robotpara.THIGH_HEIGHT;
		thigh.limb << 0, 0, -robotpara.THIGH_HEIGHT;

		calf.m = robotpara.CALF_MASS;
		calf.dcom << 0.0, 0, -0.5 * robotpara.CALF_HEIGHT;
		calf.limb << 0, 0, -robotpara.CALF_HEIGHT;

		foot.m = robotpara.FOOT_MASS;
		foot.dcom << 0.5 * RobotParaClass::ANKLE_X_OFFSET(), 0, -0.7 * RobotParaClass::ANKLE_HEIGHT();
		foot.limb << 0, 0, -RobotParaClass::ANKLE_HEIGHT();

		pelvis.m = robotpara.PELVIS_MASS;
		pelvis.dcom << 0, 0, 0.8 * robotpara.PELVIS_HEIGHT;
		pelvis.limb << robotpara.HIP_BODY_OFFSET_X, 0, robotpara.PELVIS_HEIGHT;

		body.m = k * robotpara.TORSO_MASS;
		body.dcom << 0.00 + 0.0, 0, 0.85 * robotpara.TORSO_HEIGHT;
		body.limb << 0.0, 0, robotpara.TORSO_HEIGHT;

		head.m = k * robotpara.HEAD_MASS;
		head.dcom << 0, 0, 0.15;
		head.limb << 0, 0, 0.30; // i assume a pseudo head size

		upperarm.m = k * robotpara.UPPER_ARM_MASS;
		upperarm.dcom << 0, 0, -0.3 * robotpara.UPPER_ARM_HEIGHT;
		upperarm.limb << 0, 0, -robotpara.UPPER_ARM_HEIGHT;

		lowerarm.m = k * robotpara.FORE_ARM_MASS;
		lowerarm.dcom << 0, 0, -0.65 * robotpara.FORE_ARM_HEIGHT;
		lowerarm.limb << 0, 0, -robotpara.FORE_ARM_HEIGHT;

		hand.m = k * robotpara.HAND_MASS;
		hand.dcom << 0, 0, -0.05;
		hand.limb << 0, 0, 0;

		Eigen::VectorXi id(5);
		id << 0, 1, 2, 3, 4; // id of main branch
		idno = id.rows();
		Eigen::Vector3i seg(0, 1, 2); // number of segments in a branch
		segno = seg.rows();

		InitRobotLink(LINK[seg(0)][id(0)], pelvis);
		InitRobotLink(LINK[seg(1)][id(0)], body);
		InitRobotLink(LINK[seg(2)][id(0)], head);

		InitRobotLink(LINK[seg(0)][id(1)], thigh);
		InitRobotLink(LINK[seg(1)][id(1)], calf);
		InitRobotLink(LINK[seg(2)][id(1)], foot);

		InitRobotLink(LINK[seg(0)][id(2)], thigh);
		InitRobotLink(LINK[seg(1)][id(2)], calf);
		InitRobotLink(LINK[seg(2)][id(2)], foot);

		InitRobotLink(LINK[seg(0)][id(3)], upperarm);
		InitRobotLink(LINK[seg(1)][id(3)], lowerarm);
		InitRobotLink(LINK[seg(2)][id(3)], hand);

		InitRobotLink(LINK[seg(0)][id(4)], upperarm);
		InitRobotLink(LINK[seg(1)][id(4)], lowerarm);
		InitRobotLink(LINK[seg(2)][id(4)], hand);

		LINK[seg(0)][id(0)].name = "pelvis";
		LINK[seg(1)][id(0)].name = "upperbody";
		LINK[seg(2)][id(0)].name = "head";

		LINK[seg(0)][id(1)].name = "left thigh";
		LINK[seg(1)][id(1)].name = "left calf";
		LINK[seg(2)][id(1)].name = "left foot";

		LINK[seg(0)][id(2)].name = "right thigh";
		LINK[seg(1)][id(2)].name = "right calf";
		LINK[seg(2)][id(2)].name = "right foot";

		LINK[seg(0)][id(3)].name = "left upper arm";
		LINK[seg(1)][id(3)].name = "left lower arm";
		LINK[seg(2)][id(3)].name = "left hand";

		LINK[seg(0)][id(4)].name = "right upper arm";
		LINK[seg(1)][id(4)].name = "right lower arm";
		LINK[seg(2)][id(4)].name = "right hand";

		LINK[0][0].rcom << LINK[0][0].dcom;   // pelvis is the root 0, so rcom is the same as displacement of com vector
		LINK[0][1].joint = hipwidth; // left leg
		LINK[0][2].joint = -hipwidth; // right leg

		for (int i = 0; i < idno; i++) {
			for (int j = 0; j < segno; j++) {
				m += LINK[j][i].m;
			}
		}
		m = robotpara.totalmass;
		//m -= 2*FOOT_MASS;

		DPRINTF("Robot's total mass in estimation is: %.3f kg\n", m);
		DPRINTF("Robot State Initialization Finished!\n");

		//delete thigh, calf, foot, pelvis, body, head, upperarm, lowerarm, hand;
		IsInitial = true;
	}
	else {
		DPRINTF("Error!!! Robot State is Already Initialized!\n");
	}
};

void RobotStateClass::UpdateRef(const std::vector<double> &comRef, const std::vector<double> &lftRef, const std::vector<double> &rftRef)
{
	for (unsigned int i = 0; i < 3; i++) {
		ComRef[i] = comRef[i];
		LftRef[i] = lftRef[i];
		RftRef[i] = rftRef[i];
	}
}

void RobotStateClass::CalcDesiredArmVelocity(const Eigen::Vector3d& LhdPosRef, const Eigen::Matrix3d& LhdOriRef, const Eigen::Vector3d& RhdPosRef, const Eigen::Matrix3d& RhdOriRef)
{
	Eigen::Vector3d EulerAng;
	EulerAng[0] = atan2(LhdOriRef(2, 1), LhdOriRef(2, 2));   //phi
	EulerAng[1] = asin(-LhdOriRef(2, 0));           //theta
	EulerAng[2] = atan2(LhdOriRef(1, 0), LhdOriRef(0, 0));    //greek Y
	SendToRobot->fin_lhd_pose.segment<3>(0) = LhdPosRef;
	SendToRobot->fin_lhd_pose.segment<3>(3) = EulerAng;

	EulerAng[0] = atan2(RhdOriRef(2, 1), RhdOriRef(2, 2));   //phi
	EulerAng[1] = asin(-RhdOriRef(2, 0));           //theta
	EulerAng[2] = atan2(RhdOriRef(1, 0), RhdOriRef(0, 0));    //greek Y
	SendToRobot->fin_rhd_pose.segment<3>(0) = RhdPosRef;
	SendToRobot->fin_rhd_pose.segment<3>(3) = EulerAng;

	// =============================================================
	Eigen::Vector3d dRhdRef = (RhdPosRef - RhdPosRef_old) / dt;
	Eigen::Vector3d dLhdRef = (LhdPosRef - LhdPosRef_old) / dt;

	RBDL::Math::SpatialTransform LHandFrame = _model->getLocalBodyFrame(LEFT_HAND);
	RBDL::Math::SpatialTransform RHandFrame = _model->getLocalBodyFrame(RIGHT_HAND);

	Eigen::Vector3d err_d_pos_R(0.0, 0.0, 0.0), err_d_ang_R(0.0, 0.0, 0.0), err_d_pos_L(0.0, 0.0, 0.0), err_d_ang_L(0.0, 0.0, 0.0);

	// double PosGain = 1.0; // for ode
	double PosGain = robotpara.kp_debug;
	err_d_pos_R = dRhdRef + PosGain * (RhdPosRef - RHandFrame.r) / dt;
	err_d_pos_L = dLhdRef + PosGain * (LhdPosRef - LHandFrame.r) / dt;

	// for orientations
	Eigen::Quaterniond quat_L(LHandFrame.E), quat_R(RHandFrame.E);

	QuaternionClass q_L, q_R, qd_L, qd_R;
	q_L.w = quat_L.w();
	q_L.x = quat_L.x();
	q_L.y = quat_L.y();
	q_L.z = quat_L.z();
	q_R.w = quat_R.w();
	q_R.x = quat_R.x();
	q_R.y = quat_R.y();
	q_R.z = quat_R.z();

	Eigen::Matrix3d LhdOriRef_local = LhdOriRef;
	Eigen::Matrix3d RhdOriRef_local = RhdOriRef;

	Eigen::Quaterniond quat_Ld(LhdOriRef_local);
	Eigen::Quaterniond quat_Rd(RhdOriRef_local);

	qd_L.w = quat_Ld.w();
	qd_L.x = quat_Ld.x();
	qd_L.y = quat_Ld.y();
	qd_L.z = quat_Ld.z();
	qd_R.w = quat_Rd.w();
	qd_R.x = quat_Rd.x();
	qd_R.y = quat_Rd.y();
	qd_R.z = quat_Rd.z();

	KDL::Vector xerr_o_L, xerr_o_R;

	xerr_o_L = QuaternionClass::error(q_L, qd_L);
	xerr_o_R = QuaternionClass::error(q_R, qd_R);

	Eigen::Vector3d wL(0.0, 0.0, 0.0), wR(0.0, 0.0, 0.0);
	// double OriGain = 15.0;// for ode
	double OriGain = robotpara.kd_debug;
	for (int i = 0; i < 3; ++i) {
		wL[i] = -OriGain * xerr_o_L[i];
		wR[i] = -OriGain * xerr_o_R[i];
	}

	// w = skew(\dot{R}*R^T)
	Eigen::Matrix3d S_L = (LhdOriRef_local - LhdOriRef_local_old) / dt * LhdOriRef_local.transpose();
	Eigen::Matrix3d S_R = (RhdOriRef_local - RhdOriRef_local_old) / dt * RhdOriRef_local.transpose();

	Eigen::Vector3d dOriLhdToComRef, dOriRhdToComRef;
	dOriLhdToComRef(0) = S_L(2, 1);
	dOriLhdToComRef(1) = S_L(0, 2);
	dOriLhdToComRef(2) = S_L(1, 0);
	dOriRhdToComRef(0) = S_R(2, 1);
	dOriRhdToComRef(1) = S_R(0, 2);
	dOriRhdToComRef(2) = S_R(1, 0);

	LhdOriRef_local_old = LhdOriRef_local;
	RhdOriRef_local_old = RhdOriRef_local;

	err_d_ang_R = dOriRhdToComRef + wR;
	err_d_ang_L = dOriLhdToComRef + wL;

	SendToRobot->des_rhd_vel.segment<3>(0) = err_d_pos_R;
	SendToRobot->des_rhd_vel.segment<3>(3) = err_d_ang_R;
	SendToRobot->des_lhd_vel.segment<3>(0) = err_d_pos_L;
	SendToRobot->des_lhd_vel.segment<3>(3) = err_d_ang_L;

	// acc ---------------------------
	Eigen::Vector3d ddRhdRef = (dRhdRef - dRhdRef_old) / dt;
	Eigen::Vector3d ddLhdRef = (dLhdRef - dLhdRef_old) / dt;

	Eigen::Vector3d err_dd_pos_R(0.0, 0.0, 0.0), err_dd_ang_R(0.0, 0.0, 0.0), err_dd_pos_L(0.0, 0.0, 0.0), err_dd_ang_L(0.0, 0.0, 0.0);

	Eigen::Vector3d dRhd_r = (RHandFrame.r - RHandFrame_r_old) / dt;
	Eigen::Vector3d dLhd_r = (LHandFrame.r - LHandFrame_r_old) / dt;

	double kp_pos = 0.01;
	double kd_pos = 10;
	// double kp_pos = robotpara.kp_debug;
	// double kd_pos = robotpara.kd_debug;
	err_dd_pos_R = ddRhdRef + kd_pos * (dRhdRef - dRhd_r) + kp_pos * (RhdPosRef - RHandFrame.r) / dt;

	err_dd_pos_L = ddLhdRef + kd_pos * (dLhdRef - dLhd_r) + kp_pos * (LhdPosRef - LHandFrame.r) / dt;

	Eigen::Vector3d ddRhdOri = (dOriRhdToComRef - dOriRhdToComRef_old) / dt;
	Eigen::Vector3d ddLhdOri = (dOriLhdToComRef - dOriLhdToComRef_old) / dt;

	// real orientation angular velocity
	S_L = (LHandFrame.E - LHandFrame_E_old) / dt * LHandFrame.E.transpose();
	S_R = (RHandFrame.E - RHandFrame_E_old) / dt * RHandFrame.E.transpose();

	Eigen::Vector3d dOriLhdToComMeasure, dOriRhdToComMeasure;
	dOriLhdToComMeasure(0) = S_L(2, 1);
	dOriLhdToComMeasure(1) = S_L(0, 2);
	dOriLhdToComMeasure(2) = S_L(1, 0);
	dOriRhdToComMeasure(0) = S_R(2, 1);
	dOriRhdToComMeasure(1) = S_R(0, 2);
	dOriRhdToComMeasure(2) = S_R(1, 0);

	double kp_ori_acc = 50;
	double kd_ori_acc = 10;
	// double kp_ori_acc = robotpara.kp_debug;
	// double kd_ori_acc = robotpara.kd_debug;
	err_dd_ang_R = ddRhdOri + kd_ori_acc * (dOriRhdToComRef - dOriRhdToComMeasure) - kp_ori_acc * vKDLtoEigen(xerr_o_R);
	err_dd_ang_L = ddLhdOri + kd_ori_acc * (dOriLhdToComRef - dOriLhdToComMeasure) - kp_ori_acc * vKDLtoEigen(xerr_o_L);

	SendToRobot->des_rhd_acc.segment<3>(0) = err_dd_pos_R;
	SendToRobot->des_rhd_acc.segment<3>(3) = err_dd_ang_R;
	SendToRobot->des_lhd_acc.segment<3>(0) = err_dd_pos_L;
	SendToRobot->des_lhd_acc.segment<3>(3) = err_dd_ang_L;

	// ---------- save old -------------------
	LhdPosRef_old = LhdPosRef;
	RhdPosRef_old = RhdPosRef;
	dRhdRef_old = dRhdRef;
	dLhdRef_old = dLhdRef;

	LHandFrame_r_old = LHandFrame.r;
	RHandFrame_r_old = RHandFrame.r;

	LHandFrame_E_old = LHandFrame.E;
	RHandFrame_E_old = RHandFrame.E;

	dOriLhdToComRef_old = dOriLhdToComRef;
	dOriRhdToComRef_old = dOriRhdToComRef;

}

void RobotStateClass::CalcDesiredFootVelocity(const Eigen::Vector3d& HipPosRef, const Eigen::Matrix3d& HipOriLftRef, const Eigen::Matrix3d& HipOriRftRef, const Eigen::Vector3d& LftPosRef, const Eigen::Matrix3d& LftOriRef, const Eigen::Vector3d& RftPosRef, const Eigen::Matrix3d& RftOriRef)
{
	Eigen::Vector3d dRftRef = (RftPosRef - RftPosRef_old) / dt;
	Eigen::Vector3d dLftRef = (LftPosRef - LftPosRef_old) / dt;
	Eigen::Vector3d dHipRef = (HipPosRef - HipPosRef_old) / dt;

	RBDL::Math::SpatialTransform LFootFrame = _model->getLocalBodyFrame(LEFT_FOOT);
	RBDL::Math::SpatialTransform RFootFrame = _model->getLocalBodyFrame(RIGHT_FOOT);

	Eigen::Quaterniond quat_L(LFootFrame.E), quat_R(RFootFrame.E);

	Eigen::Vector3d wL(0.0, 0.0, 0.0), wR(0.0, 0.0, 0.0);
	QuaternionClass q_L, q_R, qd_L, qd_R;
	q_L.w = quat_L.w();
	q_L.x = quat_L.x();
	q_L.y = quat_L.y();
	q_L.z = quat_L.z();
	q_R.w = quat_R.w();
	q_R.x = quat_R.x();
	q_R.y = quat_R.y();
	q_R.z = quat_R.z();

	Eigen::Matrix3d h_R_lft = HipOriLftRef.transpose() * LftOriRef;
	Eigen::Matrix3d h_R_rft = HipOriRftRef.transpose() * RftOriRef;

	Eigen::Quaterniond quat_Ld(h_R_lft);
	Eigen::Quaterniond quat_Rd(h_R_rft);

	qd_L.w = quat_Ld.w();
	qd_L.x = quat_Ld.x();
	qd_L.y = quat_Ld.y();
	qd_L.z = quat_Ld.z();
	qd_R.w = quat_Rd.w();
	qd_R.x = quat_Rd.x();
	qd_R.y = quat_Rd.y();
	qd_R.z = quat_Rd.z();

	KDL::Vector xerr_o_L, xerr_o_R;

	xerr_o_L = QuaternionClass::error(q_L, qd_L);
	xerr_o_R = QuaternionClass::error(q_R, qd_R);

	double OriGain = robotpara.ik_ori_gain;
	for (int i = 0; i < 3; ++i) {
		wL[i] = -OriGain * xerr_o_L[i];
		wR[i] = -OriGain * xerr_o_R[i];
	}

	Eigen::Vector3d err_d_pos_R(0.0, 0.0, 0.0), err_d_ang_R(0.0, 0.0, 0.0), err_d_pos_L(0.0, 0.0, 0.0), err_d_ang_L(0.0, 0.0, 0.0);

	// Eigen::Vector3d rft_pos_off_ini = HipOriRftRef.transpose() * (RftPosRef - HipPosRef) - vKDLtoEigen((RFootFrame).p);
	// Eigen::Vector3d lft_pos_off_ini = HipOriRftRef.transpose() * (LftPosRef - HipPosRef) - vKDLtoEigen((LFootFrame).p);
	// COUT(rft_pos_off_ini.transpose(), lft_pos_off_ini.transpose(), 0.5*(rft_pos_off_ini+lft_pos_off_ini).transpose());

	// Eigen::Vector3d ft_off(0.00350817, 0.0, -3.06159e-05); // for old ComanODE
	Eigen::Vector3d ft_off(0.0, 0.0, 0.0);
	Eigen::Vector3d Rft_off = HipOriRftRef.transpose() * RftOriRef * ft_off;
	Eigen::Vector3d Lft_off = HipOriLftRef.transpose() * LftOriRef * ft_off;

	double PosGain = robotpara.ik_pos_gain;
	err_d_pos_R = HipOriRftRef.transpose() * (dRftRef - dHipRef) + PosGain * (HipOriRftRef.transpose() * (RftPosRef - HipPosRef) - RFootFrame.r - Rft_off) / dt;

	err_d_pos_L = HipOriLftRef.transpose() * (dLftRef - dHipRef) + PosGain * (HipOriLftRef.transpose() * (LftPosRef - HipPosRef) - LFootFrame.r - Lft_off) / dt;

	Eigen::Matrix3d LftCom = HipOriLftRef.transpose() * LftOriRef;
	Eigen::Matrix3d RftCom = HipOriRftRef.transpose() * RftOriRef;

	// w = skew(\dot{R}*R^T)
	Eigen::Matrix3d S_L = (LftCom - LftCom_old) / dt * LftCom.transpose();
	Eigen::Matrix3d S_R = (RftCom - RftCom_old) / dt * RftCom.transpose();

	Eigen::Vector3d dOriLftToComRef, dOriRftToComRef;
	dOriLftToComRef(0) = S_L(2, 1);
	dOriLftToComRef(1) = S_L(0, 2);
	dOriLftToComRef(2) = S_L(1, 0);
	dOriRftToComRef(0) = S_R(2, 1);
	dOriRftToComRef(1) = S_R(0, 2);
	dOriRftToComRef(2) = S_R(1, 0);

	LftCom_old = LftCom;
	RftCom_old = RftCom;

	err_d_ang_R = dOriRftToComRef + wR;
	err_d_ang_L = dOriLftToComRef + wL;

	SendToRobot->des_rft_vel.segment<3>(0) = err_d_pos_R;
	SendToRobot->des_rft_vel.segment<3>(3) = err_d_ang_R;
	SendToRobot->des_lft_vel.segment<3>(0) = err_d_pos_L;
	SendToRobot->des_lft_vel.segment<3>(3) = err_d_ang_L;

	// acc ---------------------------
	Eigen::Vector3d ddRftRef = (dRftRef - dRftRef_old) / dt;
	Eigen::Vector3d ddLftRef = (dLftRef - dLftRef_old) / dt;
	Eigen::Vector3d ddHipRef = (dHipRef - dHipRef_old) / dt;

	Eigen::Vector3d err_dd_pos_R(0.0, 0.0, 0.0), err_dd_ang_R(0.0, 0.0, 0.0), err_dd_pos_L(0.0, 0.0, 0.0), err_dd_ang_L(0.0, 0.0, 0.0);

	Eigen::Vector3d dRft_r = (RFootFrame.r - RFootFrame_r_old) / dt;
	Eigen::Vector3d dLft_r = (LFootFrame.r - LFootFrame_r_old) / dt;

	double kp_acc = robotpara.ik_kp_pos_acc_gain;
	double kd_acc = robotpara.ik_kd_pos_acc_gain;
	err_dd_pos_R = HipOriRftRef.transpose() * (ddRftRef - ddHipRef) + kd_acc * (HipOriRftRef.transpose() * (dRftRef - dHipRef) - dRft_r) + kp_acc * (HipOriRftRef.transpose() * (RftPosRef - HipPosRef) - RFootFrame.r - Rft_off) / dt;

	err_dd_pos_L = HipOriLftRef.transpose() * (ddLftRef - ddHipRef) + kd_acc * (HipOriLftRef.transpose() * (dLftRef - dHipRef) - dLft_r) + kp_acc * (HipOriLftRef.transpose() * (LftPosRef - HipPosRef) - LFootFrame.r - Lft_off) / dt;

	Eigen::Vector3d ddRftOri = (dOriRftToComRef - dOriRftToComRef_old) / dt;
	Eigen::Vector3d ddLftOri = (dOriLftToComRef - dOriLftToComRef_old) / dt;

	// real orientation angular velocity
	S_L = (LFootFrame.E - LFootFrame_E_old) / dt * LFootFrame.E.transpose();
	S_R = (RFootFrame.E - RFootFrame_E_old) / dt * RFootFrame.E.transpose();

	Eigen::Vector3d dOriLftToComMeasure, dOriRftToComMeasure;
	dOriLftToComMeasure(0) = S_L(2, 1);
	dOriLftToComMeasure(1) = S_L(0, 2);
	dOriLftToComMeasure(2) = S_L(1, 0);
	dOriRftToComMeasure(0) = S_R(2, 1);
	dOriRftToComMeasure(1) = S_R(0, 2);
	dOriRftToComMeasure(2) = S_R(1, 0);

	double kp_ori_acc = robotpara.ik_kp_ori_acc_gain;
	double kd_ori_acc = robotpara.ik_kd_ori_acc_gain;
	// double kp_ori_acc = robotpara.kp_debug;
	// double kd_ori_acc = robotpara.kd_debug;
	err_dd_ang_R = ddRftOri + kd_ori_acc * (dOriRftToComRef - dOriRftToComMeasure) - kp_ori_acc * vKDLtoEigen(xerr_o_R);
	err_dd_ang_L = ddLftOri + kd_ori_acc * (dOriLftToComRef - dOriLftToComMeasure) - kp_ori_acc * vKDLtoEigen(xerr_o_L);

	SendToRobot->des_rft_acc.segment<3>(0) = err_dd_pos_R;
	SendToRobot->des_rft_acc.segment<3>(3) = err_dd_ang_R;
	SendToRobot->des_lft_acc.segment<3>(0) = err_dd_pos_L;
	SendToRobot->des_lft_acc.segment<3>(3) = err_dd_ang_L;

	// ---------- save old -------------------
	LftPosRef_old = LftPosRef;
	RftPosRef_old = RftPosRef;
	HipPosRef_old = HipPosRef;

	dRftRef_old = dRftRef;
	dLftRef_old = dLftRef;
	dHipRef_old = dHipRef;

	LFootFrame_r_old = LFootFrame.r;
	RFootFrame_r_old = RFootFrame.r;

	LFootFrame_E_old = LFootFrame.E;
	RFootFrame_E_old = RFootFrame.E;

	dOriLftToComRef_old = dOriLftToComRef;
	dOriRftToComRef_old = dOriRftToComRef;

	// SendToRobot->Update();

}