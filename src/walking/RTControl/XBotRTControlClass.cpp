/*****************************************************************************
XBotRTControlClass.cpp
*****************************************************************************/
#include "RTControl/XBotRTControlClass.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#define USE_XBOT_LOGGER
#include <time.h>


// #include "ZMPWalk/swingfootcartclass.h"

#ifdef USE_OPENSOT
#include "IK/IKClass_OpenSoT.h"
#endif

XBotRTControlClass::XBotRTControlClass()
	: zc(RobotParaClass::Z_C())
#ifdef USE_OPENSOT
	, zh(RobotParaClass::WAIST_HEIGHT() - RobotParaClass::HOMING_LIFT_HEIGHT())
#else
	, zh(RobotParaClass::FULL_LEG() - RobotParaClass::HOMING_LIFT_HEIGHT())
#endif
	, xstate(0.0, 0.0, 0.0)
	, ystate(0.0, 0.0, 0.0)
	, xcop(0.0)
	, ycop(0.0)
	, LeftFootPos(RobotParaClass::HIP_TO_ANKLE_X_OFFSET(), RobotParaClass::HALF_HIP_WIDTH(), 0)
	, RightFootPos(RobotParaClass::HIP_TO_ANKLE_X_OFFSET(), -RobotParaClass::HALF_HIP_WIDTH(), 0)
	, PelvisPos(0, 0, RobotParaClass::FULL_LEG())
	, IsStart(false)
	, IsInit(false)
	, HipO_comL(0.0)
	, HipO_comR(0.0)
	, dtime(0)
	, walkdtime(0)
	, walkstarttime(0.0)
	, StaEnableX(false)
	, StaEnableY(false)
	, StaEnableZ(false)
	, KeyBoardSwitch(1)
	, steptime(RobotPara().Tstep)
	, steplength(0.0)
	, stepwidth(0.0)
	, stepheight(0.0)
	, turnangle(0.0)
	, JointNUM(RobotParaClass::JOINT_NUM())
	, jointestime(0)
	, Is_testJoint(false)
	, En_testJoint(false)
	, jointmax(RobotParaClass::JOINT_NUM(), 0.0)
	, jointmin(RobotParaClass::JOINT_NUM(), 0.0)
	, IsFixed(RobotPara().IsFixedWalk)
	, cycN(0)
	, traj_ref_old(RobotParaClass::JOINT_NUM(), 0.0)
	, initHipx(0.0), initHipy(0.0)
	, Is_xCOPinCenter(false), Is_yCOPinCenter(false)
	, dcounter(0), dbuffertime(0)
	, IsInitZMPold(false)
	, realtime(0.0)
	, Fz_ref(2, 0)
	, deltaFtZ(2, 0)
	, TurnYaw(0.0)
	, HipPos(0, 0, 0)
	, dt(RobotParaClass::dT())
	, deltaXYZ(0, 0, 0)
	, IsLoggerSaved(false)
	, vFullLeg(0, 0, RobotParaClass::FULL_LEG())
	, IsSetHandRefToWorld(false)
	, hand_startT(0.0)
	, hand_startT_d(0)
	, WaistComp(0.0, 0.0, 0.0)
	, IsMoveHand(false)
	, srdf_homing_angles(RobotParaClass::JOINT_NUM(), 0.0)
	, walkingVel(0.0, 0.0, 0.0)
	, EnableFtPos(false)
	, EnableFtOri(false)
 	, IsStartWalk(false)
//	, IsStartWalk(true)
{
	deltaFtOri_left = deltaFtOri_right = Eigen::Matrix3d::Identity();
	deltaHip = deltaFtPos_l = deltaFtPos_r = Eigen::Vector3d::Zero();
	com_ref = lft_ref = rft_ref = zmp_ref = Eigen::Vector3d::Zero();

	LeftHandPos = RobotPara().LEFT_HAND_HOME_POS;
	RightHandPos = RobotPara().RIGHT_HAND_HOME_POS;
	LeftHandOGlobal = LeftHandO = RobotPara().LEFT_HAND_HOME_ORI;
	RightHandOGlobal = RightHandO = RobotPara().RIGHT_HAND_HOME_ORI;

	LeftHandPosGlobal = vFullLeg + LeftHandPos;
	RightHandPosGlobal = vFullLeg + RightHandPos;

	LeftFootPos = RobotPara().LEFT_FOOT_HOME_POS;
	RightFootPos = RobotPara().RIGHT_FOOT_HOME_POS;
	LeftFootO = RobotPara().LEFT_FOOT_HOME_ORI;
	RightFootO = RobotPara().RIGHT_FOOT_HOME_ORI;

	LEFT_FOOT_HOME_POS  = RobotPara().LEFT_FOOT_HOME_POS ;
	RIGHT_FOOT_HOME_POS = RobotPara().RIGHT_FOOT_HOME_POS;
	LEFT_FOOT_HOME_ORI  = RobotPara().LEFT_FOOT_HOME_ORI ;
	RIGHT_FOOT_HOME_ORI = RobotPara().RIGHT_FOOT_HOME_ORI;
	LEFT_HAND_HOME_POS  = RobotPara().LEFT_HAND_HOME_POS ;
	RIGHT_HAND_HOME_POS = RobotPara().RIGHT_HAND_HOME_POS;
	LEFT_HAND_HOME_ORI  = RobotPara().LEFT_HAND_HOME_ORI ;
	RIGHT_HAND_HOME_ORI = RobotPara().RIGHT_HAND_HOME_ORI;

	std::vector<double> homing_angles(RobotPara().HOMING_POS());
	std::transform(homing_angles.begin(), homing_angles.end(), homing_angles.begin(), std::bind1st(std::multiplies<double>(), M_PI / 180.0));
	HomingInit(homing_angles);

	// set initial elbow angles to avoid singularity
	const std::vector<double>& OffsetAng = RobotPara().HOMING_POS();
	const RobotStateClass& irobot = _WBS.getRobotState();
	for (int i = 0; i < RobotParaClass::JOINT_NUM(); i++) {
		irobot.SendToRobot->q[i] = DEGTORAD(OffsetAng[i]);
	}

	deltaHipO = Eigen::Matrix3d::Identity(3, 3);
	body_pitch_des = DEGTORAD(0);

	Ksway = RobotPara().Ksway;
	// LeftFootO << Eigen::Matrix3d::Identity(3, 3);
	// RightFootO << Eigen::Matrix3d::Identity(3, 3);
	HipO_Left << Eigen::Matrix3d::Identity(3, 3);
	HipO_Right << Eigen::Matrix3d::Identity(3, 3);
	HipO_Left_comp << Eigen::Matrix3d::Identity(3, 3);
	HipO_Right_comp << Eigen::Matrix3d::Identity(3, 3);
	HipO_Turn << Eigen::Matrix3d::Identity(3, 3);
	WaistO << Eigen::Matrix3d::Identity(3, 3);

#ifdef USE_XBOT_LOGGER
	xbot_logger = XBot::MatLogger::getLogger("/tmp/RTControl_log");
	initLogger(logger_len);
#endif

	if (RBDL_API_VERSION == irobot._model->rbdl250) {
		n_RBDL_floating_jnt = 2; // for rbdl 2.5.0
	}
	else if (RBDL_API_VERSION == irobot._model->rbdl240) {
		n_RBDL_floating_jnt = 6; // for rbdl 2.4.0
	}
	q_rbdl = Eigen::VectorXd::Zero(n_RBDL_floating_jnt + JointNUM);
	e_i = Eigen::VectorXd::Zero(JointNUM);

	_WBS.EnableGravityCompensation(false);

	q_tmp = Eigen::VectorXd::Zero(37);

	console = new Read_XDDP_pipe(std::string("boards_console"), 1024);

	DPRINTF("============== XBotRTControlClass is ready... ==========\n\n\n\n");
	
	///for admittance control
        body_thetax.setZero();
	det_hip_posotion.setZero(); det_hip_pose.setZero();
	det_foot_rpy_lr.setZero();
	det_footz_lr.setZero();
	
	ZMPxy_realx.setZero();
	thetaxyx.setZero();
	comxyzx.setZero();
	Lfootxyzx.setZero();
	Rfootxyzx.setZero();
	M_L.setZero();
	M_R.setZero(); 
	F_L.setZero();
	F_R.setZero();
	
	F_L(2) = F_R(2)= 9.8/2*RobotPara().totalmass;
	
        j_count =0;
	bjx1 = 0;
	tx = 0;
	td=0;

        ankle_Ori_left.setZero();
	ankle_Ori_right.setZero();
	det_ank_foot.setZero();
        
	LeftFootPosx.setZero();
	LeftFootPosx(1) = RobotParaClass::HALF_HIP_WIDTH();
	RightFootPosx.setZero();
	RightFootPosx(1) = -RobotParaClass::HALF_HIP_WIDTH();	

        t_compute =0;	
	
	
}

XBotRTControlClass::~XBotRTControlClass()
{
	savedata();
	delete console;
	DPRINTF("========= XBotRTControlClass destroyed. =============\n");
}


void XBotRTControlClass::initOpenSoTIK(std::string path_to_config_file)
{
#ifdef USE_OPENSOT
	ik_sot.reset(new IKClass_OpenSoT(path_to_config_file));
#endif
}

void XBotRTControlClass::Init(const double &dT)
{
	IsInit = true;
	DPRINTF("Gait parameters initialization finished.\n");
}

void XBotRTControlClass::HomingInit(const std::vector<double> &homing_angles)
{
	const RobotStateClass& irobot = _WBS.getRobotState();

	irobot._model->Update(homing_angles);
	RBDL::Math::SpatialTransform LHandFrame = irobot._model->lwrist;
	RBDL::Math::SpatialTransform RHandFrame = irobot._model->rwrist;
	RBDL::Math::SpatialTransform LFootFrame = irobot._model->lsole;
	RBDL::Math::SpatialTransform RFootFrame = irobot._model->rsole;

	LEFT_FOOT_HOME_POS = LFootFrame.r;
	RIGHT_FOOT_HOME_POS = RFootFrame.r;
	LEFT_FOOT_HOME_ORI = LFootFrame.E;
	RIGHT_FOOT_HOME_ORI = RFootFrame.E;

	LEFT_HAND_HOME_POS = LHandFrame.r;
	RIGHT_HAND_HOME_POS = RHandFrame.r;
	LEFT_HAND_HOME_ORI = LHandFrame.E;
	RIGHT_HAND_HOME_ORI = RHandFrame.E;

	for (int i = 0; i < JointNUM; i++) {
		irobot.SendToRobot->q_ref[i] = irobot.SendToRobot->q[i] = homing_angles[i];
	}
	srdf_homing_angles = homing_angles;
}

void XBotRTControlClass::MoveToInitialPosition()
{
	const RobotStateClass& irobot = _WBS.getRobotState();

	double start_time = RobotPara().setParaForSimOrReal(1.0, 3.0);
// 	std::cout<< "start_time:"<<start_time<<std::endl;

	if (realtime == 0.0) {
		DPRINTF("\n======== Start moving to initial postion ===============\n");
	}

	if (realtime <= start_time) {
		double scale = 0.5 * (1 - std::cos(M_PI * realtime / start_time));

		// for analytic IK, PelvisPos[2] is the leg's first joint height, for OpenSOT, PelvisPos[2] is the actual waist height
		PelvisPos[2] = RobotParaClass::FULL_LEG() - RobotParaClass::HOMING_LIFT_HEIGHT() * scale;
#ifdef USE_OPENSOT
		PelvisPos[2] = RobotParaClass::WAIST_HEIGHT() - (RobotParaClass::HOMING_LIFT_HEIGHT()) * scale;
#endif
		initHipx = 0.025*scale;

		Eigen::Vector3d vWorldToWaist(0.0, 0.0, RobotParaClass::WAIST_HEIGHT());
		// Eigen::Vector3d vWorldToWaist = irobot._model->waist_pos.r;
		if (RobotPara().name == "walkman") {
			vWorldToWaist[0] = -0.051; // for new walkman upper body only, because the urdf of the waist was changed
		}
		// Eigen::Vector3d vWorldToWaist = vFullLeg;
		Eigen::Vector3d LftHomingPosGlobal = vWorldToWaist + LEFT_FOOT_HOME_POS; // actual foot position in global frame
		Eigen::Vector3d RftHomingPosGlobal = vWorldToWaist + RIGHT_FOOT_HOME_POS;
		Eigen::Vector3d LFootHomingDistance = Eigen::Vector3d(0.0, RobotParaClass::HALF_FOOT_DIS(), 0.0) - LftHomingPosGlobal; // difference with ref foot position
		Eigen::Vector3d RFootHomingDistance = Eigen::Vector3d(0.0, -RobotParaClass::HALF_FOOT_DIS(), 0.0) - RftHomingPosGlobal;

		// Eigen::Vector3d LftHomingPosGlobal = Eigen::Vector3d(0, 0, RobotParaClass::FULL_LEG()) + LEFT_FOOT_HOME_POS; // actual ankle position in global frame
		// Eigen::Vector3d RftHomingPosGlobal = Eigen::Vector3d(0, 0, RobotParaClass::FULL_LEG()) + RIGHT_FOOT_HOME_POS;
		// Eigen::Vector3d LFootHomingDistance = Eigen::Vector3d(0.0, RobotParaClass::HALF_FOOT_DIS(), RobotParaClass::ANKLE_HEIGHT()) - LftHomingPosGlobal; // difference with ref ankle position
		// Eigen::Vector3d RFootHomingDistance = Eigen::Vector3d(0.0, -RobotParaClass::HALF_FOOT_DIS(), RobotParaClass::ANKLE_HEIGHT()) - RftHomingPosGlobal;

		LeftFootPos = LftHomingPosGlobal + scale * LFootHomingDistance;
		RightFootPos = RftHomingPosGlobal + scale * RFootHomingDistance;
		// LeftFootPos[2] -= RobotParaClass::ANKLE_HEIGHT();
		// RightFootPos[2] -= RobotParaClass::ANKLE_HEIGHT();
		// LeftFootO = ZMPwalk.SwingFoot->SwingFootRGen(0.0, start_time, realtime, LEFT_FOOT_HOME_ORI, Eigen::Matrix3d::Identity());
		// RightFootO = ZMPwalk.SwingFoot->SwingFootRGen(0.0, start_time, realtime, RIGHT_FOOT_HOME_ORI, Eigen::Matrix3d::Identity());

		Eigen::Vector3d LHandHomingDistance(0.0, 0.0, 0.0);
		Eigen::Vector3d RHandHomingDistance(-0.0, -0.0, -0.0);
		// Eigen::Vector3d LHandHomingDistance(0.10, 0.0, 0.15);
		// Eigen::Vector3d RHandHomingDistance(0.10, -0.0, 0.15);
		LeftHandPos = LEFT_HAND_HOME_POS + scale * LHandHomingDistance;
		RightHandPos = RIGHT_HAND_HOME_POS + scale * RHandHomingDistance;

		// LeftHandPosGlobal = PelvisPos + LeftHandPos;
		// RightHandPosGlobal = PelvisPos + RightHandPos;

		// Eigen::Matrix3d Lhd_end = LEFT_HAND_HOME_ORI;
		Eigen::Matrix3d Lhd_end = Ry(-M_PI * 0.5);
		// LeftHandO = ZMPwalk.SwingFoot->SwingFootRGen(0.0, start_time, realtime, LEFT_HAND_HOME_ORI, Lhd_end);
		// LeftHandOGlobal = WaistO * LeftHandO;

		// Eigen::Matrix3d Rhd_end = RIGHT_HAND_HOME_ORI;
		Eigen::Matrix3d Rhd_end = Ry(-M_PI * 0.5);
		// RightHandO = ZMPwalk.SwingFoot->SwingFootRGen(0.0, start_time, realtime, RIGHT_HAND_HOME_ORI, Rhd_end);
		// RightHandOGlobal = WaistO * RightHandO;

		// COUT("\nRIGHT_HAND_HOME_ORI:\n", RIGHT_HAND_HOME_ORI);
		// COUT("RIGHT_HAND_HOME_POS:", RIGHT_HAND_HOME_POS.transpose());

		Eigen::Vector3d com_ref_eigen = PelvisPos;
		Eigen::Vector3d lft_ref_eigen = LeftFootPos;
		Eigen::Vector3d rft_ref_eigen = RightFootPos;
		Eigen::Vector3d zmp_ref_eigen(0.0, 0.0, 0.0);
		Eigen::Vector3d fz_ref_eigen(0.5, 0.5, 0);

		_WBS.UpdateRef(com_ref_eigen, lft_ref_eigen, rft_ref_eigen, zmp_ref_eigen, fz_ref_eigen);

		for (int i = 3; i < 15; i++) {
			_WBS.q_off[i] = DEGTORAD(RobotPara().offset_pos[i]) * scale;
// 			DPRINTF("%.2f\t",RobotPara().offset_pos[i]);
		}
// 		DPRINTF("%.2f\t",RobotPara().offset_pos[LEFT_FOOT_PITCH]);
		for (int i = 15; i < srdf_homing_angles.size(); i++) {
			irobot.SendToRobot->q_ref[i] = irobot.SendToRobot->q[i] = srdf_homing_angles[i] + (DEGTORAD(RobotPara().HOMING_POS()[i]) - srdf_homing_angles[i]) * scale;
		}
// 		DPRINTF("\n");

		if (realtime == start_time) {
			DPRINTF("Finish moving to initial postion.\n");
			// DPRINTF("CoM height is %.4f m.\n\n", irobot._model->com_ft[2]);
			DPRINTF("CoM is at %.4f\t%.4f\t%.4f m.\n\n", irobot._model->com_ft[0], irobot._model->com_ft[1], irobot._model->com_ft[2]);
			DPRINTF("Pelvis is at %.4f\t%.4f\t%.4f m.\n\n", PelvisPos[0], PelvisPos[1], PelvisPos[2]);
			DPRINTF("\n================================================\n");
			DPRINTF("Normal Keyboard Control is running, Press\n");
			DPRINTF("v:     Switch to AutoWalk Keyboard.\n");
			DPRINTF("x/y/z: Enable stabilizer in x/y/z, respectively.\n");
			DPRINTF("d:     Disable stabilizer in x, y and z.\n");
			DPRINTF("c:     Start/Restart Walking.\n");
			DPRINTF("s:     Stop Walking.\n");
			DPRINTF("F:     Enable/Disable Fixed Walking.\n");
			DPRINTF("B:     Enable/Disable Stop wiht parallel feet.\n");
			DPRINTF("e:     Enable standing reactive stepping.\n");
			DPRINTF("================================================\n");
			// KeyBoardControl('x');
			// KeyBoardControl('y');
#ifdef USE_OPENSOT
			Eigen::VectorXd q_postural = ik_sot->getSolution(); // get the size
			JointRefToXBot(q_postural); // get the last solution
			ik_sot->setPosturalReference(q_postural); // set homing postural
#endif
		}
	}
	else {
		if (!IsInit) {
			Init(dt);
		}
	}
}


void XBotRTControlClass::UpdateWBS(const double & dT, const std::vector<double> &qall, const Eigen::Vector3d &EulerAng, const Eigen::Vector3d &Acc, const Eigen::Vector3d &AngleRate, const std::vector<double> &FTSensor, const std::vector<double> &HandFTSensor)
{
	//***************** implementation of WBS class *****************

	_WBS.setFilterPara(20, 4);
	Eigen::Matrix3d Rpelvis_abs = Rz(EulerAng[2]) * Ry(EulerAng[1]) * Rx(EulerAng[0]);
	_WBS.UpdateIMU(dT, Rpelvis_abs, Acc, AngleRate, EulerAng);
	_WBS.UpdateRobotState(qall, dT, FTSensor);
// 	_WBS.UpdateHandFT(HandFTSensor);
	//***************************************************************
///	DPRINTF("using updatawbs===== hand=====\n");
}

//////// using this one
void XBotRTControlClass::UpdateWBS(const std::vector<double> &qall, const Eigen::Matrix3d &Rpelvis_abs, const Eigen::Vector3d &Acc, const Eigen::Vector3d &AngleRate, const std::vector<double> &FTSensor, const std::vector<double> &HandFTSensor)
{
	//************* implementation of WBS class ********************
	_WBS.setFilterPara(20, 4);
	_WBS.UpdateIMU(RobotParaClass::dT(), Rpelvis_abs, Acc, AngleRate);
	//DPRINTF("using UpdateIMU======no hand=====\n");
	_WBS.UpdateRobotState(qall, RobotParaClass::dT(), FTSensor);
// 	_WBS.UpdateHandFT(HandFTSensor);
	//**************************************************************

    ///DPRINTF("using UpdateRobotState======no hand=====\n");
}

void XBotRTControlClass::Run(const double &dT, std::vector<double> &pos, const std::vector<float> &OffsetAng)
{
	this->Run();
}

void XBotRTControlClass::Run()
{
  
  
    clock_t t_start,t_finish;
    
	  t_start = clock();    
  //// run: logic: firstly: move to initial postion: upper arm put down. after 3s, keep the initial position;
  ////// then run ./xddp_console, to keyboard input the c/s to start or stop the real-time loop
	const RobotStateClass& irobot = _WBS.getRobotState();

	char cmd;
	user_input(cmd);

	realtime = dtime * dt;
	MoveToInitialPosition();
        
	if (IsInit && IsStartWalk) {
		if (walkdtime == 0) {
			walkstarttime = dt * dtime;
		}
		double UpdateTcycle = 1 * dt;
		double time = realtime - walkstarttime;
		if (IsFixed) {
		  DPRINTF("========= IsFixed. =============\n");
			
		}
		else {		  
			WalkingReactStepping();
			
		}

		COMTrajGen();
		walkdtime++;
		
	}


	if (IsInit) {
// 	  Admittance_controller();
 //	  std::cout<<"admittance controller"<<std::endl;
	}

	SolveIK();

#ifdef USE_XBOT_LOGGER
	addToLog();
#endif
	dtime++;
	
//  	std::cout<<"Isinit:"<< IsInit<<std::endl; 
// 	std::cout<<"IsStartWalk:"<< IsStartWalk<<std::endl;
// 	std::cout<<"IsFixed:"<< IsFixed<<std::endl;

	  t_finish = clock();   
	  
	  t_compute = (double)(t_finish - t_start)/CLOCKS_PER_SEC ;	  
    
	
}

static int jnt_idx = 12;
void XBotRTControlClass::KeyBoardControl(char cmd)
{
        //StartWalking();
	switch (cmd)
	{
	case 'c':
	{
		DPRINTF("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! start  Walking!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");	  
		StartWalking();
	}
	break;

	case 's':
	{
		DPRINTF("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  Stop Walking !!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

		StopWalking();
	}
	break;

	case 'e':
	{
		EnableStandingReact();
	}
	break;

	case 'E':
	{
		EnableWalkingReact();
	}
	break;

	case 'v':
	{
		KeyBoardSwitch *= -1;
		if (KeyBoardSwitch == 1) {
			DPRINTF("\n================================================\n");
			DPRINTF("Switched to Normal RTControl Control.\n");
			DPRINTF("v:     Switch to AutoWalk Keyboard.\n");
			DPRINTF("x/y/z: Enable stabilizer in x/y/z, respectively.\n");
			DPRINTF("d:     Disable stabilizer in x, y and z.\n");
			DPRINTF("c:     Start/Restart Walking.\n");
			DPRINTF("s:     Stop Walking.\n");
			DPRINTF("F:     Enable/Disable Fixed Walking.\n");
			DPRINTF("B:     Enable/Disable Stop wiht parallel feet.\n");
			DPRINTF("e:     Enable standing reactive stepping.\n");
			DPRINTF("================================================\n");
		}
		else if (KeyBoardSwitch == -1) {
			DPRINTF("\n================================================\n");
			DPRINTF("Switched to RTControl Keyboard Control.\n");
			DPRINTF("v:     Switch to Normal Keyboard.\n");
			DPRINTF("o/p:   Inc./Dec. Step Length.\n");
			DPRINTF("k/l:   Inc./Dec. Step Width.\n");
			DPRINTF("n/m:   Turn Left/Right.\n");
			DPRINTF("O/P:   Inc./Dec. Walking Velocity.\n");
			DPRINTF("U/I:   Inc./Dec. Step Time.\n");
			DPRINTF("================================================\n");
		}
		else {
			DPRINTF("Error! No Such Keyboard Control.\n");
		}
	}
	break;

	default:
		break;
	}

	if (KeyBoardSwitch == -1) {
		RTControlKeyBoardControl(cmd);
	}
	else {
		double interval = 0.1;
		switch (cmd)
		{

		case 'x':
		{
			StaEnableX = true;
			DPRINTF("Enable Stabilizer X!\n");
		}
		break;

		case 'y':
		{
			StaEnableY = true;
			DPRINTF("Enable Stabilizer Y!\n");
		}
		break;

		case 'z':
		{
			StaEnableZ = true;
			DPRINTF("Enable Stabilizer Z!\n");
		}
		break;

		case 'd':
		{
			StaEnableX = false;
			StaEnableY = false;
			StaEnableZ = false;
			DPRINTF("Disable Stabilizer X Y Z!\n");
		}
		break;

		case 'X':
		{
			HandStaEnableX = true;
			DPRINTF("Enable Hand Stabilizer X!\n");
		}
		break;

		case 'Y':
		{
			HandStaEnableY = true;
			DPRINTF("Enable Hand Stabilizer Y!\n");
		}
		break;

		case 'Z':
		{
			HandStaEnableZ = true;
			DPRINTF("Enable Hand Stabilizer Z!\n");
		}
		break;

		case 'D':
		{
			HandStaEnableX = false;
			HandStaEnableY = false;
			HandStaEnableZ = false;
			DPRINTF("Disable Hand Stabilizer X Y Z!\n");
		}
		break;

		case 'j': {
			Is_testJoint = true;
			En_testJoint = true;
			DPRINTF("Start Joint Test.\n");
		}
		break;

		case 'h': {
			En_testJoint = false;
			DPRINTF("Disable Joint Test.\n");
		}
		break;

		case 'H': {
			// ZMPwalk.IsClearFixedFootHolds(true);
			IsMoveHand = true;
			En_testJoint = true;
			if (!IsSetHandRefToWorld) {
				hand_startT = realtime;
				hand_startT_d = dtime;
				LeftHandPosGlobal_start = LeftHandPosGlobal;
				RightHandPosGlobal_start = RightHandPosGlobal;
				LeftHandOGlobal_start = LeftHandOGlobal;
				RightHandOGlobal_start = RightHandOGlobal;
				LeftHandPos_start = LeftHandPos;
				RightHandPos_start = RightHandPos;
				LeftHandO_start = LeftHandO;
				RightHandO_start = RightHandO;
				DPRINTF("\n======== Start moving hands ===============\n");
			}
			IsSetHandRefToWorld = true;
		}
		break;

		case 'i':
		{
			RobotPara().kd_debug += interval;
			DPRINTF("Increase kd_debug to %.2f\n", RobotPara().kd_debug);
		}
		break;

		case 'o':
		{
			RobotPara().kd_debug -= interval;
			DPRINTF("Decrease kd_debug to %.2f\n", RobotPara().kd_debug);
		}
		break;

		default:
			break;
		}
	}
}

void XBotRTControlClass::RTControlKeyBoardControl(char cmd)
{
	double delta_sl = 0.002;
	double delta_sw = 0.002;
	double delta_ta = 2.5;
	double maxVel = 0.2;
	double minVel = -maxVel;

	switch (cmd)
	{
	case 'z':
	{
		EnableFtPos = true;
		DPRINTF("Enable Foot Position Control!\n");
	}
	break;

	case 'Z':
	{
		EnableFtOri = true;
		DPRINTF("Enable Foot Orientation Control!\n");
	}
	break;

	case 'd':
	{
		EnableFtPos = false;
		EnableFtOri = false;
		DPRINTF("Disable Landing Foot Control!\n");
	}
	break;

	case 'o':
	{
		steplength += delta_sl;
		DPRINTF("Increase Step Length to %.3f m!\n", steplength);
	}
	break;

	case 'p':
	{
		steplength -= delta_sl;
		DPRINTF("Decrease Step Length to %.3f m!\n", steplength);
	}
	break;

	case 'k':
	{
		stepwidth += delta_sw;
		DPRINTF("Increase Step Width to %.3f m!\n", stepwidth);
	}
	break;

	case 'l':
	{
		stepwidth -= delta_sw;
		DPRINTF("Decrease Step Width to %.3f m!\n", stepwidth);
	}
	break;

	case 'n':
	{
		turnangle += delta_ta;
		DPRINTF("Change Heading Angle to %.3f degree!\n", turnangle);
	}
	break;

	case 'm':
	{
		turnangle -= delta_ta;
		DPRINTF("Change Heading Angle to %.3f degree!\n", turnangle);
	}
	break;

	case 'g':
	{
		initHipx += 0.001;
		DPRINTF("Increase deltaHip X to %.3f m!\n", initHipx);
	}
	break;

	case 'h':
	{
		initHipx -= 0.001;
		DPRINTF("Decrease deltaHip X to %.3f m!\n", initHipx);
	}
	break;

	case 'G':
	{
		initHipy += 0.001;
		DPRINTF("Increase deltaHip Y to %.3f m!\n", initHipy);
	}
	break;

	case 'H':
	{
		initHipy -= 0.001;
		DPRINTF("Decrease deltaHip Y to %.3f m!\n", initHipy);
	}
	break;

	case 'u':
	{
		Ksway += 0.01;
		DPRINTF("Increase Ksway to %.3f\n", Ksway);
	}
	break;

	case 'i':
	{
		Ksway -= 0.01;
		DPRINTF("Decrease Ksway to %.3f\n", Ksway);
	}
	break;

	case 'N':
	{
		stepheight += 0.0005;
		clamp(stepheight, 0.0, RobotParaClass::HOMING_LIFT_HEIGHT());
		// clamp(stepheight, -RobotParaClass::HOMING_LIFT_HEIGHT(), RobotParaClass::HOMING_LIFT_HEIGHT());
		DPRINTF("Change Step Height to %.4f m!\n", stepheight);
	}
	break;

	case 'M':
	{
		stepheight -= 0.0005;
		clamp(stepheight, 0.0, RobotParaClass::HOMING_LIFT_HEIGHT());
		// clamp(stepheight, -RobotParaClass::HOMING_LIFT_HEIGHT(), RobotParaClass::HOMING_LIFT_HEIGHT());
		DPRINTF("Change Step Height to %.4f m!\n", stepheight);
	}
	break;

	default:
		break;
	}
}

void XBotRTControlClass::COMTrajGen()
{

	// TurnYaw = std::atan2(HipO_Turn(1, 0), HipO_Turn(0, 0));

	// // leave it like this temporarily
	// Eigen::Matrix3d TurnYawO = Rz(TurnYaw);
	// Eigen::Vector3d Local0 = 0.5 * (LeftFootPos + RightFootPos);
	// Local0[2] = 0;
	// Eigen::Vector3d Local1 = TurnYawO.transpose() * (hip_pos_eigen - Local0);
	// Local1[1] *= Ksway;
	// hip_pos_eigen = TurnYawO * Local1 + Local0;
	// PelvisPos << hip_pos_eigen[0], hip_pos_eigen[1], zh + deltaCOMz;


        double kappx=1;
// 	double t_xx =0;
// 	
// /*	if (walkdtime*dt>=6.5)
// 	{
// 	  t_xx = (walkdtime -(int)6.5/dt)*0.000008;
// 	  
// 	  kappx += t_xx;
// 	  if (kappx>=0.99)
// 	  {
// 	    kappx=0.99;
// 	  }	  
// 	}*/	
// 	
// 	if (walkdtime*dt>=14)
// 	{
// 	  t_xx = (walkdtime -(int)14/dt)*0.0001;
// 	  
// 	  kappx -= t_xx;
// 	  if (kappx<=0.96)
// 	  {
// 	    kappx=0.96;
// 	  }	  
// 	}


// 	PelvisPos[1] *= Ksway;
  	PelvisPos[1] *= 1;
	
	PelvisPos[0] *= kappx;
}

void XBotRTControlClass::savedata()
{
	if (!IsLoggerSaved) {
#ifdef USE_XBOT_LOGGER
		DPRINTF("=========        Logging Data .....      =============\n");
		xbot_logger->flush();
#endif
		IsLoggerSaved = true;

		DPRINTF("=========     Finish Logging Data .....  =============\n");
	}
}






void XBotRTControlClass::UpdateWbsRef()
{
	Eigen::Matrix3d TurnYawO = Rz(TurnYaw);
	for (unsigned int i = 0; i < 3; i++) {
		lft_ref[i] = LeftFootPos[i];
		rft_ref[i] = RightFootPos[i];
	}
	com_ref[0] = PelvisPos[0];
	com_ref[1] = PelvisPos[1];
	com_ref[2] = PelvisPos[2];

	const RobotStateClass& irobot = _WBS.getRobotState();
	irobot.SendToRobot->gLftRef = LeftFootPos;
	irobot.SendToRobot->gRftRef = RightFootPos;
	irobot.SendToRobot->gComRef = PelvisPos;
	irobot.SendToRobot->gZmpRef[0] = zmp_ref[0];
	irobot.SendToRobot->gZmpRef[1] = zmp_ref[1];

	Eigen::Vector3d com_ref_eigen(0, 0, 0), lft_ref_eigen(0, 0, 0), rft_ref_eigen(0, 0, 0), zmp_ref_eigen(0, 0, 0), fz_ref_eigen(0, 0, 0);
	for (unsigned int i = 0; i < 2; i++) {
		com_ref_eigen[i] = com_ref[i];
		lft_ref_eigen[i] = lft_ref[i];
		rft_ref_eigen[i] = rft_ref[i];
		zmp_ref_eigen[i] = zmp_ref[i];
		fz_ref_eigen[i] = Fz_ref[i];
	}
	com_ref_eigen = TurnYawO.transpose() * com_ref_eigen;
	lft_ref_eigen = TurnYawO.transpose() * lft_ref_eigen;
	rft_ref_eigen = TurnYawO.transpose() * rft_ref_eigen;
	zmp_ref_eigen = TurnYawO.transpose() * zmp_ref_eigen;
	for (unsigned int i = 2; i < 3; i++) {
		com_ref_eigen[i] = com_ref[i];
		lft_ref_eigen[i] = lft_ref[i];
		rft_ref_eigen[i] = rft_ref[i];
		zmp_ref_eigen[i] = zmp_ref[i];
	}

	double delaytime = RobotPara().setParaForSimOrReal(0.0, 0.0);

	_WBS.UpdateRef(com_ref_eigen, lft_ref_eigen, rft_ref_eigen, zmp_ref_eigen, fz_ref_eigen);

}



void XBotRTControlClass::Admittance_controller()
{
   const RobotStateClass& irobot = _WBS.getRobotState();
   UpdateWbsRef();
  
 det_hip_posotion = Sta.COMdampingCtrl(bjx1,Lfootxyzx,Rfootxyzx,zmp_ref,irobot);
   det_hip_pose = Sta.COMangleCtrl(bjx1,thetaxyx,comxyzx,Lfootxyzx,Rfootxyzx,irobot);
  det_foot_rpy_lr = Sta.FootdampiingCtrol_LR(bjx1, j_count, tx, td, M_L, M_R,irobot,IsStartWalk);
 det_footz_lr = Sta.ForcediffCtrol_LR(bjx1, F_L,F_R,irobot,IsStartWalk);
  
  
  HipO_Turn = Rz(body_thetax[2]+det_hip_pose[2])*Ry(body_thetax[1]+det_hip_pose[1])*Rx(body_thetax[0]+det_hip_pose[0]);
  
  deltaHip = det_hip_posotion;
  
  
  /// deltaFtOri_left, deltaFtOri_right are the matrix rotation of the local framework
  deltaFtOri_left = Rz(det_hip_pose(2))*Ry(det_hip_pose(1))*Rx(det_hip_pose(0));
  
  deltaFtOri_right = Rz(det_hip_pose(5))*Ry(det_hip_pose(4))*Rx(det_hip_pose(3));
  

  
}







void XBotRTControlClass::SolveIK()
{

	const RobotStateClass& irobot = _WBS.getRobotState();
	
        
	
// 	if (RobotPara().name == "coman") {
// 	  std::vector<double> qL(6, 0);
// 	  std::vector<double> qR(6, 0);
// 	}
// 	else
// 	{
// 	  Eigen::Vector6d qL, qR;	  
// 	}
	
	
	
//  	Eigen::Vector6d qL, qR;

	Eigen::Matrix3d TurnYawO = Rz(TurnYaw);
	Eigen::Vector3d modCopHip(initHipx, initHipy, 0);
	modCopHip = TurnYawO * modCopHip;

	HipPos.setZero();
	HipPos[0] += modCopHip[0];
	HipPos[1] += modCopHip[1];

	HipPos += PelvisPos;
	HipPos += deltaHip;
	HipPos += WaistComp;
	
	LeftFootPos.setZero();
	LeftFootPos += LeftFootPosx;
	LeftFootPos(2) += det_footz_lr(0);
	
	RightFootPos.setZero();
	RightFootPos += RightFootPosx;
	RightFootPos(2) += det_footz_lr(1);			
	
	

	// DPRINTF("deltaHip x: %.3f,\ty: %.3f,\tz: %.3f\n", deltaHip[0], deltaHip[1], deltaHip[2]);


	Eigen::Matrix3d HipO_Left_Ref = HipO_Left_comp;
	Eigen::Matrix3d HipO_Right_Ref = HipO_Right_comp;

	double yaw = 0.0;
	HipO_Left = HipO_Turn * HipO_Left_Ref;
	HipO_Right = HipO_Turn * HipO_Right_Ref;
	WaistO = HipO_Turn;

	Eigen::Vector3d LeftAnklePos = LeftFootPos;
	LeftAnklePos += deltaFtPos_l;
	LeftAnklePos[2] += deltaFtZ[0] + RobotParaClass::ANKLE_HEIGHT(); // probably has potential risk that into considering the orientation of the foot
	Eigen::Vector3d RightAnklePos = RightFootPos;
	RightAnklePos += deltaFtPos_r;
	RightAnklePos[2] += deltaFtZ[1] + RobotParaClass::ANKLE_HEIGHT();

	Eigen::Vector3d FinalLeftHandPos = LeftHandPos;
	Eigen::Vector3d FinalRightHandPos = RightHandPos;

	Eigen::Matrix3d FinalLeftHandOri  = LeftHandO;
	Eigen::Matrix3d FinalRightHandOri = RightHandO;

	irobot.SendToRobot->HipPos = HipPos;
	irobot.SendToRobot->LeftAnklePos = LeftAnklePos;
	irobot.SendToRobot->RightAnklePos = RightAnklePos;
	irobot.SendToRobot->HipO_Left = HipO_Left;
	irobot.SendToRobot->HipO_Right = HipO_Right;
	irobot.SendToRobot->LeftFootO = LeftFootO;
	irobot.SendToRobot->RightFootO = RightFootO;

	Eigen::Vector3d FinalLeftFootPos = LeftFootPos;
	FinalLeftFootPos[2] += deltaFtPos_l[2];
	Eigen::Vector3d FinalRightFootPos = RightFootPos;
	FinalRightFootPos[2] += deltaFtPos_r[2];

	Eigen::Matrix3d FinalLeftFootOri = LeftFootO * deltaFtOri_left;
	Eigen::Matrix3d FinalRightFootOri = RightFootO * deltaFtOri_right;

	if (RobotPara().name == "walkman") {
	        Eigen::Vector6d qL, qR;
		IK.BigmanLegIK(HipPos, HipO_Left, LeftAnklePos, FinalLeftFootOri, "L", qL, RobotPara().name);
		IK.BigmanLegIK(HipPos, HipO_Right, RightAnklePos, FinalRightFootOri, "R", qR, RobotPara().name);
		irobot.SendToRobot->q[RIGHT_HIP_PITCH ] = qR[2];
		irobot.SendToRobot->q[RIGHT_HIP_ROLL  ] = qR[0];
		irobot.SendToRobot->q[RIGHT_HIP_YAW   ] = qR[1];
		irobot.SendToRobot->q[RIGHT_KNEE_PITCH] = qR[3];
		irobot.SendToRobot->q[RIGHT_FOOT_ROLL ] = qR[5];
		irobot.SendToRobot->q[RIGHT_FOOT_PITCH] = qR[4];
		irobot.SendToRobot->q[LEFT_HIP_PITCH  ] = qL[2];
		irobot.SendToRobot->q[LEFT_HIP_ROLL   ] = qL[0];
		irobot.SendToRobot->q[LEFT_HIP_YAW    ] = qL[1];
		irobot.SendToRobot->q[LEFT_KNEE_PITCH ] = qL[3];
		irobot.SendToRobot->q[LEFT_FOOT_ROLL  ] = qL[5];
		irobot.SendToRobot->q[LEFT_FOOT_PITCH ] = qL[4];
		irobot.SendToRobot->q[WAIST_YAW ] = -yaw;
	}
	else if (RobotPara().name == "cogimon") {
	  Eigen::Vector6d qL, qR;
	  //DPRINTF("========= Cogimon COgimon Cogimon COgimon=============\n");
#ifdef USE_OPENSOT
		if (!IsSetHandRefToWorld)  {
			LeftHandPosGlobal = HipPos + WaistO * LeftHandPos;
			RightHandPosGlobal = HipPos + WaistO * RightHandPos;
			LeftHandOGlobal = WaistO * LeftHandO;
			RightHandOGlobal = WaistO * RightHandO;
			ik_sot->setCurrentPosturalReference();
			
		}
		else {
			// MoveHands();
		  DPRINTF("========= IsSetHandRefToWorld=============\n");
		}
		FinalLeftHandPos  = LeftHandPosGlobal;
		FinalRightHandPos = RightHandPosGlobal;
		FinalLeftHandOri  = LeftHandOGlobal;
		FinalRightHandOri = RightHandOGlobal;
                

		ik_sot->setLowerBodyReference(HipPos, WaistO, FinalLeftFootPos, FinalLeftFootOri, FinalRightFootPos, FinalRightFootOri);
		// ik_sot->setHandLocalReference(LeftHandPos, LeftHandO, RightHandPos, RightHandO);
		// ik_sot->setHandGlobalReference(FinalLeftHandPos, FinalLeftHandOri, FinalRightHandPos, FinalRightHandOri);

 		ik_sot->SolveIK(irobot._model->q_all_floating);
		SortFromXBotToJointName(ik_sot->getSolution(), irobot.SendToRobot->q);
#endif
	}
	else if (RobotPara().name == "coman") {
	        std::vector<double> qL(6, 0);
	        std::vector<double> qR(6, 0);
		IK.LegInverseKinematics(HipPos, HipO_Left, LeftAnklePos, FinalLeftFootOri, irobot.hipwidth, qL);
		IK.LegInverseKinematics(HipPos, HipO_Right, RightAnklePos, FinalRightFootOri, -irobot.hipwidth , qR);
		irobot.SendToRobot->q[RIGHT_HIP_PITCH ] = qR[0]; //right hip pitch
		irobot.SendToRobot->q[RIGHT_HIP_ROLL  ] = qR[1]; //right hip roll
		irobot.SendToRobot->q[RIGHT_HIP_YAW   ] = qR[2]; //right hip yaw
		irobot.SendToRobot->q[RIGHT_KNEE_PITCH] = qR[3]; //right knee
		irobot.SendToRobot->q[RIGHT_FOOT_ROLL ] = qR[4]; //right ankle pitch
		irobot.SendToRobot->q[RIGHT_FOOT_PITCH] = qR[5]; //right ankle roll
		irobot.SendToRobot->q[LEFT_HIP_PITCH  ] = qL[0]; //left hip pitch
		irobot.SendToRobot->q[LEFT_HIP_ROLL   ] = qL[1]; //left hip roll
		irobot.SendToRobot->q[LEFT_HIP_YAW    ] = qL[2]; //left hip yaw
		irobot.SendToRobot->q[LEFT_KNEE_PITCH ] = qL[3]; //left knee
		irobot.SendToRobot->q[LEFT_FOOT_ROLL  ] = qL[4]; //left ankle pitch
		irobot.SendToRobot->q[LEFT_FOOT_PITCH ] = qL[5]; //left ankle roll
		irobot.SendToRobot->q[WAIST_YAW ] = -yaw;
	}	

	double ki = 0.0;
	// for (int i = 3; i < 15; i++) {
	for (int i = 0; i < JointNUM; i++) {
		e_i[i] += irobot.q_all[i] - irobot.SendToRobot->q[i];
		irobot.SendToRobot->q_ref[i] = irobot.SendToRobot->q[i] + _WBS.q_off[i] + (ki * e_i[i]) * dt;
		// DPRINTF("%.4f,\t", RADTODEG(irobot.SendToRobot->q_ref[i]));
	}
}

void XBotRTControlClass::SortFromXBotToJointName(const Eigen::VectorXd &from, std::vector<double> &to)
{
// 	assert(from.size() == JointNUM);
// 	assert(to.size() == JointNUM);

	const RobotStateClass& irobot = _WBS.getRobotState();

	q_rbdl.segment(n_RBDL_floating_jnt, from.size()) = from;
	irobot._model->vRBDLToJointName(q_rbdl, to);

	// for ( const auto& jnt : from ) {
	// 	to[RobotPara().getJointName(jnt.first)] = jnt.second;
	// 	std::cerr << RobotPara().getJointName(jnt.first) << ", Key:[" << jnt.first << "] Value:[" << jnt.second << "]\n";
	// }
}

void XBotRTControlClass::SortFromXBotToJointName(const Eigen::VectorXd &from, Eigen::VectorXd &to)
{
// 	assert(from.size() == JointNUM);
// 	assert(to.size() == JointNUM);

	const RobotStateClass& irobot = _WBS.getRobotState();

	q_rbdl.segment(n_RBDL_floating_jnt, JointNUM) = from;
	irobot._model->vRBDLToJointName(q_rbdl, to);
}

template <class T>
void XBotRTControlClass::SortFromJointNameToXBot(const T &from, std::unordered_map<std::string, double> &to)
{
// 	assert(from.size() == JointNUM);
// 	assert(to.size() == JointNUM);

	for ( const auto& jnt : to ) {
		jnt.second = from[RobotPara().getJointName(jnt.first)];
		// std::cerr << getJointName(jnt.first) << ", Key:[" << jnt.first << "] Value:[" << jnt.second << "]\n";
	}
}

void XBotRTControlClass::JointRefToXBot(Eigen::VectorXd &to)
{
// 	assert(to.size() == JointNUM);
	const RobotStateClass& irobot = _WBS.getRobotState();

	irobot._model->vJointNameToRBDL(irobot.SendToRobot->q_ref, q_tmp);
	to = q_tmp.segment(6, to.size());
}

void XBotRTControlClass::JointRefToXBot_LowerBody(Eigen::VectorXd &to)
{
// 	assert(to.size() == JointNUM);
	const RobotStateClass& irobot = _WBS.getRobotState();

	// COUT("SendToRobot->q:", irobot.SendToRobot->q.transpose());
	// COUT("to:", to.transpose());

	irobot._model->vJointNameToRBDL(irobot.SendToRobot->q_ref, q_tmp);
//         to.segment(0, 12) = q_tmp.segment(6, 12); //only update the lower body // rbdl 2.4.0, not tested in rbdl 2.5.0, but should be the same
	to.segment(0, 12) = q_tmp.segment(6, 12);
}


int XBotRTControlClass::user_input(void *buffer, ssize_t buff_size)
{
	int nbytes = console->read(buffer, buff_size);
	return nbytes;
}


int XBotRTControlClass::user_input(char &cmd)
{
	int nbytes = user_input((void*)&cmd, sizeof(cmd));
	if (nbytes <= 0) {
		return nbytes;
	}

	KeyBoardControl(cmd);

	return nbytes;
}

#ifdef USE_XBOT_LOGGER
void XBotRTControlClass::initLogger(int buffer_size, int interleave)
{
	xbot_logger->createScalarVariable("time", interleave, buffer_size);
	xbot_logger->createScalarVariable("WhichFoot", interleave, buffer_size);
	xbot_logger->createScalarVariable("WhichFootRef", interleave, buffer_size);
	xbot_logger->createScalarVariable("Fzl_ref", interleave, buffer_size);
	xbot_logger->createScalarVariable("Fzr_ref", interleave, buffer_size);
	xbot_logger->createScalarVariable("FallPredictor", interleave, buffer_size);
	xbot_logger->createScalarVariable("E_p", interleave, buffer_size);
	xbot_logger->createScalarVariable("E_k", interleave, buffer_size);
	xbot_logger->createScalarVariable("irobot_scale", interleave, buffer_size);

	xbot_logger->createVectorVariable("ComRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("LftRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("RftRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("ZmpRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gComRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gLftRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gRftRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gZmpRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gcom", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gdcom", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gddcom", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("glft", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("grft", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("cop", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gcop", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("glcop", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("grcop", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gCapturePoint", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("IMU_Euler", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("IMU_LinearVel_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("IMU_LinearAcc_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("IMU_AngularVel_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("IMU_AngularAcc_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("FT_foot_left", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("FT_foot_right", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("FT_fl_filter", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("FT_fr_filter", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("q_msr", RobotParaClass::JOINT_NUM(), interleave, buffer_size);
	xbot_logger->createVectorVariable("q_des", RobotParaClass::JOINT_NUM(), interleave, buffer_size);
	xbot_logger->createVectorVariable("q_ref", RobotParaClass::JOINT_NUM(), interleave, buffer_size);
	xbot_logger->createVectorVariable("dq_msr", RobotParaClass::JOINT_NUM(), interleave, buffer_size);
	xbot_logger->createVectorVariable("q_msrx",34, interleave, buffer_size);	
	xbot_logger->createVectorVariable("dq_msrx", 34, interleave, buffer_size);
	xbot_logger->createVectorVariable("ddq_msrx", 34, interleave, buffer_size);
	xbot_logger->createVectorVariable("tau_msrx", 34, interleave, buffer_size);	
	xbot_logger->createVectorVariable("tau_msr", RobotParaClass::JOINT_NUM(), interleave, buffer_size);	
	xbot_logger->createVectorVariable("L_com", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("LhdRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("RhdRef", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("deltaHip", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("deltaFtZ", 2, interleave, buffer_size);
	xbot_logger->createVectorVariable("deltaFtPos_l", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("deltaFtPos_r", 3, interleave, buffer_size);

	xbot_logger->createVectorVariable("gcom_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gdcom_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gddcom_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("glft_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("grft_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("cop_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("gcop_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("glcop_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("grcop_raw", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("lankle", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("rankle", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("Lft", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("Rft", 3, interleave, buffer_size);

	xbot_logger->createVectorVariable("des_lft_vel", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("des_rft_vel", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("des_lft_acc", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("des_rft_acc", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("des_lhd_vel", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("des_rhd_vel", 6, interleave, buffer_size);

	xbot_logger->createMatrixVariable("IMU_abs", 3, 3, interleave, buffer_size);

	xbot_logger->createVectorVariable("PelvisPos", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("LeftFootPos", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("RightFootPos", 3, interleave, buffer_size);

	xbot_logger->createVectorVariable("HipPos", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("LeftAnklePos", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("RightAnklePos", 3, interleave, buffer_size);

	xbot_logger->createMatrixVariable("HipO_Left", 3, 3, interleave, buffer_size);
	xbot_logger->createMatrixVariable("HipO_Right", 3, 3, interleave, buffer_size);
	xbot_logger->createMatrixVariable("LeftFootO", 3, 3, interleave, buffer_size);
	xbot_logger->createMatrixVariable("RightFootO", 3, 3, interleave, buffer_size);

	xbot_logger->createVectorVariable("stabilizer_torque", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("applyDeltaPosX", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("applyDeltaPosY", 3, interleave, buffer_size);

	xbot_logger->createScalarVariable("TurnYaw", interleave, buffer_size);
	xbot_logger->createScalarVariable("LftYaw", interleave, buffer_size);
	xbot_logger->createScalarVariable("RftYaw", interleave, buffer_size);
	
	xbot_logger->createVectorVariable("det_hip_posotion", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("det_hip_pose", 3, interleave, buffer_size);
	xbot_logger->createVectorVariable("det_foot_rpy_lr", 6, interleave, buffer_size);
	xbot_logger->createVectorVariable("det_footz_lr", 2, interleave, buffer_size);			
	xbot_logger->createVectorVariable("time_cost", 1, interleave, buffer_size);		

	Sta.initLogger(xbot_logger, logger_len);

	InternalLoggerInit();
}

void XBotRTControlClass::addToLog()
{
	const RobotStateClass& irobot = _WBS.getRobotState();

	xbot_logger->add("time", realtime);
	xbot_logger->add("WhichFoot", (irobot.WhichFoot));
	xbot_logger->add("WhichFootRef", (irobot.WhichFootRef));
	xbot_logger->add("Fzl_ref", (irobot.FzRef[0]));
	xbot_logger->add("Fzr_ref", (irobot.FzRef[1]));
	xbot_logger->add("FallPredictor", (irobot.fall_state));
	xbot_logger->add("E_p", irobot._model->Ep);
	xbot_logger->add("E_k", irobot._model->Ek);
	xbot_logger->add("irobot_scale", (irobot.scale));

	xbot_logger->add("ComRef", irobot.ComRef);
	xbot_logger->add("LftRef", irobot.LftRef);
	xbot_logger->add("RftRef", irobot.RftRef);
	xbot_logger->add("ZmpRef", irobot.ZmpRef);
	xbot_logger->add("gComRef", com_ref);
	xbot_logger->add("gLftRef", lft_ref);
	xbot_logger->add("gRftRef", rft_ref);
	xbot_logger->add("gZmpRef", zmp_ref);
	xbot_logger->add("gcom", irobot.gcom);
	xbot_logger->add("gdcom", irobot.gdcom);
	xbot_logger->add("gddcom", irobot.gddcom);
	xbot_logger->add("glft", irobot.glft);
	xbot_logger->add("grft", irobot.grft);
	xbot_logger->add("cop", irobot.cop);
	xbot_logger->add("gcop", irobot.gcop);
	xbot_logger->add("glcop", irobot.glcop);
	xbot_logger->add("grcop", irobot.grcop);
	xbot_logger->add("gCapturePoint", irobot.gCapturePoint);
	xbot_logger->add("IMU_Euler", irobot.IMU_Euler);
	xbot_logger->add("IMU_LinearVel_raw", irobot.IMU_LinearVel_raw);
	xbot_logger->add("IMU_LinearAcc_raw", irobot.IMU_LinearAcc_raw);
	xbot_logger->add("IMU_AngularVel_raw", irobot.IMU_AngularVel_raw);
	xbot_logger->add("IMU_AngularAcc_raw", irobot.IMU_AngularAcc_raw);
	xbot_logger->add("FT_foot_left", irobot.FT_foot_left);
	xbot_logger->add("FT_foot_right", irobot.FT_foot_right);
	xbot_logger->add("FT_fl_filter", irobot.FT_fl_filter);
	xbot_logger->add("FT_fr_filter", irobot.FT_fr_filter);
	xbot_logger->add("q_msr", irobot.q_all_mesure);
	xbot_logger->add("q_des", irobot.SendToRobot->q);
	xbot_logger->add("q_ref", irobot.SendToRobot->q_ref);
	xbot_logger->add("dq_msr", irobot.qdot_all_mesure);
	xbot_logger->add("q_msrx", irobot._model->q_all_floating);		
	xbot_logger->add("dq_msrx", irobot._model->dq_all_floating);	
	xbot_logger->add("ddq_msrx", irobot._model->ddq_all_floating);	
	xbot_logger->add("tau_msrx", irobot._model->tau_all_floating);	
	xbot_logger->add("tau_msr", irobot.tau_all);
	xbot_logger->add("L_com", irobot._model->angular_momentum);
	xbot_logger->add("LhdRef", irobot.LhdRef);
	xbot_logger->add("RhdRef", irobot.RhdRef);
	xbot_logger->add("deltaHip", deltaHip);
	xbot_logger->add("deltaFtZ", deltaFtZ);
	xbot_logger->add("deltaFtPos_l", deltaFtPos_l);
	xbot_logger->add("deltaFtPos_r", deltaFtPos_r);

	xbot_logger->add("gcom_raw", irobot.gcom_raw);
	xbot_logger->add("gdcom_raw", irobot.gdcom_raw);
	xbot_logger->add("gddcom_raw", irobot.gddcom_raw);
	xbot_logger->add("glft_raw", irobot.glft_raw);
	xbot_logger->add("grft_raw", irobot.grft_raw);
	xbot_logger->add("cop_raw", irobot.cop_raw);
	xbot_logger->add("gcop_raw", irobot.gcop_raw);
	xbot_logger->add("glcop_raw", irobot.glcop_raw);
	xbot_logger->add("grcop_raw", irobot.grcop_raw);
	xbot_logger->add("lankle", irobot.lankle);
	xbot_logger->add("rankle", irobot.rankle);
	xbot_logger->add("Lft", irobot.Lft);
	xbot_logger->add("Rft", irobot.Rft);

	xbot_logger->add("des_lft_vel", irobot.SendToRobot->des_lft_vel);
	xbot_logger->add("des_rft_vel", irobot.SendToRobot->des_rft_vel);
	xbot_logger->add("des_lft_acc", irobot.SendToRobot->des_lft_acc);
	xbot_logger->add("des_rft_acc", irobot.SendToRobot->des_rft_acc);
	xbot_logger->add("des_lhd_vel", irobot.SendToRobot->des_lhd_vel);
	xbot_logger->add("des_rhd_vel", irobot.SendToRobot->des_rhd_vel);

	xbot_logger->add("IMU_abs", irobot.IMU_abs);

	xbot_logger->add("PelvisPos", irobot.SendToRobot->HipPos);
	xbot_logger->add("LeftFootPos", irobot.SendToRobot->LeftAnklePos);
	xbot_logger->add("RightFootPos", irobot.SendToRobot->RightAnklePos);

	xbot_logger->add("HipPos", irobot.SendToRobot->HipPos);
	xbot_logger->add("LeftAnklePos", irobot.SendToRobot->LeftAnklePos);
	xbot_logger->add("RightAnklePos", irobot.SendToRobot->RightAnklePos);

	xbot_logger->add("HipO_Left", irobot.SendToRobot->HipO_Left);
	xbot_logger->add("HipO_Right", irobot.SendToRobot->HipO_Right);
	xbot_logger->add("LeftFootO", irobot.SendToRobot->LeftFootO);
	xbot_logger->add("RightFootO", irobot.SendToRobot->RightFootO);

	xbot_logger->add("stabilizer_torque", irobot.SendToRobot->stabilizer_torque);
	xbot_logger->add("applyDeltaPosX", irobot.SendToRobot->applyDeltaPosX);
	xbot_logger->add("applyDeltaPosY", irobot.SendToRobot->applyDeltaPosY);

	xbot_logger->add("TurnYaw", TurnYaw);
	xbot_logger->add("LftYaw", LftYaw);
	xbot_logger->add("RftYaw", RftYaw);

	
	xbot_logger->add("det_hip_posotion", det_hip_posotion);
	xbot_logger->add("det_hip_pose", det_hip_pose);
	xbot_logger->add("det_foot_rpy_lr", det_foot_rpy_lr);
	xbot_logger->add("det_footz_lr", det_footz_lr);		
	xbot_logger->add("time_cost", t_compute);		
	Sta.addToLog(xbot_logger);

	InternalLoggerLoop();
}
#endif
