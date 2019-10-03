/*****************************************************************************
WholeBodySensingClass.cpp
*****************************************************************************/
#include "WholeBodySensingClass.h"
#include "Filters/FilterClass.h"
#include "Filters/MeanFilterClass.h"
#include "Filters/KalmanFilterClass.h"

WholeBodySensingClass::WholeBodySensingClass()
//raw_data_cop(21,0), filter_data_cop(21,0),
//raw_data_GLOBAL_1(12,0), filter_data_GLOBAL_1(12,0), raw_data_GLOBAL_2(18,0), filter_data_GLOBAL_2(18,0),
	: FilterCutOff(30.0)
	, N_ButWth(4)
	, IsRefDelayInit(false)
	, Fz_ratio_l(0)
	, Fz_ratio_r(0)
	, Fzmin(30)
	, FZqueue(50, std::vector<double>(2))
	, oldRef(5, std::vector<double>(3, 0))
	, oldHandRef(2, std::vector<double>(3, 0))
	, olddRef(4, Eigen::Vector3d::Zero())
	, olddHandRef(2, Eigen::Vector3d::Zero())
	, oldAngRef(3, std::vector<double>(3, 0))
	, oldRelAngRef(2, std::vector<double>(3, 0))
	, IMU_odometry_raw(0, 0, 0)
	, IMU_dodometry_raw(0, 0, 0)
	, IMU_ddodometry_raw(0, 0, 0)
	, IsUpdateFtYaw(false)
	, dtimeFT(0)
	, gftYawTemp(0.0)
	, gftYawTemp1(0.0)
	, ContactIndex(0)
	, qall_offset(RobotParaClass::JOINT_NUM(), 0.0)
	, qall_temp(RobotParaClass::JOINT_NUM(), 0.0)
	, FT_filterDS(12, 0.0)
	, IsRemoveOffset(false)
        , yaw_off(0.0)
{
	q_off = Eigen::VectorXd::Zero(RobotParaClass::JOINT_NUM());

	LftCom_old << Eigen::Matrix3d::Identity();
	RftCom_old << Eigen::Matrix3d::Identity();

	FT_AlexFilter.reset(new MeanFilterClass);
	gcop_AlexFilter.reset(new MeanFilterClass);
	xKFC.reset(new KalmanFilterClass);
	yKFC.reset(new KalmanFilterClass);
	zKFC.reset(new KalmanFilterClass);

	glft_k_Filter.reset(new FilterClass);
	grft_k_Filter.reset(new FilterClass);
	gftOrigin_Filter.reset(new FilterClass);
	gftcom_Filter.reset(new FilterClass);
	gdftcom_Filter.reset(new FilterClass);
	gddftcom_Filter.reset(new FilterClass);
	gftcop_Filter.reset(new FilterClass);
	gftlcop_Filter.reset(new FilterClass);
	gftrcop_Filter.reset(new FilterClass);
	glft_Filter.reset(new FilterClass);
	grft_Filter.reset(new FilterClass);
	fOrigin_Filter.reset(new FilterClass);
	gOrigin_Filter.reset(new FilterClass);
	gcom_Filter.reset(new FilterClass);
	gdcom_Filter.reset(new FilterClass);
	gddcom_Filter.reset(new FilterClass);
	gcop_Filter.reset(new FilterClass);
	glcop_Filter.reset(new FilterClass);
	grcop_Filter.reset(new FilterClass);
	gGeomCenSP_Filter.reset(new FilterClass);
	lcop_Filter.reset(new FilterClass);
	rcop_Filter.reset(new FilterClass);
	dlcop_Filter.reset(new FilterClass);
	drcop_Filter.reset(new FilterClass);
	cop_in_lft_Filter.reset(new FilterClass);
	cop_in_rft_Filter.reset(new FilterClass);
	cop_Filter.reset(new FilterClass);
	fcop_Filter.reset(new FilterClass);
	ccop_local_Filter.reset(new FilterClass);
	IMU_LinearAcc_Filter.reset(new FilterClass);
	IMU_LinearVel_Filter.reset(new FilterClass);
	IMU_AngularVel_Filter.reset(new FilterClass);
	IMU_AngularAcc_Filter.reset(new FilterClass);
	IMU_odometry_Filter.reset(new FilterClass);
	FT_Filter.reset(new FilterClass);
	FT_hand_Filter.reset(new FilterClass);

	InitWBS(RobotParaClass::dT());

	DPRINTF("=============== WholeBodySensingClass is constructed... =============\n");
}

void WholeBodySensingClass::CheckSupportState()
{
	double eps = 1e-6;
	if (irobot.FT_foot_right(2) < eps) {
		irobot.Fz_zerocut[0] = eps;
	}
	else {
		irobot.Fz_zerocut[0] = irobot.FT_foot_right(2);
	}

	if (irobot.FT_foot_left(2) < eps) {
		irobot.Fz_zerocut[1] = eps;
	}
	else {
		irobot.Fz_zerocut[1] = irobot.FT_foot_left(2);
	}

	unsigned int N_sample = (int)floor((0.02) / irobot.dt + 0.5);
	unsigned int R = 0, L = 0;

	FZqueue.push_back(irobot.Fz_zerocut);
	FZqueue.pop_front();
	//printf("queue size is %i :",FZqueue.size());

	if (N_sample > FZqueue.size()) {
		N_sample =  FZqueue.size();
		DPRINTF("Error!!! Too long WhichFoot decision time! Reset it to %fs.", FZqueue.size()*irobot.dt);
	}

	for (unsigned int i = 1; i <= N_sample; i++) {
		if (FZqueue[FZqueue.size() - i][0] >= Fzmin) {
			R++;
		}
		if (FZqueue[FZqueue.size() - i][1] >= Fzmin) {
			L++;
		}
	}

	//if (N_sample > FT_AlexFilter->returnBufferSize())
	//{
	//	N_sample =  FT_AlexFilter->returnBufferSize();
	//}

	//for(int i=0;i<N_sample;i++){
	//	if(FT_AlexFilter->returnBuffer(i)[2]>=Fzmin){
	//		a++;
	//	}
	//	if(FT_AlexFilter->returnBuffer(i)[8]>=Fzmin){
	//		b++;
	//	}
	//}

	if (R == N_sample && L == 0) {
		irobot.Fz_condition[0] = irobot.Fz_zerocut[0];
		irobot.Fz_condition[1] = eps;
		irobot.WhichFoot = OnRFoot;
	}
	else if (R == 0 && L == N_sample) {
		irobot.Fz_condition[0] = eps;
		irobot.Fz_condition[1] = irobot.Fz_zerocut[1];
		irobot.WhichFoot = OnLFoot;
	}
	else if (R == N_sample && L == N_sample) {
		irobot.Fz_condition[0] = irobot.Fz_zerocut[0];
		irobot.Fz_condition[1] = irobot.Fz_zerocut[1];
		irobot.WhichFoot = OnBoth;
	}
	else if (R == 0 && L == 0) {
		irobot.Fz_condition[0] = eps;
		irobot.Fz_condition[1] = eps;
		irobot.WhichFoot = InAir;
	}
	else {// keep the previous state
		irobot.Fz_condition[0] = irobot.Fz_zerocut[0];
		irobot.Fz_condition[1] = irobot.Fz_zerocut[1];
	}

	//if( (WhichFoot_old!=irobot.WhichFoot) && (irobot.WhichFoot==OnBoth)){
	//	IsUpdateFtYaw=true;
	//}
	//else{
	//	IsUpdateFtYaw=false;
	//}

	if (WhichFoot_old != irobot.WhichFoot) {
		irobot.WhichFoot_old = WhichFoot_old;
		UpdateFtYawContactIndex_old = ContactIndex;
		ContactIndex++;
	}
	WhichFoot_old = irobot.WhichFoot;

	//std::vector<double> Fz_filter = Fz_AlexFilter->applyMeanFilter(20, irobot.Fz_condition); // filtering Fz_condition
	irobot.Fzr = irobot.Fz_zerocut[0];
	irobot.Fzl = irobot.Fz_zerocut[1];

	Fz_ratio_l = irobot.Fzl / (irobot.Fzl + irobot.Fzr);
	Fz_ratio_r = irobot.Fzr / (irobot.Fzl + irobot.Fzr);

	double half_weight_ratio = 0.45;
	if (((irobot.Fzl >= half_weight_ratio * irobot.m * RobotParaClass::G()) && (irobot.Fzr >= half_weight_ratio * irobot.m * RobotParaClass::G())) && (irobot.WhichFoot == OnBoth)) {
		if (UpdateFtYawContactIndex_old != ContactIndex) {
			IsUpdateFtYaw = true;
			UpdateFtYawContactIndex_old = ContactIndex;
		}
	}
	else {
		IsUpdateFtYaw = false;
	}

	// if (irobot.Fz_zerocut[0] >= Fzmin)
	// {
	// 	irobot.Fzr = irobot.Fz_zerocut[0] - Fzmin;
	// }
	// else {
	// 	irobot.Fzr = 0;
	// }

	// if (irobot.Fz_zerocut[1] >= Fzmin)
	// {
	// 	irobot.Fzl = irobot.Fz_zerocut[1] - Fzmin;
	// }
	// else {
	// 	irobot.Fzl = 0;
	// }

	double Tf1 = 1.0 / 10.0; // T=1/f for filter;
	double Tf2 = 1.0 / 2.0; // T=1/f for filter;

	if ( (irobot.FT_fl_filter(2) + irobot.FT_fr_filter(2)) > 2 * Fzmin ) { //normalized unit based on GRF/mg
		irobot.scale = (Tf1 * irobot.scale + irobot.dt * 1.0) / (Tf1 + irobot.dt); //if load on the ground, coefficient is 1
	} else {
		irobot.scale = (Tf2 * irobot.scale + irobot.dt * 0.0) / (Tf2 + irobot.dt);
	}

	Tf1 = 1.0 / 20.0; // T=1/f for filter;
	Tf2 = 1.0 / 10.0; // T=1/f for filter;

	if ( irobot.WhichFoot == OnLFoot ) { // for LandingFtO_PosMode, compliant ankle during landing
		irobot.RF_scale = (Tf1 * irobot.RF_scale + irobot.dt * 1.0) / (Tf1 + irobot.dt); //if swing in the air, coefficient is 1
	}
	else if (irobot.Fzl > 0.45 * irobot.m * RobotParaClass::G()) {
		irobot.RF_scale = (Tf2 * irobot.RF_scale + irobot.dt * 0.0) / (Tf2 + irobot.dt);
	}
	if ( irobot.WhichFoot == OnRFoot ) { // for LandingFtO_PosMode, compliant ankle during landing
		irobot.LF_scale = (Tf1 * irobot.LF_scale + irobot.dt * 1.0) / (Tf1 + irobot.dt); //if swing in the air, coefficient is 1
	}
	else if (irobot.Fzr > 0.45 * irobot.m * RobotParaClass::G()) {
		irobot.LF_scale = (Tf2 * irobot.LF_scale + irobot.dt * 0.0) / (Tf2 + irobot.dt);
	}

}

void WholeBodySensingClass::UpdateFK2Global()
{
	irobot.glft_k_old = irobot.glft_k_raw;
	irobot.grft_k_old = irobot.grft_k_raw;

	double eps = 1e-6;

	if (IsUpdateFtYaw) {
		dtimeFT++;
		Eigen::Matrix3d glftOtemp = irobot.IMU_rel * irobot.Rot_lft2hip;
		Eigen::Matrix3d grftOtemp = irobot.IMU_rel * irobot.Rot_rft2hip;
		double glftyaw = atan2(glftOtemp(1, 0), glftOtemp(0, 0));
		double grftyaw = atan2(grftOtemp(1, 0), grftOtemp(0, 0));
		if (irobot.WhichFoot_old == OnLFoot) {
			gftYawTemp += (grftyaw - glftyaw);
			gftYawTemp1 = abs(grftyaw - glftyaw) > abs(gftYawTemp1) ? (grftyaw - glftyaw) : gftYawTemp1;
		}
		else {
			gftYawTemp += (glftyaw - grftyaw);
			gftYawTemp1 = abs(glftyaw - grftyaw) > abs(gftYawTemp1) ? (glftyaw - grftyaw) : gftYawTemp1;
		}
		//std::cout <<"glftyaw "<<57.3*glftyaw<<"	grftyaw"<<57.3*grftyaw<<std::endl;
	}
	else {
		if (dtimeFT != 0) {
			//irobot.gftyaw += gftYawTemp/dtimeFT;
			irobot.gftyaw += gftYawTemp1;
			irobot.gftO = Rz(irobot.gftyaw);
			//std::cout <<"gft yaw "<<57.3*irobot.gftyaw<<"	"<<57.3*gftYawTemp/dtimeFT<<"	"<<57.3*gftYawTemp1 <<std::endl;
		}
		dtimeFT = 0;
		gftYawTemp = 0.0;
	}

	if (irobot.WhichFoot == OnRFoot) { // update global left foot only during right foot single support
		irobot.glft_k_raw = irobot.grft_k_old + irobot.grftO * irobot.IMU_rel * (irobot.Lft - irobot.Rft);
		irobot.gftcom_raw = irobot.grft_k_old + irobot.grftO * irobot.IMU_rel * (irobot.com - irobot.Rft);
		irobot.gftcop_raw = irobot.grft_k_old + irobot.grftO * irobot.IMU_rel * (irobot.cop_in_rft_raw);
	}
	else if (irobot.WhichFoot_old == OnRFoot) {
		if ((irobot.grftO * irobot.IMU_rel * (irobot.Lft - irobot.Rft))[2] > eps) {
			irobot.glft_k_raw = irobot.grft_k_old + irobot.grftO * irobot.IMU_rel * (irobot.Lft - irobot.Rft);
			irobot.gftcom_raw = irobot.grft_k_old + irobot.grftO * irobot.IMU_rel * (irobot.com - irobot.Rft);
			irobot.gftcop_raw = irobot.grft_k_old + irobot.grftO * irobot.IMU_rel * (irobot.cop_in_rft_raw);
		}
		else {
			//Eigen::Matrix3d gftOtemp = (irobot.grftO*irobot.IMU_rel*irobot.Rot_rft2hip)*irobot.Rot_rft2hip.transpose()*irobot.Rot_lft2hip;
			//Eigen::Matrix3d gftOtemp = irobot.grftO*irobot.IMU_rel*irobot.Rot_rft2hip.transpose()*irobot.Rot_lft2hip;
			//irobot.glftO = Rz(atan2(gftOtemp(1,0), gftOtemp(0,0)));
			irobot.glftO = irobot.gftO;
			irobot.gftcom_raw = irobot.glft_k_old + irobot.glftO * irobot.IMU_rel * (irobot.com - irobot.Lft);
			irobot.gftcop_raw = irobot.glft_k_old + irobot.glftO * irobot.IMU_rel * (irobot.cop_in_lft_raw);
		}
	}

	if (irobot.WhichFoot == OnLFoot) { // update global right foot only during left foot single support
		irobot.grft_k_raw = irobot.glft_k_old + irobot.glftO * irobot.IMU_rel * (irobot.Rft - irobot.Lft);
		irobot.gftcom_raw = irobot.glft_k_old + irobot.glftO * irobot.IMU_rel * (irobot.com - irobot.Lft);
		irobot.gftcop_raw = irobot.glft_k_old + irobot.glftO * irobot.IMU_rel * (irobot.cop_in_lft_raw);
	}
	else if (irobot.WhichFoot_old == OnLFoot) {
		if ((irobot.glftO * irobot.IMU_rel * (irobot.Rft - irobot.Lft))[2] > eps) {
			irobot.grft_k_raw = irobot.glft_k_old + irobot.glftO * irobot.IMU_rel * (irobot.Rft - irobot.Lft);
			irobot.gftcom_raw = irobot.glft_k_old + irobot.glftO * irobot.IMU_rel * (irobot.com - irobot.Lft);
			irobot.gftcop_raw = irobot.glft_k_old + irobot.glftO * irobot.IMU_rel * (irobot.cop_in_lft_raw);
		}
		else {
			//Eigen::Matrix3d gftOtemp = irobot.glftO*irobot.IMU_rel*irobot.Rot_lft2hip.transpose()*irobot.Rot_rft2hip;
			//irobot.grftO = Rz(atan2(gftOtemp(1,0), gftOtemp(0,0)));
			irobot.grftO = irobot.gftO;
			irobot.gftcom_raw = irobot.grft_k_old + irobot.grftO * irobot.IMU_rel * (irobot.com - irobot.Rft);
			irobot.gftcop_raw = irobot.grft_k_old + irobot.grftO * irobot.IMU_rel * (irobot.cop_in_rft_raw);
		}
	}

	if (irobot.WhichFoot_old == InAir || irobot.WhichFoot_old == Nowhere) { // for first step, need to confirm when the robot jumps
		//Eigen::Matrix3d gftOtemp = irobot.glftO*irobot.IMU_rel*irobot.Rot_lft2hip.transpose()*irobot.Rot_rft2hip;
		//irobot.grftO = Rz(atan2(gftOtemp(1,0), gftOtemp(0,0)));
		irobot.grftO = irobot.gftO;
		irobot.gftcom_raw = irobot.grft_k_old + irobot.grftO * irobot.IMU_rel * (irobot.com - irobot.Rft);
		irobot.gftcop_raw = irobot.grft_k_old + irobot.grftO * irobot.IMU_rel * (irobot.cop_in_rft_raw);
	}

	//Eigen::Vector3d gftOrigin_raw =  Fz_ratio_l *irobot.glft_k_raw + Fz_ratio_r*irobot.grft_k_raw; // it seems that gOrigin is the fOrigin in the world coordinate, fOrigin is the fOrigin point in hip coordinate.

	//if(irobot.WhichFoot==OnRFoot){// update global left foot only during right foot single support
	//	irobot.gftcom_raw	= gftOrigin_raw + irobot.grftO*irobot.IMU_rel*(irobot.ft_com);
	//}
	//else if(irobot.WhichFoot_old==OnRFoot){
	//	if((irobot.grftO*irobot.IMU_rel*(irobot.Lft-irobot.Rft))[2]>eps){
	//		irobot.gftcom_raw	= gftOrigin_raw + irobot.grftO*irobot.IMU_rel*(irobot.ft_com);
	//	}
	//	else{
	//		irobot.gftcom_raw	= gftOrigin_raw + irobot.glftO*irobot.IMU_rel*(irobot.ft_com);
	//	}
	//}

	//if(irobot.WhichFoot==OnLFoot){// update global right foot only during left foot single support
	//	irobot.gftcom_raw	= gftOrigin_raw + irobot.glftO*irobot.IMU_rel*(irobot.ft_com);
	//}
	//else if(irobot.WhichFoot_old==OnLFoot){
	//	if((irobot.glftO*irobot.IMU_rel*(irobot.Rft-irobot.Lft))[2]>eps){
	//		irobot.gftcom_raw	= gftOrigin_raw + irobot.glftO*irobot.IMU_rel*(irobot.ft_com);
	//	}
	//	else{
	//		irobot.gftcom_raw	= gftOrigin_raw + irobot.grftO*irobot.IMU_rel*(irobot.ft_com);
	//	}
	//}

	//if(irobot.WhichFoot_old==InAir || irobot.WhichFoot_old==Nowhere){ // for first step, need to confirm when the robot jumps
	//	irobot.gftcom_raw	= gftOrigin_raw + irobot.grftO*irobot.IMU_rel*(irobot.ft_com);
	//}


	irobot.glft_k = glft_k_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.glft_k_raw);
	irobot.grft_k = grft_k_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.grft_k_raw);

	irobot.gftOrigin_raw =  Fz_ratio_l * irobot.glft_k_raw + Fz_ratio_r * irobot.grft_k_raw;
	irobot.gftOrigin = gftOrigin_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.gftOrigin_raw);

	irobot.gftcom_old = irobot.gftcom_raw;
	irobot.gdftcom_old = irobot.gdftcom;

	irobot.gftcom = gftcom_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.gftcom_raw);
	irobot.gdftcom_raw	= (irobot.gftcom_raw - irobot.gftcom_old) / irobot.dt;
	irobot.gdftcom = gdftcom_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.gdftcom_raw);
	irobot.gddftcom_raw	= (irobot.gdftcom - irobot.gdftcom_old) / irobot.dt;
	irobot.gddftcom = gddftcom_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.gddftcom_raw);

//   irobot.gftlcop_raw	= irobot.gftOrigin_raw + irobot.IMU_abs*(irobot.Rot_lft2hip*irobot.lcop_raw + irobot.leftstance);
	//irobot.gftlcop = gftlcop_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.gftlcop_raw);

//   irobot.gftrcop_raw	= irobot.gftOrigin_raw + irobot.IMU_abs*(irobot.Rot_rft2hip*irobot.rcop_raw + irobot.rightstance);
	//irobot.gftrcop = gftrcop_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.gftrcop_raw);

//   irobot.gftcop_raw =  irobot.gftOrigin_raw + irobot.IMU_abs*(irobot.cop_raw - irobot.fOrigin_raw);
	irobot.gftcop = gftcop_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.gftcop_raw);

}

void WholeBodySensingClass::Update2Global()
{        
//     if(irobot.t<0.01){
//         DPRINTF("t: %.3f, Wh: %d, Wh_old: %d, gYaw: %d, yaw_off: %d \n", irobot.t, irobot.WhichFoot, irobot.WhichFoot_old, 57.3*irobot.gYaw, 57.3*yaw_off); 
// //         DPRINTF("t: %.3f, Lft: %.3f\t%.3f\t%.3f,\t Rft: %.3f\t%.3f\t%.3f\n", irobot.t, irobot.Lft[0], irobot.Lft[1], irobot.Lft[2], irobot.Rft[0], irobot.Rft[1], irobot.Rft[2]);
//         DPRINTF("t: %.3f, glft: %.3f\t%.3f\t%.3f,\t grft: %.3f\t%.3f\t%.3f\n", irobot.t, irobot.glft[0], irobot.glft[1], irobot.glft[2], irobot.grft[0], irobot.grft[1], irobot.grft[2]);
//         DPRINTF("t: %.3f, glft_old: %.3f\t%.3f\t%.3f,\t grft_old: %.3f\t%.3f\t%.3f\n", irobot.t, irobot.glft_old[0], irobot.glft_old[1], irobot.glft_old[2], irobot.grft_old[0], irobot.grft_old[1], irobot.grft_old[2]);
//         DPRINTF("t: %.3f, glft_raw: %.3f\t%.3f\t%.3f,\t grft_raw: %.3f\t%.3f\t%.3f\n", irobot.t, irobot.glft_raw[0], irobot.glft_raw[1], irobot.glft_raw[2], irobot.grft_raw[0], irobot.grft_raw[1], irobot.grft_raw[2]);
//     }
	double eps = 1e-6;

	if (irobot.WhichFoot == OnRFoot) { // update global left foot only during right foot single support
		irobot.glft_raw = irobot.grft_old + irobot.IMU_abs * (irobot.Lft - irobot.Rft);
	}
	else if (irobot.WhichFoot_old == OnRFoot) {
		if ((irobot.IMU_abs * (irobot.Lft - irobot.Rft))[2] > eps) {
			irobot.glft_raw = irobot.grft_old + irobot.IMU_abs * (irobot.Lft - irobot.Rft);
		}
	}

	if (irobot.WhichFoot == OnLFoot) { // update global right foot only during left foot single support
		irobot.grft_raw	= irobot.glft_old + irobot.IMU_abs * (irobot.Rft - irobot.Lft);
	}
	else if (irobot.WhichFoot_old == OnLFoot) {
		if ((irobot.IMU_abs * (irobot.Rft - irobot.Lft))[2] > eps) {
			irobot.grft_raw	= irobot.glft_old + irobot.IMU_abs * (irobot.Rft - irobot.Lft);
		}
	}

	//double StairHeight=0.02;
	//if(irobot.WhichFoot==OnBoth){
	//	if(abs(irobot.glft_raw[2]-irobot.grft_raw[2])>StairHeight){
	//		irobot.glft_raw[2]=0;
	//		irobot.grft_raw[2]=0;
	//	}
	//}

	irobot.glft = glft_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.glft_raw);
	irobot.grft = grft_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.grft_raw);
        
//     if(irobot.t<0.01){
//         
//         DPRINTF("t: %.3f, glft_raw: %.3f\t%.3f\t%.3f,\t grft_raw: %.3f\t%.3f\t%.3f\n", irobot.t, irobot.glft_raw[0], irobot.glft_raw[1], irobot.glft_raw[2], irobot.grft_raw[0], irobot.grft_raw[1], irobot.grft_raw[2]);
//         DPRINTF("t: %.3f, after glft: %.3f\t%.3f\t%.3f,\t grft: %.3f\t%.3f\t%.3f\n", irobot.t, irobot.glft[0], irobot.glft[1], irobot.glft[2], irobot.grft[0], irobot.grft[1], irobot.grft[2]);
//     }



	irobot.fOrigin_raw =  Fz_ratio_l * irobot.Lft + Fz_ratio_r * irobot.Rft;
	irobot.gOrigin_raw =  Fz_ratio_l * irobot.glft_raw + Fz_ratio_r * irobot.grft_raw; // it seems that gOrigin is the fOrigin in the world coordinate, fOrigin is the fOrigin point in hip coordinate.

	irobot.fOrigin = fOrigin_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.fOrigin_raw);
	irobot.gOrigin = gOrigin_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.gOrigin_raw);
	Waist2Global();
        
        irobot.glft_old = irobot.glft_raw;
        irobot.grft_old = irobot.grft_raw;
}

void WholeBodySensingClass::Waist2Global() // calc link componets from waist to global coordinate's frame
{
	// for (int i=0;i<irobot.idno;i++){
	// 	for (int j=0;j<irobot.segno;j++){
	// 		//irobot.LINK[j][i].ft_com	= irobot.LINK[j][i].rcom	-irobot.fOrigin;
	// 		//irobot.LINK[j][i].ft_dcom	= irobot.LINK[j][i].drcom	-irobot.fOrigin;
	// 		//irobot.LINK[j][i].ft_ddcom= irobot.LINK[j][i].ddrcom	-irobot.fOrigin;
	// 		//irobot.LINK[j][i].ft_joint = irobot.LINK[j][i].joint	-irobot.fOrigin;
	// 		//irobot.LINK[j][i].grcom	= irobot.gOrigin + irobot.IMU_abs*(irobot.LINK[j][i].rcom - irobot.fOrigin);
	// 	}
	// };
	//irobot.ft_com_old   = irobot.ft_com;
	irobot.ft_com   = irobot.com - irobot.fOrigin_raw; // fOrigin is the fOrigin point with respect to hip frame, irobot.com is COM in hip frame
	// so irobot.ft_com is the vector from fOrigin to COM in hip local frame (hip rot matrix eye(3))
	//irobot.ft_dcom  = (irobot.ft_com-irobot.ft_com_old)/irobot.dt;
	//irobot.ft_ddcom = irobot.ddcom	-irobot.fOrigin;
	//irobot.leftstance_old	= 	irobot.leftstance;
	//irobot.rightstance_old	= 	irobot.rightstance;
	irobot.leftstance	= 	irobot.Lft - irobot.fOrigin_raw;
	irobot.rightstance	= 	irobot.Rft - irobot.fOrigin_raw;

	//irobot.glft	= irobot.grft_old + irobot.IMU*(irobot.Lft-irobot.Rft);
	//irobot.grft	= irobot.glft_old + irobot.IMU*(irobot.Rft-irobot.Lft);
	//irobot.glft	= irobot.gOrigin + irobot.IMU*irobot.leftstance;
	//irobot.grft	= irobot.gOrigin + irobot.IMU*irobot.rightstance;
	//irobot.ghip	= ((irobot.glft-irobot.Lft)+(irobot.grft-irobot.Rft))/2;

	irobot.gcom_old = irobot.gcom_raw;
	irobot.gdcom_old = irobot.gdcom;

	irobot.ft_com_rel_old = irobot.ft_com_rel;
	irobot.ft_com_rel = irobot.IMU_rel * (irobot.com - 0.5 * (irobot.Lft + irobot.Rft));
	irobot.ft_dcom_rel = (irobot.ft_com_rel - irobot.ft_com_rel_old) / irobot.dt;

	irobot.gcom_raw	= irobot.gOrigin_raw + irobot.IMU_abs * irobot.ft_com;	// gcom is the estimated COM in world coordinate
	irobot.gcom = gcom_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.gcom_raw);

	irobot.gdcom_raw	= (irobot.gcom_raw - irobot.gcom_old) / irobot.dt;
	irobot.gdcom = gdcom_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.gdcom_raw);

	irobot.gddcom_raw	= (irobot.gdcom - irobot.gdcom_old) / irobot.dt;
	irobot.gddcom = gddcom_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.gddcom_raw);

	irobot.glcop_raw	= irobot.gOrigin_raw + irobot.IMU_abs * (irobot.Rot_lft2hip * irobot.lcop_raw + irobot.leftstance);
	irobot.glcop = glcop_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.glcop_raw);

	irobot.grcop_raw	= irobot.gOrigin_raw + irobot.IMU_abs * (irobot.Rot_rft2hip * irobot.rcop_raw + irobot.rightstance);
	irobot.grcop = grcop_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.grcop_raw);

	//irobot.gcop_raw =  Fz_ratio_l *irobot.glcop_raw + Fz_ratio_r*irobot.grcop_raw;
	irobot.gcop_raw =  irobot.gOrigin_raw + irobot.IMU_abs * (irobot.cop_raw - irobot.fOrigin_raw);
	irobot.gcop = gcop_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.gcop_raw);
	// irobot.gcop = gcop_AlexFilter->applyFilter(100,  irobot.gcop_raw);

	// irobot.gGeomCenDS_raw	= irobot.gOrigin + irobot.IMU_abs*(irobot.GeomCenDS - irobot.fOrigin);
	// irobot.gGeomCenDS = gGeomCenDS_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.gGeomCenDS_raw);

	// irobot.gCapturePoint = sqrt(RobotParaClass::Z_C() / RobotParaClass::G()) * irobot.gdcom;

	Eigen::Vector3d gcom_ft(0.0, 0.0, 0.0);
	// if (irobot.WhichFoot == OnRFoot) {
	// 	gcom_ft = irobot.gcom - irobot.grft;
	// }
	// else if (irobot.WhichFoot == OnLFoot) {
	// 	gcom_ft = irobot.gcom - irobot.glft;
	// }
	// else {
	// 	gcom_ft = irobot.gcom - 0.5 * (irobot.grft + irobot.glft);
	// }
	gcom_ft = irobot.gcom - 0.5 * (irobot.grft + irobot.glft);
	irobot.gCapturePoint = gcom_ft + sqrt(RobotParaClass::Z_C() / RobotParaClass::G()) *  Ry(irobot.gYaw).transpose() * irobot.gdcom;
	
//         DPRINTF("========= gcom update. =============\n");
// 	DPRINTF("========= gcop update. =============\n");

};

void WholeBodySensingClass::CalcCop()
{
	/*	for COP calculation */
	double pxrDS = 0.0;
	double pyrDS = 0.0;
	double pxlDS = 0.0;
	double pylDS = 0.0;
	double pxDS = 0.0;
	double pyDS = 0.0;
	double pzDS = RobotParaClass::FT_HEIGHT();

	//begin computation of ZMP based on force without filtering
	// right foot cop
	if (irobot.FT_foot_right(2) > Fzmin) {
		pxrDS = (-irobot.FT_foot_right(4) - pzDS * irobot.FT_foot_right(0)) / irobot.Fz_zerocut[0];
		pyrDS = (irobot.FT_foot_right(3) - pzDS * irobot.FT_foot_right(1)) / irobot.Fz_zerocut[0];
		if (pxrDS > (0.5 * RobotParaClass::FOOT_LENGTH() + RobotParaClass::ANKLE_X_OFFSET())) {
			pxrDS = (0.5 * RobotParaClass::FOOT_LENGTH() + RobotParaClass::ANKLE_X_OFFSET());
		}
		else if (pxrDS < -(0.5 * RobotParaClass::FOOT_LENGTH() - RobotParaClass::ANKLE_X_OFFSET())) {
			pxrDS = -(0.5 * RobotParaClass::FOOT_LENGTH() - RobotParaClass::ANKLE_X_OFFSET());
		}
		if (pyrDS > (0.5 * RobotParaClass::FOOT_WIDTH())) {
			pyrDS = (0.5 * RobotParaClass::FOOT_WIDTH());
		}
		else if (pyrDS < -(0.5 * RobotParaClass::FOOT_WIDTH())) {
			pyrDS = -(0.5 * RobotParaClass::FOOT_WIDTH());
		}
	}
	else {
		pxrDS = 0.0;
		pyrDS = 0.0;
	}
	// left foot cop
	if (irobot.FT_foot_left(2) > Fzmin) {
		pxlDS = (-irobot.FT_foot_left(4) - pzDS * irobot.FT_foot_left(0)) / irobot.Fz_zerocut[1];
		pylDS = (irobot.FT_foot_left(3) - pzDS * irobot.FT_foot_left(1)) / irobot.Fz_zerocut[1];
		if (pxlDS > (0.5 * RobotParaClass::FOOT_LENGTH() + RobotParaClass::ANKLE_X_OFFSET())) {
			pxlDS = (0.5 * RobotParaClass::FOOT_LENGTH() + RobotParaClass::ANKLE_X_OFFSET());
		}
		else if (pxlDS < -(0.5 * RobotParaClass::FOOT_LENGTH() - RobotParaClass::ANKLE_X_OFFSET())) {
			pxlDS = -(0.5 * RobotParaClass::FOOT_LENGTH() - RobotParaClass::ANKLE_X_OFFSET());
		}
		if (pylDS > (0.5 * RobotParaClass::FOOT_WIDTH())) {
			pylDS = (0.5 * RobotParaClass::FOOT_WIDTH());
		}
		else if (pylDS < -(0.5 * RobotParaClass::FOOT_WIDTH())) {
			pylDS = -(0.5 * RobotParaClass::FOOT_WIDTH());
		}
	}
	else {
		pxlDS = 0.0;
		pylDS = 0.0;
	}

	//end of computation of ZMP based on force without filtering

	irobot.lcop_raw << pxlDS, pylDS, 0;
	irobot.rcop_raw << pxrDS, pyrDS, 0;
	irobot.lcop = lcop_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.lcop_raw);
	irobot.rcop = rcop_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.rcop_raw);

	irobot.dlcop_raw = (irobot.lcop_raw - irobot.lcop_old) / irobot.dt;
	irobot.drcop_raw = (irobot.drcop_raw - irobot.rcop_old) / irobot.dt;
	irobot.dlcop = dlcop_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.dlcop_raw);
	irobot.drcop = drcop_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.drcop_raw);

	irobot.lcop_old = irobot.lcop_raw;
	irobot.rcop_old = irobot.drcop_raw;


	//if (irobot.WhichFoot==InAir){
	//	for(int i=0;i<3;i++){
	//		irobot.cop_raw[i]=0;
	//		irobot.cop_in_lft_raw[i]=0;
	//		irobot.cop_in_rft_raw[i]=0;
	//	}
	//	//irobot.cop << 0,0,0;
	//	////irobot.dcop << 0,0,0;
	//	//irobot.cop_in_lft << 0,0,0;
	//	//irobot.cop_in_rft << 0,0,0;
	//}
	//else
	//{
	//	if (irobot.WhichFoot==OnRFoot){
	//		for(int i=0;i<3;i++){
	//			irobot.cop_in_lft_raw[i]=0;
	//		}
	//		//irobot.cop_in_lft << 0,0,0;
	//	}
	//	if (irobot.WhichFoot==OnLFoot){
	//		for(int i=0;i<3;i++){
	//			irobot.cop_in_rft_raw[i]=0;
	//		}
	//		//irobot.cop_in_rft << 0,0,0;
	//	}
	//	irobot.cop_in_lft_raw = Fz_ratio_l * irobot.Rot_lft2hip*irobot.lcop_raw + Fz_ratio_r * (irobot.Rot_rft2hip*irobot.rcop_raw + irobot.Rft-irobot.Lft);
	//	irobot.cop_in_rft_raw = Fz_ratio_l * (irobot.Rot_lft2hip*irobot.lcop_raw + irobot.Lft-irobot.Rft) + Fz_ratio_r * irobot.Rot_rft2hip*irobot.rcop_raw;
	//	irobot.cop_raw = Fz_ratio_l * (irobot.Lft+irobot.Rot_lft2hip*irobot.lcop_raw) + Fz_ratio_r * (irobot.Rft+irobot.Rot_rft2hip*irobot.rcop_raw);
	//	irobot.fcop_raw = Fz_ratio_l * (irobot.IMU_abs*irobot.Rot_lft2hip*irobot.lcop_raw) +Fz_ratio_r * (irobot.IMU_abs*irobot.Rot_rft2hip*irobot.rcop_raw);
	//	irobot.cop_local_raw = irobot.cop_raw - irobot.GeomCenDS;
	//}

	irobot.cop_in_lft_raw = Fz_ratio_l * irobot.Rot_lft2hip * irobot.lcop_raw + Fz_ratio_r * (irobot.Rot_rft2hip * irobot.rcop_raw + irobot.Rft - irobot.Lft);
	irobot.cop_in_rft_raw = Fz_ratio_l * (irobot.Rot_lft2hip * irobot.lcop_raw + irobot.Lft - irobot.Rft) + Fz_ratio_r * irobot.Rot_rft2hip * irobot.rcop_raw;
	irobot.cop_raw = Fz_ratio_l * (irobot.Lft + irobot.Rot_lft2hip * irobot.lcop_raw) + Fz_ratio_r * (irobot.Rft + irobot.Rot_rft2hip * irobot.rcop_raw);
//  irobot.fcop_raw = Fz_ratio_l * (irobot.IMU_rel*irobot.Rot_lft2hip*irobot.lcop_raw) +Fz_ratio_r * (irobot.IMU_rel*irobot.Rot_rft2hip*irobot.rcop_raw);
	irobot.cop_local_raw = irobot.cop_raw - irobot.GeomCenDS;

	irobot.cop_in_lft = cop_in_lft_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.cop_in_lft_raw);
	irobot.cop_in_rft = cop_in_rft_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.cop_in_rft_raw);
	irobot.cop = cop_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.cop_raw);
//	irobot.fcop = fcop_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.fcop_raw);
	irobot.cop_local = ccop_local_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.cop_local_raw);

	irobot.fcop_raw = irobot.IMU_rel * (irobot.cop_local_raw);
	irobot.fcop = fcop_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.fcop_raw);
}

void WholeBodySensingClass::ForwardKinematics()
{
	Eigen::Matrix3d R01, R12, R23, R34, R45, R56;

	//// pelvis, trunk, head
	irobot.LINK[1][0].R  = Ry(DEGTORAD(-20)) * Rx(irobot.LINK[1][0].q(0)) * Ry(irobot.LINK[1][0].q(1) + DEGTORAD(20)) * Rz(irobot.LINK[1][0].q(2)); // trunk

	//irobot.LINK[2][0].R  = irobot.LINK[1][0].R;

	irobot.LINK[1][0].joint = irobot.LINK[0][0].limb;
	irobot.LINK[2][0].joint = irobot.LINK[1][0].joint + irobot.LINK[1][0].R * irobot.LINK[1][0].limb; // this is the origin of head coordinate, also the reference frame for two arms in inverse kinematics
	irobot.LINK[1][0].rcom = irobot.LINK[1][0].joint + irobot.LINK[1][0].R * irobot.LINK[1][0].dcom;
	irobot.LINK[2][0].rcom = irobot.LINK[2][0].joint + irobot.LINK[2][0].R * irobot.LINK[2][0].dcom;

	//// left leg
	R01 = Ry(irobot.LINK[0][1].q(0));
	R12 = Rx(irobot.LINK[0][1].q(1));
	R23 = Rz(irobot.LINK[0][1].q(2));
	R34 = Ry(irobot.LINK[1][1].q(1));
	R45 = Rx(irobot.LINK[2][1].q(0));
	R56 = Ry(irobot.LINK[2][1].q(1));

	irobot.LINK[0][1].R = R01 * R12 * R23;
	irobot.LINK[1][1].R = irobot.LINK[0][1].R * R34;
	irobot.Rot_lft2hip = irobot.LINK[2][1].R = irobot.LINK[1][1].R * R45 * R56;
	irobot.LINK[1][1].joint = irobot.LINK[0][1].joint + irobot.LINK[0][1].R * irobot.LINK[0][1].limb;
	irobot.LINK[2][1].joint = irobot.LINK[1][1].joint + irobot.LINK[1][1].R * irobot.LINK[1][1].limb;

	irobot.LINK[0][1].rcom = irobot.LINK[0][1].joint + irobot.LINK[0][1].R * irobot.LINK[0][1].dcom;
	irobot.LINK[1][1].rcom = irobot.LINK[1][1].joint + irobot.LINK[1][1].R * irobot.LINK[1][1].dcom;
	irobot.LINK[2][1].rcom = irobot.LINK[2][1].joint + irobot.LINK[2][1].R * irobot.LINK[2][1].dcom;

	//// right leg
	R01 = Ry(irobot.LINK[0][2].q(0));
	R12 = Rx(irobot.LINK[0][2].q(1));
	R23 = Rz(irobot.LINK[0][2].q(2));
	R34 = Ry(irobot.LINK[1][2].q(1));
	R45 = Rx(irobot.LINK[2][2].q(0));
	R56 = Ry(irobot.LINK[2][2].q(1));

	irobot.LINK[0][2].R = R01 * R12 * R23;
	irobot.LINK[1][2].R = irobot.LINK[0][2].R * R34;
	irobot.Rot_rft2hip = irobot.LINK[2][2].R = irobot.LINK[1][2].R * R45 * R56;
	irobot.LINK[1][2].joint = irobot.LINK[0][2].joint + irobot.LINK[0][2].R * irobot.LINK[0][2].limb;
	irobot.LINK[2][2].joint = irobot.LINK[1][2].joint + irobot.LINK[1][2].R * irobot.LINK[1][2].limb;

	irobot.LINK[0][2].rcom = irobot.LINK[0][2].joint + irobot.LINK[0][2].R * irobot.LINK[0][2].dcom;
	irobot.LINK[1][2].rcom = irobot.LINK[1][2].joint + irobot.LINK[1][2].R * irobot.LINK[1][2].dcom;
	irobot.LINK[2][2].rcom = irobot.LINK[2][2].joint + irobot.LINK[2][2].R * irobot.LINK[2][2].dcom;

	// left arm
	R01 = Ry(irobot.LINK[0][3].q(0));
	R12 = Rx(irobot.LINK[0][3].q(1));
	R23 = Rz(irobot.LINK[0][3].q(2));
	R34 = Ry(irobot.LINK[1][3].q(1));

	irobot.LINK[0][3].R = irobot.LINK[1][0].R * R01 * R12 * R23;
	irobot.LINK[1][3].R = irobot.LINK[0][3].R * R34;
	irobot.LINK[2][3].R = irobot.LINK[1][3].R;

	irobot.LINK[0][3].joint = irobot.LINK[2][0].joint + irobot.LINK[1][0].R * irobot.shoulder;
	irobot.LINK[1][3].joint = irobot.LINK[0][3].joint + irobot.LINK[0][3].R * irobot.LINK[0][3].limb;
	irobot.LINK[2][3].joint = irobot.LINK[1][3].joint + irobot.LINK[1][3].R * irobot.LINK[1][3].limb; // left hand vector, now doesnt exist

	irobot.LINK[0][3].rcom = irobot.LINK[0][3].joint + irobot.LINK[0][3].R * irobot.LINK[0][3].dcom;
	irobot.LINK[1][3].rcom = irobot.LINK[1][3].joint + irobot.LINK[1][3].R * irobot.LINK[1][3].dcom;
	irobot.LINK[2][3].rcom = irobot.LINK[2][3].joint + irobot.LINK[2][3].R * irobot.LINK[2][3].dcom;

	// right arm
	R01 = Ry(irobot.LINK[0][4].q(0));
	R12 = Rx(irobot.LINK[0][4].q(1));
	R23 = Rz(irobot.LINK[0][4].q(2));
	R34 = Ry(irobot.LINK[1][4].q(1));

	irobot.LINK[0][4].R = irobot.LINK[1][0].R * R01 * R12 * R23;
	irobot.LINK[1][4].R = irobot.LINK[0][4].R * R34;
	irobot.LINK[2][4].R = irobot.LINK[1][4].R;

	irobot.LINK[0][4].joint = irobot.LINK[2][0].joint + irobot.LINK[1][0].R * (-irobot.shoulder);
	irobot.LINK[1][4].joint = irobot.LINK[0][4].joint + irobot.LINK[0][4].R * irobot.LINK[0][4].limb;
	irobot.LINK[2][4].joint = irobot.LINK[1][4].joint + irobot.LINK[1][4].R * irobot.LINK[1][4].limb; //right hand vector, now doesnt exist

	irobot.LINK[0][4].rcom = irobot.LINK[0][4].joint + irobot.LINK[0][4].R * irobot.LINK[0][4].dcom;
	irobot.LINK[1][4].rcom = irobot.LINK[1][4].joint + irobot.LINK[1][4].R * irobot.LINK[1][4].dcom;
	irobot.LINK[2][4].rcom = irobot.LINK[2][4].joint + irobot.LINK[2][4].R * irobot.LINK[2][4].dcom;

	// now calculate the overall COM of the robot
	Eigen::Vector3d numerator_m, numerator_v, numerator_a;
	numerator_m << 0, 0, 0; // clear to zero before calculation
	numerator_v << 0, 0, 0;
	numerator_a << 0, 0, 0;
	for (int i = 0; i < irobot.idno; i++)
	{
		for (int j = 0; j < irobot.segno; j++)
		{
			irobot.LINK[j][i].dq = (irobot.LINK[j][i].q - irobot.LINK[j][i].q_old) / irobot.dt;
			irobot.LINK[j][i].ddq = (irobot.LINK[j][i].dq - irobot.LINK[j][i].dq_old) / irobot.dt;
			irobot.LINK[j][i].drcom = (irobot.LINK[j][i].rcom - irobot.LINK[j][i].rcom_old) / irobot.dt;
			irobot.LINK[j][i].ddrcom = (irobot.LINK[j][i].drcom - irobot.LINK[j][i].drcom_old) / irobot.dt;
			numerator_m += irobot.LINK[j][i].m * irobot.LINK[j][i].rcom;
			numerator_v += irobot.LINK[j][i].m * irobot.LINK[j][i].drcom;
			numerator_a += irobot.LINK[j][i].m * irobot.LINK[j][i].ddrcom;
		}
	};

	//irobot.com_old = irobot.com;
	//irobot.dcom_old = irobot.dcom;
	//irobot.ddcom_old = irobot.ddcom;
	irobot.com = numerator_m / irobot.m;
	irobot.dcom = numerator_v / irobot.m;
	irobot.ddcom = numerator_a / irobot.m;
	irobot.lankle = irobot.LINK[2][1].joint;
	irobot.rankle = irobot.LINK[2][2].joint;
	irobot.Lft = irobot.lankle + irobot.Rot_lft2hip * irobot.LINK[2][1].limb; // left foot center vector
	irobot.Rft = irobot.rankle + irobot.Rot_rft2hip * irobot.LINK[2][2].limb; // right foot center vector
	irobot.Larm = irobot.LINK[2][3].joint;   // hand center
	irobot.Rarm = irobot.LINK[2][4].joint;
	irobot.Lcom_old = irobot.Lcom;
	irobot.dLcom_old = irobot.dLcom;

	// calculate the Overall angular momentum
	irobot.Icom << Eigen::Matrix3d::Zero();
	irobot.Lcom << Eigen::Vector3d::Zero();
	irobot.Ift << Eigen::Matrix3d::Zero();
	for (int i = 0; i < irobot.idno; i++) {
		for (int j = 0; j < irobot.segno; j++) {
			//irobot.LINK[j][i].qcom = irobot.LINK[j][i].grcom; // For ExtTorque Estimation, all torques around Global origin. In GLOBAL frame
			irobot.LINK[j][i].qcom = irobot.IMU_rel * (irobot.LINK[j][i].rcom - irobot.GeomCenDS); // For ExtTorque Estimation, all torques around GeomCenDS. In GLOBAL frame
			//irobot.LINK[j][i].dqcom	= (irobot.LINK[j][i].qcom - irobot.LINK[j][i].qcom_old)/irobot.dt;
			//irobot.LINK[j][i].qcom_old = irobot.LINK[j][i].qcom;

			////////
			irobot.Ift(0, 0) += irobot.LINK[j][i].m * (irobot.LINK[j][i].qcom(1) * irobot.LINK[j][i].qcom(1) + irobot.LINK[j][i].qcom(2) * irobot.LINK[j][i].qcom(2));
			irobot.Ift(1, 1) += irobot.LINK[j][i].m * (irobot.LINK[j][i].qcom(0) * irobot.LINK[j][i].qcom(0) + irobot.LINK[j][i].qcom(2) * irobot.LINK[j][i].qcom(2));
			irobot.Ift(2, 2) += irobot.LINK[j][i].m * (irobot.LINK[j][i].qcom(0) * irobot.LINK[j][i].qcom(0) + irobot.LINK[j][i].qcom(1) * irobot.LINK[j][i].qcom(1));
			irobot.Ift(0, 1) -= irobot.LINK[j][i].m * (irobot.LINK[j][i].qcom(0) * irobot.LINK[j][i].qcom(1));
			irobot.Ift(0, 2) -= irobot.LINK[j][i].m * (irobot.LINK[j][i].qcom(0) * irobot.LINK[j][i].qcom(2));
			irobot.Ift(1, 2) -= irobot.LINK[j][i].m * (irobot.LINK[j][i].qcom(1) * irobot.LINK[j][i].qcom(2));
			irobot.Ift(1, 0) = irobot.Ift(0, 1);
			irobot.Ift(2, 0) = irobot.Ift(0, 2);
			irobot.Ift(2, 1) = irobot.Ift(1, 2);
		}
	}

	for (int i = 0; i < irobot.idno; i++) {
		for (int j = 0; j < irobot.segno; j++) {

			irobot.LINK[j][i].pcom = irobot.LINK[j][i].rcom - irobot.com;	// For ExtTorque Estimation, all torques around com. In HIP frame
			irobot.LINK[j][i].dpcom	= (irobot.LINK[j][i].pcom - irobot.LINK[j][i].pcom_old) / irobot.dt;
			irobot.LINK[j][i].pcom_old = irobot.LINK[j][i].pcom;

			////////
			irobot.Icom(0, 0) += irobot.LINK[j][i].m * (irobot.LINK[j][i].pcom(1) * irobot.LINK[j][i].pcom(1) + irobot.LINK[j][i].pcom(2) * irobot.LINK[j][i].pcom(2));
			irobot.Icom(1, 1) += irobot.LINK[j][i].m * (irobot.LINK[j][i].pcom(0) * irobot.LINK[j][i].pcom(0) + irobot.LINK[j][i].pcom(2) * irobot.LINK[j][i].pcom(2));
			irobot.Icom(2, 2) += irobot.LINK[j][i].m * (irobot.LINK[j][i].pcom(0) * irobot.LINK[j][i].pcom(0) + irobot.LINK[j][i].pcom(1) * irobot.LINK[j][i].pcom(1));
			irobot.Icom(0, 1) -= irobot.LINK[j][i].m * (irobot.LINK[j][i].pcom(0) * irobot.LINK[j][i].pcom(1));
			irobot.Icom(0, 2) -= irobot.LINK[j][i].m * (irobot.LINK[j][i].pcom(0) * irobot.LINK[j][i].pcom(2));
			irobot.Icom(1, 2) -= irobot.LINK[j][i].m * (irobot.LINK[j][i].pcom(1) * irobot.LINK[j][i].pcom(2));
			irobot.Icom(1, 0) = irobot.Icom(0, 1);
			irobot.Icom(2, 0) = irobot.Icom(0, 2);
			irobot.Icom(2, 1) = irobot.Icom(1, 2);

			// Angular Momentum
			//irobot.LINK[j][i].Lcom = irobot.LINK[j][i].pcom.cross(irobot.LINK[j][i].m*(irobot.LINK[j][i].drcom-irobot.dcom));
			irobot.LINK[j][i].Lcom = irobot.LINK[j][i].pcom.cross(irobot.LINK[j][i].m * irobot.LINK[j][i].dpcom);
			irobot.Lcom += irobot.LINK[j][i].Lcom;
		}
	}

	irobot.dLcom = (irobot.Lcom - irobot.Lcom_old) / irobot.dt;
	irobot.gIcom = irobot.IMU_abs * irobot.Icom * irobot.IMU_abs.transpose();
	irobot.gIft = irobot.IMU_abs * irobot.Ift * irobot.IMU_abs.transpose();

	for (int i = 0; i < irobot.idno; i++) {
		for (int j = 0; j < irobot.segno; j++) {
			irobot.LINK[j][i].q_old = irobot.LINK[j][i].q;
			irobot.LINK[j][i].dq_old = irobot.LINK[j][i].dq;
			irobot.LINK[j][i].ddq_old = irobot.LINK[j][i].ddq;
			irobot.LINK[j][i].rcom_old = irobot.LINK[j][i].rcom;
			irobot.LINK[j][i].drcom_old = irobot.LINK[j][i].drcom;
			irobot.LINK[j][i].ddrcom_old = irobot.LINK[j][i].ddrcom;
			irobot.LINK[j][i].joint_old = irobot.LINK[j][i].joint;
		}
	}

};

void WholeBodySensingClass::RobotModelFK()
{
	// Eigen::Matrix3d Rot_w2hip = irobot._model->Rot_World_to_Pelvis;
	irobot.com = irobot._model->com_hip;
	irobot.dcom = irobot._model->dcom_hip;
	irobot.lankle = irobot._model->lankle.r;
	irobot.rankle = irobot._model->rankle.r;
	irobot.Lft = irobot._model->lsole.r;
	irobot.Rft = irobot._model->rsole.r;
	irobot.Rot_lft2hip = irobot._model->lsole.E;
	irobot.Rot_rft2hip = irobot._model->rsole.E;
	irobot.Larm = irobot._model->lwrist.r;
	irobot.Rarm = irobot._model->rwrist.r;
}

void WholeBodySensingClass::UpdateUsefulVariableAfterFK()
{
	irobot.IMU_odometry_raw = IMU_odometry_raw + irobot.IMU_abs * irobot.com;
	irobot.IMU_dodometry_raw = IMU_dodometry_raw + irobot.IMU_abs * irobot.dcom;
	irobot.IMU_odometry_raw[2] += RobotParaClass::FULL_LEG(); // should not be full leg, Mon 17 Jul 2017 05:06:59 PM CEST

	// calculate GeomCenDS as the center of two ankles
	irobot.GeomCenDS = 0.5 * (irobot.lankle + irobot.rankle);
	// irobot.GeomCenDS = 0.5 * (irobot.Lft + irobot.Rft);

	irobot.GeomCenLft	= 	irobot.lankle - irobot.GeomCenDS;
	irobot.GeomCenRft	= 	irobot.rankle - irobot.GeomCenDS;
	irobot.GeomCenCOM	= 	irobot.com - irobot.GeomCenDS;
	irobot.gGeomCenLft	= 	irobot.IMU_rel * irobot.GeomCenLft;
	irobot.gGeomCenRft	= 	irobot.IMU_rel * irobot.GeomCenRft;
	irobot.gGeomCenCOM	= 	irobot.IMU_rel * irobot.GeomCenCOM;

	// calculate GeomCenDS as the center of two ankles projections on the ground
	irobot.GeomCenDS = 0.5 * (irobot.Lft + irobot.Rft);

	irobot.cLft = irobot.Lft - irobot.com;
	irobot.cRft = irobot.Rft - irobot.com;
	irobot.gcLft = irobot.IMU_rel * irobot.cLft;
	irobot.gcRft = irobot.IMU_rel * irobot.cRft;

	irobot.dgcLft = (irobot.gcLft - irobot.gcLft_old)/irobot.dt;
	irobot.gcLft_old = irobot.gcLft;
	irobot.dgcRft = (irobot.gcRft - irobot.gcRft_old)/irobot.dt;
	irobot.gcRft_old = irobot.gcRft;

	irobot.cGeomCenDS = irobot.GeomCenDS - irobot.com;
	irobot.gcGeomCenDS = irobot.IMU_rel * irobot.cGeomCenDS;
        irobot.dgcGeomCenDS    = (irobot.gcGeomCenDS - irobot.gcGeomCenDS_old)/irobot.dt;
        irobot.gcGeomCenDS_old = irobot.gcGeomCenDS;
	//irobot.dGeomCenDS	= (irobot.GeomCenDS - irobot.GeomCenDS_old)/irobot.dt;
	//irobot.GeomCenDS_old = irobot.GeomCenDS;

	// irobot.GeomCenLft	= 	irobot.Lft - irobot.GeomCenDS;
	// irobot.GeomCenRft	= 	irobot.Rft - irobot.GeomCenDS;
	// irobot.GeomCenCOM	= 	irobot.com - irobot.GeomCenDS;
	// irobot.gGeomCenLft	= 	irobot.IMU_rel * irobot.GeomCenLft;
	// irobot.gGeomCenRft	= 	irobot.IMU_rel * irobot.GeomCenRft;
	// irobot.gGeomCenCOM	= 	irobot.IMU_rel * irobot.GeomCenCOM;

}


void WholeBodySensingClass::UpdateIMU(double dt, const Eigen::Matrix3d &Rpelvis_abs, const Eigen::Vector3d &LnAcc, const Eigen::Vector3d &AgVel)
{
// 	InitWBS(dt);
	Eigen::Vector3d EulerAng(0, 0, 0);

	// Eigen::Vector3d AgVel(0,0,0), EulerAng(0,0,0);
	EulerAng[0] = std::atan2(Rpelvis_abs(2, 1), Rpelvis_abs(2, 2));   //phi
	EulerAng[1] = std::asin(-Rpelvis_abs(2, 0));           //theta
	EulerAng[2] = std::atan2(Rpelvis_abs(1, 0), Rpelvis_abs(0, 0));    //greek Y

	//AgVel += irobot.dt*AgAcc;

	UpdateIMU(dt, Rpelvis_abs, LnAcc, AgVel, EulerAng);;
}

void WholeBodySensingClass::UpdateIMU(double dt, const Eigen::Matrix3d &Rpelvis_abs, const Eigen::Vector3d &LnAcc, const Eigen::Vector3d &AgVel, const Eigen::Vector3d &EulerAng)
{
// 	InitWBS(dt);
	//irobot.gRoll  = atan2(Rpelvis_abs(2,1), Rpelvis_abs(2,2));     //phi
	//irobot.gPitch = asin(-Rpelvis_abs(2,0));            //theta
	//irobot.gYaw   = atan2(Rpelvis_abs(1,0), Rpelvis_abs(0,0));      //greek Y

	irobot.IMU_Euler = EulerAng;
        irobot.IMU_Euler[2] -= yaw_off;
	irobot.gRoll  = std::atan2(Rpelvis_abs(2, 1), Rpelvis_abs(2, 2));   //phi
	irobot.gPitch = std::asin(-Rpelvis_abs(2, 0));           //theta
	irobot.gYaw   = std::atan2(Rpelvis_abs(1, 0), Rpelvis_abs(0, 0)) - yaw_off;    //greek Y

	irobot.IMU_rel = Ry(irobot.gPitch) * Rx(irobot.gRoll);	// pelvis rotational matrix without yaw
// 	irobot.IMU_abs = Rpelvis_abs;
        irobot.IMU_abs = Rz(irobot.gYaw) * irobot.IMU_rel;

	irobot.IMU_LinearAcc_raw = LnAcc;
	irobot.IMU_LinearAcc = IMU_LinearAcc_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.IMU_LinearAcc_raw);

	irobot.IMU_LinearVel_raw += irobot.dt * irobot.IMU_LinearAcc_raw;
	//irobot.IMU_LinearVel_raw += irobot.dt*irobot.IMU_LinearAcc;
	irobot.IMU_LinearVel = IMU_LinearVel_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.IMU_LinearVel_raw);

	irobot.IMU_AngularVel_raw = AgVel;
	irobot.IMU_AngularVel = IMU_AngularVel_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.IMU_AngularVel_raw);

	irobot.IMU_AngularAcc_raw = (irobot.IMU_AngularVel - irobot.IMU_AngularVel_old) / irobot.dt;
	irobot.IMU_AngularAcc = IMU_AngularAcc_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.IMU_AngularAcc_raw);

	irobot.IMU_AngularVel_old = irobot.IMU_AngularVel;

	irobot.IMU_ddodometry_raw = irobot.IMU_abs * irobot.IMU_LinearAcc_raw;
	IMU_dodometry_raw += irobot.dt * irobot.IMU_ddodometry_raw;
	//irobot.IMU_dodometry = IMU_dodometry_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.IMU_dodometry_raw);
	IMU_odometry_raw += irobot.dt * IMU_dodometry_raw;
	//irobot.IMU_odometry = IMU_odometry_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, irobot.IMU_odometry_raw);
}

void WholeBodySensingClass::UpdateRobotState(const double (&qL)[6], const double (&qR)[6], const double (&qW)[3], const double (&qaL)[4], const double (&qaR)[4], double dt, const Eigen::Matrix3d &Rpelvis_abs, const std::vector<double> &FTSensor)
{
// 	InitWBS(dt);
	//Eigen::Vector3d LnAcc(0,0,0), AgAcc(0,0,0);
	//UpdateIMU(Rpelvis_abs, LnAcc, AgAcc);

	Eigen::Vector3d LnAcc(0, 0, 0), AgVel(0, 0, 0), EulerAng(0, 0, 0);
	EulerAng[0] = std::atan2(Rpelvis_abs(2, 1), Rpelvis_abs(2, 2));   //phi
	EulerAng[1] = std::asin(-Rpelvis_abs(2, 0));           //theta
	EulerAng[2] = std::atan2(Rpelvis_abs(1, 0), Rpelvis_abs(0, 0));    //greek Y
	UpdateIMU(dt, Rpelvis_abs, LnAcc, AgVel, EulerAng);
	UpdateRobotState(qL, qR, qW, qaL, qaR, dt, FTSensor);
	
	//DPRINTF("=============== UpdateRobotState11111111XXXX=============\n");
	
}

void WholeBodySensingClass::UpdateJointTorqueFB(const std::vector<double> &tall)
{
	irobot.tau_all = tall;
}

void WholeBodySensingClass::UpdateRobotState(const std::vector<double> &qall, double dt, const std::vector<double> &FTSensor, const std::vector<double> &tall)
{
	irobot.tau_all = tall;
	UpdateRobotState(qall, dt, FTSensor);
	//DPRINTF("=============== UpdateRobotState2222222222222222222=============\n");
}

void WholeBodySensingClass::UpdateRobotState(const std::vector<double> &qall, double dt, const std::vector<double> &FTSensor)
{
	std::vector<double> qW(3, 0), qL(6, 0), qR(6, 0), qaL(4, 0), qaR(4, 0);
	// if (!IsRemoveOffset) {
	// 	for (int i = 3; i < 15; ++i) {
	// 		qall_offset[i] = qall[i]; // is not right. Fri 21 Apr 2017 12:22:56 PM CEST
	// 	}
	// 	IsRemoveOffset = true;
	// }

	// std::vector<double>qall_temp(qall.size(), 0);
	// qall_temp = qall;
	for (int i = 0; i < qall.size(); ++i) {
//                 qall_temp[i] = qall[i] - qall_offset[i];
		qall_temp[i] = qall[i] - q_off[i];
		// COUT(i,irobot.RobotPara().getJointName(i));
	}
	irobot.q_all = qall_temp;

	irobot.q_all_mesure_old = irobot.q_all_mesure;
	irobot.q_all_mesure = qall_temp; // for rbdl
	for (int i = 0; i < irobot.q_all_mesure.size(); ++i) {
		irobot.qdot_all_mesure[i] = (irobot.q_all_mesure[i] - irobot.q_all_mesure_old[i]) / dt;
	}

	qW[0] = qall_temp[WAIST_ROLL];	//WAIST_ROLL,			//  0
	qW[1] = qall_temp[WAIST_PITCH];	//WAIST_PITCH,			//  1
	qW[2] = qall_temp[WAIST_YAW];	//WAIST_YAW,			//  2
	qR[0] =	qall_temp[RIGHT_HIP_PITCH];	//RIGHT_HIP_PITCH,		//  3
	qR[1] = qall_temp[RIGHT_HIP_ROLL];	//RIGHT_HIP_ROLL,		//  5
	qR[2] = qall_temp[RIGHT_HIP_YAW];	//RIGHT_HIP_YAW,		//  6
	qR[3] = qall_temp[RIGHT_KNEE_PITCH];	//RIGHT_KNEE_PITCH,		//  7
	qR[4] = qall_temp[RIGHT_FOOT_ROLL];	//RIGHT_FOOT_ROLL,		//  8
	qR[5] = qall_temp[RIGHT_FOOT_PITCH];	//RIGHT_FOOT_PITCH,		//  9
	qL[0] = qall_temp[LEFT_HIP_PITCH];	//LEFT_HIP_PITCH,		//  4
	qL[1] = qall_temp[LEFT_HIP_ROLL];	//LEFT_HIP_ROLL,		// 10
	qL[2] = qall_temp[LEFT_HIP_YAW];	//LEFT_HIP_YAW,			// 11
	qL[3] = qall_temp[LEFT_KNEE_PITCH];	//LEFT_KNEE_PITCH,		// 12
	qL[4] = qall_temp[LEFT_FOOT_ROLL];	//LEFT_FOOT_ROLL,		// 13
	qL[5] = qall_temp[LEFT_FOOT_PITCH];	//LEFT_FOOT_PITCH,		// 14

	qaR[0] = qall_temp[RIGHT_SHOULDER_PITCH];	//RIGHT_SHOULDER_PITCH,	// 15
	qaR[1] = qall_temp[RIGHT_SHOULDER_ROLL];	//RIGHT_SHOULDER_ROLL,	// 16
	qaR[2] = qall_temp[RIGHT_SHOULDER_YAW];	//RIGHT_SHOULDER_YAW,	// 17
	qaR[3] = qall_temp[RIGHT_ELBOW_PITCH];	//RIGHT_ELBOW_PITCH,	// 18
	qaL[0] = qall_temp[LEFT_SHOULDER_PITCH];	//LEFT_SHOULDER_PITCH,	// 19
	qaL[1] = qall_temp[LEFT_SHOULDER_ROLL];	//LEFT_SHOULDER_ROLL,	// 20
	qaL[2] = qall_temp[LEFT_SHOULDER_YAW];	//LEFT_SHOULDER_YAW,	// 21
	qaL[3] = qall_temp[LEFT_ELBOW_PITCH];	//LEFT_ELBOW_PITCH,		// 22

	this->UpdateRobotState(qL, qR, qW, qaL, qaR, dt, FTSensor);
	//DPRINTF("=============== UpdateRobotState3333333333333=============\n");
}

void WholeBodySensingClass::UpdateRobotState(const std::vector<double> &qL, const std::vector<double> &qR, const std::vector<double> &qW, const std::vector<double> &qaL, const std::vector<double> &qaR, double dt, const std::vector<double> &FTSensor)
{
	this->UpdateRobotStateTemp(qL, qR, qW, qaL, qaR, dt, FTSensor);
	//DPRINTF("=============== UpdateRobotState3333xxxxxxxxxxxx=============\n");
}

void WholeBodySensingClass::UpdateRobotState(const double (&qL)[6], const double (&qR)[6], const double (&qW)[3], const double (&qaL)[4], const double (&qaR)[4], double dt, const std::vector<double> &FTSensor)
{
	this->UpdateRobotStateTemp(qL, qR, qW, qaL, qaR, dt, FTSensor);
	//DPRINTF("=============== UpdateRobotState33333yyyyyyyyyy=============\n");
}

// template <class T>
void WholeBodySensingClass::UpdateHandFT(const std::vector<double> &HandFTSensor)
{
	FT_filterDS = FT_hand_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, HandFTSensor);

	irobot.FT_hand_right << HandFTSensor[0], HandFTSensor[1], HandFTSensor[2], HandFTSensor[3], HandFTSensor[4], HandFTSensor[5];
	irobot.FT_hand_left << HandFTSensor[6], HandFTSensor[7], HandFTSensor[8], HandFTSensor[9], HandFTSensor[10], HandFTSensor[11];
	irobot.FT_hr_filter << FT_filterDS[0], FT_filterDS[1], FT_filterDS[2], FT_filterDS[3], FT_filterDS[4], FT_filterDS[5];
	irobot.FT_hl_filter << FT_filterDS[6], FT_filterDS[7], FT_filterDS[8], FT_filterDS[9], FT_filterDS[10], FT_filterDS[11];

	RBDL::Math::SpatialTransform LHandFrame = irobot._model->getLocalBodyFrame(LEFT_HAND);
	RBDL::Math::SpatialTransform RHandFrame = irobot._model->getLocalBodyFrame(RIGHT_HAND);

	irobot.FT_hand_right_in_hip.segment<3>(0) = RHandFrame.E * irobot.FT_hand_right.segment<3>(0);
	irobot.FT_hand_right_in_hip.segment<3>(3) = RHandFrame.E * irobot.FT_hand_right.segment<3>(3);
	irobot.FT_hand_left_in_hip.segment<3>(0) = LHandFrame.E * irobot.FT_hand_left.segment<3>(0);
	irobot.FT_hand_left_in_hip.segment<3>(3) = LHandFrame.E * irobot.FT_hand_left.segment<3>(3);
	irobot.FT_hr_filter_in_hip.segment<3>(0) = RHandFrame.E * irobot.FT_hr_filter.segment<3>(0);
	irobot.FT_hr_filter_in_hip.segment<3>(3) = RHandFrame.E * irobot.FT_hr_filter.segment<3>(3);
	irobot.FT_hl_filter_in_hip.segment<3>(0) = LHandFrame.E * irobot.FT_hl_filter.segment<3>(0);
	irobot.FT_hl_filter_in_hip.segment<3>(3) = LHandFrame.E * irobot.FT_hl_filter.segment<3>(3);
}

template <class T1, class T2, class T3, class T4>
void WholeBodySensingClass::UpdateRobotStateTemp(const T1 &qL, const T1 &qR, const T2 &qW, const T3 &qaL, const T3 &qaR, double dt, const T4 &FTSensor)
{
  
        irobot.IsReady = true;
	FT_filterDS = FT_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, FTSensor);

	irobot.FT_foot_right << FTSensor[0], FTSensor[1], FTSensor[2], FTSensor[3], FTSensor[4], FTSensor[5];
	irobot.FT_foot_left << FTSensor[6], FTSensor[7], FTSensor[8], FTSensor[9], FTSensor[10], FTSensor[11];
	irobot.FT_fr_filter << FT_filterDS[0], FT_filterDS[1], FT_filterDS[2], FT_filterDS[3], FT_filterDS[4], FT_filterDS[5];
	irobot.FT_fl_filter << FT_filterDS[6], FT_filterDS[7], FT_filterDS[8], FT_filterDS[9], FT_filterDS[10], FT_filterDS[11];

	irobot.LINK[1][0].q << qW[0], qW[1], qW[2]; // Waist Roll, Pitch, Yaw
	irobot.LINK[0][1].q << qL[0], qL[1], qL[2]; // Left Leg Hip Pitch, Roll, Yaw
	irobot.LINK[0][2].q << qR[0], qR[1], qR[2]; // Right Leg Hip Pitch, Roll, Yaw
	irobot.LINK[0][3].q << qaL[0], qaL[1], qaL[2]; // Left Shoulder Pitch, Roll, Yaw
	irobot.LINK[0][4].q << qaR[0], qaR[1], qaR[2]; // Right Shoulder Pitch, Roll, Yaw
	irobot.LINK[1][1].q << 0, qL[3], 0; // Left Knee Pitch
	irobot.LINK[1][2].q << 0, qR[3], 0; // Right Knee Pitch
	irobot.LINK[1][3].q << 0, qaL[3], 0; // Left Elbow Pitch
	irobot.LINK[1][4].q << 0, qaR[3], 0; // Right Elbow Pitch
	irobot.LINK[2][1].q << qL[4], qL[5], 0; // Left Ankle Roll and Pitch
	irobot.LINK[2][2].q << qR[4], qR[5], 0; // Right Ankle Roll and Pitch

	CheckSupportState();

	// ForwardKinematics();
	UpdateRBDL();
	UpdateUsefulVariableAfterFK();
	CalcCop(); // calculate COP in feet

	if (irobot.IsReady) {
		irobot.t += dt;
		Update2Global();	// global estimation can only be done after knowing the contact state ( CalcCop() )
		UpdateFK2Global();
		// UpdateKFC();
		//DPRINTF("=============== updata.... gcom=============\n");
		
	}
	
        if (!irobot.IsReady && irobot.WhichFoot == OnBoth && irobot.IsInitial) {
                irobot.IsReady = true;
                yaw_off = irobot.gYaw;
                irobot.WhichFoot_old = InAir;
                DPRINTF("WBS is Ready!\n");
                Eigen::Vector3d com_local = irobot._model->com_ft;
                DPRINTF("The robot COM is now at %.3f %.3f %.3f m.\n", com_local[0], com_local[1], com_local[2]);
                // DPRINTF("The WBS COM is now at %.3f %.3f %.3f m.\n", irobot.gcom[0], irobot.gcom[1], irobot.gcom[2]);
		//DPRINTF("=============== no update gcom=============\n");
        }
        
//        DPRINTF("=============== update gcom possible=============\n");
// 	DPRINTF("=============== update gcom possible=============:",irobot.IsReady);
// 	DPRINTF("=============== update gcom possible=============:", irobot.WhichFoot);
// 	DPRINTF("=============== update gcom possible=============:", irobot.IsInitial);
	
	
        
//         DPRINTF("t: %.3f, Wh: %d, Wh_old: %d, gYaw: %.2f, yaw_off: %.2f \n", irobot.t, irobot.WhichFoot, irobot.WhichFoot_old, 57.3*irobot.gYaw, 57.3*yaw_off); 
}

const RobotStateClass& WholeBodySensingClass::getRobotState()
{
	return irobot;
}

void WholeBodySensingClass::InitWBS(const double &dt)
{
	if (!irobot.IsInitial) {
		irobot.dt = dt;
		// InitKFC(dt);
		irobot.Init();
	}
}

void WholeBodySensingClass::UpdateRBDL()
{

	irobot._model->q_all_floating.segment(0, 3) = Eigen::Vector3d::Zero();
	irobot._model->q_all_floating[3] = 0 * irobot.gRoll;
	irobot._model->q_all_floating[4] = 0 * irobot.gPitch;
	irobot._model->q_all_floating[5] = 0 * irobot.gYaw;

	irobot._model->UpdateKinematicsOnce();
	switch (irobot.WhichFootRef)
	{
	case OnLFoot:
		irobot._model->base_pos = -irobot._model->lsole.r;
		break;
	case OnRFoot:
		irobot._model->base_pos = -irobot._model->rsole.r;
		break;
	default:
	case OnBoth:
		irobot._model->base_pos = -0.5 * (irobot._model->lsole.r + irobot._model->rsole.r);
		break;
	}

	irobot._model->q_all_floating.segment(0, 3) = irobot._model->base_pos;

	// irobot.InverseDynamics();
	irobot._model->Update(irobot.q_all_mesure);
	// irobot._model->Update(irobot.ReadFromRobot->q);
	RobotModelFK();
}

void WholeBodySensingClass::InitKFC(const double &dt)
{
	Eigen::MatrixXd A(3, 3), H(5, 3), P(3, 3), R(5, 5), Q(3, 3);

	A <<	1.0, dt, 0.5 * dt*dt,
	   0.0, 1.0, dt,
	   0.0, 0.0, 1.0;

	H <<	0.9, 0.0, 0.0,  // WBS gcom
	   0.0, 0.5, 0.0,  // WBS gdcom
	   0.1, 0.0, 0.0,  // IMU gcom
	   0.0, 0.5, 0.0,  // IMU gdcom
	   0.0, 0.0, 1.0;  // IMU gddcom

	P <<	1.0, 0.0, 0.0,
	   0.0, 1.0, 0.0,
	   0.0, 0.0, 1.0;

	R <<	0.01, 0.0, 0.0, 0.0, 0.0,
	   0.0, 0.1, 0.0, 0.0, 0.0,
	   0.0, 0.0, 1.0, 0.0, 0.0,
	   0.0, 0.0, 0.0, 0.1, 0.0,
	   0.0, 0.0, 0.0, 0.0, 0.001;

	Q <<	0.5, 0.0, 0.0,
	   0.0, 0.3, 0.0,
	   0.0, 0.0, 0.5;

	xKFC->Init(A, H, P, R, Q);
	yKFC->Init(A, H, P, R, Q);
	zKFC->Init(A, H, P, R, Q);
}

void WholeBodySensingClass::UpdateKFC()
{
	Eigen::VectorXd Zx(5), Zy(5), Zz(5);
	Zx << irobot.gftcom_raw[0], irobot.gdftcom_raw[0], irobot.IMU_odometry_raw[0], irobot.IMU_dodometry_raw[0], irobot.IMU_ddodometry_raw[0];
	Zy << irobot.gftcom_raw[1], irobot.gdftcom_raw[1], irobot.IMU_odometry_raw[1], irobot.IMU_dodometry_raw[1], irobot.IMU_ddodometry_raw[1];
	Zz << irobot.gftcom_raw[2], irobot.gdftcom_raw[2], irobot.IMU_odometry_raw[2], irobot.IMU_dodometry_raw[2], irobot.IMU_ddodometry_raw[2];

	Eigen::Vector3d Xstate = xKFC->Update(Zx);
	Eigen::Vector3d Ystate = xKFC->Update(Zy);
	Eigen::Vector3d Zstate = xKFC->Update(Zz);

	irobot.KFC_gcom << Xstate[0], Ystate[0], Zstate[0];
	irobot.KFC_gdcom << Xstate[1], Ystate[1], Zstate[1];
	irobot.KFC_gddcom << Xstate[2], Ystate[2], Zstate[2];
}

//Eigen::Vector3d WholeBodySensingClass::com()
//{
//	return irobot.com;
//}
//
//Eigen::Vector3d WholeBodySensingClass::gcom()
//{
//	return irobot.gcom;
//}

void WholeBodySensingClass::UpdateRef(const std::vector<double> &ComRef, const std::vector<double> &LftRef, const std::vector<double> &RftRef)
{
	for (unsigned int i = 0; i < 3; i++) {
		irobot.ComRef[i] = ComRef[i];
		irobot.LftRef[i] = LftRef[i];
		irobot.RftRef[i] = RftRef[i];
	}
}

void WholeBodySensingClass::UpdateRef(const Eigen::Vector3d  &ComRef, const Eigen::Vector3d &LftRef, const Eigen::Vector3d &RftRef, const Eigen::Vector3d &ZmpRef, const Eigen::Vector3d &FzRef)
{
	for (unsigned int i = 0; i < 3; i++) {
		irobot.ComRef[i] = ComRef[i];
		irobot.LftRef[i] = LftRef[i];
		irobot.RftRef[i] = RftRef[i];
		irobot.ZmpRef[i] = ZmpRef[i];
		irobot.FzRef[i] = FzRef[i];
	}
}

void WholeBodySensingClass::UpdateRef(const double &delay, const Eigen::Vector3d &ComRef, const Eigen::Vector3d &LftRef, const Eigen::Vector3d &RftRef, const Eigen::Vector3d &ZmpRef)
{
	Eigen::Vector3d FzRef(0.5, 0.5, 0);
	this->UpdateRef(delay, ComRef, LftRef, RftRef, ZmpRef, FzRef);
}

void WholeBodySensingClass::UpdateRef(const double &delay, const Eigen::Vector3d &ComRef, const Eigen::Vector3d &LftRef, const Eigen::Vector3d &RftRef, const Eigen::Vector3d &ZmpRef, const Eigen::Vector3d &FzRef)
{
	std::vector<double> ComRefVec(3, 0.0), LftRefVec(3, 0.0), RftRefVec(3, 0.0), ZmpRefVec(3, 0.0), FzRefVec(3, 0.0);
	for (unsigned int i = 0; i < 3; i++) {
		ComRefVec[i] = ComRef[i];
		LftRefVec[i] = LftRef[i];
		RftRefVec[i] = RftRef[i];
		ZmpRefVec[i] = ZmpRef[i];
		FzRefVec[i] = FzRef[i];
	}
	this->UpdateRef(delay, ComRefVec, LftRefVec, RftRefVec, ZmpRefVec, FzRefVec);
}

void WholeBodySensingClass::UpdateRef(const std::vector<double> &ComRef, const std::vector<double> &LftRef, const std::vector<double> &RftRef, const std::vector<double> &ZmpRef)
{
	std::vector<double> FzRef(3, 0.5);
	this->UpdateRef(0.0, ComRef, LftRef, RftRef, ZmpRef, FzRef);
}

void WholeBodySensingClass::UpdateRef(const double &delay, const std::vector<double> &ComRef, const std::vector<double> &LftRef, const std::vector<double> &RftRef, const std::vector<double> &ZmpRef)
{
	std::vector<double> FzRef(3, 0.5);
	this->UpdateRef(delay, ComRef, LftRef, RftRef, ZmpRef, FzRef);
}

void WholeBodySensingClass::UpdateRef(const double &delay, const std::vector<double> &ComRef, const std::vector<double> &LftRef, const std::vector<double> &RftRef, const std::vector<double> &ZmpRef, const std::vector<double> &FzRef)
{

//     DPRINTF("3\n");
//         if(delay == 0.0){
//             UpdateRef(ComRef, LftRef, RftRef, ZmpRef, FzRef);
//         }
//         else{
	if (!IsRefDelayInit) {
		RefDelayQueue.resize((unsigned int)(round((delay) / irobot.dt + 0.5)), std::vector<std::vector<double> >(5, std::vector<double>(3, 0)));
		AngRefDelayQueue.resize((unsigned int)(round((delay) / irobot.dt + 0.5)), std::vector<std::vector<double> >(3, std::vector<double>(3, 0)));
		RelAngRefDelayQueue.resize((unsigned int)(round((delay) / irobot.dt + 0.5)), std::vector<std::vector<double> >(2, std::vector<double>(3, 0)));
		HandRefDelayQueue.resize((unsigned int)(round((delay) / irobot.dt + 0.5)), std::vector<std::vector<double> >(2, std::vector<double>(3, 0)));
		tempRef.resize(5, std::vector<double>(3, 0));
		IsRefDelayInit = true;
	}
	if (IsRefDelayInit) {
		tempRef[0] = ComRef;
		tempRef[1] = LftRef;
		tempRef[2] = RftRef;
		tempRef[3] = ZmpRef;
		tempRef[4] = FzRef;
		RefDelayQueue.push_back(tempRef);
		RefDelayQueue.pop_front();
		for (unsigned int i = 0; i < 3; i++) {
			irobot.ComRef[i] = RefDelayQueue[0][0][i];
			irobot.LftRef[i] = RefDelayQueue[0][1][i];
			irobot.RftRef[i] = RefDelayQueue[0][2][i];
			irobot.ZmpRef[i] = RefDelayQueue[0][3][i];
			irobot.FzRef[i] = RefDelayQueue[0][4][i];
			irobot.dComRef[i] = (irobot.ComRef[i] - oldRef[0][i]) / irobot.dt;
			irobot.dLftRef[i] = (irobot.LftRef[i] - oldRef[1][i]) / irobot.dt;
			irobot.dRftRef[i] = (irobot.RftRef[i] - oldRef[2][i]) / irobot.dt;
			irobot.dZmpRef[i] = (irobot.ZmpRef[i] - oldRef[3][i]) / irobot.dt;
			irobot.dFzRef[i] = (irobot.FzRef[i] - oldRef[4][i]) / irobot.dt;
		}
		irobot.ddComRef = (irobot.dComRef - olddRef[0]) / irobot.dt;
		irobot.ddLftRef = (irobot.dLftRef - olddRef[1]) / irobot.dt;
		irobot.ddRftRef = (irobot.dRftRef - olddRef[2]) / irobot.dt;
		irobot.ddZmpRef = (irobot.dZmpRef - olddRef[3]) / irobot.dt;
		this->oldRef = RefDelayQueue[0];
		this->olddRef[0] = irobot.dComRef;
		this->olddRef[1] = irobot.dLftRef;
		this->olddRef[2] = irobot.dRftRef;
		this->olddRef[3] = irobot.dZmpRef;
	}
//         }
}

void WholeBodySensingClass::UpdateHandPosRef(const Eigen::Vector3d &LhdRef, const Eigen::Vector3d &RhdRef)
{
	std::vector<double> LhdRefVec(3, 0.0), RhdRefVec(3, 0.0);
	for (unsigned int i = 0; i < 3; i++) {
		LhdRefVec[i] = LhdRef[i];
		RhdRefVec[i] = RhdRef[i];
	}
	this->UpdateHandPosRef(LhdRefVec, RhdRefVec);
}

void WholeBodySensingClass::UpdateHandPosRef(const std::vector<double> &LhdRef, const std::vector<double> &RhdRef)
{
	if (IsRefDelayInit) {
		std::vector<std::vector<double> > tempRef;
		tempRef.push_back(LhdRef);
		tempRef.push_back(RhdRef);
		HandRefDelayQueue.push_back(tempRef);
		HandRefDelayQueue.pop_front();
		for (unsigned int i = 0; i < 3; i++) {
			irobot.LhdRef[i] = HandRefDelayQueue[0][0][i];
			irobot.RhdRef[i] = HandRefDelayQueue[0][1][i];
			irobot.dLhdRef[i] = (irobot.LhdRef[i] - oldHandRef[0][i]) / irobot.dt;
			irobot.dRhdRef[i] = (irobot.RhdRef[i] - oldHandRef[1][i]) / irobot.dt;
		}
		irobot.ddLhdRef = (irobot.dLhdRef - olddHandRef[0]) / irobot.dt;
		irobot.ddRhdRef = (irobot.dRhdRef - olddHandRef[1]) / irobot.dt;
		this->oldHandRef = HandRefDelayQueue[0];
		this->olddHandRef[0] = irobot.dLhdRef;
		this->olddHandRef[1] = irobot.dRhdRef;
	}
	else {
		DPRINTF("\n\n\n\n\n\nplease initialize HandRefDelayQueue\n\n\n\n\n\n");
	}
}



// void WholeBodySensingClass::UpdateAngRef(const Eigen::Vector3d &OriComRef, const Eigen::Vector3d &OriLftRef, const Eigen::Vector3d &OriRftRef)
// {
// 	std::vector<double> ComRefVec, LftRefVec, RftRefVec;
// 	for(unsigned int i=0; i<3; i++){
// 		ComRefVec.push_back(OriComRef[i]);
// 		LftRefVec.push_back(OriLftRef[i]);
// 		RftRefVec.push_back(OriRftRef[i]);
// 	}
// 	this->UpdateAngRef(ComRefVec,LftRefVec,RftRefVec);
// }

// void WholeBodySensingClass::UpdateAngRef(const std::vector<double> &OriComRef, const std::vector<double> &OriLftRef, const std::vector<double> &OriRftRef)
// {
//     if(IsRefDelayInit){
//         std::vector<std::vector<double> > tempRef;
//         tempRef.push_back(OriComRef);
//         tempRef.push_back(OriLftRef);
//         tempRef.push_back(OriRftRef);
//         AngRefDelayQueue.push_back(tempRef);
//         AngRefDelayQueue.pop_front();
//         for(unsigned int i=0; i<3; i++){
//             irobot.OriComRef[i] = AngRefDelayQueue[0][0][i];
//             irobot.OriLftRef[i] = AngRefDelayQueue[0][1][i];
//             irobot.OriRftRef[i] = AngRefDelayQueue[0][2][i];
// 			irobot.dOriComRef[i] = (irobot.OriComRef[i] - oldAngRef[0][i])/irobot.dt;
// 			irobot.dOriLftRef[i] = (irobot.OriLftRef[i] - oldAngRef[1][i])/irobot.dt;
// 			irobot.dOriRftRef[i] = (irobot.OriRftRef[i] - oldAngRef[2][i])/irobot.dt;
//         }
// 		this->oldAngRef = AngRefDelayQueue[0];
//     }
// }

void WholeBodySensingClass::UpdateRelAngRef(const Eigen::Matrix3d &OriComRefL, const Eigen::Matrix3d &OriComRefR, const Eigen::Matrix3d &OriLftRef, const Eigen::Matrix3d &OriRftRef)
{
	Eigen::Vector3d OriLftCom, OriRftCom;
	Eigen::Matrix3d LftCom = OriComRefL.transpose() * OriLftRef;
	Eigen::Matrix3d RftCom = OriComRefR.transpose() * OriRftRef;

	OriLftCom[0] = atan2(LftCom(2, 1), LftCom(2, 2));
	OriLftCom[1] = asin(-LftCom(2, 0));
	OriLftCom[2] = atan2(LftCom(1, 0), LftCom(0, 0));

	OriRftCom[0] = atan2(RftCom(2, 1), RftCom(2, 2));
	OriRftCom[1] = asin(-RftCom(2, 0));
	OriRftCom[2] = atan2(RftCom(1, 0), RftCom(0, 0));

	std::vector<double> OriLftToComRef(3, 0.0), OriRftToComRef(3, 0.0);
	for (unsigned int i = 0; i < 3; i++) {
		OriLftToComRef[i] = OriLftCom[i];
		OriRftToComRef[i] = OriRftCom[i];
	}
	this->UpdateRelAngRef(OriLftToComRef, OriRftToComRef);


	Eigen::Matrix3d S_L = (LftCom - LftCom_old) / irobot.dt * LftCom.transpose();
	Eigen::Matrix3d S_R = (RftCom - RftCom_old) / irobot.dt * RftCom.transpose();

	irobot.dOriLftToComRef2(0) = S_L(2, 1);
	irobot.dOriLftToComRef2(1) = S_L(0, 2);
	irobot.dOriLftToComRef2(2) = S_L(1, 0);

	irobot.dOriRftToComRef2(0) = S_R(2, 1);
	irobot.dOriRftToComRef2(1) = S_R(0, 2);
	irobot.dOriRftToComRef2(2) = S_R(1, 0);

	LftCom_old = LftCom;
	RftCom_old = RftCom;


}

void WholeBodySensingClass::UpdateRelAngRef(const std::vector<double> &OriLftToComRef, const std::vector<double> &OriRftToComRef)
{
	if (IsRefDelayInit) {
		std::vector<std::vector<double> > tempRef;
		tempRef.push_back(OriLftToComRef);
		tempRef.push_back(OriRftToComRef);
		RelAngRefDelayQueue.push_back(tempRef);
		RelAngRefDelayQueue.pop_front();
		for (unsigned int i = 0; i < 3; i++) {
			irobot.OriLftToComRef[i] = RelAngRefDelayQueue[0][0][i];
			irobot.OriRftToComRef[i] = RelAngRefDelayQueue[0][1][i];
			irobot.dOriLftToComRef[i] = (irobot.OriLftToComRef[i] - oldRelAngRef[0][i]) / irobot.dt;
			irobot.dOriRftToComRef[i] = (irobot.OriRftToComRef[i] - oldRelAngRef[1][i]) / irobot.dt;
		}
		this->oldRelAngRef = RelAngRefDelayQueue[0];
	}
}

void WholeBodySensingClass::SaveData(const bool &IsSave, const std::vector<double> &rawdata)
{
	if (IsSave) {
		if (!irobot.IsSavedata && irobot.IsInitial) {
			irobot.IsSavedata = IsSave;
			irobot.savedata.reserve(int(60 / irobot.dt));
		}
		if (irobot.IsSavedata) {
			irobot.savedata.push_back(rawdata);
		}
	}
}

//template void WholeBodySensingClass::UpdateRobotStateTemp<std::vector<double>, std::vector<double>, std::vector<double>,std::vector<double>>(const std::vector<double> &,const std::vector<double> &,const std::vector<double> &,const std::vector<double> &,const std::vector<double> &,double, const std::vector<double> &);





