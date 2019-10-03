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

//	for(int i=0; i<3; i++){
//		mKd.push_back(0);
//		mBd.push_back(0);
//		A.push_back(0);
//		mEqui0.push_back(0);
//		mEnable.push_back(0);
//		deltaHip.push_back(0);
//	}
};

void StabilizerClass::Kd(double x_stiff, double y_stiff, double z_stiff)
{
	if ( x_stiff < 0 || y_stiff < 0 || z_stiff < 0) {
		DPRINTF("Error!!! All Stiffness Should Be Bigger Than 0!");
	}
	else {
		mKd[0] = x_stiff;
		mKd[1] = y_stiff;
		mKd[2] = z_stiff;
	}
}

void StabilizerClass::Bd(double x_damp, double y_damp, double z_damp)
{
	if ( x_damp < 0 || y_damp < 0 || z_damp < 0) {
		DPRINTF("Error!!! All Damping Should Be Bigger Than 0!");
	}
	else {
		mBd[0] = x_damp;
		mBd[1] = y_damp;
		mBd[2] = z_damp;
	}
}

void StabilizerClass::Enable(int x_mEnable, int y_mEnable, int z_mEnable)
{
	if (x_mEnable != 0) {
		mEnable[0] = 1;
	}
	else {
		mEnable[0] = 0;
	}

	if (y_mEnable != 0) {
		mEnable[1] = 1;
	}
	else {
		mEnable[1] = 0;
	}

	if (z_mEnable != 0) {
		mEnable[2] = 1;
	}
	else {
		mEnable[2] = 0;
	}

	//mEnable[0]=x_mEnable;
	//mEnable[1]=y_mEnable;
	//mEnable[2]=z_mEnable;
}

void StabilizerClass::Equi0(double x_equi, double y_equi)
{
	//if(x_equi>0.5*RobotParaClass::FOOT_LENGTH()){
	//	x_equi = 0.5*RobotParaClass::FOOT_LENGTH();
	//}
	//else if(x_equi<-0.5*RobotParaClass::FOOT_LENGTH()){
	//	x_equi = -0.5*RobotParaClass::FOOT_LENGTH();
	//}

	//if(y_equi>HALF_HIP_WIDTH){
	//	y_equi = HALF_HIP_WIDTH;
	//}
	//else if(y_equi<-HALF_HIP_WIDTH){
	//	y_equi = -HALF_HIP_WIDTH;
	//}

	mEqui0[0] = x_equi;
	mEqui0[1] = y_equi;
}

void StabilizerClass::VerticalScale(double input)
{
	if (input > 1.2) {
		input = 1.2;
		DPRINTF("Too big VerticalScale!!! Reset it to %f!", input);
	}
	else if (input < 0.8) {
		input = 0.8;
		DPRINTF("Too small VerticalScale!!! Reset it to %f!", input);
	}
	mVerticalScale = input;
}

//double StabilizerClass::VerticalScale()
//{
//	return mVerticalScale;
//}

//void StabilizerClass::zc(double input)
//{
//	mZc=input;
//}

//double StabilizerClass::zc()
//{
//	return mZc;
//}

const Eigen::Vector3d& StabilizerClass::StabilizerCart(const RobotStateClass &irobot) //pass deltahip out
{
	deltaHip = StabilizerCartTemp(irobot, irobot.LftRef, irobot.RftRef);
	return deltaHip;
}

//template <>
const Eigen::Vector3d& StabilizerClass::StabilizerCart(const RobotStateClass &irobot, const std::vector<double> &LftRef, const std::vector<double> &RftRef) //pass deltahip out
{
	deltaHip = StabilizerCartTemp(irobot, LftRef, RftRef);
	return deltaHip;
}

//template <>
const Eigen::Vector3d& StabilizerClass::StabilizerCart(const RobotStateClass &irobot, const Eigen::Vector3d &LftRef, const Eigen::Vector3d &RftRef) //pass deltahip out
{
	deltaHip = StabilizerCartTemp(irobot, LftRef, RftRef);
	return deltaHip;
}

template <class T>
const Eigen::Vector3d& StabilizerClass::StabilizerCartTemp(const RobotStateClass &irobot, const T &LftRef, const T &RftRef) //pass deltahip out
{
	// if ((mEnable[0] + mEnable[1] + mEnable[2]) == 0) {
	// 	if (!mEnable[0]) {
	// 		double Tf = 1.0 / 3.0;
	// 		deltaHip[0] = (Tf * deltaHip[0]) / (Tf + irobot.dt);
	// 	}
	// 	if (!mEnable[1]) {
	// 		double Tf = 1.0 / 3.0;
	// 		deltaHip[1] = (Tf * deltaHip[1]) / (Tf + irobot.dt);
	// 	}
	// 	if (!mEnable[2]) {
	// 		double Tf = 1.0 / 3.0;
	// 		deltaHip[2] = (Tf * deltaHip[2]) / (Tf + irobot.dt);
	// 	}
	// 	return deltaHip;
	// }

	/*  torque created by GRF */
	// double torque_x = 0; // torque_x
	// double torque_y = 0; // torque_y
	double xd = 0;
	double yd = 0;
	double x0 = 0;
	double y0 = 0;
	// double dT = irobot.dt;
	double m = irobot.m;
	mass = irobot.m;

	Eigen::Vector3d lft_Force, rft_Force, txy_act;
	//double irobotmass = (irobot.m /*- 2*FOOT_MASS*/);
	//mg << 0,0, -irobotmass*RobotParaClass::G();

	Eigen::Vector3d gGeomCenLft = irobot.gGeomCenLft;
	Eigen::Vector3d gGeomCenRft = irobot.gGeomCenRft;

	gGeomCenLft[2] = 0;
	gGeomCenRft[2] = 0;

	lft_Force = irobot.IMU_rel * irobot.Rot_lft2hip * irobot.FT_foot_left.head(3);
	rft_Force = irobot.IMU_rel * irobot.Rot_rft2hip * irobot.FT_foot_right.head(3);
	txy_act = irobot.IMU_rel * (irobot.Rot_lft2hip * irobot.FT_foot_left.tail(3)
	                            + irobot.Rot_rft2hip * irobot.FT_foot_right.tail(3))
	          + (gGeomCenLft).cross(lft_Force)
	          + (gGeomCenRft).cross(rft_Force) ;

//        double a1 = (irobot.IMU_rel * (irobot.Rot_lft2hip * irobot.FT_foot_left.tail(3)))[1];
//        double a2 = (irobot.IMU_rel * (irobot.Rot_rft2hip * irobot.FT_foot_right.tail(3)))[1];
//        double a3 = ((gGeomCenLft).cross(lft_Force))[1];
//        double a4 = ((gGeomCenRft).cross(rft_Force))[1];

	//	/*  compute the torque created by the GRF */
//    Eigen::Vector3d t_overall= irobot.FT_foot_left.tail(3)+irobot.FT_foot_right.tail(3);
//       txy_act[0] = t_overall[0]+(irobot.FT_foot_left[2]-irobot.FT_foot_right[2])*(LftRef[1]-RftRef[1])*0.5;
//       txy_act[1] = t_overall[1]-(irobot.FT_foot_left[2]-irobot.FT_foot_right[2])*(LftRef[0]-RftRef[0])*0.5;
//    txy_act[0] = t_overall[0]+(lft_Force[2]-rft_Force[2])*(LftRef[1]-RftRef[1])*0.5;
//    txy_act[1] = t_overall[1]-(lft_Force[2]-rft_Force[2])*(LftRef[0]-RftRef[0])*0.5;

	Eigen::Vector3d GeomCenRef = 0.5 * (irobot.LftRef + irobot.RftRef);
	GeomCenRef[2] = 0.0;

	//Eigen::Vector3d ZMPLocalRef = irobot.ZmpRef - GeomCenRef;
	//double td_x1 = irobot.m*RobotParaClass::G()*(ZMPLocalRef[1]);
	//double td_y1 = irobot.m*RobotParaClass::G()*(ZMPLocalRef[0]);


	Eigen::Vector3d ComLocalRef = irobot.ComRef - GeomCenRef;
	Eigen::Vector3d cZmpRef = - irobot.ComRef + irobot.ZmpRef;

	//txy_act	  = irobot.IMU_rel.transpose()*txy_act	;
	//ComLocalRef = irobot.IMU_abs.transpose()*ComLocalRef;
	//cZmpRef = irobot.IMU_abs.transpose()*cZmpRef;

	td_x = m * RobotParaClass::G() * (cZmpRef[1] + ComLocalRef[1] + yd_old);
	td_y = m * RobotParaClass::G() * (cZmpRef[0] + ComLocalRef[0] + xd_old);

	/*-------------- here is the main law 1 ----------------*/
	/*------------ stabilizer internal filter --------------*/
	std::vector<double> t_xy(2, 0);

	t_xy[0] = txy_act[0] - td_x;
	t_xy[1] = txy_act[1] + td_y;
// 	t_xy = txy_MeanFilter->applyFilter(1, t_xy);

	t_xy = txy_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, t_xy);

//    if(!IsInitF){
//        tx_Filter->butterworth(irobot.dt, FilterCutOff, N_ButWth);
//        ty_Filter->butterworth(irobot.dt, FilterCutOff, N_ButWth);
//        IsInitF = true;
//    }

//    double t_x = txy_act[0] - td_x;
//    double t_y = txy_act[1] - td_y;

//    t_x = tx_Filter->applyFilter(t_x);
//    t_y = ty_Filter->applyFilter(t_y);

//    torque_x=t_x;
//       torque_y=t_y;

	//t_xy = txy_MeanFilter->applyFilter(30, t_xy);

	torque_x = t_xy[0];
	torque_y = t_xy[1];

	ReactStepping(torque_x, torque_y);

	// sagittal
	x0 = mEqui0[0];
	//A[0]=mKd[0]*x0+(mKd[0]-Ks[0])/Ks[0]*torque_y/mZc+(mBd[0])/Ks[0]*dty/mZc-m*RobotParaClass::G()/mZc*(xd_old-torque_y/mZc/Ks[0]);
	//A[0]=mKd[0]*x0-torque_y/mZc-m*RobotParaClass::G()/mZc*(ComLocalRef[0] + xd_old);
	A[0] = mKd[0] * x0 - torque_y / mZc;
	xd = (dT / (mKd[0] * dT + mBd[0])) * A[0] + mBd[0] / (mKd[0] * dT + mBd[0]) * xd_old;

	// lateral
	y0 = mEqui0[1];
	//A[1]=mKd[1]*y0+torque_x/mZc-m*RobotParaClass::G()/mZc*(ComLocalRef[1] + yd_old);
	//A[1]=mKd[1]*y0-(mKd[1]-Ks[1])/Ks[1]*torque_x/mZc-(mBd[1])/Ks[1]*dtx/mZc-m*RobotParaClass::G()/mZc*(yd_old-torque_x/mZc/Ks[1]);
	A[1] = mKd[1] * y0 + torque_x / mZc;
	yd = (dT / (mKd[1] * dT + mBd[1])) * A[1] + mBd[1] / (mKd[1] * dT + mBd[1]) * yd_old;

	std::vector<double> Fext(1, 0);
	// Fext[0]  = 1-(irobot.FT_fl_filter[2]+irobot.FT_fr_filter[2])/(mVerticalScale*m*RobotParaClass::G()); // include the bias force to counterbalance gravity
	Fext[0]  = 1 - (irobot.FT_foot_left[2] + irobot.FT_foot_right[2]) / (mVerticalScale * m * RobotParaClass::G()); // include the bias force to counterbalance gravity
	// Fext[0]  = (mVerticalScale * m * RobotParaClass::G()) - (irobot.FT_foot_left[2] + irobot.FT_foot_right[2]); // include the bias force to counterbalance gravity
	// Fext = Fz_MeanFilter->applyFilter(1, Fext);
// 	Fext = FextZ_hip_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, Fext);
	if (mEnable[0]) {
		deltaHip[0] = irobot.scale * xd;
	}
	else {
		double Tf = 1.0 / 3.0;
		deltaHip[0] = (Tf * deltaHip[0]) / (Tf + irobot.dt);
	}

	if (mEnable[1]) {
		deltaHip[1] = irobot.scale * yd;
	}
	else {
		double Tf = 1.0 / 3.0;
		deltaHip[1] = (Tf * deltaHip[1]) / (Tf + irobot.dt);
	}

	if (mEnable[2]) {
		deltaHip[2] = irobot.scale * (Fext[0] * dT + mBd[2] * zd_old) / (mKd[2] * dT + mBd[2]); //delta hip
	}
	else {
		double Tf = 1.0 / 3.0;
		deltaHip[2] = (Tf * deltaHip[2]) / (Tf + irobot.dt);
	}

	/* --------------- */
	// put some threshold for the hip position modification
	clamp(deltaHip[0], -0.1, 0.1);
	clamp(deltaHip[1], -0.15, 0.15);
	clamp(deltaHip[2], -0.1, 0.005);

	xd_old = deltaHip[0];
	yd_old = deltaHip[1];
	zd_old = deltaHip[2];

	tx_old = torque_x;
	ty_old = torque_y;

	// COUT("deltaHip",deltaHip[0],deltaHip[1]);

	return deltaHip;
}

const std::vector<double>& StabilizerClass::StabilizerCartZ(const bool enable, const double K, const double B, const RobotStateClass & irobot)
{
	double dT = irobot.dt;
	double m = irobot.m;
	// double Fzl_ref = irobot.FzRef[0] * m * RobotParaClass::G() * mVerticalScale;
	// double Fzr_ref = irobot.FzRef[1] * m * RobotParaClass::G() * mVerticalScale;
	double Fzl_ref = irobot.FzRef[0];
	double Fzr_ref = irobot.FzRef[1];

	std::vector<double> Fext(2, 0);
	Fext[0]  = irobot.FT_foot_left[2] / (m * RobotParaClass::G() * mVerticalScale) - Fzl_ref;
	Fext[1]  = irobot.FT_foot_right[2] / (m * RobotParaClass::G() * mVerticalScale) - Fzr_ref;
	// Fext = FzFT_MeanFilter->applyFilter(1, Fext);
	Fext = FzFT_Filter->ApplyButterworth(irobot.dt, FilterCutOff, N_ButWth, Fext);
	// // Fext = FzFT_Filter->ApplyButterworth(irobot.dt, 5, 3, Fext);

	if (enable) {
		deltaFtZ[0] = irobot.scale * (Fext[0] * dT + B * deltaFtZ_old[0]) / (K * dT + B);
		deltaFtZ[1] = irobot.scale * (Fext[1] * dT + B * deltaFtZ_old[1]) / (K * dT + B);
	}
	else {
		double Tf = 1.0 / 3.0;
		deltaFtZ[0] = (Tf * deltaFtZ[0]) / (Tf + irobot.dt);
		deltaFtZ[1] = (Tf * deltaFtZ[1]) / (Tf + irobot.dt);
	}

	double max = 0.1;
	double min = -0.005;

	clamp(deltaFtZ[0], min, max);
	clamp(deltaFtZ[1], min, max);

	if (enable) {
		DPRINTF("deltaFtZ[0]: %.4f, deltaFtZ[1]: %.4f, Fext[0]: %.4f, Fext[1]: %.4f\n", deltaFtZ[0], deltaFtZ[1], Fext[0], Fext[1]);
	}

	//deltaFtZ = FzFT_Filter->ApplyButterworth(irobot.dt, 10.0, N_ButWth, deltaFtZ);
	deltaFtZ_old[0] = deltaFtZ[0];
	deltaFtZ_old[1] = deltaFtZ[1];
	return deltaFtZ;
}

void StabilizerClass::ReactStepping(const double & tq_x, const double & tq_y)
{
	double fx =  - tq_y / mZc;
	double fy =  tq_x / mZc;

	double acc_x = fx / mass;
	double acc_y = fy / mass;

	mdeltaPos[0] = acc_x * mZc / 9.806;
	mdeltaPos[1] = acc_y * mZc / 9.806;

	// COUT(mdeltaPos[0], mdeltaPos[1]);

}

void StabilizerClass::LandingFtZctrl(const bool& enable, const RobotStateClass& irobot, Eigen::Vector3d& lft_pos_delta, Eigen::Vector3d& rft_pos_delta)
{
	double Fz_ref_diff = irobot.FzRef[0] - irobot.FzRef[1];
	// double Fzl_msr = irobot.Fzl;
	// double Fzr_msr = irobot.Fzr;
	// double Fz_msr_diff = (Fzl_msr - Fzr_msr) / (Fzl_msr + Fzr_msr);
	// if ((Fzl_msr + Fzr_msr) < 30.0) {
	// 	Fz_msr_diff = Fz_ref_diff;
	// }

	Eigen::Vector3d lft_Force = irobot.IMU_rel * irobot.Rot_lft2hip * irobot.FT_foot_left.head(3);
	Eigen::Vector3d rft_Force = irobot.IMU_rel * irobot.Rot_rft2hip * irobot.FT_foot_right.head(3);

	if (lft_Force[2] < 0.0) lft_Force[2] = 0.0;
	if (rft_Force[2] < 0.0) rft_Force[2] = 0.0;
	double Fz_total = lft_Force[2] + rft_Force[2];
	double eps = 1.0;
	if (Fz_total <= eps) Fz_total = eps;
	lft_Force /= Fz_total;
	rft_Force /= Fz_total;
	Ft_force_diff_ref << 0.0, 0.0, Fz_ref_diff;
	Ft_force_diff_msr = lft_Force - rft_Force;
	// Ft_force_diff_msr[2] = Fz_msr_diff;

	Eigen::Vector3d damping(4, 4, irobot.RobotPara().kd_debug);
	Eigen::Vector3d settle_time(1.5, 1.5, 1.5);

	double deltaFtPos_l, deltaFtPos_r;

	if (enable) {
		// if (irobot.FzRef[0] < 0.35 || irobot.FzRef[1] < 0.35) {
		// if (irobot.WhichFootRef == OnLFoot || irobot.WhichFootRef == OnRFoot) {
		if (irobot.WhichFoot != InAir) {
			deltaFtPos = irobot.scale * FootPosZ_ctrl->update(damping, Ft_force_diff_ref, Ft_force_diff_msr, settle_time, dT);

			if (irobot.WhichFootRef == OnLFoot) {
				deltaFtPos_l = irobot.scale * LFootPosZ_ctrl->update(2.0, irobot.FzRef[0], lft_Force[2], 0.2, dT);
			}
			else {
				deltaFtPos_l = irobot.scale * LFootPosZ_ctrl->update(5.0, irobot.FzRef[0], lft_Force[2], 2.0, dT);
			}

			if (irobot.WhichFootRef == OnRFoot) {
				deltaFtPos_r = irobot.scale * RFootPosZ_ctrl->update(2.0, irobot.FzRef[1], rft_Force[2], 0.2, dT);
			}
			else {
				deltaFtPos_r = irobot.scale * RFootPosZ_ctrl->update(5.0, irobot.FzRef[1], rft_Force[2], 2.0, dT);
			}
		}
	}
	else {
		double Tf = 1.0 / 3.0;
		deltaFtPos = (Tf) / (Tf + dT) * deltaFtPos;
		FootPosZ_ctrl->reset(deltaFtPos);
	}

	clamp(deltaFtPos[0], -0.03, 0.03);
	clamp(deltaFtPos[1], -0.03, 0.03);
	clamp(deltaFtPos[2], -0.04, 0.04);

	lft_pos_delta = -0.5 * deltaFtPos;
	rft_pos_delta = 0.5 * deltaFtPos;
	// lft_pos_delta[2] = -0.5 * deltaFtPos_l + 0.5 * deltaFtPos_r;
	// rft_pos_delta[2] = -0.5 * deltaFtPos_r + 0.5 * deltaFtPos_l;

	// COUT(deltaFtPos_l, deltaFtPos_r);


	// DPRINTF("enable=%d, FzlRef = %.2f, lft_Force[2] = %.3f, lft_pos_delta[2] = %.3f \n", enable, irobot.FzRef[0], lft_Force[2], lft_pos_delta[2]);
//         DPRINTF("enable=%d, lft_Force[2] = %.3f, rft_Force[2] = %.3f \n", enable, irobot.FT_foot_left[2], irobot.FT_foot_right[2]);
}

double StabilizerClass::LandingFtZctrl(double damping, double Fz_ref_diff, double Fzl_msr, double Fzr_msr, double settle_time)
{
	if (Fzl_msr < 0.0) Fzl_msr = 0.0;
	if (Fzr_msr < 0.0) Fzr_msr = 0.0;
	double Fz_msr_diff = (Fzl_msr - Fzr_msr) / (Fzl_msr + Fzr_msr);
	if ((Fzl_msr + Fzr_msr) < 30.0) {
		Fz_msr_diff = Fz_ref_diff;
	}

	double deltaFtz = FootPosZ_ctrl->update(damping, Fz_ref_diff, Fz_msr_diff, settle_time, dT);
	clamp(deltaFtz, -0.05, 0.05);

	// COUT("Fzl", Fzl_msr, "Fzr", Fzr_msr);
	// COUT("deltaFtz", deltaFtz, "ref", Fz_ref_diff, "msr", Fz_msr_diff);
	return deltaFtz;
}

void StabilizerClass::LandingFtOctrl(const bool& enable, const RobotStateClass& irobot, Eigen::Matrix3d& deltaFtOri_left, Eigen::Matrix3d& deltaFtOri_right)
{
	// Eigen::Vector3d fl_ref(0.0, 0.0, irobot.FzRef[0]);
	// Eigen::Vector3d fr_ref(0.0, 0.0, irobot.FzRef[1]);
	Eigen::Vector3d fl_ref(0.0, 0.0, irobot.FzRef[0]*irobot.m);
	Eigen::Vector3d fr_ref(0.0, 0.0, irobot.FzRef[1]*irobot.m);

	Eigen::Vector3d tau_tot_ref = -(irobot.LftRef - irobot.ZmpRef).cross(fl_ref) - (irobot.RftRef - irobot.ZmpRef).cross(fr_ref);

	Eigen::Vector3d lft_tau_ref(0.0, 0.0, 0.0), rft_tau_ref(0.0, 0.0, 0.0);
	lft_tau_ref[1] = irobot.FzRef[0] * tau_tot_ref[1];
	rft_tau_ref[1] = irobot.FzRef[1] * tau_tot_ref[1];
	if (tau_tot_ref[0] >= 0) lft_tau_ref[0] = tau_tot_ref[0];
	else rft_tau_ref[0] = tau_tot_ref[0];

	Eigen::Vector3d lft_Force, rft_Force, lft_tau, rft_tau;
	lft_Force = irobot.IMU_rel * irobot.Rot_lft2hip * irobot.FT_foot_left.head(3);
	rft_Force = irobot.IMU_rel * irobot.Rot_rft2hip * irobot.FT_foot_right.head(3);

	if (lft_Force[2] < 0.0) lft_Force[2] = 0.0;
	if (rft_Force[2] < 0.0) rft_Force[2] = 0.0;
	double Fz_total = lft_Force[2] + rft_Force[2];
	double eps = 1.0;
	if (Fz_total <= eps) Fz_total = eps;
	lft_Force /= Fz_total;
	rft_Force /= Fz_total;

	Eigen::Vector3d gGeomCenLft = irobot.IMU_rel * (irobot.Lft - irobot.cop);
	Eigen::Vector3d gGeomCenRft = irobot.IMU_rel * (irobot.Rft - irobot.cop);

	lft_tau = -irobot.IMU_rel * (irobot.Rot_lft2hip * irobot.FT_foot_left.tail(3));
	// - (gGeomCenLft).cross(lft_Force);
	rft_tau = -irobot.IMU_rel * (irobot.Rot_rft2hip * irobot.FT_foot_right.tail(3));
	// - (gGeomCenRft).cross(rft_Force);
	// lft_tau /= Fz_total;
	// rft_tau /= Fz_total;

	// Eigen::Vector3d damping(0.8, 0.8, 1e3);
	Eigen::Vector3d damping(10.0, 3.0, 1e3);
	Eigen::Vector3d settle_time(1.0, 1.0, 1.0);


	// COUT("lft_tau", lft_tau.transpose(), "rft_tau", rft_tau.transpose());


	if (enable) {
		if (irobot.FzRef[0] < 0.35)
		{
			deltaFtAng_l = LFootOri_ctrl->update(damping, lft_tau_ref, lft_tau, settle_time, dT);
		}

		if (irobot.FzRef[1] < 0.35)
		{
			// deltaFtAng_r = RFootOri_ctrl->update(damping, rft_tau_ref, rft_tau, settle_time, dT);
		}
	}
	else {
		double Tf = 1.0 / 3.0;
		deltaFtAng_l = (Tf) / (Tf + dT) * deltaFtAng_l;
		deltaFtAng_r = (Tf) / (Tf + dT) * deltaFtAng_r;

		LFootOri_ctrl->reset(deltaFtAng_l);
		RFootOri_ctrl->reset(deltaFtAng_r);
	}

	clamp(deltaFtAng_l[0], DEGTORAD(-10.0), DEGTORAD(10.0));
	clamp(deltaFtAng_l[1], DEGTORAD(-20.0), DEGTORAD(20.0));
	clamp(deltaFtAng_l[2], DEGTORAD(-1.0), DEGTORAD(1.0));
	clamp(deltaFtAng_r[0], DEGTORAD(-10.0), DEGTORAD(10.0));
	clamp(deltaFtAng_r[1], DEGTORAD(-20.0), DEGTORAD(20.0));
	clamp(deltaFtAng_r[2], DEGTORAD(-1.0), DEGTORAD(1.0));

	// COUT("left", irobot.FzRef[0], lft_tau_ref[0], lft_tau[0], 57.3 * deltaFtAng_l[0]);
	// COUT("left", irobot.FzRef[0], lft_tau_ref[1], lft_tau[1], 57.3 * deltaFtAng_l[1]);
	// COUT("righ", rft_tau_ref[1], rft_tau[1], 57.3 * deltaFtAng_r[1]);


	deltaFtOri_left = Rz(0 * deltaFtAng_l[2]) * Ry(deltaFtAng_l[1]) * Rx(deltaFtAng_l[0]);
	deltaFtOri_right = Rz(0 * deltaFtAng_r[2]) * Ry(deltaFtAng_r[1]) * Rx(deltaFtAng_r[0]);

}

void StabilizerClass::LandingFtOctrl(const RobotStateClass& irobot, const double & tq_x, const double & tq_y, double damping, double settle_time, Eigen::Matrix3d& deltaFtOri_left, Eigen::Matrix3d& deltaFtOri_right)
{
	// double Fz_ratio_l = irobot.Fzl / (irobot.Fzl + irobot.Fzr);
	// double tyl = Fz_ratio_l * tq_y;
	// double tyr = (1 - Fz_ratio_l) * tq_y;

	// Eigen::Vector3d Tq_diff_left, Tq_diff_right;
	// Tq_diff_left[1] = tyl;
	// Tq_diff_right[1] = tyr;
	// if (tq_x >= 0) Tq_diff_left[0] = tq_x;
	// else Tq_diff_right[0] = tq_x;

	// Eigen::Vector3d Tq_msr_left = irobot.FT_foot_left.tail(3);
	// Eigen::Vector3d Tq_msr_right = irobot.FT_foot_left.tail(3);

	// Eigen::Vector3d deltaFtO_l = LFootOri_ctrl->update(damping, Tq_diff_left, Tq_msr_left, settle_time, dT);
	// clamp(deltaFtO_l[0], DEGTORAD(-10), DEGTORAD(10));
	// clamp(deltaFtO_l[1], DEGTORAD(-15), DEGTORAD(15));
	// deltaFtOri_left = Ry(deltaFtO_l[1]) * Rx(0 * deltaFtO_l[0]);

	// Eigen::Vector3d deltaFtO_r = RFootOri_ctrl->update(damping, Tq_diff_right, Tq_msr_right, settle_time, dT);
	// clamp(deltaFtO_r[0], DEGTORAD(-10), DEGTORAD(10));
	// clamp(deltaFtO_r[1], DEGTORAD(-15), DEGTORAD(15));
	// deltaFtOri_right = Ry(deltaFtO_r[1]) * Rx(0 * deltaFtO_r[0]);

	// COUT(tq_x, tq_y, 57.3 * deltaFtO_l[1], 57.3 * deltaFtO_r[1]);
	// COUT(tq_x, tq_y, 57.3 * deltaFtO_l[0], 57.3 * deltaFtO_l[1], 57.3 * deltaFtO_r[0], 57.3 * deltaFtO_r[1]);

	//
	// COUT("Fzl", Fzl_msr, "Fzr", Fzr_msr);
	// COUT("deltaFtz", deltaFtz, "ref", ref, "msr", Fz_msr_diff);

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
