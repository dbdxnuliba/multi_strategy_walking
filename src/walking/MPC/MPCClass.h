/**
MPCClass.h

Description:	Header file of MPCClass

*/
#pragma once

#include "QP/QPBaseClass.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>


#include <vector> 

#include "RobotPara/RobotParaClass.h"

#include "KMP/kmp.h"
#include <armadillo>



using namespace Eigen;
using namespace std;
using namespace arma;



/// constant variable defintion
const int _footstepsnumber = 5;       //  number of _footstepnumber
const double _dt = 0.05;                //sampling time
const int _nh = 30;                    /// =PreviewT/_dt: number of sampling time for predictive window: <= 2*_nT; (_dt defined in MpcRTControlClass.h: dt_mpc)  	
const double _tstep = 0.7;              ///step period
const int _nT = round(_tstep/_dt);      /// _tstep/_dt)  the number of one step cycle
const int _nstep = 2;                   /// maximal footstep locations where the predictive windwo covers
const int _Nt = 5*_nh + 3*_nstep;       /// _Nt = 5*_nh + 3*_nstep;  the number of the variable of the optimization problem
const int _nsum = (_footstepsnumber-1)*_nT; /// number of whole control loop
const double _height_offset = 0.05;     
const double _height_squat_time =2; 


class MPCClass : public QPBaseClass
{
public:
	MPCClass();
	virtual ~MPCClass() {};
	
	/******************* KMP class preparation **************/
        kmp kmp_leg_L;
	kmp kmp_leg_R;

	int    _inDim_kmp; 	      		    //input dimension
	int    _outDim_kmp; 	      		    //output dimension
	int    _pvFlag_kmp;			    // output: pos (and vel)
	///////////// adjust KMP parameters
	double _lamda_kmp, _kh_kmp;	    	    //set kmp parameters 
	vec    _query_kmp;            	    // input
	vec    _mean_kmp;  	            // output:mean
	mat    _data_kmp;
	
	
	
	
	
	
	
	
	
	
	
	
	
	/*********************************************** For MPCClass definition *********************/
	
	//void FootStepNumberInputs(int footstepsnumber);
	void FootStepInputs(double stepwidth, double steplength, double stepheight);	
	void Initialize();

	////////////////=================================================//////////////////
	/// for step timing optimization
        void step_timing_opti_loop(int i,Eigen::Matrix<double,18,1> estimated_state, Eigen::Vector3d _Rfoot_location_feedback, Eigen::Vector3d _Lfoot_location_feedback,double lamda, bool _stopwalking);
        int Indexfind(double goalvari, int xyz);
	void solve_stepping_timing(); 		

	
	/// for height +angular momentum + step optimization
	Eigen::MatrixXd Matrix_ps(Eigen::MatrixXd a, int nh, Eigen::MatrixXd cxps);
	Eigen::MatrixXd Matrix_pu(Eigen::MatrixXd a, Eigen::MatrixXd b, int nh, Eigen::MatrixXd cxpu);
	
	///////////////=================================================/////////////////////////////
	
	void Solve();
	
	////// for trajectory intepolation
	void Foot_trajectory_solve(int j_index, bool _stopwalking);
	int Get_maximal_number(double dtx);	
	int Get_maximal_number_reference();
	/// when no mpc is used, the CoM height trajectory should be generated by interpolation!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//////////////////////////////////////////////////////////////////////////////
	void CoM_height_solve(int j_index, bool _stopwalking);	
	
	
	Vector3d X_CoM_position_squat(int walktime, double dt_sample);
	// current state based on the past one and two actual sampling time;
	Vector3d XGetSolution_CoM_position(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3);
	Vector3d XGetSolution_Foot_positionR(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3);
	Vector3d XGetSolution_Foot_positionL(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3);	
	Vector3d XGetSolution_body_inclination(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3);
	Vector6d XGetSolution_Foot_position_KMP(int walktime, double dt_sample, int j_index, bool _stopwalking);
	
	
	
	
	/////**********************save the optimal trajectory generated by MPC********************///
	void File_wl_steptiming();	
	
	
	int _n_loop_omit;		

	std::string _robot_name;
	double _robot_mass;
	double _lift_height;	

	int _method_flag;
	
	int _n_end_walking;
	
        /////**********************save the optimal trajectory generated by kmp********************///
        void File_wl_kmp();
	
	
	int _j_period;
	Eigen::Matrix<double,_footstepsnumber,1> _tx;		///store the the start time of each walkng cycle		
	Eigen::Matrix<double,_footstepsnumber,1> _ts, _td;  //time period of single support and double support		 
	
////////// generate the COM trajectory during the DSP		
        int _nTdx;
	
	int _bjx1, _bjx2, _mx;	
	
	int _j_count;	
	
        Vector3d _F_R, _F_L,_M_R,_M_L;	
	Vector3d _ZMPxy_realx;	
	
	Vector3d _comxyzx,_comvxyzx,_comaxyzx,_thetaxyx,_thetavxyx,_thetaaxyx,_Lfootxyzx,_Rfootxyzx;	
		
	void Zmp_distributor(int walktime, double dt_sample);


		
	
protected: 


private:    
        /// update objective function and constraints matrices for step timing optimization
	void step_timing_object_function(int i);  
  
	void step_timing_constraints(int i);  
	
	
        //parameters declaration  
        // all the variable for step timing optimization
        Eigen::Matrix<double,_footstepsnumber,1> _steplength, _stepwidth, _stepheight,_lift_height_ref;
        Eigen::Matrix<double,_footstepsnumber,1> _Lxx_ref, _Lyy_ref;  ///reference steplengh and stepwidth
        Eigen::Matrix<double,_footstepsnumber,1> _footx_ref, _footy_ref, _footz_ref;    //footstep locations generation
	Eigen::Matrix<double,_footstepsnumber,1> _footx_offline, _footy_offline, _footz_offline;    //footstep locations generation
	/// support foot location feedbacke for step time optimization	
	Eigen::Matrix<double,_footstepsnumber,1> _footx_real_feed, _footy_real_feed;	        
        
        
        
	Eigen::Matrix<double,_nsum,1> _t;	            ///whole sampling time sequence

	Eigen::Matrix<double,1,_nsum> _comx, _comvx, _comax;
	Eigen::Matrix<double,1,_nsum> _comy, _comvy, _comay;
	Eigen::Matrix<double,1,_nsum> _comz, _comvz, _comaz;
	Eigen::Matrix<double,1,_nsum> _Lxx_ref_real, _Lyy_ref_real,_Ts_ref_real; 
	
	/// para
	double _hcom;
	double _g;	
	double _Wn;
	double _Wndt;
	
	Eigen::Matrix<double,1,_nsum> _px, _py, _pz;
	Eigen::Matrix<double,1,_nsum> _zmpvx, _zmpvy;
	Eigen::Matrix<double,1,_footstepsnumber> _COMx_is, _COMx_es, _COMvx_is;
	Eigen::Matrix<double,1,_footstepsnumber> _COMy_is, _COMy_es, _COMvy_is;
	Eigen::Matrix<double,1,_nsum> _comx_feed, _comvx_feed,_comax_feed;
	Eigen::Matrix<double,1,_nsum> _comy_feed, _comvy_feed,_comay_feed;
        
	//optimal variables
	Eigen::Matrix<double,8,_nsum> _Vari_ini;
	Eigen::Matrix<double,8,1> _vari_ini;

	// weight coefficient	
	double _aax,_aay,_aaxv,_aayv,_bbx,_bby,_rr1,_rr2;
	double _aax1,_aay1,_bbx1,_bby1,_rr11,_rr21;
	
	//constraints on step timing parameters
	double _t_min, _t_max;
	
	// swing foot velocity constraints  parameters
	double _footx_vmax, _footx_vmin,_footy_vmax,_footy_vmin;	
	
	// swing foot velocity constraints  parameters
	double _comax_max, _comax_min,_comay_max, _comay_min;	


	
	// %%% physical	
	double _mass,  _rad,  _j_ini;	
	
	//external force
        double _FX, _FY;	
	double _t_last;
	double _det_xa,_det_ya;
	double _det_xv,_det_yv;
	double _det_xp,_det_yp;
	
	Eigen::Matrix<double,1,_nsum> _tcpu;	
	
	
	

	
	int xyz0; //flag for find function 
	int xyz1;  
	int xyz2;	
	
	int _periond_i; ///
	
	int _ki, _k_yu;
	double _Tk;
	
	double _Lxx_refx,_Lyy_refy,_Lxx_refx1,_Lyy_refy1;
	double _tr1_ref,_tr2_ref,_tr1_ref1,_tr2_ref1;
	
	double _tr1_min,_tr2_min,_tr1_max,_tr2_max,_tr11_min,_tr21_min,_tr11_max,_tr21_max;
	
	Eigen::Matrix<double,1,8> _SS1,_SS2,_SS3,_SS4,_SS5,_SS6,_SS7,_SS8;
	
	Eigen::Matrix<double,1,1> _comvx_endref,_comvy_endref;
	
	Eigen::Matrix<double,1,1> _AxO,_BxO,_Cx, _Axv,_Bxv,_Cxv;
	Eigen::Matrix<double,1,1> _AyO,_ByO,_Cy, _Ayv,_Byv,_Cyv;
	
	Eigen::Matrix<double,8,8> _SQ_goal0,_SQ_goal,_SQ_goal1,_SQ_goal20,_SQ_goal2,_SQ_goal3;
	Eigen::Matrix<double,8,1> _Sq_goal,_Sq_goal1,_Sq_goal2,_Sq_goal3;
	Eigen::Matrix<double,8,8> _Ax,_Ay;
	Eigen::Matrix<double,1,1> _Bx,_By;
	Eigen::Matrix<double,1,1> _ixi,_iyi;	
	
	//constraints on step timing parameters
	Eigen::Matrix<double,1,8> _trx1_up, _trx1_lp,_trx2_up, _trx2_lp,_trx3_up, _trx3_lp,_trx4_up, _trx4_lp;	
	Eigen::Matrix<double,1,1> _det_trx1_up, _det_trx1_lp,_det_trx2_up,_det_trx2_lp,_det_trx3_up, _det_trx3_lp,_det_trx4_up, _det_trx4_lp;		
	Eigen::Matrix<double,8,8> _trx;
	Eigen::Matrix<double,8,1> _det_trx;
	
	// tr1 & tr2: equation constraints
	Eigen::Matrix<double,1,8> _trx12, _trx121;	
	Eigen::Matrix<double,1,1> _det_trx12, _det_trx121;		
	Eigen::Matrix<double,2,8> _trxx;
	Eigen::Matrix<double,2,1> _det_trxx;	
	
	///foot location constraints 
	double _footx_max, _footx_min,_footy_max,_footy_min;	
	Eigen::Matrix<double,1,8> _h_lx_up,  _h_lx_lp, _h_ly_up, _h_ly_lp,_h_lx_up1, _h_lx_lp1, _h_ly_up1, _h_ly_lp1;
	Eigen::Matrix<double,1,1> _det_h_lx_up,_det_h_lx_lp,_det_h_ly_up,_det_h_ly_lp,_det_h_lx_up1,_det_h_lx_lp1,_det_h_ly_up1,_det_h_ly_lp1;
	Eigen::Matrix<double,8,8> _h_lx_upx;
	Eigen::Matrix<double,8,1> _det_h_lx_upx;


	///foot location constraints 	
	Eigen::Matrix<double,1,8> _h_lvx_up,  _h_lvx_lp, _h_lvy_up, _h_lvy_lp,_h_lvx_up1, _h_lvx_lp1, _h_lvy_up1, _h_lvy_lp1;
	Eigen::Matrix<double,1,1> _det_h_lvx_up,_det_h_lvx_lp,_det_h_lvy_up,_det_h_lvy_lp,_det_h_lvx_up1,_det_h_lvx_lp1,_det_h_lvy_up1,_det_h_lvy_lp1;
	Eigen::Matrix<double,8,8> _h_lvx_upx;
	Eigen::Matrix<double,8,1> _det_h_lvx_upx;	
	
	/// CoM accelearation boundary
	double _AA, _CCx, _BBx, _CCy, _BBy,_AA1x,_AA2x,_AA3x,_AA1y,_AA2y,_AA3y;
	Eigen::Matrix<double,1,8> _CoM_lax_up,  _CoM_lax_lp,  _CoM_lay_up,  _CoM_lay_lp;
	Eigen::Matrix<double,1,1> _det_CoM_lax_up,  _det_CoM_lax_lp,  _det_CoM_lay_up,  _det_CoM_lay_lp;
	Eigen::Matrix<double,4,8> _CoM_lax_upx;
	Eigen::Matrix<double,4,1> _det_CoM_lax_upx;	
	
	
	
	/// CoM velocity_inremental boundary
	double _VAA, _VCCx, _VBBx, _VCCy, _VBBy,_VAA1x,_VAA2x,_VAA3x,_VAA1y,_VAA2y,_VAA3y;
	Eigen::Matrix<double,1,8> _CoM_lvx_up,  _CoM_lvx_lp,  _CoM_lvy_up,  _CoM_lvy_lp;
	Eigen::Matrix<double,1,1> _det_CoM_lvx_up,  _det_CoM_lvx_lp,  _det_CoM_lvy_up,  _det_CoM_lvy_lp;
	Eigen::Matrix<double,4,8> _CoM_lvx_upx;
	Eigen::Matrix<double,4,1> _det_CoM_lvx_upx;	
	
	/// CoM initial velocity_ boundary
	double _VAA1x1,_VAA2x1,_VAA3x1,_VAA1y1,_VAA2y1,_VAA3y1;
	Eigen::Matrix<double,1,8> _CoM_lvx_up1,  _CoM_lvx_lp1,  _CoM_lvy_up1,  _CoM_lvy_lp1;
	Eigen::Matrix<double,1,1> _det_CoM_lvx_up1,  _det_CoM_lvx_lp1,  _det_CoM_lvy_up1,  _det_CoM_lvy_lp1;
	Eigen::Matrix<double,4,8> _CoM_lvx_upx1;
	Eigen::Matrix<double,4,1> _det_CoM_lvx_upx1;	
	
	

	
	
	
	/// for foot trajectory generation
	Eigen::Matrix<double,_nh,1> _t_f;	

	int _bjxx;	

	Eigen::Matrix<double, 3,_footstepsnumber> _footxyz_real;
	
	Eigen::Matrix<double, 1,_nsum> _Lfootx, _Lfooty,_Lfootz, _Lfootvx, _Lfootvy,_Lfootvz, _Lfootax, _Lfootay,_Lfootaz;
	Eigen::Matrix<double, 1,_nsum> _Rfootx, _Rfooty,_Rfootz, _Rfootvx, _Rfootvy,_Rfootvz, _Rfootax, _Rfootay,_Rfootaz;	
        double _ry_left_right;	
	
	
	////result CoM_foot_trajection_generation
	Eigen::Matrix<double,3,_nsum> _CoM_position_optimal;
	Eigen::Matrix<double,3,_nsum> _torso_angle_optimal;
	Eigen::Matrix<double,3,_nsum> _L_foot_optition_optimal;
	Eigen::Matrix<double,3,_nsum> _R_foot_optition_optimal;
	Eigen::Matrix<double,3,_footstepsnumber> _foot_location_optimal;	
	
	
	
	/////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//// for kMP swing generation: private variable for leg status storage	
	/////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// 	Eigen::VectorXd _flag_global;
	
	
	Eigen::Matrix<double, 1,10000> _Lfootx_kmp, _Lfooty_kmp,_Lfootz_kmp, _Lfootvx_kmp, _Lfootvy_kmp,_Lfootvz_kmp;
	Eigen::Matrix<double, 1,10000> _Rfootx_kmp, _Rfooty_kmp,_Rfootz_kmp, _Rfootvx_kmp, _Rfootvy_kmp,_Rfootvz_kmp;
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
	// other variables for step height+angular optimization
	///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	
	Eigen::RowVectorXd _zmpx_real, _zmpy_real;		
	Eigen::RowVectorXd _thetax, _thetavx, _thetaax;
	Eigen::RowVectorXd _thetay, _thetavy, _thetaay;	
	Eigen::RowVectorXd _thetaz, _thetavz, _thetaaz;	
	
	Eigen::RowVectorXd _torquex_real, _torquey_real;	
	
	// CoM+angular momentum state and contro input
	Eigen::MatrixXd _xk,_yk,_zk,_thetaxk,_thetayk;
	Eigen::RowVectorXd _x_vacc_k,_y_vacc_k,_z_vacc_k,_thetax_vacc_k,_thetay_vacc_k;
	
	std::vector<Eigen::Vector3d> _footsteps;
	
	// initial parameters for MPC

	Eigen::MatrixXd _Hcom1;

	Eigen::VectorXd _ggg;
// 	int    _nh;
	Eigen::MatrixXd _a,_b,_c,_cp,_cv,_ca;
	
	
	//vertical height constraints
	Eigen::VectorXd _z_max;
	Eigen::VectorXd _z_min;
	
	
	//footz
	Eigen::MatrixXd _Zsc;
	
	//predictive model
	Eigen::MatrixXd _pps,_ppu,_pvs,_pvu,_pas,_pau;
	
	
	Eigen::VectorXd _footx_real, _footy_real, _footz_real;
	Eigen::VectorXd _footx_real_next, _footy_real_next, _footz_real_next;
	Eigen::VectorXd _footx_real_next1, _footy_real_next1, _footz_real_next1;
	



	Eigen::MatrixXd _Lfootxyz, _Rfootxyz;
	
	

	

	
	// zmp-constraints
	Eigen::VectorXd _zmpx_ub, _zmpx_lb, _zmpy_ub, _zmpy_lb;
	
	// com-support range
	Eigen::VectorXd _comx_max,  _comx_min,  _comy_max, _comy_min;
	
	// angle range
	Eigen::VectorXd _thetax_max,  _thetax_min,  _thetay_max, _thetay_min;
	
	
	// torque range
	Eigen::VectorXd _torquex_max, _torquex_min, _torquey_max, _torquey_min; 
	



	// solution preparation
	Eigen::MatrixXd _V_optimal;
	Eigen::VectorXd _flag;
	Eigen::VectorXd _flag_global;
	
	
	double _Rx,     _Ry,     _Rz;
	double _alphax, _alphay, _alphaz;
	double _beltax, _beltay, _beltaz;
	double _gamax,  _gamay,  _gamaz;
	double _Rthetax, _Rthetay;
	double _alphathetax, _alphathetay;
	double _beltathetax, _beltathetay;
	
	// time cost consumption

	Eigen::RowVectorXd _tcpu_iterative;
	Eigen::RowVectorXd _tcpu_prepara;
	Eigen::RowVectorXd _tcpu_prepara2;
	Eigen::RowVectorXd _tcpu_qp;	
	
	
// predictive model control_tracking with time_varying height		

	Eigen::VectorXd _bjx, _tnx;
	
	Eigen::MatrixXd _v_i, _VV_i;
	
	int _n_vis;
	
        Eigen::VectorXd _Lx_ref, _Ly_ref,_Lz_ref;
	
// 	int _Nt;
// 	int _nstep;
	Eigen::MatrixXd _V_ini;
	
	int xxx, xxx1,xxx2;

        Eigen::VectorXd _fx, _fy;
	Eigen::VectorXd _fxx_global, _fyy_global;
	
	Eigen::MatrixXd _comx_center_ref, _comy_center_ref,_comz_center_ref;
	
	Eigen::MatrixXd _thetax_center_ref, _thetay_center_ref;      
	
	int _loop;
	
	// sqp model
	
	Eigen::MatrixXd _ppu_2, _pvu_2;
	
	Eigen::MatrixXd _WX, _WY, _WZ, _WthetaX, _WthetaY, _PHIX, _PHIY, _PHIZ, _Q_goal, _q_goal, _Q_goal1, _q_goal1;
	
	Eigen::MatrixXd A_unit, B_unit, C_unit;	
	Eigen::MatrixXd _Sjx,_Sjy,_Sjz,_Sjthetax,_Sjthetay,_Sfx,_Sfy,_Sfz;
	
	// inequality constraints
	
	Eigen::MatrixXd _H_q_upx,_F_zmp_upx,_H_q_lowx,_F_zmp_lowx,_H_q_upy,_F_zmp_upy,_H_q_lowy,_F_zmp_lowy;
	
	Eigen::MatrixXd _H_h_upz,_F_h_upz,_H_h_lowz,_F_h_lowz, _delta_footz_up, _delta_footz_low;
	
	Eigen::MatrixXd _H_hacc_lowz,_F_hacc_lowz, _delta_footzacc_up;
	
	Eigen::MatrixXd _q_upx,_qq_upx,_q_lowx,_qq_lowx,_q_upy,_qq_upy,_q_lowy,_qq_lowy,_qq1_upx,_qq1_lowx,_qq1_upy,_qq1_lowy;
	
	Eigen::MatrixXd _t_upx,_tt_upx,_t_lowx,_tt_lowx,_t_upy,_tt_upy,_t_lowy,_tt_lowy,_tt1_upx,_tt1_lowx,_tt1_upy,_tt1_lowy;
	
	Eigen::MatrixXd _H_q_footx_up,_F_foot_upx,_H_q_footx_low,_F_foot_lowx,_H_q_footy_up,_F_foot_upy,_H_q_footy_low,_F_foot_lowy;
	
	Eigen::MatrixXd _Footvx_max,_Footvx_min,_Footvy_max,_Footvy_min, _footubxv,_footlbxv,_footubyv,_footlbyv;
	
	// equality constraints
	Eigen::MatrixXd _H_q_footz,_F_footz;
	
	Eigen::MatrixXd _h_h, _hhhx;
	Eigen::MatrixXd _a_hx, _a_hxx, _a_hy, _a_hyy;
	
	
	
	
	
	// zmp constraints
	Eigen::MatrixXd _Si;
	Eigen::MatrixXd _phi_i_x_up,_p_i_x_t_up,_del_i_x_up,_phi_i_x_low,_p_i_x_t_low,_del_i_x_low;
	Eigen::MatrixXd _phi_i_y_up,_p_i_y_t_up,_del_i_y_up,_phi_i_y_low,_p_i_y_t_low,_del_i_y_low;
	
	//foot location constraints
	Eigen::MatrixXd _Sfoot;
	
	//com support leg constraints
	Eigen::MatrixXd _S1;	
	
	Eigen::MatrixXd _V_inix;
	
// // 	int xxx_vector=30;check check check!!!!
// 	vector <Eigen::MatrixXd> ZMPx_constraints_offfline;
// 	vector <Eigen::MatrixXd> ZMPy_constraints_offfline;
// 	
// 	vector <Eigen::MatrixXd> ZMPx_constraints_half;
// 	vector <Eigen::MatrixXd> ZMPy_constraints_half;
	

	
////==========================================================================	
	///////////////for polynomial intepolation
	Eigen::Matrix<double, 4,4>  _AAA_inv;
	
	void solve_AAA_inv(Eigen::Matrix<double, 4, 1> t_plana); 
	
	Eigen::Matrix<double, 7, 7> solve_AAA_inv_x(Eigen::Vector3d t_plan); 
		
	
	

	////for lowel level control: ZMP optimal distribution
// 	Vector3d _F_R, _F_L,_M_R,_M_L;
	Matrix<double,3,3> _Co_L, _Co_R;
		 
	void Force_torque_calculate(Vector3d comxyzx1,Vector3d comaxyzx1,Vector3d thetaaxyx1,Vector3d Lfootxyz,Vector3d Rfootxyz);

	void zmp_interpolation(int t_int,int walktime, double dt_sample);
	
// 	void Zmp_distributor(int walktime, double dt_sample);	
	
	

	
	

	
	

	protected:
	void Rfooty_plan(int arg1);
// 	footx_ref_comx_feed(int i);
};






