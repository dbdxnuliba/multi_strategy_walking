/*****************************************************************************
RobotParaClass.h

*****************************************************************************/
#pragma once

#include <iostream>
#include <string>
#include <map>
#include <boost/bimap.hpp>
#include "yaml-cpp/yaml.h"
// #include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>

#include "utils/utils.h"
#include <Eigen/Dense>

#ifdef PARSE_OFF_TRAJ
#include "utils/TrajParser/TrajParserClass.h"
#endif

#include <unordered_map>

typedef boost::bimap< int, std::string > bm_type;

/** @file */
enum LinkName {
	PELVIS,						//!< 0
	RIGHT_THIGH_DUMMY1,			//!< 1
	RIGHT_THIGH_DUMMY2,			//!< 2
	RIGHT_THIGH,				//!< 3
	RIGHT_CALF,					//!< 4
	RIGHT_FOOT_DUMMY,			//!< 5
	RIGHT_FOOT,					//!< 6
	LEFT_THIGH_DUMMY1,			//!< 7
	LEFT_THIGH_DUMMY2,			//!< 8
	LEFT_THIGH,					//!< 9
	LEFT_CALF,					//!< 10
	LEFT_FOOT_DUMMY,			//!< 11
	LEFT_FOOT,					//!< 12
	WAIST_DUMMY1,				//!< 13
	WAIST_DUMMY2,				//!< 14
	TORSO,						//!< 15
	NECK,						//!< 16
	HEAD,						//!< 17
	RIGHT_UPPER_ARM_DUMMY1,		//!< 18
	RIGHT_UPPER_ARM_DUMMY2,		//!< 19
	RIGHT_UPPER_ARM,			//!< 20
	RIGHT_ELBOW_FORE_ARM,		//!< 21
	RIGHT_FORE_ARM,				//!< 22
	RIGHT_HAND_DUMMY1,			//!< 23
	// RIGHT_HAND_DUMMY2,		//
	RIGHT_HAND,					//!< 24
	LEFT_UPPER_ARM_DUMMY1,		//!< 25
	LEFT_UPPER_ARM_DUMMY2,		//!< 26
	LEFT_UPPER_ARM,				//!< 27
	LEFT_ELBOW_FORE_ARM,		//!< 28
	LEFT_FORE_ARM,				//!< 29
	LEFT_HAND_DUMMY1,			//!< 30
	// LEFT_HAND_DUMMY2,		//
	LEFT_HAND,					//!< 31
};

/** @file */
enum JointName {
	WAIST_ROLL,					//!< 0
	WAIST_PITCH,				//!< 1
	WAIST_YAW,					//!< 2
	RIGHT_HIP_PITCH,			//!< 3
	RIGHT_HIP_ROLL,				//!< 4
	RIGHT_HIP_YAW,				//!< 5
	RIGHT_KNEE_PITCH,			//!< 6
	RIGHT_FOOT_ROLL,			//!< 7
	RIGHT_FOOT_PITCH,			//!< 8
	LEFT_HIP_PITCH,				//!< 9
	LEFT_HIP_ROLL,				//!< 10
	LEFT_HIP_YAW,				//!< 11
	LEFT_KNEE_PITCH,			//!< 12
	LEFT_FOOT_ROLL,				//!< 13
	LEFT_FOOT_PITCH,			//!< 14
	RIGHT_SHOULDER_PITCH,		//!< 15
	RIGHT_SHOULDER_ROLL,		//!< 16
	RIGHT_SHOULDER_YAW,			//!< 17
	RIGHT_ELBOW_PITCH,			//!< 18
	RIGHT_FOREARM_YAW,			//!< 19
	RIGHT_WRIST_PITCH,			//!< 20
	RIGHT_WRIST_ROLL,			//!< 21
	LEFT_SHOULDER_PITCH,		//!< 22
	LEFT_SHOULDER_ROLL,			//!< 23
	LEFT_SHOULDER_YAW,			//!< 24
	LEFT_ELBOW_PITCH,			//!< 25
	LEFT_FOREARM_YAW,			//!< 26
	LEFT_WRIST_PITCH,			//!< 27
	LEFT_WRIST_ROLL,			//!< 28
	NECK_PITCH,					//!< 29
	HEAD_PITCH,					//!< 30
};//



class RobotParaClass
{
public:
    
	RobotParaClass();
	~RobotParaClass();

	bool IsOnlyLowerBody;

	// urdf::Model _urdf_model;
	urdf::ModelInterfaceSharedPtr _urdf_model;
	// boost::shared_ptr<urdf::ModelInterface> _urdf_model;

	std::string name;
	// static std::string name;
	std::string urdf_path;
	std::string opensot_cfg_path;

	bool TRUE_SIM_FALSE_REALROBOT;

	std::vector<double> homing_pos, homing_pos_for_coman, offset_pos, offset_pos_for_coman;

	static std::vector<double> robot_push_force;
	static std::string FallOrStab;

	bool IsFixedWalk;
	static bool HasEntrySway;

	static int robot_count;
	int RobotNo;
	static double g;
	static double dt;
	double totalmass;
	static int link_num, joint_num;
	static double foot_length, foot_width, foot_height;

	static double ankle_x_offset, half_hip_width, half_foot_dis;
	static double full_leg, ankle_height, ft_height;
	static double waist_height;
	
	static double hip_to_ankle_x_offset;

	static double homing_lift_height;
	double LIFT_HEIGHT;
	static double z_c;
	double Ksway;
	static double PreT;
	double Tstep;

	double HipCompL, HipCompR;

	double Kx, Bx, Ky, By, Kz, Bz;
	double Kx_Hand, Bx_Hand, Ky_Hand, By_Hand, Kz_Hand, Bz_Hand;
	double K_roll_Hand, B_roll_Hand, K_pitch_Hand, B_pitch_Hand, K_yaw_Hand, B_yaw_Hand;

	double kp_torque, kd_torque;
	double kp_imp, kd_imp;
	mutable double kp_adm, kd_adm;
	double kp_pos, kd_pos;

	double ik_pos_gain, ik_ori_gain;
	double ik_kp_pos_acc_gain, ik_kd_pos_acc_gain;
	double ik_kp_ori_acc_gain, ik_kd_ori_acc_gain;
	mutable double kp_debug, kd_debug;

	double THIGH_HEIGHT, CALF_HEIGHT, HALF_SHOULDER_WIDTH, HIP_BODY_OFFSET_X, PELVIS_HEIGHT, TORSO_HEIGHT, UPPER_ARM_HEIGHT, FORE_ARM_HEIGHT;

	double THIGH_MASS, CALF_MASS, FOOT_MASS, PELVIS_MASS, TORSO_MASS, HEAD_MASS, UPPER_ARM_MASS, FORE_ARM_MASS, HAND_MASS;

	Eigen::Vector3d LEFT_HAND_HOME_POS, RIGHT_HAND_HOME_POS;
	Eigen::MatrixXd LEFT_HAND_HOME_ORI, RIGHT_HAND_HOME_ORI;
	Eigen::Vector3d LEFT_FOOT_HOME_POS, RIGHT_FOOT_HOME_POS;
	Eigen::MatrixXd LEFT_FOOT_HOME_ORI, RIGHT_FOOT_HOME_ORI;

	inline const std::string& RobotName() {return name;};
	inline static const double& Z_C() {return z_c;};
	inline static const double& HALF_HIP_WIDTH() {return half_hip_width;};
	inline static const double& HALF_FOOT_DIS() {return half_foot_dis;};
	inline static const double& G() {return g;};
	inline static const double& dT() {return dt;};
	inline static const double& FULL_LEG() {return full_leg;};
	inline static const double& WAIST_HEIGHT() {return waist_height;};
	inline static const double& HOMING_LIFT_HEIGHT() {return homing_lift_height;};
	inline static const double& PreviewT() {return PreT;};
	inline static const double& ANKLE_X_OFFSET() {return ankle_x_offset;};
	inline static const double& HIP_TO_ANKLE_X_OFFSET() {return hip_to_ankle_x_offset;};
	inline static const double& FOOT_LENGTH() {return foot_length;};
	inline static const double& FOOT_WIDTH() {return foot_width;};
	inline static const double& FOOT_HEIGHT() {return foot_height;};
	inline static const double& ANKLE_HEIGHT() {return ankle_height;};
	inline static const double& FT_HEIGHT() {return ft_height;};
	inline static const int& LINK_NUM() {return link_num;};
	inline static const int& JOINT_NUM() {return joint_num;};
	inline static const int& ROBOT_COUNT() {return robot_count;};
	const std::vector<double>& HOMING_POS() const {return homing_pos;};

	// Member functions that do not modify the class instance should be declared as "const", otherwise will cause "error: passing xxx as 'this' argument of xxx discards qualifiers"
	const std::vector<double>& HOMING_POS_FOR_COMAN() const {return homing_pos_for_coman;};

	template <class T>
	inline T setParaForSimOrReal(T simpara, T realpara) const
	{
		if (TRUE_SIM_FALSE_REALROBOT) return simpara;
		else return realpara;
	};

	// inline boost::shared_ptr< const urdf::Joint > getJoint(const int& joint_name) const
	inline urdf::JointConstSharedPtr getJoint(const int& joint_name) const
	{
		// return _urdf_model->getJoint(_map_JointName_to_String.at(joint_name));
		return _urdf_model->getJoint(_bimap_JointName.left.find(joint_name)->second);
	};

	// inline boost::shared_ptr< const urdf::Joint > getJoint(const std::string& joint_name) const
	inline urdf::JointConstSharedPtr getJoint(const std::string& joint_name) const
	{
		return _urdf_model->getJoint(joint_name);
	};

	// inline boost::shared_ptr< const urdf::Link > getLink(const int& link_name) const
	inline urdf::LinkConstSharedPtr getLink(const int& link_name) const
	{
		// return _urdf_model->getLink(_map_LinkName_to_String.at(link_name));
		return _urdf_model->getLink(_bimap_LinkName.left.find(link_name)->second);
	};

	// inline boost::shared_ptr< const urdf::Link > getLink(const std::string& link_name) const
	inline urdf::LinkConstSharedPtr getLink(const std::string& link_name) const
	{
		// return _urdf_model->getLink(_map_LinkName_to_String.at(link_name));
		return _urdf_model->getLink(link_name);
	};

	inline std::string getLinkName(const int& link_name) const {return _bimap_LinkName.left.find(link_name)->second;};
	inline int getLinkName(const std::string& link_name) const {return _bimap_LinkName.right.find(link_name)->second;};

	inline std::string getJointName(const int& joint_name) const {return _bimap_JointName.left.find(joint_name)->second;};
	inline int getJointName(const std::string& joint_name) const {return _bimap_JointName.right.find(joint_name)->second;};

	// std::string getJointName(const int& joint_name) const;

	// template <class T>
	// void SortFromXBotToJointName(const std::unordered_map<std::string, double> &from, T &to);

	// template <class T>
	// void SortFromJointNameToXBot(const T &from, std::unordered_map<std::string, double> &to);

#ifdef PARSE_OFF_TRAJ
	boost::shared_ptr<TrajParserClass> arm_off_traj;

	template <typename T1, typename T2>
	static void set_POS_FROM_OFF_TRAJ(const T1 &from, T2 &to)
	{
		to[WAIST_ROLL ]				= from[ 0];
		to[WAIST_PITCH]				= from[ 1];
		to[WAIST_YAW  ]				= from[ 2];
		to[RIGHT_SHOULDER_PITCH]	= from[ 3];
		to[RIGHT_SHOULDER_ROLL ]	= from[ 4];
		to[RIGHT_SHOULDER_YAW  ]	= from[ 5];
		to[RIGHT_ELBOW_PITCH   ]	= from[ 6];
		// to[LEFT_SHOULDER_PITCH ]	= from[10];
		// to[LEFT_SHOULDER_ROLL  ]	= from[11];
		// to[LEFT_SHOULDER_YAW   ]	= from[12];
		// to[LEFT_ELBOW_PITCH    ]	= from[13];
		to[RIGHT_FOREARM_YAW ]		= from[ 7];
		to[RIGHT_WRIST_PITCH ]		= from[ 8];
		to[RIGHT_WRIST_ROLL  ]		= from[ 9];
		to[LEFT_FOREARM_YAW  ]		= from[14];
		to[LEFT_WRIST_PITCH  ]		= from[15];
		to[LEFT_WRIST_ROLL   ]		= from[16];
	};
#endif

private:
	std::map<int, std::string> _map_LinkName_to_String, _map_JointName_to_String;
	void createLinkAndJointNameMap();

	bm_type _bimap_LinkName, _bimap_JointName;

	double getTotalMass(const urdf::ModelInterfaceSharedPtr &model);
	void set_POS_FOR_COMAN(const std::vector<double> &from, std::vector<double> &to);

	void printTree(urdf::LinkConstSharedPtr link, int level = 0);

};
