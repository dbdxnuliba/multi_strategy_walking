/*****************************************************************************
RobotParaClass.cpp

Description:	cpp file of RobotParaClass

*****************************************************************************/
#include "RobotPara/RobotParaClass.h"

double RobotParaClass::z_c = 0;
double RobotParaClass::half_hip_width = 0.1;
double RobotParaClass::half_foot_dis = 0.1;
double RobotParaClass::g = 9.81;
double RobotParaClass::dt = 0.001;
double RobotParaClass::full_leg = 1.0;
double RobotParaClass::waist_height = 0.0;
double RobotParaClass::homing_lift_height = 0.0;
double RobotParaClass::ankle_x_offset = 0.0;
double RobotParaClass::hip_to_ankle_x_offset = 0.0;
double RobotParaClass::ankle_height = 0.1;
double RobotParaClass::foot_length = 0.2;
double RobotParaClass::foot_width = 0.1;
double RobotParaClass::foot_height = 0.02;
double RobotParaClass::ft_height = 0.05;
double RobotParaClass::PreT = 2.0;
int RobotParaClass::link_num = 32;
int RobotParaClass::joint_num = 31;
int RobotParaClass::robot_count = 0;

std::vector<double> RobotParaClass::robot_push_force(3, 0.0);
std::string RobotParaClass::FallOrStab = "stab";
// std::string RobotParaClass::name = "robot";
bool RobotParaClass::HasEntrySway = true;

RobotParaClass::RobotParaClass()
	: TRUE_SIM_FALSE_REALROBOT(true)
	, HipCompL(0.0)
	, HipCompR(0.0)
	, IsOnlyLowerBody(false)
{
	std::cout << "\n\n\n========= Constructing RobotPara =================" << std::endl;
	createLinkAndJointNameMap();
	std::string file_root = ROBOT_PARA_YAML_PATH;
	std::string file_path = file_root + "robot_para.yaml";
	YAML::Node robot = YAML::LoadFile(file_path);
	// std::cout << robot << std::endl;

	g = robot["gravity"].as<double>();
	// TRUE_SIM_FALSE_REALROBOT = robot["TRUE_SIM_FALSE_REALROBOT"].as<bool>();
#ifdef REAL_ROBOT
	TRUE_SIM_FALSE_REALROBOT = false;
#endif

	if (TRUE_SIM_FALSE_REALROBOT) {
#ifdef IN_GAZEBO
		dt = robot["for_simulation"]["dt_gaz"].as<double>();
		std::cout << "So, we are in Gazebo, heh......" << std::endl;
#else
		dt = robot["for_simulation"]["dt_ode"].as<double>();
		std::cout << "So, we are in ODE, heh......" << std::endl;
#endif
	}
	else {
		dt = robot["for_realrobot"]["dt"].as<double>();
		std::cout << "So, we are using real robot, heh......" << std::endl;
	}

	std::cout << "dt = " << dt << std::endl;

	/************ read specific robot para *******************/
	file_path  = file_root + robot["robot_para_file"].as<std::string>();
	std::cout << "Reading robot's parameters from:\t" << file_path << std::endl;
	robot = YAML::LoadFile(file_path);

	// don't know why, if using NAME as static member, will have segmentation fault
	name = robot["name"].as<std::string>();
	// std::cout << "Robot's name is:\t" << name << std::endl;
	urdf_path = file_root + robot["urdf_path"].as<std::string>();
	// urdf_path = robot["urdf_path"].as<std::string>();
	std::cout << "Robot's urdf path is:\t" << urdf_path << std::endl;

	opensot_cfg_path = file_root + robot["opensot_cfg_path"].as<std::string>();
	std::cout << "Robot's opensot_cfg_path is:\t" << opensot_cfg_path << std::endl;

	_urdf_model = urdf::parseURDFFile(urdf_path);
	if (!_urdf_model) {
		std::cerr << "Failed to parse urdf file." << std::endl;
	}

	// if (!_urdf_model->initFile(urdf_path)) {
	// 	std::cerr << "Failed to parse urdf file." << std::endl;
	// }

	if (name != _urdf_model->getName()) {
		std::cerr << "Wrong robot urdf file loaded." << std::endl;
	}

	urdf::LinkConstSharedPtr root_link = _urdf_model->getRoot();
	if (!root_link) std::cerr << "Couldn't get root link." << std::endl;;

	std::cout << "root Link: " << root_link->name << " has " << root_link->child_links.size() << " child(ren)" << std::endl;

	// print entire tree
	// printTree(root_link);

	if (_urdf_model->joints_.size() - 1 <= 15) {
		IsOnlyLowerBody = true;
	}
	COUT("IsOnlyLowerBody", IsOnlyLowerBody);

	std::cout << name << " has " << _urdf_model->joints_.size() - 1 << " joints." << std::endl;

	// for (auto it : _map_LinkName_to_String) {
	// 	_map_LinkName_to_String[it.first] = robot["robot_link"][it.second].as<std::string>();
	// }
	// for (auto it : _map_JointName_to_String) {
	// 	_map_JointName_to_String[it.first] = robot["robot_joint"][it.second].as<std::string>();
	// }
	// LINK_NUM = _map_LinkName_to_String.size();
	// JOINT_NUM = _map_JointName_to_String.size();

	joint_num = 31;
	link_num = 32;
	// joint_num = _urdf_model->joints_.size() - 1;
	// link_num = _urdf_model->links_.size() - 1;

	COUT("link size", _urdf_model->links_.size() - 1);
	COUT("joint size", _urdf_model->joints_.size() - 1);

	// load link names from yaml, and upade the bimap
	COUT("_bimap_LinkName.size() before", _bimap_LinkName.size());
	std::vector<std::string> nameToRemove;
	for ( bm_type::const_iterator iter = _bimap_LinkName.begin(); iter != _bimap_LinkName.end(); ++iter ) {
		bm_type::left_iterator it = _bimap_LinkName.left.find(iter->left);
		// std::string name = iter->right;
		if (robot["robot_link"][iter->right]) {
			bool successful_replace = _bimap_LinkName.left.replace_data( it, robot["robot_link"][iter->right].as<std::string>() );
			assert( successful_replace );
		}
		else {
			nameToRemove.push_back(iter->right);
		}
		// COUT(iter->left, iter->right);
	}
	if (!nameToRemove.empty()) {
		for (auto it : nameToRemove) {
			_bimap_LinkName.right.erase(it);
			COUT("remove link: ", it);
		}
		nameToRemove.clear();
	}
	COUT("_bimap_LinkName.size() after", _bimap_LinkName.size());
	// link_num = _bimap_LinkName.size();

	// load joint names from yaml, and upade the bimap
	COUT("_bimap_JointName.size() before", _bimap_JointName.size());
	// joint_num = _bimap_JointName.size();
	homing_pos.resize(joint_num);
	offset_pos.resize(joint_num);
	for ( bm_type::const_iterator iter = _bimap_JointName.begin(); iter != _bimap_JointName.end(); ++iter ) {
		bm_type::left_iterator it = _bimap_JointName.left.find(iter->left);
		// std::cout<<homing_pos[iter->left]<<"\t";
		if (TRUE_SIM_FALSE_REALROBOT) {
			homing_pos[iter->left] = robot["ode_homing_pos"][iter->right].as<double>();
		}
		else {
			homing_pos[iter->left] = robot["robot_homing_pos"][iter->right].as<double>();
			if (robot["robot_offset_pos"]) {
				offset_pos[iter->left] = robot["robot_offset_pos"][iter->right].as<double>();
			}
		}
		// std::cout<<homing_pos[iter->left]<<"\n";
		// COUT("b:", iter->left, iter->right);
		if (robot["robot_joint"][iter->right]) {
			bool successful_replace = _bimap_JointName.left.replace_data( it, robot["robot_joint"][iter->right].as<std::string>() );
			assert( successful_replace );
		}
		else {
			nameToRemove.push_back(iter->right);
		}
		// COUT("a:", iter->left, iter->right);
	}
	if (!nameToRemove.empty()) {
		for (auto it : nameToRemove) {
			_bimap_JointName.right.erase(it);
			COUT("remove joint: ", it);
		}
		nameToRemove.clear();
	}
	COUT("_bimap_JointName.size() after", _bimap_JointName.size());

#ifdef PARSE_OFF_TRAJ

	arm_off_traj.reset(new TrajParserClass());
	Eigen::VectorXd start_pos = 180.0 / M_PI * arm_off_traj->getRef(); // to degree
	arm_off_traj->resetTraj();
	set_POS_FROM_OFF_TRAJ(start_pos, homing_pos);
	COUT("\n\n\nThe start pos load from offline traj:", start_pos.transpose(), "\n\n\n");
#endif


	// set_HOMING_POS_FOR_COMAN();
	set_POS_FOR_COMAN(homing_pos, homing_pos_for_coman);
	set_POS_FOR_COMAN(offset_pos, offset_pos_for_coman);

	for (int i = 0; i < homing_pos_for_coman.size(); i++) {
		homing_pos_for_coman[i] += offset_pos_for_coman[i];
	}

	// boost::shared_ptr<urdf::Joint> jnt;
	urdf::JointConstSharedPtr jnt;
	double x, y, z;

	// for calf height
	jnt = getLink(LEFT_FOOT_DUMMY)->parent_joint;
	CALF_HEIGHT = std::abs(jnt->parent_to_joint_origin_transform.position.z);
	// for thigh height and half hip width
	x = y = z = 0;
	jnt = getLink(LEFT_FOOT)->parent_joint; // modified for cogimon
	// jnt = getLink(LEFT_FOOT_DUMM)->parent_joint;
	while (jnt->parent_link_name != getLinkName(PELVIS)) {
		// while (jnt->parent_link_name != getLinkName(LEFT_THIGH_DUMMY1)) { // no need the first link for cogimon lower body only
		jnt = getLink(jnt->parent_link_name)->parent_joint;
		x += jnt->parent_to_joint_origin_transform.position.x;
		y += jnt->parent_to_joint_origin_transform.position.y;
		z += jnt->parent_to_joint_origin_transform.position.z;
		// COUT(jnt->name, jnt->parent_link_name);
		// COUT(x, y, z);
	}
	// half_hip_width = std::abs(y);
	// THIGH_HEIGHT = std::abs(z);
	THIGH_HEIGHT = std::abs(z) - CALF_HEIGHT; // modified for cogimon
	hip_to_ankle_x_offset = x;

	if (!IsOnlyLowerBody) {
		// for TORSO_HEIGHT
		x = y = z = 0;
		// jnt = getLink(NECK)->parent_joint;
		jnt = getLink(LEFT_UPPER_ARM_DUMMY1)->parent_joint;
		while (jnt->parent_link_name != getLinkName(PELVIS)) {
			z += jnt->parent_to_joint_origin_transform.position.z;
			jnt = getLink(jnt->parent_link_name)->parent_joint;
		}
		TORSO_HEIGHT = std::abs(z);

		// for PELVIS_HEIGHT
		jnt = getLink(WAIST_DUMMY1)->parent_joint;
		PELVIS_HEIGHT = std::abs(jnt->parent_to_joint_origin_transform.position.z);

		// for UPPER_ARM_HEIGHT
		x = y = z = 0;
		jnt = getLink(LEFT_FORE_ARM)->parent_joint;
		while (jnt->parent_link_name != getLinkName(LEFT_UPPER_ARM_DUMMY1)) {
			jnt = getLink(jnt->parent_link_name)->parent_joint;
			z += std::abs(jnt->parent_to_joint_origin_transform.position.z);
		}
		UPPER_ARM_HEIGHT = std::abs(z);

		// for HALF_SHOULDER_WIDTH
		x = y = z = 0;
		jnt = getLink(LEFT_FORE_ARM)->parent_joint;
		while (jnt->parent_link_name != getLinkName(TORSO)) {
			jnt = getLink(jnt->parent_link_name)->parent_joint;
			y += jnt->parent_to_joint_origin_transform.position.y;
		}
		HALF_SHOULDER_WIDTH = std::abs(y);

		// for FORE_ARM_HEIGHT
		x = y = z = 0;
		jnt = getLink(LEFT_HAND)->parent_joint;
		while (jnt->parent_link_name != getLinkName(LEFT_UPPER_ARM)) {
			z += jnt->parent_to_joint_origin_transform.position.z;
			jnt = getLink(jnt->parent_link_name)->parent_joint;
		}
		FORE_ARM_HEIGHT = std::abs(z);

		// for PELVIS_HEIGHT
		jnt = getLink(WAIST_DUMMY1)->parent_joint;
		HIP_BODY_OFFSET_X = jnt->parent_to_joint_origin_transform.position.x;
	}
	else {
		TORSO_HEIGHT = PELVIS_HEIGHT = UPPER_ARM_HEIGHT = HALF_SHOULDER_WIDTH = FORE_ARM_HEIGHT = HIP_BODY_OFFSET_X = 0.0;
	}

	COUT("TORSO_HEIGHT", TORSO_HEIGHT);
	COUT("PELVIS_HEIGHT", PELVIS_HEIGHT);
	COUT("HALF_SHOULDER_WIDTH", HALF_SHOULDER_WIDTH);
	COUT("UPPER_ARM_HEIGHT", UPPER_ARM_HEIGHT);
	COUT("FORE_ARM_HEIGHT", FORE_ARM_HEIGHT);
	COUT("HIP_BODY_OFFSET_X", HIP_BODY_OFFSET_X);

	/* Tue 25 Apr 2017 03:21:16 PM CEST
	   cast boost::shared_ptr<urdf::Geometry> to boost::shared_ptr<urdf::Box>
	   http://stackoverflow.com/questions/624854/static-cast-with-boostshared-ptr
	   */
	boost::shared_ptr<urdf::Box> FootBox = boost::static_pointer_cast<urdf::Box>(getLink(LEFT_FOOT)->collision->geometry);

	foot_length = FootBox->dim.x;
	foot_width = FootBox->dim.y;
	// foot_height = FootBox->dim.z;

	urdf::Vector3 FootBoxOrigin = getLink(LEFT_FOOT)->collision->origin.position;
	ankle_x_offset = FootBoxOrigin.x;
	ankle_height = std::abs(FootBoxOrigin.z);

	x = y = z = 0;
	jnt = getLink(LEFT_FOOT)->parent_joint; // modified for cogimon
	while (jnt->parent_link_name != getLinkName(PELVIS)) {
		jnt = getLink(jnt->parent_link_name)->parent_joint;
		x += jnt->parent_to_joint_origin_transform.position.x;
		y += jnt->parent_to_joint_origin_transform.position.y;
		z += jnt->parent_to_joint_origin_transform.position.z;
	}
	half_hip_width = std::abs(y);
	waist_height = std::abs(z) + ankle_height;

	// foot_length = robot["robot_para"]["FOOT_LENGTH"].as<double>();
	// foot_width = robot["robot_para"]["FOOT_WIDTH"].as<double>();
	foot_height = robot["robot_para"]["FOOT_HEIGHT"].as<double>();
	// ankle_x_offset = robot["robot_para"]["ANKLE_X_OFFSET"].as<double>();
	// ankle_height = robot["robot_para"]["ANKLE_HEIGHT"].as<double>();
	// THIGH_HEIGHT = robot["robot_para"]["THIGH_HEIGHT"].as<double>();
	// CALF_HEIGHT = robot["robot_para"]["CALF_HEIGHT"].as<double>();
	// half_hip_width = robot["robot_para"]["HALF_HIP_WIDTH"].as<double>();
	double foot_dis_increment = robot["robot_para"]["HALF_FOOT_DIS_INCREMENT"].as<double>();
	half_foot_dis = half_hip_width + foot_dis_increment;
	// half_foot_dis = robot["robot_para"]["HALF_FOOT_DIS"].as<double>();

	full_leg = THIGH_HEIGHT + CALF_HEIGHT + ankle_height;

	// TORSO_HEIGHT		= 0.2029;   // z axis
	// PELVIS_HEIGHT		= 0.1191;
	// HALF_SHOULDER_WIDTH	= 0.1535;
	// UPPER_ARM_HEIGHT		= 0.1800;
	// FORE_ARM_HEIGHT		= 0.1800;
	// HIP_BODY_OFFSET_X	= 0.0203;

	COUT("foot_length", foot_length);
	COUT("foot_width", foot_width);
	COUT("foot_height", foot_height);
	COUT("ANKLE_X_OFFSET", ankle_x_offset);
	COUT("hip_to_ankle_x_offset", hip_to_ankle_x_offset);
	COUT("ankle_height", ankle_height);

	COUT("HALF_HIP_WIDTH", half_hip_width);
	COUT("HALF_FOOT_DIS", half_foot_dis);
	COUT("THIGH_HEIGHT", THIGH_HEIGHT);
	COUT("CALF_HEIGHT", CALF_HEIGHT);
	COUT("ANKLE_HEIGHT", ankle_height);
	COUT("TORSO_HEIGHT", TORSO_HEIGHT);
	COUT("PELVIS_HEIGHT", PELVIS_HEIGHT);
	COUT("full_leg", full_leg);
	COUT("HALF_SHOULDER_WIDTH", HALF_SHOULDER_WIDTH);
	COUT("UPPER_ARM_HEIGHT", UPPER_ARM_HEIGHT);
	COUT("FORE_ARM_HEIGHT", FORE_ARM_HEIGHT);
	COUT("HIP_BODY_OFFSET_X", HIP_BODY_OFFSET_X);


	IsFixedWalk = robot["gait_para"]["IsFixedWalk"].as<bool>();
	HasEntrySway = robot["gait_para"]["HasEntrySway"].as<bool>();
	homing_lift_height = robot["gait_para"]["homing_lift_height"].as<double>();
	LIFT_HEIGHT = robot["gait_para"]["lift_height"].as<double>();
	z_c = robot["gait_para"]["z_c"].as<double>();

	// homing_lift_height = full_leg * 0.1;

	COUT("HOMING_LIFT_HEIGHT", homing_lift_height);
	COUT("LIFT_HEIGHT", LIFT_HEIGHT);
	COUT("z_c", z_c);

	PreT = robot["gait_para"]["PreviewT"].as<double>();
	Tstep = robot["gait_para"]["Tstep"].as<double>();

	if (TRUE_SIM_FALSE_REALROBOT) {
		Ksway = robot["gait_para"]["for_simulation"]["Ksway"].as<double>();
		ft_height = ankle_height;
	}
	else {
		Ksway = robot["gait_para"]["for_realrobot"]["Ksway"].as<double>();
		ft_height = robot["gait_para"]["for_realrobot"]["FT_HEIGHT"].as<double>();
		HipCompL = robot["gait_para"]["for_realrobot"]["HipCompL"].as<double>();
		HipCompR = robot["gait_para"]["for_realrobot"]["HipCompR"].as<double>();
	}

	COUT("Ksway", Ksway);
	COUT("FT_HEIGHT", ft_height);

	Kx = robot["stabilizer"]["Kx"].as<double>();
	Bx = robot["stabilizer"]["Bx"].as<double>();
	Ky = robot["stabilizer"]["Ky"].as<double>();
	By = robot["stabilizer"]["By"].as<double>();
	Kz = robot["stabilizer"]["Kz"].as<double>();
	Bz = robot["stabilizer"]["Bz"].as<double>();

	Kx_Hand = robot["hand_stabilizer"]["Kx"].as<double>();
	Bx_Hand = robot["hand_stabilizer"]["Bx"].as<double>();
	Ky_Hand = robot["hand_stabilizer"]["Ky"].as<double>();
	By_Hand = robot["hand_stabilizer"]["By"].as<double>();
	Kz_Hand = robot["hand_stabilizer"]["Kz"].as<double>();
	Bz_Hand = robot["hand_stabilizer"]["Bz"].as<double>();

	K_roll_Hand = robot["hand_stabilizer"]["K_roll"].as<double>();
	B_roll_Hand = robot["hand_stabilizer"]["B_roll"].as<double>();
	K_pitch_Hand = robot["hand_stabilizer"]["K_pitch"].as<double>();
	B_pitch_Hand = robot["hand_stabilizer"]["B_pitch"].as<double>();
	K_yaw_Hand = robot["hand_stabilizer"]["K_yaw"].as<double>();
	B_yaw_Hand = robot["hand_stabilizer"]["B_yaw"].as<double>();

	// double addtional_w = 6.0; //kg for real robot
	double addtional_w = setParaForSimOrReal(0.0, 10.0); //kg
	// double addtional_w = 30.0; //kg for gazebo with pennacchio

	totalmass = getTotalMass(_urdf_model) + addtional_w;
	COUT("Robot total mass is:", totalmass);

	if (!IsOnlyLowerBody) {
		TORSO_MASS = addtional_w + getLink(TORSO)->inertial->mass;
		UPPER_ARM_MASS = getLink(LEFT_UPPER_ARM_DUMMY1)->inertial->mass + getLink(LEFT_UPPER_ARM_DUMMY2)->inertial->mass + getLink(LEFT_UPPER_ARM)->inertial->mass;
		FORE_ARM_MASS = getLink(LEFT_ELBOW_FORE_ARM)->inertial->mass + getLink(LEFT_FORE_ARM)->inertial->mass + getLink(LEFT_HAND_DUMMY1)->inertial->mass;
		HAND_MASS = getLink(LEFT_HAND)->inertial->mass;
		PELVIS_MASS = getLink(PELVIS)->inertial->mass + getLink(WAIST_DUMMY1)->inertial->mass;
		if (_urdf_model->getName() != "cogimon") {
			if (_urdf_model->getName() != "walkman") {
				PELVIS_MASS += getLink(WAIST_DUMMY2)->inertial->mass;
			}
			HEAD_MASS = getLink(NECK)->inertial->mass + getLink(HEAD)->inertial->mass;
		}
	}
	else {
		PELVIS_MASS = getLink(PELVIS)->inertial->mass;
		HEAD_MASS = TORSO_MASS = UPPER_ARM_MASS = FORE_ARM_MASS = HAND_MASS = 0.0;
	}

	THIGH_MASS = getLink(LEFT_THIGH)->inertial->mass + getLink(LEFT_THIGH_DUMMY2)->inertial->mass + getLink(LEFT_THIGH_DUMMY1)->inertial->mass;
	CALF_MASS = getLink(LEFT_CALF)->inertial->mass + getLink(LEFT_FOOT_DUMMY)->inertial->mass;
	FOOT_MASS = getLink(LEFT_FOOT)->inertial->mass;

	COUT("HEAD_MASS", HEAD_MASS	);
	COUT("TORSO_MASS", TORSO_MASS	);
	COUT("PELVIS_MASS", PELVIS_MASS	);
	COUT("THIGH_MASS", THIGH_MASS	);
	COUT("CALF_MASS", CALF_MASS	);
	COUT("FOOT_MASS", FOOT_MASS	);
	COUT("UPPER_ARM_MASS", UPPER_ARM_MASS);
	COUT("FORE_ARM_MASS", FORE_ARM_MASS);
	COUT("HAND_MASS", HAND_MASS	);

	kp_torque = robot["joint_gain"]["kp_torque"].as<double>();
	kd_torque = robot["joint_gain"]["kd_torque"].as<double>();
	kp_imp = robot["joint_gain"]["kp_imp"].as<double>();
	kd_imp = robot["joint_gain"]["kd_imp"].as<double>();
	kp_pos = robot["joint_gain"]["kp_pos"].as<double>();
	kd_pos = robot["joint_gain"]["kd_pos"].as<double>();
	kp_adm = robot["joint_gain"]["kp_adm"].as<double>();
	kd_adm = robot["joint_gain"]["kd_adm"].as<double>();

	if (TRUE_SIM_FALSE_REALROBOT) {
#ifdef IN_GAZEBO
		ik_pos_gain = robot["cart_ref_gains_for_IKQP"]["k_pos_gaz"].as<double>();
		ik_ori_gain = robot["cart_ref_gains_for_IKQP"]["k_ori_gaz"].as<double>();
		ik_kp_pos_acc_gain = robot["cart_ref_gains_for_IKQP"]["kp_pos_acc_gaz"].as<double>();
		ik_kd_pos_acc_gain = robot["cart_ref_gains_for_IKQP"]["kd_pos_acc_gaz"].as<double>();
		ik_kp_ori_acc_gain = robot["cart_ref_gains_for_IKQP"]["kp_ori_acc_gaz"].as<double>();
		ik_kd_ori_acc_gain = robot["cart_ref_gains_for_IKQP"]["kd_ori_acc_gaz"].as<double>();
#else
		ik_pos_gain = robot["cart_ref_gains_for_IKQP"]["k_pos_ode"].as<double>();
		ik_ori_gain = robot["cart_ref_gains_for_IKQP"]["k_ori_ode"].as<double>();
		ik_kp_pos_acc_gain = robot["cart_ref_gains_for_IKQP"]["kp_pos_acc_ode"].as<double>();
		ik_kd_pos_acc_gain = robot["cart_ref_gains_for_IKQP"]["kd_pos_acc_ode"].as<double>();
		ik_kp_ori_acc_gain = robot["cart_ref_gains_for_IKQP"]["kp_ori_acc_ode"].as<double>();
		ik_kd_ori_acc_gain = robot["cart_ref_gains_for_IKQP"]["kd_ori_acc_ode"].as<double>();
#endif
	}
	else {
		ik_pos_gain = robot["cart_ref_gains_for_IKQP"]["k_pos_rob"].as<double>();
		ik_ori_gain = robot["cart_ref_gains_for_IKQP"]["k_ori_rob"].as<double>();
		ik_kp_pos_acc_gain = robot["cart_ref_gains_for_IKQP"]["kp_pos_acc_rob"].as<double>();
		ik_kd_pos_acc_gain = robot["cart_ref_gains_for_IKQP"]["kd_pos_acc_rob"].as<double>();
		ik_kp_ori_acc_gain = robot["cart_ref_gains_for_IKQP"]["kp_ori_acc_rob"].as<double>();
		ik_kd_ori_acc_gain = robot["cart_ref_gains_for_IKQP"]["kd_ori_acc_rob"].as<double>();
	}

	COUT("ik_pos_gain", ik_pos_gain);
	COUT("ik_ori_gain", ik_ori_gain);
	COUT("ik_kp_pos_acc_gain", ik_kp_pos_acc_gain);
	COUT("ik_kd_pos_acc_gain", ik_kd_pos_acc_gain);
	COUT("ik_kp_ori_acc_gain", ik_kp_ori_acc_gain);
	COUT("ik_kd_ori_acc_gain", ik_kd_ori_acc_gain);

	kp_debug = robot["for_debug"]["kp_debug"].as<double>();
	kd_debug = robot["for_debug"]["kd_debug"].as<double>();

	robot_count ++;

	RobotNo = robot_count;

	std::cout << "Finish init RobotParaClass No." << robot_count << std::endl;
	std::cout << "=====================================================\n\n\n" << std::endl;

};

RobotParaClass::~RobotParaClass()
{

};

void RobotParaClass::createLinkAndJointNameMap()
{
	_bimap_LinkName.insert( bm_type::value_type(PELVIS					, "PELVIS"					) );
	_bimap_LinkName.insert( bm_type::value_type(RIGHT_THIGH_DUMMY1		, "RIGHT_THIGH_DUMMY1"		) );
	_bimap_LinkName.insert( bm_type::value_type(RIGHT_THIGH_DUMMY2		, "RIGHT_THIGH_DUMMY2"		) );
	_bimap_LinkName.insert( bm_type::value_type(RIGHT_THIGH				, "RIGHT_THIGH"				) );
	_bimap_LinkName.insert( bm_type::value_type(RIGHT_CALF				, "RIGHT_CALF"				) );
	_bimap_LinkName.insert( bm_type::value_type(RIGHT_FOOT_DUMMY		, "RIGHT_FOOT_DUMMY"		) );
	_bimap_LinkName.insert( bm_type::value_type(RIGHT_FOOT				, "RIGHT_FOOT"				) );
	_bimap_LinkName.insert( bm_type::value_type(LEFT_THIGH_DUMMY1		, "LEFT_THIGH_DUMMY1"		) );
	_bimap_LinkName.insert( bm_type::value_type(LEFT_THIGH_DUMMY2		, "LEFT_THIGH_DUMMY2"		) );
	_bimap_LinkName.insert( bm_type::value_type(LEFT_THIGH				, "LEFT_THIGH"				) );
	_bimap_LinkName.insert( bm_type::value_type(LEFT_CALF				, "LEFT_CALF"				) );
	_bimap_LinkName.insert( bm_type::value_type(LEFT_FOOT_DUMMY			, "LEFT_FOOT_DUMMY"			) );
	_bimap_LinkName.insert( bm_type::value_type(LEFT_FOOT				, "LEFT_FOOT"				) );
	_bimap_LinkName.insert( bm_type::value_type(WAIST_DUMMY1			, "WAIST_DUMMY1"			) );
	_bimap_LinkName.insert( bm_type::value_type(WAIST_DUMMY2			, "WAIST_DUMMY2"			) );
	_bimap_LinkName.insert( bm_type::value_type(TORSO					, "TORSO"					) );
	_bimap_LinkName.insert( bm_type::value_type(NECK					, "NECK"					) );
	_bimap_LinkName.insert( bm_type::value_type(HEAD					, "HEAD"					) );
	_bimap_LinkName.insert( bm_type::value_type(RIGHT_UPPER_ARM_DUMMY1	, "RIGHT_UPPER_ARM_DUMMY1"	) );
	_bimap_LinkName.insert( bm_type::value_type(RIGHT_UPPER_ARM_DUMMY2	, "RIGHT_UPPER_ARM_DUMMY2"	) );
	_bimap_LinkName.insert( bm_type::value_type(RIGHT_UPPER_ARM			, "RIGHT_UPPER_ARM"			) );
	_bimap_LinkName.insert( bm_type::value_type(RIGHT_ELBOW_FORE_ARM	, "RIGHT_ELBOW_FORE_ARM"	) );
	_bimap_LinkName.insert( bm_type::value_type(RIGHT_FORE_ARM			, "RIGHT_FORE_ARM"			) );
	_bimap_LinkName.insert( bm_type::value_type(RIGHT_HAND_DUMMY1		, "RIGHT_HAND_DUMMY1"		) );
	// _bimap_LinkName.insert( bm_type::value_type(RIGHT_HAND_DUMMY2		, "RIGHT_HAND_DUMMY2"		) );
	_bimap_LinkName.insert( bm_type::value_type(RIGHT_HAND				, "RIGHT_HAND"				) );
	_bimap_LinkName.insert( bm_type::value_type(LEFT_UPPER_ARM_DUMMY1	, "LEFT_UPPER_ARM_DUMMY1"	) );
	_bimap_LinkName.insert( bm_type::value_type(LEFT_UPPER_ARM_DUMMY2	, "LEFT_UPPER_ARM_DUMMY2"	) );
	_bimap_LinkName.insert( bm_type::value_type(LEFT_UPPER_ARM			, "LEFT_UPPER_ARM"			) );
	_bimap_LinkName.insert( bm_type::value_type(LEFT_ELBOW_FORE_ARM		, "LEFT_ELBOW_FORE_ARM"		) );
	_bimap_LinkName.insert( bm_type::value_type(LEFT_FORE_ARM			, "LEFT_FORE_ARM"			) );
	_bimap_LinkName.insert( bm_type::value_type(LEFT_HAND_DUMMY1		, "LEFT_HAND_DUMMY1"		) );
	// _bimap_LinkName.insert( bm_type::value_type(LEFT_HAND_DUMMY2		, "LEFT_HAND_DUMMY2"		) );
	_bimap_LinkName.insert( bm_type::value_type(LEFT_HAND				, "LEFT_HAND"				) );


	_bimap_JointName.insert( bm_type::value_type(WAIST_ROLL           , "WAIST_ROLL"           ) );
	_bimap_JointName.insert( bm_type::value_type(WAIST_PITCH          , "WAIST_PITCH"          ) );
	_bimap_JointName.insert( bm_type::value_type(WAIST_YAW            , "WAIST_YAW"            ) );
	_bimap_JointName.insert( bm_type::value_type(RIGHT_HIP_PITCH      , "RIGHT_HIP_PITCH"      ) );
	_bimap_JointName.insert( bm_type::value_type(LEFT_HIP_PITCH       , "LEFT_HIP_PITCH"       ) );
	_bimap_JointName.insert( bm_type::value_type(RIGHT_HIP_ROLL       , "RIGHT_HIP_ROLL"       ) );
	_bimap_JointName.insert( bm_type::value_type(RIGHT_HIP_YAW        , "RIGHT_HIP_YAW"        ) );
	_bimap_JointName.insert( bm_type::value_type(RIGHT_KNEE_PITCH     , "RIGHT_KNEE_PITCH"     ) );
	_bimap_JointName.insert( bm_type::value_type(RIGHT_FOOT_ROLL      , "RIGHT_FOOT_ROLL"      ) );
	_bimap_JointName.insert( bm_type::value_type(RIGHT_FOOT_PITCH     , "RIGHT_FOOT_PITCH"     ) );
	_bimap_JointName.insert( bm_type::value_type(LEFT_HIP_ROLL        , "LEFT_HIP_ROLL"        ) );
	_bimap_JointName.insert( bm_type::value_type(LEFT_HIP_YAW         , "LEFT_HIP_YAW"         ) );
	_bimap_JointName.insert( bm_type::value_type(LEFT_KNEE_PITCH      , "LEFT_KNEE_PITCH"      ) );
	_bimap_JointName.insert( bm_type::value_type(LEFT_FOOT_ROLL       , "LEFT_FOOT_ROLL"       ) );
	_bimap_JointName.insert( bm_type::value_type(LEFT_FOOT_PITCH      , "LEFT_FOOT_PITCH"      ) );
	_bimap_JointName.insert( bm_type::value_type(RIGHT_SHOULDER_PITCH , "RIGHT_SHOULDER_PITCH" ) );
	_bimap_JointName.insert( bm_type::value_type(RIGHT_SHOULDER_ROLL  , "RIGHT_SHOULDER_ROLL"  ) );
	_bimap_JointName.insert( bm_type::value_type(RIGHT_SHOULDER_YAW   , "RIGHT_SHOULDER_YAW"   ) );
	_bimap_JointName.insert( bm_type::value_type(RIGHT_ELBOW_PITCH    , "RIGHT_ELBOW_PITCH"    ) );
	_bimap_JointName.insert( bm_type::value_type(RIGHT_FOREARM_YAW    , "RIGHT_FOREARM_YAW"    ) );
	_bimap_JointName.insert( bm_type::value_type(RIGHT_WRIST_PITCH    , "RIGHT_WRIST_PITCH"    ) );
	_bimap_JointName.insert( bm_type::value_type(RIGHT_WRIST_ROLL     , "RIGHT_WRIST_ROLL"     ) );
	_bimap_JointName.insert( bm_type::value_type(LEFT_SHOULDER_PITCH  , "LEFT_SHOULDER_PITCH"  ) );
	_bimap_JointName.insert( bm_type::value_type(LEFT_SHOULDER_ROLL   , "LEFT_SHOULDER_ROLL"   ) );
	_bimap_JointName.insert( bm_type::value_type(LEFT_SHOULDER_YAW    , "LEFT_SHOULDER_YAW"    ) );
	_bimap_JointName.insert( bm_type::value_type(LEFT_ELBOW_PITCH     , "LEFT_ELBOW_PITCH"     ) );
	_bimap_JointName.insert( bm_type::value_type(LEFT_FOREARM_YAW     , "LEFT_FOREARM_YAW"     ) );
	_bimap_JointName.insert( bm_type::value_type(LEFT_WRIST_PITCH     , "LEFT_WRIST_PITCH"     ) );
	_bimap_JointName.insert( bm_type::value_type(LEFT_WRIST_ROLL      , "LEFT_WRIST_ROLL"      ) );
	_bimap_JointName.insert( bm_type::value_type(NECK_PITCH           , "NECK_PITCH"           ) );
	_bimap_JointName.insert( bm_type::value_type(HEAD_PITCH           , "HEAD_PITCH"           ) );


	// _map_LinkName_to_String[PELVIS					] = "PELVIS"				;
	// _map_LinkName_to_String[RIGHT_THIGH_DUMMY1		] = "RIGHT_THIGH_DUMMY1"	;
	// _map_LinkName_to_String[RIGHT_THIGH_DUMMY2		] = "RIGHT_THIGH_DUMMY2"	;
	// _map_LinkName_to_String[RIGHT_THIGH				] = "RIGHT_THIGH"			;
	// _map_LinkName_to_String[RIGHT_CALF				] = "RIGHT_CALF"			;
	// _map_LinkName_to_String[RIGHT_FOOT_DUMMY		] = "RIGHT_FOOT_DUMMY"		;
	// _map_LinkName_to_String[RIGHT_FOOT				] = "RIGHT_FOOT"			;
	// _map_LinkName_to_String[LEFT_THIGH_DUMMY1		] = "LEFT_THIGH_DUMMY1"		;
	// _map_LinkName_to_String[LEFT_THIGH_DUMMY2		] = "LEFT_THIGH_DUMMY2"		;
	// _map_LinkName_to_String[LEFT_THIGH				] = "LEFT_THIGH"			;
	// _map_LinkName_to_String[LEFT_CALF				] = "LEFT_CALF"				;
	// _map_LinkName_to_String[LEFT_FOOT_DUMMY			] = "LEFT_FOOT_DUMMY"		;
	// _map_LinkName_to_String[LEFT_FOOT				] = "LEFT_FOOT"				;
	// _map_LinkName_to_String[WAIST_DUMMY1			] = "WAIST_DUMMY1"			;
	// _map_LinkName_to_String[WAIST_DUMMY2			] = "WAIST_DUMMY2"			;
	// _map_LinkName_to_String[TORSO					] = "TORSO"					;
	// _map_LinkName_to_String[NECK					] = "NECK"					;
	// _map_LinkName_to_String[HEAD					] = "HEAD"					;
	// _map_LinkName_to_String[RIGHT_UPPER_ARM_DUMMY1	] = "RIGHT_UPPER_ARM_DUMMY1";
	// _map_LinkName_to_String[RIGHT_UPPER_ARM_DUMMY2	] = "RIGHT_UPPER_ARM_DUMMY2";
	// _map_LinkName_to_String[RIGHT_UPPER_ARM			] = "RIGHT_UPPER_ARM"		;
	// _map_LinkName_to_String[RIGHT_ELBOW_FORE_ARM	] = "RIGHT_ELBOW_FORE_ARM"	;
	// _map_LinkName_to_String[RIGHT_FORE_ARM			] = "RIGHT_FORE_ARM"		;
	// _map_LinkName_to_String[RIGHT_HAND_DUMMY1		] = "RIGHT_HAND_DUMMY1"		;
	// _map_LinkName_to_String[RIGHT_HAND_DUMMY2		] = "RIGHT_HAND_DUMMY2"		;
	// _map_LinkName_to_String[RIGHT_HAND				] = "RIGHT_HAND"			;
	// _map_LinkName_to_String[LEFT_UPPER_ARM_DUMMY1	] = "LEFT_UPPER_ARM_DUMMY1"	;
	// _map_LinkName_to_String[LEFT_UPPER_ARM_DUMMY2	] = "LEFT_UPPER_ARM_DUMMY2"	;
	// _map_LinkName_to_String[LEFT_UPPER_ARM			] = "LEFT_UPPER_ARM"		;
	// _map_LinkName_to_String[LEFT_ELBOW_FORE_ARM		] = "LEFT_ELBOW_FORE_ARM"	;
	// _map_LinkName_to_String[LEFT_FORE_ARM			] = "LEFT_FORE_ARM"			;
	// _map_LinkName_to_String[LEFT_HAND_DUMMY1		] = "LEFT_HAND_DUMMY1"		;
	// _map_LinkName_to_String[LEFT_HAND_DUMMY2		] = "LEFT_HAND_DUMMY2"		;
	// _map_LinkName_to_String[LEFT_HAND				] = "LEFT_HAND"				;


	// _map_JointName_to_String[WAIST_YAW             ] = "WAIST_YAW"            ;
	// _map_JointName_to_String[WAIST_PITCH           ] = "WAIST_PITCH"          ;
	// _map_JointName_to_String[WAIST_ROLL            ] = "WAIST_ROLL"           ;
	// _map_JointName_to_String[RIGHT_HIP_PITCH       ] = "RIGHT_HIP_PITCH"      ;
	// _map_JointName_to_String[LEFT_HIP_PITCH        ] = "LEFT_HIP_PITCH"       ;
	// _map_JointName_to_String[RIGHT_HIP_ROLL        ] = "RIGHT_HIP_ROLL"       ;
	// _map_JointName_to_String[RIGHT_HIP_YAW         ] = "RIGHT_HIP_YAW"        ;
	// _map_JointName_to_String[RIGHT_KNEE_PITCH      ] = "RIGHT_KNEE_PITCH"     ;
	// _map_JointName_to_String[RIGHT_FOOT_ROLL       ] = "RIGHT_FOOT_ROLL"      ;
	// _map_JointName_to_String[RIGHT_FOOT_PITCH      ] = "RIGHT_FOOT_PITCH"     ;
	// _map_JointName_to_String[LEFT_HIP_ROLL         ] = "LEFT_HIP_ROLL"        ;
	// _map_JointName_to_String[LEFT_HIP_YAW          ] = "LEFT_HIP_YAW"         ;
	// _map_JointName_to_String[LEFT_KNEE_PITCH       ] = "LEFT_KNEE_PITCH"      ;
	// _map_JointName_to_String[LEFT_FOOT_ROLL        ] = "LEFT_FOOT_ROLL"       ;
	// _map_JointName_to_String[LEFT_FOOT_PITCH       ] = "LEFT_FOOT_PITCH"      ;
	// _map_JointName_to_String[RIGHT_SHOULDER_PITCH  ] = "RIGHT_SHOULDER_PITCH" ;
	// _map_JointName_to_String[RIGHT_SHOULDER_ROLL   ] = "RIGHT_SHOULDER_ROLL"  ;
	// _map_JointName_to_String[RIGHT_SHOULDER_YAW    ] = "RIGHT_SHOULDER_YAW"   ;
	// _map_JointName_to_String[RIGHT_ELBOW_PITCH     ] = "RIGHT_ELBOW_PITCH"    ;
	// _map_JointName_to_String[RIGHT_FOREARM_YAW     ] = "RIGHT_FOREARM_YAW"    ;
	// _map_JointName_to_String[RIGHT_WRIST_PITCH     ] = "RIGHT_WRIST_PITCH"    ;
	// _map_JointName_to_String[RIGHT_WRIST_ROLL      ] = "RIGHT_WRIST_ROLL"     ;
	// _map_JointName_to_String[LEFT_SHOULDER_PITCH   ] = "LEFT_SHOULDER_PITCH"  ;
	// _map_JointName_to_String[LEFT_SHOULDER_ROLL    ] = "LEFT_SHOULDER_ROLL"   ;
	// _map_JointName_to_String[LEFT_SHOULDER_YAW     ] = "LEFT_SHOULDER_YAW"    ;
	// _map_JointName_to_String[LEFT_ELBOW_PITCH      ] = "LEFT_ELBOW_PITCH"     ;
	// _map_JointName_to_String[LEFT_FOREARM_YAW      ] = "LEFT_FOREARM_YAW"     ;
	// _map_JointName_to_String[LEFT_WRIST_PITCH      ] = "LEFT_WRIST_PITCH"     ;
	// _map_JointName_to_String[LEFT_WRIST_ROLL       ] = "LEFT_WRIST_ROLL"      ;

}

// double RobotParaClass::getTotalMass(const urdf::Model & model)
double RobotParaClass::getTotalMass(const urdf::ModelInterfaceSharedPtr & model)
{
	// std::vector<boost::shared_ptr<urdf::Link> > input_links;
	std::vector<urdf::LinkSharedPtr > input_links;

	model->getLinks(input_links);

	double total_mass = 0.0;

	// std::cout << "Found " << input_links.size() << " links in input URDF " << std::endl;
	for (int i = 0; i < input_links.size(); i++ )
	{
		if ( input_links[i]->inertial ) {
			// COUT(input_links[i]->name, input_links[i]->inertial->mass);
			total_mass = total_mass + input_links[i]->inertial->mass;
		}
	}

	// double total_mass2 = 0.0;
	// for (auto it : _map_LinkName_to_String)
	// {
	// 	std::string linkname = it.second;

	// 	if (_urdf_model->getLink(linkname)->inertial) {
	// 		COUT(it.second, _urdf_model->getLink(linkname)->inertial->mass);
	// 		total_mass2 = total_mass2 + _urdf_model->getLink(linkname)->inertial->mass;
	// 	}
	// }

	return total_mass;
}

void RobotParaClass::set_POS_FOR_COMAN(const std::vector<double> &from, std::vector<double> &to)
{
	to.resize(33);
	to[ 0] = from[WAIST_YAW	 ];
	to[ 1] = from[WAIST_PITCH];
	to[ 2] = from[WAIST_ROLL ];
	to[ 3] = from[RIGHT_HIP_PITCH ];
	to[ 4] = from[LEFT_HIP_PITCH  ];
	to[ 5] = from[RIGHT_HIP_ROLL  ];
	to[ 6] = from[RIGHT_HIP_YAW   ];
	to[ 7] = from[RIGHT_KNEE_PITCH];
	to[ 8] = from[RIGHT_FOOT_PITCH];
	to[ 9] = from[RIGHT_FOOT_ROLL ];
	to[10] = from[LEFT_HIP_ROLL   ];
	to[11] = from[LEFT_HIP_YAW    ];
	to[12] = from[LEFT_KNEE_PITCH ];
	to[13] = from[LEFT_FOOT_PITCH ];
	to[14] = from[LEFT_FOOT_ROLL  ];
	to[15] = from[RIGHT_SHOULDER_PITCH];
	to[16] = from[RIGHT_SHOULDER_ROLL ];
	to[17] = from[RIGHT_SHOULDER_YAW  ];
	to[18] = from[RIGHT_ELBOW_PITCH   ];
	to[19] = from[LEFT_SHOULDER_PITCH ];
	to[20] = from[LEFT_SHOULDER_ROLL  ];
	to[21] = from[LEFT_SHOULDER_YAW   ];
	to[22] = from[LEFT_ELBOW_PITCH    ];
	to[23] = from[NECK_PITCH  ];
	to[24] = from[HEAD_PITCH  ];
	to[25] = from[RIGHT_FOREARM_YAW ];
	to[26] = from[RIGHT_WRIST_PITCH ];
	to[27] = from[RIGHT_WRIST_ROLL  ];
	to[28] = from[LEFT_FOREARM_YAW  ];
	to[29] = from[LEFT_WRIST_PITCH  ];
	to[30] = from[LEFT_WRIST_ROLL   ];
	// to[31] = from[RIGHT_HAND_GRASP  ];
	// to[32] = from[LEFT_HAND_GRASP   ];
};

void RobotParaClass::printTree(urdf::LinkConstSharedPtr link, int level)
{
// 	level += 2;
// 	int count = 0;
// 	for (std::vector<urdf::Link>::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
// 	{
// 		if (*child)
// 		{
// 			for (int j = 0; j < level; j++) std::cout << "  "; //indent
// 			std::cout << "child(" << (count++) + 1 << "):  " << (*child)->name  << std::endl;
// 			// first grandchild
// 			printTree(*child, level);
// 		}
// 		else
// 		{
// 			for (int j = 0; j < level; j++) std::cout << " "; //indent
// 			std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
// 		}
// 	}

}

