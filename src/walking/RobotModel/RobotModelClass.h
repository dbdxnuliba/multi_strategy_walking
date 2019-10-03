/*****************************************************************************
RobotModelClass.h
*****************************************************************************/
#pragma once

#include "rbdl/rbdl.h"
namespace RBDL = RigidBodyDynamics;

#include "utils/Eigen_utils.hpp"
#include "utils/QuaternionClass.h"

#include "RobotPara/RobotParaClass.h"

class RBDLModelClass
{
public:
	RBDLModelClass() {};
	virtual ~RBDLModelClass() {};
	
        //////// ******************************* robotpara
	void InitRBDL(const RobotParaClass& robotpara);                            //////
	
	void UpdateRBDL(const std::vector<double>& qall_msr);
	void UpdateRBDL(const Eigen::VectorXd& qall_msr);

	int getJointNameFromRBDLJntID(const int& rbdlID)
	{
		for (auto it : _map_JointName_to_rbdl) {
			if (it.second == rbdlID) return it.first;
		};
	};

	inline const int& getRBDLJointID(const int& joint_name) {return _map_JointName_to_rbdl[joint_name];};
	inline const int& getRBDLBodyID(const int& link_name) {return _map_LinkName_to_rbdl[link_name];};

	RBDL::Math::SpatialTransform getLocalBodyFrame(const int& link_name);
	Eigen::Vector3d PositionFromGlobal2Hip(const Eigen::Vector3d& from);
	Eigen::Vector3d VelocityFromGlobal2Hip(const Eigen::Vector3d& vel_from);
	Eigen::Matrix3d getGloballBodyFrame(const int& link_name);
	Eigen::Vector3d getGloballBodyPosition(const int& link_name);
	Eigen::Vector3d getGloballBodyVelocity(const int& link_name);

	const RBDL::Math::Matrix3d& getOrientationFromWorldToBody(const int& link_name);
	const RBDL::Math::MatrixNd& InertiaMatrix();
	const RBDL::Math::VectorNd& NonlinearEffects();
	// Eigen::VectorXd sortVecFromRBDLtoJointName(const Eigen::VectorXd& from);

	template <typename T>
	void getRequiredTorques(T& torque_all) // if it is the template func used by other cpp file, the implementation should be in the header file, otherwise will cause " undefined reference to" error because other cpp file cannot see the implementation
	{
		InverseDynamics();
		vRBDLToJointName(tau_all_floating, torque_all, 0, torque_all.size() - 1);
	};

	void CalcCenterOfMass(const bool& IsUpdateKinematics = true);
	void InverseDynamics();
	Eigen::VectorXd InverseDynamicsContacts(RBDL::ConstraintSet& CS);
	void ForwardDynamicsContacts(RBDL::ConstraintSet& CS);
	// Eigen::MatrixXd CMM() {return centroidal_momentum_matrix();};
	Eigen::MatrixXd CalcCMM();

	/** \brief Returns the spatial jacobian of given frame with respect to required reference frame
	 *
	 * \param tar_body_id id of the target body to which the target frame are expressed
	 * \param tar_body_Rotation_tar_frame the orientation of target frame with respect to target body frame
	 * \param tar_body_Translation_tar_frame the position of target frame expressed in body frame
	 * \param ref_body_id id of the body to which the spatial velocity are expressed (default: 0, which means the reference frame is base frame).
	 * \param ref_body_Rotation_ref_frame the orientation of reference frame with respect to reference body frame (defual: I, no rotation)
	 * \param ref_body_Translation_ref_frame the position of reference frame expressed in reference body frame (defual: 0, no translation)
	 * \param IsUpdateKinematics whether UpdateKinematics() should be called or not
	 * (default: false).
	 *
	 * \returns a jacobian matrix of the target frame wrt reference frame (rotational part above translational part)
	 */
	Eigen::MatrixXd CalcFrameJacobian ( unsigned int tar_body_id,
	                                    RBDL::Math::Matrix3d tar_body_Rotation_tar_frame,
	                                    RBDL::Math::Vector3d tar_body_Translation_tar_frame,
	                                    unsigned int ref_body_id = 0,
	                                    RBDL::Math::Matrix3d ref_body_Rotation_ref_frame = RBDL::Math::Matrix3d::Identity(),
	                                    RBDL::Math::Vector3d ref_body_Translation_ref_frame = RBDL::Math::Vector3d::Zero(),
	                                    bool IsUpdateKinematics = false );


	/** \brief Returns the spatial transform of given frame with respect to required reference frame
	 *
	 * \param model the rigid body model
	 * \param Q the curent genereralized positions
	 * \param tar_body_id id of the target body to which the target frame are expressed
	 * \param tar_body_Rotation_tar_frame the orientation of target frame with respect to target body frame
	 * \param tar_body_Translation_tar_frame the position of target frame expressed in body frame
	 * \param ref_body_id id of the body to which the spatial velocity are expressed (default: 0, which means the reference frame is base frame).
	 * \param ref_body_Rotation_ref_frame the orientation of reference frame with respect to reference body frame (defual: I, no rotation)
	 * \param ref_body_Translation_ref_frame the position of reference frame expressed in reference body frame (defual: 0, no translation)
	 *
	 * \returns a spatial transform of the target frame wrt reference frame(spactial transform: E->rotation matrix, r->translation vector)
	 */
	RBDL::Math::SpatialTransform CalcFrameTransform (
	    unsigned int tar_body_id,
	    RBDL::Math::Matrix3d tar_body_Rotation_tar_frame,
	    RBDL::Math::Vector3d tar_body_Translation_tar_frame,
	    unsigned int ref_body_id = 0,
	    RBDL::Math::Matrix3d ref_body_Rotation_ref_frame = RBDL::Math::Matrix3d::Identity(),
	    RBDL::Math::Vector3d ref_body_Translation_ref_frame = RBDL::Math::Vector3d::Zero());

	/**
	 * @brief      transfer a vector with total dof length from predefined joint name order to rbdl order (6 floating dof + joint num)
	 *
	 * @param[in]  from     The from
	 * @param      to_rbdl  To rbdl
	 *
	 * @tparam     T1       std::vector or Eigen::VectorXd
	 * @tparam     T2       std::vector or Eigen::VectorXd
	 */
	template <typename T1, typename T2>
	void vJointNameToRBDL(const T1& from, T2& to_rbdl)
	{
		for (auto it : _map_JointName_to_rbdl) {
			if (RBDL_API_VERSION == rbdl250) {
				to_rbdl[it.second + 4] = from[it.first]; // for rbdl 2.5.0
			}
			else if (RBDL_API_VERSION == rbdl240) {
				to_rbdl[it.second] = from[it.first]; // for rbdl 2.4.0
			}
		}
	};

	/**
	 * @brief      sort vector "from_rbdl" to "to" from rbdl order to link_name order
	 *
	 * @param[in]  from_rbdl  The from rbdl
	 * @param      to         resulted vector in link_name order
	 * @param[in]  start      The start link_name id
	 * @param[in]  end        The end link_name id
	 *
	 * @tparam     T1         std::vector or Eigen::VectorXd
	 * @tparam     T2         std::vector or Eigen::VectorXd
	 */
	template <typename T1, typename T2>
	void vRBDLToJointName(const T1& from_rbdl, T2& to, const int& start = 0, const int& end = RobotParaClass::JOINT_NUM() - 1)
	{
		for (auto it : _map_JointName_to_rbdl) {
			if ((it.first >= start) && (it.first <= end)) {
				assert(it.second < from_rbdl.size());
				assert(it.first < to.size());
				// to[it.first] = from_rbdl[it.second]; // for rbdl 2.4.0, has 6 dof at the start
				to[it.first] = from_rbdl[it.second]; // for rbdl 2.5.0
			}
		}
	};

	RBDL::ConstraintSet constraint_set_lft, constraint_set_rft, constraint_set_bothft;

	RBDL::Math::MatrixNd Jcom;
	RBDL::Math::MatrixNd dJcom;
	RBDL::Math::MatrixNd _CMM;
	RBDL::Math::MatrixNd _dCMM;
	RBDL::Math::VectorNd q_all_floating;
	RBDL::Math::VectorNd dq_all_floating;
	RBDL::Math::VectorNd ddq_all_floating;
	RBDL::Math::VectorNd tau_all_floating;
	RBDL::Math::Vector3d com_floating, com_ft;
	RBDL::Math::Vector3d com_hip, dcom_hip;
	RBDL::Math::Vector3d com_velocity, angular_momentum;
	RBDL::Math::VectorNd q_all_fixed;
	RBDL::Math::VectorNd dq_all_fixed;
	double Ep, Ek; // potential and kinetic energy of the robot

	double mass;

	RBDL::Model* getRBDLModel() {return &_rbdl_model;};

	int rbdl240, rbdl250;

	Eigen::Matrix3d Rot_World_to_Pelvis;
	RBDL::Math::SpatialTransform local_origin_PelvisProj;
	RBDL::Math::SpatialTransform lwrist, rwrist;
	RBDL::Math::SpatialTransform lsole, rsole;
	RBDL::Math::SpatialTransform lankle, rankle;
	Eigen::Vector3d base_pos;
	RBDL::Math::SpatialTransform waist_pos;

	void UpdateKinematicsOnce();

protected:
	RBDL::Model _rbdl_model;

	Eigen::MatrixXd getJacobian(const int& body_id, const bool& IsFullJacobian = false, const bool& IsUpdateKinematics = false);

	Eigen::MatrixXd getJdot(const int& body_id, const bool& IsFullJacobian = false);

	Eigen::VectorXd getJdotQdot(const int& body_id, const bool& IsFullJacobian = false);

private:
	double dt;
	double nDoF_floating;
	double nLink_floating;

	Eigen::Vector3d _ankle_offset;

	RBDL::Math::MatrixNd _H; //! inertial matrix
	RBDL::Math::VectorNd _C; //! gravity + centrifugal and coriolis forces

	RBDL::Math::VectorNd q_all_floating_old, dq_all_floating_old;

	std::map<int, int> _map_LinkName_to_rbdl, _map_JointName_to_rbdl;

	RBDL::Math::MatrixNd _CMM_old;
	Eigen::MatrixXd compute_P();
	Eigen::MatrixXd compute_J();
	Eigen::MatrixXd centroidal_momentum_matrix();
};

//===============================================================

#ifdef USE_KDL
class RobotModelClass: public KDLModelClass, public RBDLModelClass
#else
class RobotModelClass: public RBDLModelClass
#endif
{
public:
	RobotModelClass() {};
	virtual ~RobotModelClass() {};

	int Init(const RobotParaClass& robotpara)
	{
		InitRBDL(robotpara);

#ifdef USE_KDL
		InitKDL(robotpara);
#endif
		// COUT("KDLModelClass::dt:",KDLModelClass::dt);
		// COUT("RBDLModelClass::dt:",RBDLModelClass::dt);
	};

	int Update(const std::vector<double>& qall_msr)
	{
		UpdateRBDL(qall_msr);
#ifdef USE_KDL
		UpdateKDL(qall_msr);
#endif
	};

	int Update(const Eigen::VectorXd& qall_msr)
	{
		UpdateRBDL(qall_msr);
#ifdef USE_KDL
		UpdateKDL(qall_msr);
#endif
	};

	Eigen::MatrixXd getJacobian(const int& body_id, const std::string& lib_name = "kdl", const bool& IsFullJacobian = false)
	{
		if (lib_name == "rbdl") {
			return RBDLModelClass::getJacobian(body_id, IsFullJacobian);
		}
#ifdef USE_KDL
		else if (lib_name == "kdl") {
			assert(!IsFullJacobian);
			return KDLModelClass::getJacobian(body_id);
		}
#endif
		else {
			assert(!"not defined lib_name to getJacobian, should be kdl or rbdl");
		}
	};

	Eigen::MatrixXd getJdot(const int& body_id, const std::string& lib_name = "kdl", const bool& IsFullJacobian = false)
	{
		if (lib_name == "rbdl") {
			return RBDLModelClass::getJdot(body_id, IsFullJacobian);
		}
#ifdef USE_KDL
		else if (lib_name == "kdl") {
			assert(!IsFullJacobian);
			return KDLModelClass::getJdot(body_id);
		}
#endif
		else {
			assert(!"not defined lib_name to getJdot, should be kdl or rbdl");
		}
	};

	Eigen::MatrixXd getJdotQdot(const int& body_id, const std::string& lib_name = "kdl", const bool& IsFullJacobian = false)
	{
		if (lib_name == "rbdl") {
			return RBDLModelClass::getJdotQdot(body_id, IsFullJacobian);
		}
#ifdef USE_KDL
		else if (lib_name == "kdl") {
			assert(!IsFullJacobian);
			return KDLModelClass::getJdotQdot(body_id);
		}
#endif
		else {
			assert(!"not defined lib_name to getJdotQdot, should be kdl or rbdl");
		}
	};

private:


};


