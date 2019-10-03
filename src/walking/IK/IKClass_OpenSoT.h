/*****************************************************************************
IKClass_OpenSoT.h

*****************************************************************************/
#pragma once

// #include <OpenSoT/tasks/velocity/all.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/utils/AutoStack.h>

#include <OpenSoT/tasks/velocity/Gaze.h>
#include <OpenSoT/tasks/velocity/MinimizeAcceleration.h>
#include <OpenSoT/tasks/velocity/Manipulability.h>
#include <OpenSoT/tasks/velocity/MinimumEffort.h>
#include <OpenSoT/constraints/TaskToConstraint.h>
// #include <qpOASES/Options.hpp>
#include <OpenSoT/solvers/iHQP.h>

#include "utils/utils.h"
#include "WBS/RobotStateClass.h"

#define tic      double tic_t = clock();
#define toc      std::cout << (clock() - tic_t)/CLOCKS_PER_SEC \
                           << " seconds" << std::endl;

class IKClass_OpenSoT
{
public:

    IKClass_OpenSoT(XBot::ModelInterface::Ptr model);
    IKClass_OpenSoT(std::string path_to_config_file);
    IKClass_OpenSoT(std::string path_to_config_file, const RobotStateClass& irobot);
    IKClass_OpenSoT(const RobotStateClass& irobot);

    ~IKClass_OpenSoT() {};

    void setFloatingBasePose();

    void init();

    void setLowerBodyLocalReference(const Eigen::Vector3d& PelvisPos,
                                    const Eigen::Matrix3d& PelvisO,
                                    const Eigen::Vector3d& LeftFootPos,
                                    const Eigen::Matrix3d& LeftFootO,
                                    const Eigen::Vector3d& RightFootPos,
                                    const Eigen::Matrix3d& RightFootO);

    void setLowerBodyReference(const Eigen::Vector3d& PelvisPos,
                               const Eigen::Matrix3d& PelvisO,
                               const Eigen::Vector3d& LeftFootPos,
                               const Eigen::Matrix3d& LeftFootO,
                               const Eigen::Vector3d& RightFootPos,
                               const Eigen::Matrix3d& RightFootO);

    void setHandLocalReference(const Eigen::Vector3d& LeftHandPos,
                               const Eigen::Matrix3d& LeftHandO,
                               const Eigen::Vector3d& RightHandPos,
                               const Eigen::Matrix3d& RightHandO);

    void setHandGlobalReference(const Eigen::Vector3d& LeftHandPos,
                                const Eigen::Matrix3d& LeftHandO,
                                const Eigen::Vector3d& RightHandPos,
                                const Eigen::Matrix3d& RightHandO);

    void setPosturalReference(const Eigen::VectorXd& x_desired);
    void setCurrentPosturalReference() {setPosturalReference(_q);};

    void SolveIK(const Eigen::VectorXd& x);

    const Eigen::VectorXd& getSolution();
    inline const Eigen::VectorXd& getSolutionFloating() {return _q;};

private:
    double _final_qdot_lim;

    Eigen::VectorXd _dq, _q, _q_float;
    Eigen::VectorXd _q_out; //!< joint position ref send to robot, without floating base

    OpenSoT::tasks::velocity::Cartesian::Ptr _l_wrist, _r_wrist;
    OpenSoT::tasks::velocity::Cartesian::Ptr _l_sole, _r_sole;
    OpenSoT::tasks::velocity::Cartesian::Ptr _waist;
    OpenSoT::tasks::velocity::Postural::Ptr _postural;
    OpenSoT::tasks::velocity::CoM::Ptr _com;
    OpenSoT::tasks::velocity::Gaze::Ptr _gaze;
    OpenSoT::tasks::velocity::MinimizeAcceleration::Ptr minAcc;

    OpenSoT::constraints::velocity::JointLimits::Ptr _joint_lims;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr _joint_vel_lims;
    OpenSoT::constraints::TaskToConstraint::Ptr _com_constr;

    OpenSoT::AutoStack::Ptr _auto_stack;
    OpenSoT::solvers::iHQP::Ptr _solver;
    // OpenSoT::solvers::QPOases_sot::Ptr _solver;

    XBot::ModelInterface::Ptr _model;

    int err_counter;
    bool IsSetHoming;
    bool IsOnlyLowerBody;
    double z_c;
};

