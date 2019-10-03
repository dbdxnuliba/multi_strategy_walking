/*****************************************************************************
IKClass_OpenSoT.cpp
*****************************************************************************/

#ifdef USE_OPENSOT
#include "IK/IKClass_OpenSoT.h"


IKClass_OpenSoT::IKClass_OpenSoT(XBot::ModelInterface::Ptr model)
    : _model(model)
    , IsSetHoming(false)
    , IsOnlyLowerBody(true) // need to change
{
    _q.setZero(_model->getJointNum());
    init();
};

IKClass_OpenSoT::IKClass_OpenSoT(std::string path_to_config_file)
    : IsSetHoming(false)
    , IsOnlyLowerBody(true) // need to change
{
    _model = XBot::ModelInterface::getModel(path_to_config_file);
    _q.setZero(_model->getJointNum());
    init();
};

IKClass_OpenSoT::IKClass_OpenSoT(std::string path_to_config_file, const RobotStateClass& irobot)
    : IsSetHoming(false)
    , IsOnlyLowerBody(irobot.RobotPara().IsOnlyLowerBody)
{
    z_c = irobot.RobotPara().Z_C();
    _model = XBot::ModelInterface::getModel(path_to_config_file);
    _q.setZero(_model->getJointNum());
    init();
};

IKClass_OpenSoT::IKClass_OpenSoT(const RobotStateClass& irobot)
    : IsSetHoming(true)
    , IsOnlyLowerBody(irobot.RobotPara().IsOnlyLowerBody)
{
    z_c = irobot.RobotPara().Z_C();
    _model = XBot::ModelInterface::getModel(irobot.RobotPara().opensot_cfg_path);
    _q.setZero(_model->getJointNum());
    std::vector<double> homing_angles(irobot.RobotPara().HOMING_POS());
    std::transform(homing_angles.begin(), homing_angles.end(), homing_angles.begin(), std::bind1st(std::multiplies<double>(), M_PI / 180.0));
    irobot._model->vJointNameToRBDL(homing_angles, _q);
    COUT("_q home", _q.transpose());
    init();
}

void IKClass_OpenSoT::setFloatingBasePose()
{
    Eigen::VectorXd q_zero;
    q_zero.setZero(_model->getJointNum());
    _model->setJointPosition(q_zero);
    _model->update();

    KDL::Frame l_sole_T_Waist, r_sole_T_Waist, foot_center_T_Waist;
    _model->getPose("Waist", "l_sole", l_sole_T_Waist);
    _model->getPose("Waist", "r_sole", r_sole_T_Waist);
    // foot_center_T_Waist.p = l_sole_T_Waist.p;
    // foot_center_T_Waist.p.y(0.5 * (l_sole_T_Waist.p.y() + r_sole_T_Waist.p.y()));

    foot_center_T_Waist.p.z(l_sole_T_Waist.p.z());
    COUT("l_sole_T_Waist:", l_sole_T_Waist);
    COUT("r_sole_T_Waist:", r_sole_T_Waist);
    COUT("foot_center_T_Waist:", foot_center_T_Waist);

    _model->setJointPosition(_q);
    _model->setFloatingBasePose(foot_center_T_Waist);
    _model->update();
    _model->getJointPosition(_q);

    COUT("_q.size():", _q.size());
    COUT("_q:", _q.transpose());

    if (!IsOnlyLowerBody) {
        KDL::Frame l_wrist_T_Waist, r_wrist_T_Waist;
        _model->getPose("LWrMot3", "Waist", l_wrist_T_Waist);
        _model->getPose("RWrMot3", "Waist", r_wrist_T_Waist);
        COUT("l_wrist_T_Waist:", l_wrist_T_Waist);
        COUT("r_wrist_T_Waist:", r_wrist_T_Waist);
    }

    KDL::Frame l_sole_T_world, r_sole_T_world, waist_T_world;
    _model->getPose("l_sole", "world", l_sole_T_world);
    _model->getPose("r_sole", "world", r_sole_T_world);
    _model->getPose("Waist", "world", waist_T_world);
    COUT("l_sole_T_world:", l_sole_T_world);
    COUT("r_sole_T_world:", r_sole_T_world);
    COUT("waist_T_world:", waist_T_world);
};

void IKClass_OpenSoT::init()
{
    err_counter = 0;

    _dq.setZero(_model->getJointNum());

    if (!IsSetHoming) {
        Eigen::VectorXd _q_home;
        _model->getRobotState("home", _q_home); // size 31+6
        COUT("_model->getJointNum()", _model->getJointNum(), "_q_home.size()", _q_home.size(), "_q_home", _q_home.transpose());

        _q = _q_home;
    }

    setFloatingBasePose();

    _q_float = _q;
    getSolution();


    // for (unsigned int i = 0; i < _q_home.size(); ++i)
    // {
    //     std::string name = _model->getJointByDofIndex(i)->getJointName();
    //     double q_min, q_max;
    //     _model->getJointByDofIndex(i)->getJointLimits(q_min, q_max);
    //     std::cout << name << ": " << q_min << " < " << _q_home(i) << " " << " < " << q_max;
    //     if (_q_home(i) > q_max || _q_home(i) < q_min)
    //         std::cout << "    VALUE NOT IN LIMITS!!!!";
    //     std::cout << std::endl;
    // }

    // ======== construct tasks for opensot ====================
    // =========================================================
    /* Create cartesian tasks */

    if (!IsOnlyLowerBody) {
        _l_wrist.reset( new OpenSoT::tasks::velocity::Cartesian("Cartesian::l_wrist", _q, *_model, "LWrMot3", "world") );
        _r_wrist.reset( new OpenSoT::tasks::velocity::Cartesian("Cartesian::r_wrist", _q, *_model, "RWrMot3", "world") );
        _gaze.reset(new OpenSoT::tasks::velocity::Gaze("Cartesian::Gaze", _q, *_model, "Waist"));
    }

    _l_sole.reset( new OpenSoT::tasks::velocity::Cartesian("Cartesian::l_sole", _q, *_model, "l_sole", "world") );
    _l_sole->setLambda(0.3);
    _r_sole.reset( new OpenSoT::tasks::velocity::Cartesian("Cartesian::r_sole", _q, *_model, "r_sole", "world") );
    _r_sole->setLambda(0.3);
    _waist.reset(new OpenSoT::tasks::velocity::Cartesian("Cartesian::waist", _q, *_model, "Waist", "world"));
    _waist->setLambda(0.3);
    _com.reset(new OpenSoT::tasks::velocity::CoM(_q, *_model));
    _com->setLambda(0.3);

    // std::vector<bool> ajm = _gaze->getActiveJointsMask();
    // for (unsigned int i = 0; i < ajm.size(); ++i) {
    //     ajm[i] = false;
    // }
    // ajm[_model->getDofIndex("WaistYaw")] = true;
    // ajm[_model->getDofIndex("WaistSag")] = true;
    // ajm[_model->getDofIndex("WaistLat")] = true;
    // _gaze->setActiveJointsMask(ajm);

    /* Create postural task */
    Eigen::VectorXd q_postural = _q;
    q_postural[6 + 3] = DEGTORAD(20);
    q_postural[6 + 3 + 6] = DEGTORAD(20);
    _postural.reset(new OpenSoT::tasks::velocity::Postural(q_postural));
    Eigen::VectorXd weight;
    weight.setOnes((_model->getJointNum()));
    // weight(0) = 100;
    // _postural->setLambda(0.0);
    _postural->setWeight(weight.asDiagonal());

    /* Create min acc task */
    OpenSoT::tasks::velocity::MinimizeAcceleration::Ptr _min_acc( new OpenSoT::tasks::velocity::MinimizeAcceleration(_q) );
    Eigen::MatrixXd W = _min_acc->getWeight();
    _min_acc->setWeight(2.*W);

    /* Manipulability task */
    if (!IsOnlyLowerBody) {
        OpenSoT::tasks::velocity::Manipulability::Ptr manipulability_right( new OpenSoT::tasks::velocity::Manipulability(_q, *_model, _r_wrist) );
        OpenSoT::tasks::velocity::Manipulability::Ptr manipulability_left( new OpenSoT::tasks::velocity::Manipulability(_q, *_model, _l_wrist) );
    }

    /* Minimum effort task */
    OpenSoT::tasks::velocity::MinimumEffort::Ptr min_effort( new OpenSoT::tasks::velocity::MinimumEffort(_q, *_model) );
    min_effort->setLambda(0.01);

    /* Create joint limits & velocity limits */
    Eigen::VectorXd qmin, qmax, qdotmax;
    _model->getJointLimits(qmin, qmax);
    _model->getVelocityLimits(qdotmax);
    double qdotmax_min = qdotmax.minCoeff();
    Eigen::VectorXd qdotlims(_q.size());
    qdotlims.setConstant(_q.size(), M_PI);

    if (!IsOnlyLowerBody) {
        qdotlims[_model->getDofIndex(_model->chain("torso").getJointId(1))] = 0.01;
    }
    _final_qdot_lim = 2.0;

    _joint_lims.reset( new OpenSoT::constraints::velocity::JointLimits(_q, qmax, qmin) );

    _joint_vel_lims.reset( new OpenSoT::constraints::velocity::VelocityLimits(qdotlims, RobotParaClass::dT()) );

    /* Create autostack and set solver */
    // _auto_stack = (_l_sole + _r_sole) /
    //              (_l_wrist + _r_wrist + _gaze) /
    //              (_postural + _min_acc) << _joint_lims << _joint_vel_lims;
    std::list<unsigned int> id;
    id.push_back(3);
    id.push_back(4);
    id.push_back(5);
    _auto_stack = (_l_sole + _r_sole) /
                  // (_l_wrist + _r_wrist) /
                  (_waist) /
                  // (_com + _waist%id) << _joint_lims << _joint_vel_lims;
                  // (_com + _waist) << _joint_lims << _joint_vel_lims;
                  (_postural) << _joint_lims << _joint_vel_lims;

    _com_constr.reset(new OpenSoT::constraints::TaskToConstraint(_com));

    _auto_stack->update(_q);
    _waist->update(_q);
    //_com_constr->update(_q);

    // _solver.reset( new OpenSoT::solvers::QPOases_sot(_auto_stack->getStack(), _auto_stack->getBounds(), 1e9) );
    _solver.reset( new OpenSoT::solvers::iHQP(_auto_stack->getStack(), _auto_stack->getBounds(), 1e9) );

    // _solver.reset(new OpenSoT::solvers::QPOases_sot(_auto_stack->getStack(), _auto_stack->getBounds(), _com_constr, 1e6));

    // qpOASES::Options opt;
    // _solver->getOptions(0, opt);
    // opt.numRefinementSteps = 0;
    // opt.numRegularisationSteps = 1;
    // for (unsigned int i = 0; i < 3; ++i)
    //     _solver->setOptions(i, opt);

    // KDL::Frame _l_sole_ref;
    // _l_sole->getReference(_l_sole_ref);
    // COUT("_l_sole_ref", _l_sole_ref);

    // KDL::Frame _l_wrist_ref;
    // _l_wrist->getReference(_l_wrist_ref);
    // COUT("_l_wrist_ref", _l_wrist_ref);
};

void IKClass_OpenSoT::setLowerBodyLocalReference(const Eigen::Vector3d& PelvisPos,
        const Eigen::Matrix3d& PelvisO,
        const Eigen::Vector3d& LeftFootPos,
        const Eigen::Matrix3d& LeftFootO,
        const Eigen::Vector3d& RightFootPos,
        const Eigen::Matrix3d& RightFootO)
{
    Eigen::Vector3d anklePos;
    Eigen::Matrix3d ankleRot;
    anklePos = PelvisO.transpose() * (LeftFootPos - PelvisPos);
    ankleRot = PelvisO.transpose() * LeftFootO;
    _l_sole->setReference(transformEigenToKDLFrame(anklePos, ankleRot));

    // COUT("_l_sole ref:",transformEigenToKDLFrame(anklePos, ankleRot));

    anklePos = PelvisO.transpose() * (RightFootPos - PelvisPos);
    ankleRot = PelvisO.transpose() * RightFootO;
    _r_sole->setReference(transformEigenToKDLFrame(anklePos, ankleRot));

    // COUT("_r_sole ref:",transformEigenToKDLFrame(anklePos, ankleRot));
};

void IKClass_OpenSoT::setLowerBodyReference(const Eigen::Vector3d& PelvisPos,
        const Eigen::Matrix3d& PelvisO,
        const Eigen::Vector3d& LeftFootPos,
        const Eigen::Matrix3d& LeftFootO,
        const Eigen::Vector3d& RightFootPos,
        const Eigen::Matrix3d& RightFootO)
{
    Eigen::Vector3d com = PelvisPos;
    com[2] = z_c;
    _com->setReference(com);
    _waist->setReference(transformEigenToKDLFrame(PelvisPos, PelvisO));
    _l_sole->setReference(transformEigenToKDLFrame(LeftFootPos, LeftFootO));
    _r_sole->setReference(transformEigenToKDLFrame(RightFootPos, RightFootO));
};

void IKClass_OpenSoT::setHandLocalReference(const Eigen::Vector3d& LeftHandPos,
        const Eigen::Matrix3d& LeftHandO,
        const Eigen::Vector3d& RightHandPos,
        const Eigen::Matrix3d& RightHandO)
{
    _l_wrist->setReference(transformEigenToKDLFrame(LeftHandPos, LeftHandO));
    _r_wrist->setReference(transformEigenToKDLFrame(RightHandPos, RightHandO));
};

void IKClass_OpenSoT::setHandGlobalReference(const Eigen::Vector3d& LeftHandPos,
        const Eigen::Matrix3d& LeftHandO,
        const Eigen::Vector3d& RightHandPos,
        const Eigen::Matrix3d& RightHandO)
{
    _l_wrist->setReference(transformEigenToKDLFrame(LeftHandPos, LeftHandO));
    _r_wrist->setReference(transformEigenToKDLFrame(RightHandPos, RightHandO));
};

void IKClass_OpenSoT::setPosturalReference(const Eigen::VectorXd& x_desired)
{
    _postural->setReference(x_desired);
};

void IKClass_OpenSoT::SolveIK(const Eigen::VectorXd& x)
{
// #define TEST_QP_TIME

#ifdef TEST_QP_TIME
    static int count = 1;
    static double aveT = 0.0;
    double t1 = get_time();
    tic;
#endif
    assert(x.size() == _q.size());

    // _q_float.tail(_q.size()-6) = x.tail(_q.size()-6); // using feedback

    /* Model update */
    _model->setJointPosition(_q_float);
    _model->update();

    // COUT("");

    // KDL::Frame _l_sole_ref;
    // _l_sole->getReference(_l_sole_ref);
    // COUT("_l_sole_ref", _l_sole_ref);
    // KDL::Frame _r_sole_ref;
    // _r_sole->getReference(_r_sole_ref);
    // COUT("_r_sole_ref", _r_sole_ref);
    // KDL::Frame _waist_ref;
    // _waist->getReference(_waist_ref);
    // COUT("_waist_ref", _waist_ref);

//     KDL::Frame _l_wrist_ref;
//     _l_wrist->getReference(_l_wrist_ref);
//     COUT("_l_wrist_ref", _l_wrist_ref);

    /* Stack update and solve */
    // COUT("_postural", _postural->getActualPositions().transpose());
    _auto_stack->update(_q_float);
    _waist->update(_q);
    //_com_constr->update(_q_float);

    if ( !_solver->solve(_dq) ) {
        _dq.setZero(_dq.size());
        DPRINTF("%d UNABLE TO SOLVE OPENSOT\n", err_counter++);
    }

#ifdef TEST_QP_TIME
    toc;
    double t2 = get_time();
    aveT += (t2 - t1);
    COUT("solveQP using ", t2 - t1, " s. Average time is ", aveT / count);
    count ++;
#endif

    /* Update q */
    _q += _dq;
    _q_float = _q;

    // COUT("_q", _q_float.transpose());
};

const Eigen::VectorXd& IKClass_OpenSoT::getSolution()
{
    if (_model->isFloatingBase()) {
        _q_out = _q.tail(_model->getJointNum() - 6);
    }
    else {
        _q_out = _q;
    }
    return _q_out;
};


#endif
