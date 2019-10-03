/*
    Created date: 1 Sept 2014
    Copyright (C) Department of Advanced Robotics, Italian Institute of Technology*/

#include <cmath>
#include <vector>
#include <string>
#include <Eigen/Dense>

namespace Eigen {typedef Matrix< double , 6 , 1> Vector6d;}

class IKClass
{
public:

    IKClass();
    ~IKClass();

    double LowerBodyIK(double xcop, double _zc, Eigen::Vector3d X, Eigen::VectorXd xzmp_ref, double delta_plus, double delta_minus);
    void LegInverseKinematics(Eigen::Vector3d Waist_P, Eigen::Matrix3d Waist_R, Eigen::Vector3d Foot_P, Eigen::Matrix3d Foot_R, Eigen::Vector3d hipoffset, std::vector<double> &q);

    void IKLowerBody(Eigen::Vector3d PelvisPos, Eigen::Matrix3d PelvisO, Eigen::Vector3d LeftFootPos, Eigen::Matrix3d LeftFootO, Eigen::Vector3d RightFootPos, Eigen::Matrix3d RightFootO, std::vector<double> &_qL, std::vector<double> &_qR);

    bool BigmanLegFK( Eigen::Vector6d q, std::string leg, Eigen::Vector3d &anklePos);
    bool BigmanLegIK( Eigen::Vector3d anklePos, Eigen::Matrix3d ankleRot, std::string leg, Eigen::Vector6d &q, std::string robot_name = "walkman");

    void BigmanLegIK(Eigen::Vector3d Waist_P, Eigen::Matrix3d Waist_R, Eigen::Vector3d Foot_P, Eigen::Matrix3d Foot_R, std::string leg, Eigen::Vector6d &q, std::string robot_name = "walkman");

//    void get_jointAngles(std::vector<double> _qL, std::vector<double> _qR);

private:
    Eigen::Vector3d upperleg;
    Eigen::Vector3d lowerleg;
    Eigen::Vector3d ankle;
    Eigen::Vector3d offset_hip;// from pelvis center to hip joint
    const double fullleg;// full leg length

    Eigen::Vector3d waist;
    Eigen::Vector3d chest; //(chest, center of 2 shoulders) with respect to frame 22
    Eigen::Vector3d upperarm;
    Eigen::Vector3d forearm;
    Eigen::Vector3d offset_shoulder;// from shoulder axis to chest center

    double RADTODEG(double x);
    double DEGTORAD(double x);

    std::vector<double> qL;
    std::vector<double> qR;

    double kneeMinDeg;
    double kneeMaxDeg;


    //  for bigman
    Eigen::Vector3d d0, d1, d2, d3, d4;
    double twoVectorsAngle( Eigen::Vector3d first, Eigen::Vector3d second);
    double twoVectorsAngleSigned( Eigen::Vector3d first, Eigen::Vector3d second, Eigen::Vector3d planeNormalVector);
    double triangleAngleFromThreeSides(double closeOne, double closeTwo, double opposite);
};

