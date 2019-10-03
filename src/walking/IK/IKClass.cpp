#include "IK/IKClass.h"
#include "utils/MatrixRotation.h"
#include "utils/utils.h"

#include <iostream>

IKClass::IKClass()
    : upperleg(0, 0, -0.22580000)
    , lowerleg(0, 0, -0.2010000)
    , ankle(0, 0, -0.0793)
    , offset_hip(0, 0.0726, 0)
    , fullleg(upperleg.norm() + lowerleg.norm() + ankle.norm())
    , waist(0.0203, 0, 0.1191)
    , chest(-0.015, 0, 0.2029)
    , upperarm(0, 0, -0.180)
    , forearm(0, 0, -0.20451)
    , offset_shoulder(0, 0.1535, 0)
    , qL(6, 0)
    , qR(6, 0)
    , kneeMinDeg(1.0)
    , kneeMaxDeg(120.0)
    , d0(0, 0.06, 0)
    , d1(0, 0.121032, 0)
    , d2(0, 0, -0.217872)
    , d3(0, 0, -0.356)
    , d4(0, 0, -0.4)
{
}


IKClass::~IKClass()
{
}


double IKClass::RADTODEG(double x)
{
    return x * 180.0 / M_PI;
}

double IKClass::DEGTORAD(double x)
{
    return x * M_PI / 180.0;
}

// one leg inverse kinematics, Geometric solution //
void IKClass::LegInverseKinematics(Eigen::Vector3d Waist_P, Eigen::Matrix3d Waist_R, Eigen::Vector3d Foot_P, Eigen::Matrix3d Foot_R, Eigen::Vector3d hipoffset, std::vector<double> &q)
{
    const double A = upperleg.norm();
    const double B = lowerleg.norm();
    Eigen::Matrix3d R06T, Rfoot_hip, R03;
    Eigen::Vector3d r;
    double C, alpha, rxz, x, y, costheta, gama; //C the distance between hip and ankle joint,reset C if C is out of range
    double kneeMin, kneeMax, kneeExtentionMax, kneeExtentionMin;

    R06T = Foot_R.transpose();
    r = R06T * (Waist_P + (Waist_R * hipoffset) - Foot_P); //the vector points from ankle to hip with respect to local foot orientation frame

    kneeMin = DEGTORAD(kneeMinDeg);
    kneeMax = DEGTORAD(kneeMaxDeg);
    kneeExtentionMax = sqrt(A * A + B * B - 2 * A * B * cos(M_PI - kneeMin));
    kneeExtentionMin = sqrt(A * A + B * B - 2 * A * B * cos(M_PI - kneeMax));

    C = r.norm();
    if (C > kneeExtentionMax)
    {
        C = kneeExtentionMax;
        q[3] = kneeMin;
    }
    else if (C < kneeExtentionMin)
    {
        C = kneeExtentionMin;
        q[3] = kneeMax;
    }
    else  // C remains
    {
        q[3] = M_PI - acos((A * A + B * B - C * C) / (2.0 * A * B));
    }
    r = C * r / r.norm(); // scale r vector
    alpha = asin(A * sin(M_PI - q[3]) / C);
    rxz = sqrt(r(0) * r(0) + r(2) * r(2));
    costheta = C / sqrt(r(0) * r(0) + r(2) * r(2)) * cos(M_PI / 2 - alpha);
    x = rxz * sqrt(1 - costheta * costheta);
    q[4] = atan2(r(1), x);

    y = C * cos(alpha) - x;
    gama = asin(y * sin(alpha) / rxz);
    q[5] = -(atan2(r(0), r(2)) + gama + alpha);

    Rfoot_hip = Waist_R.transpose() * Foot_R;
    R03 = Rfoot_hip * Ry(-q[5]) * Rx(-q[4]) * Ry(-q[3]);

    q[0] = atan2(R03(0, 2), R03(2, 2));
    q[1] = atan2( -R03(1, 2), (R03(0, 2) * sin(q[0]) + R03(2, 2) * cos(q[0])) );
    q[2] = atan2(R03(1, 0), R03(1, 1));
}


void IKClass::IKLowerBody(Eigen::Vector3d PelvisPos, Eigen::Matrix3d PelvisO, Eigen::Vector3d LeftFootPos, Eigen::Matrix3d LeftFootO, Eigen::Vector3d RightFootPos, Eigen::Matrix3d RightFootO, std::vector<double> &_qL, std::vector<double> &_qR)
{
    LegInverseKinematics(PelvisPos, PelvisO, LeftFootPos, LeftFootO, offset_hip, _qL);
    LegInverseKinematics(PelvisPos, PelvisO, RightFootPos, RightFootO, -offset_hip, _qR);
}

//void IKClass::get_jointAngles(std::vector<double> _qL, std::vector<double> _qR)
//{
//    _qL=qL;
//    _qR=qR;
//}


// =============== for walkman =======================================
double IKClass::twoVectorsAngle( Eigen::Vector3d first, Eigen::Vector3d second)
{
    first.normalize();
    second.normalize();

    double s = first.cross(second).norm();
    double c = first.dot(second);
    return std::atan2(s, c);

}

double IKClass::twoVectorsAngleSigned( Eigen::Vector3d first, Eigen::Vector3d second, Eigen::Vector3d planeNormalVector)
{
    first.normalize();
    second.normalize();
    planeNormalVector.normalize();

    errno = 0;

    //From some reasons the dot product of normalized vectors was bigger than 1 and that results in nan.
    double dotProduct =  first.dot(second);
    if (dotProduct >  1) dotProduct =  1;
    if (dotProduct < -1) dotProduct = -1;
    double angle = std::acos( dotProduct );
    if (errno != 0) {
        switch (errno) {
        case EDOM: std::cout << "Error EDOM " << std::endl; break;
        case ERANGE: std::cout << "Error ERANGE " << std::endl; break;
        case EILSEQ: std::cout << "Error EILSEQ " << std::endl; break;
        }
    }

    Eigen::Vector3d crossP = first.cross(second);

    if (planeNormalVector.dot(crossP) < 0)
        angle = -angle;

    return angle;
}

double IKClass::triangleAngleFromThreeSides(double closeOne, double closeTwo, double opposite)
{
    double acosValue = (closeOne * closeOne + closeTwo * closeTwo - opposite * opposite) / (2 * closeOne * closeTwo);
    if (acosValue >  1) acosValue =  1;
    if (acosValue < -1) acosValue = -1;
    return acos( acosValue );
}

bool IKClass::BigmanLegFK( Eigen::Vector6d q, std::string leg, Eigen::Vector3d &anklePos)
{
    Eigen::Matrix3d R01, R12, R23, R34, R03, R04;
    Eigen::Vector3d hipPos, kneePos;

    R01 = Rx(q[0]);   // hip roll
    R12 = Rz(q[1]);   // hip yaw
    R23 = Ry(q[2]);   // hip pitch
    R34 = Ry(q[3]);   // knee
    //R45 = Ry(q[4]); // ankle pitch
    //R56 = Rx(q[5]); // ankle roll

    if (leg == "L") {
        hipPos = d0 + R01 * (d1 + d2);
    } else if (leg == "R") {
        hipPos = -d0 + R01 * (-d1 + d2);
    }

    R03 = R01 * R12 * R23;
    R04 = R01 * R12 * R23 * R34;

    kneePos = R03 * d3 + hipPos;
    anklePos = kneePos + R04 * d4; // R45 R56 are not needed to calcuate the ankle position

    anklePos(0) = anklePos(0) + 0.0170;   // 0.0170 - x shift from pelvis

    //Check if there is no nan's inside the vector
    if (anklePos != anklePos) {
        return false;
    }
    return true;
}

void IKClass::BigmanLegIK(Eigen::Vector3d Waist_P, Eigen::Matrix3d Waist_R, Eigen::Vector3d Foot_P, Eigen::Matrix3d Foot_R, std::string leg, Eigen::Vector6d &q, std::string robot_name)
{
    Eigen::Vector3d anklePos = Waist_R.transpose() * (Foot_P - Waist_P);
    Eigen::Matrix3d ankleRot = Waist_R.transpose() * Foot_R;
    BigmanLegIK(anklePos, ankleRot, leg, q, robot_name);
}

bool IKClass::BigmanLegIK( Eigen::Vector3d anklePos, Eigen::Matrix3d ankleRot, std::string leg, Eigen::Vector6d &q, std::string robot_name)
{
    if (q.rows() != 6)
        q.resize(6);//q[6]: hip roll, hip yaw, hip pitch, knee pitch, ankle pitch, ankle roll;

    //Robot parameters
    if (robot_name == "bigman") {
        anklePos(0) = anklePos(0) - 0.0170;   // 0.0170 - x shift from pelvis // bigman
    }
    else if (robot_name == "walkman") {
        anklePos(0) = anklePos(0) - 0.017;   // if I substract 0.051 in MoveToInit - x shift from pelvis // walkman
        // anklePos(0) = anklePos(0) - 0.068;   // 0.068 - x shift from pelvis // walkman
    }
    else{
        std::cout<<"wrong robot name for bigmanIK\n";
    }

    //pelvis ZY plane parameters
    Eigen::Vector3d XYpelvis_normalVector(1, 0, 0);

    // ==================== HIP ROLL ====================
    //Ankle roll joint x axis before rotation
    Eigen::Vector3d ankleX_point  = anklePos;
    Eigen::Vector3d ankleX_vector = ankleRot.block(0, 0, 3, 1); //(1:3,1);

    //Plane and line intersection point
    double ankle_d = (-ankleX_point).dot(XYpelvis_normalVector) / ankleX_vector.dot(XYpelvis_normalVector);
    Eigen::Vector3d XYpelvis_ankleXIntersection = ankle_d * ankleX_vector + ankleX_point;

    double hipRollToYaw, hipRollToAnkleDist, hr_alpha, hr_beta;

    //Calculate hip roll angle
    if (leg == "L") {
        hipRollToYaw = d1.norm();
        hipRollToAnkleDist = (-d0 + XYpelvis_ankleXIntersection).norm();
        hr_alpha = acos(hipRollToYaw / hipRollToAnkleDist);
        hr_beta  = twoVectorsAngle(-d0, XYpelvis_ankleXIntersection - d0);
        q(0) = (hr_alpha + hr_beta - M_PI);
    } else if (leg == "R") {
        hipRollToYaw = d1.norm();
        hipRollToAnkleDist = (d0 + XYpelvis_ankleXIntersection).norm();
        hr_alpha = acos(hipRollToYaw / hipRollToAnkleDist);
        hr_beta  = twoVectorsAngle(d0, XYpelvis_ankleXIntersection + d0);
        q(0) = -(hr_alpha + hr_beta - M_PI);
    }

    // ==================== HIP YAW ====================
    //Normal vector of plane containing ankle X-axis and hip-yaw axis
    Eigen::Matrix3d hipRollRot = Rx(q(0));
    Eigen::Matrix3d hipRotation = Rx(q(0));
    Eigen::Vector3d hipYaw_pos;
    if (leg == "L") {
        hipYaw_pos = d0 + hipRotation * d1;
    } else if (leg == "R") {
        hipYaw_pos = -d0 - hipRotation * d1;
    }

    Eigen::Vector3d ankleToHipYaw = hipYaw_pos - ankleX_point;
    Eigen::Vector3d ankleXHipYawPlane_normal = ankleToHipYaw.cross(ankleX_vector);
    //Normal vector conaining XZ-axes of hip yaw plane before yaw rotation
    Eigen::Vector3d hipYawXZ_normal = hipRollRot.block(0, 1, 3, 1); //(1:3,2);
    //Vector3d hipYaw_Z = hipRollRot.block(0,2,3,1); //(1:3,3);
    q(1) = twoVectorsAngleSigned(hipYawXZ_normal, ankleXHipYawPlane_normal, hipRollRot.block(0, 2, 3, 1)); //(1:3,3));
    hipRotation = hipRotation * Rz(q(1));

    // ==================== KNEE PITCH ====================
    Eigen::Vector3d hipPitch_pos = hipYaw_pos + hipRotation * d2;
    Eigen::Vector3d hipPitchToAnkle_vector = ankleX_point - hipPitch_pos;
    if (hipPitchToAnkle_vector.norm() > (d3.norm() + d4.norm() - 0.000)) {
        //std::cout << leg << ": Knee is getting overstretched. Desired: " << hipPitchToAnkle_vector.norm() << ", Limit" << d3.norm() + d4.norm() - 0.000 << std::endl;
        //return false;
        q(3) = M_PI - triangleAngleFromThreeSides(std::abs(d3(2)), std::abs(d4(2)), (d3.norm() + d4.norm() - 0.000));
    }
    else
        q(3) = M_PI - triangleAngleFromThreeSides(std::abs(d3(2)), std::abs(d4(2)), hipPitchToAnkle_vector.norm());

    // ==================== HIP PITCH ====================
    Eigen::Vector3d hipPitchToAnkleXPelvisXYintersection = XYpelvis_ankleXIntersection - hipPitch_pos;
    Eigen::Vector3d ankleToHipPitch = hipPitch_pos - ankleX_point;
    double hipYaw_ankleX_anklePos_angle = twoVectorsAngleSigned(hipPitchToAnkleXPelvisXYintersection, -ankleToHipPitch, ankleXHipYawPlane_normal);
    q(2) = -triangleAngleFromThreeSides(std::abs(d3(2)), hipPitchToAnkle_vector.norm(), std::abs(d4(2)));
    q(2) = q(2) + hipYaw_ankleX_anklePos_angle;

    // ==================== ANKLE PITCH ====================
    hipRotation = hipRotation * Ry(q(2));
    Eigen::Matrix3d kneePitchRot = Ry(q(3));
    Eigen::Matrix3d pelvisToKneePitchRot = hipRotation * kneePitchRot;
    q(4) =  twoVectorsAngleSigned(pelvisToKneePitchRot.block(0, 0, 3, 1), ankleX_vector, pelvisToKneePitchRot.block(0, 1, 3, 1));

    // ==================== ANKLE ROLL ====================
    Eigen::Matrix3d pelvisToAnklePitchRot = hipRotation * kneePitchRot * Ry(q(4));
    q(5) =  twoVectorsAngleSigned(pelvisToAnklePitchRot.block(0, 1, 3, 1), ankleRot.block(0, 1, 3, 1), pelvisToAnklePitchRot.block(0, 0, 3, 1));

    //Check if there is no nan's inside the vector
    if (q != q) {
        return false;
    }
    return true;
}

