#ifndef __EIGEN_UTILS_HPP
#define __EIGEN_UTILS_HPP

#include <Eigen/Geometry>
#include <stdio.h>
#include <string>
#include <kdl_parser/kdl_parser.hpp>



inline void MapEigenToArray(const Eigen::MatrixXd& from, double * to)
{
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(to, from.rows(), from.cols()) = from;
};

inline void MapEigenToArray(const Eigen::MatrixXi& from, int * to)
{
  Eigen::Map<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(to, from.rows(), from.cols()) = from;
};

namespace Eigen {typedef Matrix< double , 6 , 1> Vector6d;}

inline Eigen::Vector3d vKDLtoEigen(const KDL::Vector &k)
{
  Eigen::Vector3d e;
  // tf::vectorKDLToEigen(k, e);
  for (int i = 0; i < 3; ++i)
    e[i] = k[i];
  return e;
}

inline KDL::Vector vEigentoKDL(const Eigen::Vector3d &e)
{
  KDL::Vector k;
  // tf::vectorEigenToKDL(e, k);
  for (int i = 0; i < 3; ++i)
    k[i] = e[i];
  return k;
}

inline Eigen::Vector6d vKDLtoEigen(const KDL::Twist &k)
{
  Eigen::Vector6d e;
  // tf::twistKDLToEigen(k, e);
  for (int i = 0; i < 6; ++i)
    e[i] = k[i];
  return e;
}

template<typename T>
inline void transformEigenToKDL(const T &e, KDL::Frame &k)
{
  for (unsigned int i = 0; i < 3; ++i)
    k.p[i] = e(i, 3);
  for (unsigned int i = 0; i < 9; ++i)
    k.M.data[i] = e(i / 3, i % 3);
}

inline KDL::Frame transformEigenToKDLFrame(const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot)
{
  KDL::Frame k;
  Eigen::Affine3d aff;
  aff.translation() = pos;
  aff.linear() = rot;
  transformEigenToKDL(aff, k);
  return k;
}
//------------------------------------------------


inline Eigen::Quaterniond normalAndYaw2Quat(Eigen::Vector3d normal, double yaw)
{
  if (normal.norm() == 0)
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

  normal.normalize();
  double n1 = normal(0);
  double n2 = normal(1);
  double n3 = normal(2);

  Eigen::Vector3d Y = normal.cross(Eigen::Vector3d(0, 0, 1));
  Eigen::Vector3d X = Y.cross(normal);
  Eigen::Matrix3d R;
  R.col(0) = X.normalized();
  R.col(1) = Y.normalized();
  R.col(2) = normal;

  // final rot = R * rotateZ[theta]
  // multiple out, and take rot(0,0) rot(1,0), that's the xy coordinate in xy
  // plane
  // x = c*R00 + R01*s, y = R10*c + R11*s
  double x = cos(yaw);
  double y = sin(yaw);
  double s = (R(0, 0) * y - R(1, 0) * x) / (R(0, 0) * R(1, 1) - R(1, 0) * R(0, 1));
  double c = (x - R(0, 1) * s) / R(0, 0);

  double theta = atan2(s, c);

  return Eigen::Quaterniond(R) * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
}

inline Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d &v)
{
  return (Eigen::Matrix3d() <<
          0, -v(2), v(1),
          v(2), 0, -v(0),
          -v(1), v(0), 0).finished();
}

inline void SkewSymmetric(const Eigen::Vector3d &v, Eigen::Matrix3d &m)
{
  m << 0, -v(2), v(1),
  v(2), 0, -v(0),
  -v(1), v(0), 0;
}

inline Eigen::Vector3d transformVec(const Eigen::Vector3d &pos, const Eigen::Quaterniond &q, const Eigen::Vector3d &off)
{
  return pos + q.toRotationMatrix() * off;
}

inline Eigen::Vector2d rotate2d(const Eigen::Vector2d &in, double yaw)
{
  return Eigen::Vector2d(cos(yaw) * in[0] - sin(yaw) * in[1], sin(yaw) * in[0] + cos(yaw) * in[1]);
}

inline void neg_quat(Eigen::Quaterniond &q)
{
  q.w() *= -1;
  q.x() *= -1;
  q.y() *= -1;
  q.z() *= -1;
}

inline Eigen::Vector3d getAxis(const Eigen::Quaterniond &q, int idx)
{
  Eigen::Matrix3d rot = q.toRotationMatrix();
  return rot.col(idx).normalized();
}

inline Eigen::Vector3d getAngVel(const Eigen::Quaterniond &quat0, Eigen::Quaterniond Qf, double T) {
  if (Qf.dot(quat0) < 0)   neg_quat(Qf);

  Eigen::AngleAxisd wQ = Eigen::AngleAxisd(Qf * (quat0.inverse()));
  return  wQ.axis() * (wQ.angle() / T);
}

// used to generate world frame angular vel / acc.
// q0 = delta * q1
// delta = q0 * q1^-1
inline Eigen::Vector3d quatMinusAbs(const Eigen::Quaterniond &Q0, Eigen::Quaterniond Q1) {
  if (Q0.dot(Q1) < 0)
    neg_quat(Q1);
  Eigen::AngleAxisd rotVec(Q0 * (Q1.inverse().normalized()));
  return rotVec.axis() * rotVec.angle();
}

// used to generate q1 frame angular vel / acc.
// q0 = q1 * delta
// delta = q1^-1 * q0
inline Eigen::Vector3d quatMinusRel(const Eigen::Quaterniond &Q0, Eigen::Quaterniond Q1) {
  if (Q0.dot(Q1) < 0)
    neg_quat(Q1);
  Eigen::AngleAxisd rotVec((Q1.inverse().normalized()) * Q0);
  return rotVec.axis() * rotVec.angle();
}

inline double quatDist(const Eigen::Quaterniond &a, const Eigen::Quaterniond &b)
{
  Eigen::Quaterniond tmp = a;
  if (tmp.dot(b) < 0)
    neg_quat(tmp);

  Eigen::AngleAxisd rotVec(tmp * (b.inverse()));
  return rotVec.angle();
}

inline Eigen::Quaterniond euler2quat(const Eigen::Vector3d &ang)
{
  Eigen::Matrix3d rot;
  rot =
    Eigen::AngleAxisd(ang[0], Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(ang[1], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(ang[2], Eigen::Vector3d::UnitZ());
  return Eigen::Quaterniond(rot).normalized();
}

inline Eigen::Quaterniond zyx2quat(double x, double y, double z)
{
  Eigen::Matrix3d rot;
  rot =
    Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX());
  return Eigen::Quaterniond(rot).normalized();
}

inline Eigen::Quaterniond zyx2quat(const Eigen::Vector3d &ang)
{
  return zyx2quat(ang[0], ang[1], ang[2]);
}

inline Eigen::Vector3d quat2zyx(const Eigen::Quaterniond &q)
{
  double q0 = q.w();
  double q1 = q.x();
  double q2 = q.y();
  double q3 = q.z();

  double phi = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
  double theta = asin(2 * (q0 * q2 - q3 * q1));
  double psi = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
  return Eigen::Vector3d(phi, theta, psi);
}

inline Eigen::Quaterniond onlyYaw(const Eigen::Quaterniond &q)
{
  Eigen::Vector3d zyx = quat2zyx(q);
  zyx[0] = zyx[1] = 0;
  return zyx2quat(zyx);
}

inline double getYaw(const Eigen::Quaterniond &q)
{
  return quat2zyx(q).z();
}

inline void printQuat(const std::string &name, const Eigen::Quaterniond &q, std::ostream &out)
{
  out << name << ", qx " << q.x() << ", qy " << q.y() << ", qz " << q.z() << ", qw " << q.w();
}

inline double avgAngle(double a, double b)
{
  double x = fmod(fabs(a - b), 2 * M_PI);
  if (x >= 0 && x <= M_PI)
    return fmod((a + b) / 2., 2 * M_PI);
  else if (x >= M_PI && x < 1.5 * M_PI)
    return fmod((a + b) / 2., 2 * M_PI) + M_PI;
  else
    return fmod((a + b) / 2., 2 * M_PI) - M_PI;
}

inline void quat2EA(const Eigen::Quaterniond &q, double *EA) {
  double Rtmp[3][3];
  double e1, e2, e3, e4, e11, e22, e33, e44, norm;
  double ie1 = q.x();
  double ie2 = q.y();
  double ie3 = q.z();
  double ie4 = q.w();

  e11 = ie1 * ie1;
  e22 = ie2 * ie2;
  e33 = ie3 * ie3;
  e44 = ie4 * ie4;
  norm = sqrt(e11 + e22 + e33 + e44);
  if (norm == 0.) {
    e4 = 1.;
    norm = 1.;
  } else {
    e4 = ie4;
  }
  norm = 1. / norm;
  e1 = ie1 * norm;
  e2 = ie2 * norm;
  e3 = ie3 * norm;
  e4 = e4 * norm;
  e11 = e1 * e1;
  e22 = e2 * e2;
  e33 = e3 * e3;
  Rtmp[0][0] = 1. - (2.*(e22 + e33));
  Rtmp[0][1] = 2.*(e1 * e2 - e3 * e4);
  Rtmp[0][2] = 2.*(e1 * e3 + e2 * e4);
  Rtmp[1][0] = 2.*(e1 * e2 + e3 * e4);
  Rtmp[1][1] = 1. - (2.*(e11 + e33));
  Rtmp[1][2] = 2.*(e2 * e3 - e1 * e4);
  Rtmp[2][0] = 2.*(e1 * e3 - e2 * e4);
  Rtmp[2][1] = 2.*(e2 * e3 + e1 * e4);
  Rtmp[2][2] = 1. - (2.*(e11 + e22));

  double th1, th2, th3, tmp;

  if (((fabs(Rtmp[0][2]) - 1.) >= -1e-15)  ) {
    th1 = atan2(Rtmp[2][1], Rtmp[1][1]);
    if ((Rtmp[0][2] > 0.)  ) {
      tmp = 1.5707963267949;
    } else {
      tmp = -1.5707963267949;
    }
    th2 = tmp;
    th3 = 0.;
  } else {
    th1 = atan2(-Rtmp[1][2], Rtmp[2][2]);
    th2 = asin(Rtmp[0][2]);
    th3 = atan2(-Rtmp[0][1], Rtmp[0][0]);
  }
  EA[0] = th1;
  EA[1] = th2;
  EA[2] = th3;
}

inline Eigen::Quaterniond EA2quat(const double *EA) {
  double cos1, cos2, cos3, sin1, sin2, sin3;
  double Rtmp[3][3];

  cos1 = cos(EA[0]);
  cos2 = cos(EA[1]);
  cos3 = cos(EA[2]);
  sin1 = sin(EA[0]);
  sin2 = sin(EA[1]);
  sin3 = sin(EA[2]);
  Rtmp[0][0] = (cos2 * cos3);
  Rtmp[0][1] = -(cos2 * sin3);
  Rtmp[0][2] = sin2;
  Rtmp[1][0] = ((cos1 * sin3) + (sin1 * (cos3 * sin2)));
  Rtmp[1][1] = ((cos1 * cos3) - (sin1 * (sin2 * sin3)));
  Rtmp[1][2] = -(cos2 * sin1);
  Rtmp[2][0] = ((sin1 * sin3) - (cos1 * (cos3 * sin2)));
  Rtmp[2][1] = ((cos1 * (sin2 * sin3)) + (cos3 * sin1));
  Rtmp[2][2] = (cos1 * cos2);

  double tmp, tmp1, tmp2, tmp3, tmp4;

  tmp = (Rtmp[0][0] + (Rtmp[1][1] + Rtmp[2][2]));
  if (((tmp >= Rtmp[0][0]) && ((tmp >= Rtmp[1][1]) && (tmp >= Rtmp[2][2]
                                                      )))  ) {
    tmp1 = (Rtmp[2][1] - Rtmp[1][2]);
    tmp2 = (Rtmp[0][2] - Rtmp[2][0]);
    tmp3 = (Rtmp[1][0] - Rtmp[0][1]);
    tmp4 = (1. + tmp);
  } else {
    if (((Rtmp[0][0] >= Rtmp[1][1]) && (Rtmp[0][0] >= Rtmp[2][2]))
       ) {
      tmp1 = (1. - (tmp - (2.*Rtmp[0][0])));
      tmp2 = (Rtmp[0][1] + Rtmp[1][0]);
      tmp3 = (Rtmp[0][2] + Rtmp[2][0]);
      tmp4 = (Rtmp[2][1] - Rtmp[1][2]);
    } else {
      if ((Rtmp[1][1] >= Rtmp[2][2])  ) {
        tmp1 = (Rtmp[0][1] + Rtmp[1][0]);
        tmp2 = (1. - (tmp - (2.*Rtmp[1][1])));
        tmp3 = (Rtmp[1][2] + Rtmp[2][1]);
        tmp4 = (Rtmp[0][2] - Rtmp[2][0]);
      } else {
        tmp1 = (Rtmp[0][2] + Rtmp[2][0]);
        tmp2 = (Rtmp[1][2] + Rtmp[2][1]);
        tmp3 = (1. - (tmp - (2.*Rtmp[2][2])));
        tmp4 = (Rtmp[1][0] - Rtmp[0][1]);
      }
    }
  }
  tmp = (1. / sqrt(((tmp1 * tmp1) + ((tmp2 * tmp2) + ((tmp3 * tmp3) + (tmp4 * tmp4))))));
  return Eigen::Quaterniond((tmp * tmp4), (tmp * tmp1), (tmp * tmp2), (tmp * tmp3));
}

inline Eigen::Quaterniond EA2quat(double r, double p, double y) {
  double EA[3] = {r, p, y};
  return EA2quat(EA);
}

inline void flattenQuat(Eigen::Quaterniond &q) {
  double EA[3];
  quat2EA(q, EA);
  EA[0] = EA[1] = 0.0;
  q = EA2quat(EA);
}

inline Eigen::Quaterniond flatQuat(const Eigen::Quaterniond &q) {
  double EA[3];
  quat2EA(q, EA);
  EA[0] = EA[1] = 0.0;
  return EA2quat(EA);
}

//frac = 0: base
//frac = 1: target
inline Eigen::Quaterniond mySlerp(Eigen::Quaterniond &base, Eigen::Quaterniond target, double frac) {
  if (base.dot(target) < 0)     neg_quat(target); //go the short direction
  return base.slerp(frac, target);
}

inline void modifyQ(Eigen::Quaterniond &q, double r, double p, double y) {
  double EA[3] = {r, p, y};
  Eigen::Quaterniond q2 = EA2quat(EA);
  q = q * q2;
}

inline void preModifyQ(Eigen::Quaterniond &q, double r, double p, double y) {
  double EA[3] = {r, p, y};
  Eigen::Quaterniond q2 = EA2quat(EA);
  q = q2 * q;
}

inline Eigen::Matrix3d getBasis(Eigen::Vector3d v0) {
  v0.normalize();
  Eigen::Vector3d zhat(0, 0, 1);
  Eigen::Vector3d v1 = zhat.cross(v0);
  Eigen::Vector3d v2 = v0.cross(v1);

  Eigen::Matrix3d m;
  m.block<1, 3>(0, 0) = v0; //top row is original vector
  m.block<1, 3>(1, 0) = v1;
  m.block<1, 3>(2, 0) = v2;

  return m;
}

inline void yawQuat(Eigen::Quaterniond &q, double yaw) {
  Eigen::Quaterniond dR = EA2quat(0, 0, yaw);
  q = dR * q;
}

inline std::ostream& operator<<(std::ostream& os, const Eigen::Quaterniond &q)
{
  Eigen::Vector3d tmp = quat2zyx(q);
  os << "EA:(" << tmp[0] << "," << tmp[1] << "," << tmp[2] << ")";
  return os;
}


#endif
