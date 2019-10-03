#ifndef _QUATERNION_CLASS_H_
#define _QUATERNION_CLASS_H_

#include <Eigen/Geometry>
#include <kdl/frames.hpp>
/**
  This class implements QuaternionClass error as in the paper:
    "Operational Space Control: A Theoretical and Empirical Comparison"
  Authors: Jun Nakanishi, Rick Cory, Michael Mistry, Jan Peters and Stefan Schaal
  The International Journal of Robotics Research, Vol. 27, No. 6, June 2008, pp. 737–757

  REMEMBER: if e is the QuaternionClass error, the orientation error is defined as:
                o_error = -Ke
            with K positive definite!
  **/
class QuaternionClass
{
public:
    double x;
    double y;
    double z;
    double w;

    QuaternionClass():
        x(0.0),
        y(0.0),
        z(0.0),
        w(1.0)
    {

    }

    QuaternionClass(double _x, double _y, double _z, double _w):
        x(_x),
        y(_y),
        z(_z),
        w(_w)
    {

    }

    /**
     * @brief dot product between two quaternions
     * @param a first QuaternionClass
     * @param b second QuaternionClass
     * @return a scalar
     */
    static double dot(const QuaternionClass& a, const QuaternionClass& b)
    {
        return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
    }

    /**
     * @brief operator * product between a QuaternionClass and a scalar
     * @param a scalar
     * @return a QuaternionClass
     */
    QuaternionClass operator*(const double a)
    {
        QuaternionClass q(x, y, z, w);

        q.x *= a;
        q.y *= a;
        q.z *= a;
        q.w *= a;

        return q;
    }

    /**
     * @brief skew operator
     * @return the skew matrix of the QuaternionClass
     */
    KDL::Rotation skew()
    {
        KDL::Rotation s(0.0,  -z,   y,
                          z, 0.0,  -x,
                         -y,   x, 0.0);
        return s;
    }

    /**
     * @brief error compute the error between two QuaternionClass to be usable in orientation control
     * @param q actual QuaternionClass
     * @param qd desired QuaternionClass
     * @return an error vector [3x1]
     *
     * REMEMBER: if e is the QuaternionClass error, the orientation error is defined as:
                o_error = -Ke
            with K positive definite!
     *
     */
    static KDL::Vector error(QuaternionClass& q, QuaternionClass& qd)
    {
        KDL::Vector e(0.0, 0.0, 0.0);

        KDL::Vector eps(q.x, q.y, q.z);
        KDL::Vector epsd(qd.x, qd.y, qd.z);

        e = qd.w*eps - q.w*epsd + qd.skew()*eps;

        return e;
    }
};

#endif