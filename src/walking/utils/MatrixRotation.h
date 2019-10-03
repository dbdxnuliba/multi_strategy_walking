#ifndef _MATRIX_ROTATION_H// header guards
#define _MATRIX_ROTATION_H

#include <cmath>
#include <Eigen/Dense>


inline Eigen::Matrix3d Rx(double angle)
{
	double ct = std::cos(angle);
	double st = std::sin(angle);
	
	Eigen::Matrix3d result;
	
	result << 1, 0,	0,
			  0,ct,	-st,
			  0,st,	ct;
	return result;
}

inline Eigen::Matrix3d Ry(double angle)
{
	double ct = std::cos(angle);
	double st = std::sin(angle);
	
	Eigen::Matrix3d result;
	
	result << ct,  0,  st,
			   0,  1,	0,
			 -st,  0,  ct;
			 
	return result;
}

inline Eigen::Matrix3d Rz(double angle)
{
	double ct = std::cos(angle);
	double st = std::sin(angle);
	
	Eigen::Matrix3d result;
	
	result << ct,  -st,	 0,
			  st,	ct,	 0, 
			   0, 	0,	 1;
			   
	return result;
}

inline Eigen::Matrix3d Rodrigues(Eigen::Vector3d w,double theta)
{
 //w should be column vector of size 3, unit vector
 //you can either specify w and dt or directly use theta=w*dt to get
 //rotational angle
	Eigen::Vector3d wn;
	Eigen::Matrix3d w_wedge, R, Reye;
	Reye = Eigen::Matrix3d::Identity(3,3);
	if (w.norm()<1e-6)
	{
		wn<<0, 0, 0;
	}
	else    
	{
		wn = w/w.norm();// normarized vector
	}
	w_wedge <<0, -wn(2), wn(1),
				wn(2), 0, -wn(0),
				-wn(1), wn(0), 0;
	
	R = Reye + w_wedge*std::sin(theta) + w_wedge*w_wedge*(1-std::cos(theta));
	return R;
// the rotational operation around the vector w
}

inline Eigen::Matrix3d Rodrigues_w(Eigen::Vector3d w,double dt)
{
 //w should be column vector of size 3, unit vector
 //you can either specify w and dt or directly use theta=w*dt to get
 //rotational angle
    double theta = w.norm()*dt;
    Eigen::Vector3d wn;
    Eigen::Matrix3d w_wedge, R, Reye;
    Reye = Eigen::Matrix3d::Identity(3,3);
    if (w.norm()<1e-6)
    {
        wn<<0, 0, 0;
    }
    else
    {
        wn = w/w.norm();// normarized vector
    }
    w_wedge <<0, -wn(2), wn(1),
                wn(2), 0, -wn(0),
                -wn(1), wn(0), 0;

    R = Reye + std::sin(theta)*w_wedge + (1-std::cos(theta))*w_wedge*w_wedge;
    return R;
// the rotational operation around the vector w
}


#endif
