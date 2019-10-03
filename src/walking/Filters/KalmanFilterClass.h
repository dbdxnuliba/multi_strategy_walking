/**
* Detailed description of file.
 */

#ifndef KF_CLASS_H
#define KF_CLASS_H

#include <iostream>
#include <Eigen/Dense>

class KalmanFilterClass
{
public:
	KalmanFilterClass();
	virtual ~KalmanFilterClass();
	void Init(const Eigen::MatrixXd &_A, const Eigen::MatrixXd &_H, const Eigen::MatrixXd &_P, const Eigen::MatrixXd &_R, const Eigen::MatrixXd &_Q);
	void Init(const Eigen::MatrixXd &_A, const Eigen::MatrixXd &_B, const Eigen::MatrixXd &_H, const Eigen::MatrixXd &_P, const Eigen::MatrixXd &_R, const Eigen::MatrixXd &_Q);
	Eigen::VectorXd Update(const Eigen::VectorXd &Z);
	Eigen::VectorXd Update(const Eigen::VectorXd &Z, const Eigen::VectorXd &U);

private:
	bool IsInit;
	Eigen::MatrixXd A;     		// Statetransition matrix 
	Eigen::MatrixXd B;			// Input matrix
	Eigen::MatrixXd H;			// Measurement matrix
	Eigen::MatrixXd P; 			// Covariance matrix Kalman filter
	Eigen::MatrixXd R;			// Measurement noise covariance matrix
	Eigen::MatrixXd Q;			// State noise covariance matrix
	Eigen::MatrixXd K;			// Gain matrix
	Eigen::VectorXd X;			// State vector
	Eigen::MatrixXd I;

	bool CheckMatrixSize(const Eigen::MatrixXd &_A, const Eigen::MatrixXd &_B, const Eigen::MatrixXd &_H, const Eigen::MatrixXd &_P, const Eigen::MatrixXd &_R, const Eigen::MatrixXd &_Q);

};

#endif
