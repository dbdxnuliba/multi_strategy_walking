/** Detailed description of file.
 */
#include "KalmanFilterClass.h"

KalmanFilterClass::KalmanFilterClass()
:IsInit(false)
{
}

KalmanFilterClass::~KalmanFilterClass()
{
}

void KalmanFilterClass::Init(const Eigen::MatrixXd &_A, const Eigen::MatrixXd &_H, const Eigen::MatrixXd &_P, const Eigen::MatrixXd &_R, const Eigen::MatrixXd &_Q)
{
	Eigen::MatrixXd Btemp = Eigen::MatrixXd::Zero(_A.rows(), _A.rows());
	Init(_A, Btemp, _H, _P, _R, _Q);
}

void KalmanFilterClass::Init(const Eigen::MatrixXd &_A, const Eigen::MatrixXd &_B, const Eigen::MatrixXd &_H, const Eigen::MatrixXd &_P, const Eigen::MatrixXd &_R, const Eigen::MatrixXd &_Q)
{
	if(!IsInit){
		if(!CheckMatrixSize(_A, _B, _H, _P, _R, _Q)){
			std::cout<<"Initialize failed!!! Matrix has invalid size!"<<std::endl;
		}
		else{
			A = _A;
			B = _B;
			H = _H;
			P = _P;
			R = _R;
			Q = _Q;
			X = Eigen::VectorXd::Zero(A.rows());
			K = Eigen::MatrixXd::Zero(H.cols(), H.rows());
			I = Eigen::MatrixXd::Identity(A.rows(),A.rows());
			IsInit = true;
		}	
	}
	else{
		std::cout<<"Error!!! KalmanFilter is already initialized!"<<std::endl;
	}
}

bool KalmanFilterClass::CheckMatrixSize(const Eigen::MatrixXd &_A, const Eigen::MatrixXd &_B, const Eigen::MatrixXd &_H, const Eigen::MatrixXd &_P, const Eigen::MatrixXd &_R, const Eigen::MatrixXd &_Q)
{
	bool flagA = false;
	bool flagB = false;
	bool flagH = false;
	bool flagP = false;
	bool flagR = false;
	bool flagQ = false;
	if(A.rows()==A.cols()){
		flagA = true;
	}
	else{
		std::cout<<"Error!!! Matrix A has wrong size!"<<std::endl;
	}

	if(B.rows()==A.rows()){
		flagB = true;
	}
	else{
		std::cout<<"Error!!! Vector B has wrong size!"<<std::endl;
	}

	if(H.cols()==A.rows()){
		flagH = true;
	}
	else{
		std::cout<<"Error!!! Matrix H has wrong size!"<<std::endl;
	}

	if(P.rows()==P.cols() && P.rows()==A.rows()){
		flagP = true;
	}
	else{
		std::cout<<"Error!!! Matrix P has wrong size!"<<std::endl;
	}

	if(R.rows()==R.cols() && R.rows()==H.rows()){
		flagR = true;
	}
	else{
		std::cout<<"Error!!! Matrix R has wrong size!"<<std::endl;
	}

	if(Q.rows()==Q.cols() && Q.rows()==A.rows()){
		flagQ = true;
	}
	else{
		std::cout<<"Error!!! Matrix Q has wrong size!"<<std::endl;
	}
	return (flagA && flagB && flagH && flagP && flagR && flagQ);
}

Eigen::VectorXd KalmanFilterClass::Update(const Eigen::VectorXd &Z)
{
	Eigen::VectorXd Utemp;
	if(IsInit){
		Utemp = Eigen::VectorXd::Zero(B.cols());
	}
	return Update(Z, Utemp);
}

Eigen::VectorXd KalmanFilterClass::Update(const Eigen::VectorXd &Z, const Eigen::VectorXd &U)
{
	if(IsInit){
		X = A*X + B*U;				//X(k|k-1)	= A*X(k-1|k-1)+B*u(k)
		P = A*P*A.transpose()+Q;	//P(k|k-1)	= A*P(k-1|k-1)A' + Q
		K = P*H.transpose()*(H*P*H.transpose()+R).inverse();		//K(k) = P(k|k-1)*H'*(H*P(k|k-1)*H'+R)^-1
		X = X + K*(Z-H*X);			//X(k|k)	= X(k|k-1)+K*(z(k)-H*X(k|k-1))
		P = (I-K*H)*P;				//P(k|k)	= (I-K*H)*P(k|k-1)
	}
	else{
		X = Eigen::VectorXd::Zero(3);
		std::cout<<"Error!!! Initialize KalmanFilter before Update!"<<std::endl;
	}
	return X;
}

