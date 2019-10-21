/*****************************************************************************
QPBaseClass.cpp
*****************************************************************************/
#include "QP/QPBaseClass.h"
#include "utils/EiQuadProg/EiQuadProg.hpp"
#include <iostream>

// #include <qpOASES.hpp>

inline bool is_bad_num(const double& num)
{
	return std::isinf(num) || std::isnan(num);
}

inline void MapEigenToArray(const Eigen::MatrixXd& from, double * to)
{
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(to, from.rows(), from.cols()) = from;
}

class QPsolver
{
public:
	QPsolver() {};
	virtual ~QPsolver() {};
	virtual void resize(const int& nVar, const int& nEq, const int& nIneq) = 0;
	virtual void solve(Eigen::MatrixXd & G, const Eigen::VectorXd & g0, const Eigen::MatrixXd & CE, const Eigen::VectorXd & ce0, const Eigen::MatrixXd & CI, const Eigen::VectorXd & ci0, Eigen::VectorXd& X) = 0;

protected:
	int _nVar;
	int _nEq;
	int _nIneq;

};
/*
//-------------------------------------------------------------------
class QPsolver_qpOASES : public QPsolver
{
public:
	QPsolver_qpOASES()
	{
		std::cout << "Using qpOASES as QP solver...\n";
	};

	~QPsolver_qpOASES()
	{
		// delete[] _H;    // cause __GI___libc_free (mem=0xc13e63f2aaaaaaab) at malloc.c:2929
		// delete[] _A;
		// delete[] _g;
		// delete[] _lb;
		// delete[] _ub;
		// delete[] _lbA ;
		// delete[] _ubA ;
		// delete[] _xOpt;
	};

	void resize(const int& nVar, const int& nEq, const int& nIneq)
	{
		_nVar = nVar;
		_nEq = nEq;
		_nIneq = nIneq;

		int nV = _nVar, nC = _nEq + _nIneq;

		_qpsolver.reset(new qpOASES::SQProblem(nV, nC));
		_qpsolver->setPrintLevel(qpOASES::PL_NONE);

		_H		= new qpOASES::real_t[nV * nV];
		_A		= new qpOASES::real_t[nC * nV];
		_g		= new qpOASES::real_t[nV];
		_lb		= new qpOASES::real_t[nV];
		_ub		= new qpOASES::real_t[nV];
		_lbA	= new qpOASES::real_t[nC];
		_ubA	= new qpOASES::real_t[nC];
		_xOpt	= new qpOASES::real_t[nV];

		MapEigenToArray(Eigen::MatrixXd::Identity(nV, nV), _H);
		MapEigenToArray(Eigen::MatrixXd::Zero(nC, nV), _A);
		MapEigenToArray(Eigen::MatrixXd::Zero(nV, 1), _g);
		MapEigenToArray(Eigen::MatrixXd::Constant(nV, 1, -qpOASES::INFTY), _lb);
		MapEigenToArray(Eigen::MatrixXd::Constant(nV, 1, qpOASES::INFTY), _ub);
		MapEigenToArray(Eigen::MatrixXd::Constant(nC, 1, -qpOASES::INFTY), _lbA);
		_ubA_eigen = Eigen::MatrixXd::Constant(nC, 1, qpOASES::INFTY);
		MapEigenToArray(_ubA_eigen, _ubA);

		qpOASES::real_t cputime = 0.01;
		int nWSR = 50;
		if (qpOASES::SUCCESSFUL_RETURN != _qpsolver->init(_H, _g, _A, _lb, _ub, _lbA, _ubA, nWSR, &cputime)) {
			std::cout << "QPsolver_qpOASES: qpOASES initialisation failed!" << std::endl;
		};

		_A_eigen.resize(nC, nV);
		_lbA_eigen.resize(nC);

	};

	void solve(Eigen::MatrixXd & G,
	           const Eigen::VectorXd & g0,
	           const Eigen::MatrixXd & CE,
	           const Eigen::VectorXd & ce0,
	           const Eigen::MatrixXd & CI,
	           const Eigen::VectorXd & ci0,
	           Eigen::VectorXd& X)
	{
		// --------- qpOASES has the following format --------
		// lbA(w 0 ) ≤ Ax ≤ ubA(w 0 )
		//
		//  however the interface uses the following form,
		//  min 0.5 * x G x + g0 x
		// s.t.
		//     CE^T x + ce0 = 0
		//     CI^T x + ci0 >= 0

		// int nV = _nVar, nC = _nEq + _nIneq;
		// COUT("CE", CE.rows(), CE.cols());
		// COUT("CI", CI.rows(), CI.cols());
		// COUT("_A_eigen", _A_eigen.rows(), _A_eigen.cols());
		// _A_eigen.block(0, 0, _nEq, nV) = CE.transpose();
		// _A_eigen.block(_nEq, 0, _nIneq, nV) = CI.transpose();

		_A_eigen << CE.transpose(), CI.transpose();
		_lbA_eigen << -ce0, -ci0;
		_ubA_eigen.segment(0, _nEq) = -ce0;

		MapEigenToArray(_A_eigen, _A);
		MapEigenToArray(_lbA_eigen, _lbA);
		MapEigenToArray(_ubA_eigen, _ubA);

		MapEigenToArray(G, _H);
		MapEigenToArray(g0, _g);

		// COUT(_ubA_eigen.size(), _lbA_eigen.size());
		// COUT(_ubA_eigen.transpose(),"\n",_lbA_eigen.transpose());

		_qpsolver->setPrintLevel(qpOASES::PL_NONE);
		qpOASES::real_t cputime = 0.01;
		int nWSR = 50;
		_qpsolver->hotstart(_H, _g, _A, _lb, _ub, _lbA, _ubA, nWSR, &cputime);
		_qpsolver->getPrimalSolution(_xOpt);

		X = Eigen::Map<Eigen::VectorXd>(_xOpt, _nVar);
	};

private:
	boost::shared_ptr<qpOASES::SQProblem> _qpsolver;

	Eigen::MatrixXd _A_eigen;
	Eigen::VectorXd _lbA_eigen, _ubA_eigen;

	qpOASES::real_t *_H, *_A, *_g, *_lb, *_ub, *_lbA, *_ubA;
	qpOASES::real_t *_xOpt;

};*/

//-------------------------------------------------------------------
class QPsolver_EiQuadProg : public QPsolver
{
public:
	QPsolver_EiQuadProg()
	{
		std::cout << "Using EiQuadProg as QP solver...\n";
	};

	void resize(const int& nVar, const int& nEq, const int& nIneq)
	{
		_qpsolver.resize(nVar, nEq, nIneq);
	};

	void solve(Eigen::MatrixXd & G,
	           const Eigen::VectorXd & g0,
	           const Eigen::MatrixXd & CE,
	           const Eigen::VectorXd & ce0,
	           const Eigen::MatrixXd & CI,
	           const Eigen::VectorXd & ci0,
	           Eigen::VectorXd& X)
	{
		_qpsolver.solve_quadprog(G, g0, CE, ce0, CI, ci0, X);
	};

private:
	Eigen::QP _qpsolver;
};

//-------------------------------------------------------------------
QPBaseClass::QPBaseClass(const std::string& qpSolverName)
	: _solver_name(qpSolverName)
{
	if (_solver_name == "EiQuadProg") {
		_qpsolver.reset(new QPsolver_EiQuadProg());
	}
// 	else if (_solver_name == "qpOASES") {
// 		_qpsolver.reset(new QPsolver_qpOASES());
// 	}
	else {
		//COUT("please define qpsolver: qpOASES or EiQuadProg");
		assert(!"please define qpsolver: qpOASES or EiQuadProg");
		exit(0);
	}

	_nVars = 0;
	_nEqCon = 0;
	_nIneqCon = 0;

	_costRowIdx = 0;
	_ineqRowIdx = 0;
	_A.resize(MAX_ROWS, MAX_VARS);
	_b.resize(MAX_ROWS);
        std::cout << "Using EiQuadProg as QP solverxxxxxxx...\n";

}

void QPBaseClass::initQP()
{
	_costRowIdx = 0;
	_ineqRowIdx = 0;

	_A.setZero();
	_b.setZero();
	_X.setZero();
	_CE.setZero();
	_ce0.setZero();
	_CI.setZero();
	_ci0.setZero();
}

void QPBaseClass::resizeQP(int nVars, int nEqCon, int nIneqCon)
{
	if (_nVars == nVars && _nEqCon == nEqCon && _nIneqCon == nIneqCon)
		return;

	_nVars = nVars;
	_nEqCon = nEqCon;
	_nIneqCon = nIneqCon;

	assert(_nVars <= MAX_VARS);
	assert(_nIneqCon <= MAX_N_INEQ);

	_qpsolver->resize(_nVars, nEqCon, nIneqCon);
	_G.resize(_nVars, _nVars);
	_g0.resize(_nVars);
	_CE.resize(_nVars, nEqCon);
	_ce0.resize(nEqCon);
	_CI.resize(_nVars, nIneqCon);
	_ci0.resize(nIneqCon);
	_X.resize(_nVars);

	debug = false;
}

void QPBaseClass::solveQP()
{
	assert(_nIneqCon >= _ineqRowIdx);

	// _G = _A.block(0, 0, _costRowIdx, _nVars).transpose() * _A.block(0, 0, _costRowIdx, _nVars);
	// _g0 = -_A.block(0, 0, _costRowIdx, _nVars).transpose() * _b.segment(0, _costRowIdx);

	_qpsolver->solve(_G, _g0, _CE, _ce0, _CI, _ci0, _X);

	for (int i = 0; i < _nVars; i++)
		assert(!std::isnan(_X[i]));

}

int QPBaseClass::finalizeCostRows(const std::string &name, int r)
{
	if (r <= 0)
		return _costRowIdx;

	for (int i = _costRowIdx; i < _costRowIdx + r; i++) {
		if (is_bad_num(_b[i])) {
			std::cout << "bad cost term: " << name << ", " << i - _costRowIdx << "th member: " << _b[i] << std::endl;
			assert(false);
			return -1;
		}
	}
	_costRowIdx += r;
	assert(_costRowIdx <= _A.rows());
	return _costRowIdx;
}

int QPBaseClass::finalizeIneqRows(const std::string &name, int r)
{
	if (r <= 0)
		return _ineqRowIdx;

	for (int i = _ineqRowIdx; i < _ineqRowIdx + r; i++) {
		if (is_bad_num(_ci0[i])) {
			std::cout << "bad ineq term: " << name << ", " << i - _ineqRowIdx << "th member: " << _ci0[i] << std::endl;
			assert(false);
			return -1;
		}
	}
	_ineqRowIdx += r;
	assert(_ineqRowIdx <= _nIneqCon);
	return _ineqRowIdx;
}

int QPBaseClass::addCostRows(const int& r)
{
	if (r <= 0)
		return _costRowIdx;

	_costRowIdx += r;
	assert(_costRowIdx <= _A.rows());
	return _costRowIdx;
}

int QPBaseClass::addIneqRows(const int& r)
{
	if (r <= 0)
		return _ineqRowIdx;

	_ineqRowIdx += r;
	assert(_ineqRowIdx <= _nIneqCon);
	return _ineqRowIdx;
}
