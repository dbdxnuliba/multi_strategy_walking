/*
 * Copyright (C) 2018 IIT-ADVR
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#pragma once
#include <Eigen/Dense>

class DampingCtrClass
{
public:
	DampingCtrClass()
		: x(0.0)
		, dx(0.0)
	{
		x_eigen = dx_eigen = Eigen::Vector3d::Zero();
	};

	~DampingCtrClass() {};

	double update(double damping, double ref, double msr, double settle_time, double dt)
	{
		dx = 1 / damping * (ref - msr) - 1 / settle_time * x;
		x += dx * dt;

		return x;
	}

	Eigen::Vector3d update(const Eigen::Vector3d& damping, const Eigen::Vector3d& ref, const Eigen::Vector3d& msr, const Eigen::Vector3d& settle_time, double dt)
	{
		dx_eigen = (ref - msr).cwiseQuotient(damping) - x_eigen.cwiseQuotient(settle_time);
		x_eigen += dt * dx_eigen;

		return x_eigen;
	}
	
	void reset(const Eigen::Vector3d& ref){x_eigen = ref;};


protected:
	double x;
	double dx;

	Eigen::Vector3d x_eigen;
	Eigen::Vector3d dx_eigen;
};

