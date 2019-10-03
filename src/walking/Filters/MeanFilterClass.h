/*****************************************************************************
MeanFilterClass.h
*****************************************************************************/
#ifndef MEAN_FILTER_CLASS_H// header guards
#define MEAN_FILTER_CLASS_H

#include <stdio.h>
#include <cstdlib>
#include <vector>
#include <Eigen/Dense>

class MeanFilterClass
{
public:
	MeanFilterClass();
	// ~MeanFilterClass();

	std::vector<double> applyFilter(int filter_size, const std::vector<double> &raw_data);
	std::vector<double> returnBuffer(int index); // index from front to back, 0 is the newest element
	int returnBufferSize();
	std::vector<double> applyMeanFilter(int filter_size, const std::vector<double> &raw_data);

    Eigen::Vector3d applyFilter(int filter_size, const Eigen::Vector3d &raw_data);
    Eigen::Vector3d applyMeanFilter(int filter_size, const Eigen::Vector3d &raw_data);

private:
	std::vector<std::vector<double> > F_buffer; 
	bool initF;
	// std::vector<std::vector<double> >::iterator it_buffer;	
};
#endif

