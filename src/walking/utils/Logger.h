#pragma once

#include <stdio.h>
#include <string.h>
#include <Eigen/Geometry>
#include <typeinfo>

#include "utils/mrdplot/mrdplot.h"
#include "utils/Eigen_utils.hpp"
#include "utils/Func_utils.h"

#define LOGGER_MAX_CHANNELS 300
#define LOGGER_MAX_CHARS	50
#define LOGGER_MAX_QUAT		50

#ifdef REAL_ROBOT
#define LOGGER_MAX_N_POINTS 300000 // for experiment, max 5 min
#else
#define LOGGER_MAX_N_POINTS 12000 // in simulation, max 1 min
#endif

enum LoggerDataType {
	LOOGER_DATA_TYPE_UNDEF = 0,
	LOGGER_DATA_TYPE_DOUBLE,
	LOGGER_DATA_TYPE_FLOAT,
	LOGGER_DATA_TYPE_INT,
	LOGGER_DATA_TYPE_LONG,
	LOGGER_DATA_TYPE_BOOL,
	LOGGER_DATA_TYPE_ULONG,
	LOGGER_DATA_TYPE_UINT
};

class DataPoint
{
public:
	char units[LOGGER_MAX_CHARS];
	char names[LOGGER_MAX_CHARS];
	char data_type;
	int channel;			// corresponds to mrdplot channel
	const void *ptr;  // ptr to data

	DataPoint()
	{
		data_type = LOOGER_DATA_TYPE_UNDEF;
		ptr = NULL;
		channel = -1;
	}
};

class Logger {
protected:
	int _ctr;
	bool _inited;
	int _nPoints;
	int _nChannels;
	int _myIdx;
	float *_data;
	float _frequency;
	DataPoint _datapoints[LOGGER_MAX_CHANNELS];
	Eigen::Quaterniond const *_qPtr[LOGGER_MAX_QUAT];
	double _EAbuff[3 * LOGGER_MAX_QUAT];
	int _nQuat;
	bool _recorded;

	void init_(double timestep)
	{
		_ctr = 0;
		_inited = true;
		_myIdx = 0;
		_nPoints = 0;
		_nChannels = 0;
		_data = new float[LOGGER_MAX_N_POINTS * LOGGER_MAX_CHANNELS];

		//_frequency = 333;
		_frequency = 1. / timestep;
		add_datapoint("ctr", "-", &_ctr);
	}

public:
	Logger() {
		_data = NULL;
		_nPoints = 0;
		_myIdx = 0;
		_frequency = 0;
		_nChannels = 0;
		_inited = false;

		_nQuat = 0;
		_recorded = false;
	};

	virtual ~Logger()
	{
		if (_data)
			delete []_data;
		_data = NULL;
	};

	void add_quat(const std::string &names, const Eigen::Quaterniond *q) {
		if (_nQuat < LOGGER_MAX_QUAT) {
			_qPtr[_nQuat] = q;
			char name[100];
			sprintf(name, "%s[RR]", names.c_str());
			add_datapoint(name, "r", &(_EAbuff[_nQuat * 3]));
			sprintf(name, "%s[PP]", names.c_str());
			add_datapoint(name, "r", &(_EAbuff[_nQuat * 3 + 1]));
			sprintf(name, "%s[YY]", names.c_str());
			add_datapoint(name, "r", &(_EAbuff[_nQuat * 3 + 2]));

			sprintf(name, "%s[X]", names.c_str());
			add_datapoint(name, "-", q->coeffs().data());
			sprintf(name, "%s[Y]", names.c_str());
			add_datapoint(name, "-", q->coeffs().data() + 1);
			sprintf(name, "%s[Z]", names.c_str());
			add_datapoint(name, "-", q->coeffs().data() + 2);
			sprintf(name, "%s[W]", names.c_str());
			add_datapoint(name, "-", q->coeffs().data() + 3);
			_nQuat++;
		}
	}

	template<typename T> bool add_datapoint(const std::string &names, const char *units, const T *ptr)
	{
		if (_nChannels >= LOGGER_MAX_CHANNELS)
			return false;

		DataPoint *dptr = &_datapoints[_nChannels];
		strncpy(dptr->names, names.c_str(), LOGGER_MAX_CHARS - 1);
		dptr->names[LOGGER_MAX_CHARS - 1] = '\0';
		strncpy(dptr->units, units, LOGGER_MAX_CHARS - 1);
		dptr->units[LOGGER_MAX_CHARS - 1] = '\0';

		_datapoints[_nChannels].ptr = (const void *)ptr;
		if (typeid(T) == typeid(bool))
			_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_BOOL;
		else if (typeid(T) == typeid(double))
			_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_DOUBLE;
		else if (typeid(T) == typeid(float))
			_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_FLOAT;
		else if (typeid(T) == typeid(int))
			_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_INT;
		else if (typeid(T) == typeid(long))
			_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_LONG;
		else if (typeid(T) == typeid(unsigned int))
			_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_UINT;
		else if (typeid(T) == typeid(unsigned long))
			_datapoints[_nChannels].data_type = LOGGER_DATA_TYPE_ULONG;
		else {
			fprintf(stdout, "Logger: adding unknown data type, name: %s\n", names.c_str());
			return false;
		}

		_nChannels++;

		return true;
	}

	template<typename T> bool add_datapoints(const std::string &names, const char *units, const T& ptr)
	{
		std::string name;
		for (int i = 0; i < ptr.size(); i++) {
			name = names + "_[" + std::to_string(i) + "]";
			add_datapoint(name, units, &ptr[i]);
		}
	}


	// template<typename T> bool add_datapoints(const std::string &names, const char *units, const std::vector<T>& ptr)
	// {
	// 	std::string name;
	// 	for(int i = 0; i < ptr.size(); i++) {
	// 		name = names + "["+std::to_string(i)+"]";
	// 	    add_datapoint(name,units,&ptr[i]);
	// 	}
	// }

	// template<typename T> bool add_datapoints(const std::string &names, const char *units, const Eigen::MatrixBase<T>& ptr)
	// {
	// 	std::string name;
	// 	for (int i = 0; i < ptr.size(); i++) {
	// 		name = names + "[" + std::to_string(i) + "]";
	// 		add_datapoint(name, units, &ptr[i]);
	// 	}
	// }

	virtual void init(double timestep) = 0;
	virtual void saveData() = 0;

	inline bool hasInited() const { return _inited; }
};

class BatchLogger : public Logger {
public:
	BatchLogger() {;}
	~BatchLogger() {;}

	void init(double timestep) {init_(timestep);};

	void saveData()
	{
		static bool err = false;

		if (_recorded)	return;		//stop wasting your time

		for (int i = 0; i < _nQuat; i++) {
			quat2EA(*(_qPtr[i]), &(_EAbuff[i * 3]));
		}

		const void *ptr;

		if (!_inited) {
			fprintf(stdout, "logger not inited.\n");
			return;
		}

		for (int i = 0; i < _nChannels; i++) {
			ptr = _datapoints[i].ptr;
			if (_datapoints[i].data_type == LOGGER_DATA_TYPE_DOUBLE)
				_data[_myIdx + i] = (float)(*((const double *)ptr));
			else if (_datapoints[i].data_type == LOGGER_DATA_TYPE_FLOAT)
				_data[_myIdx + i] = (float)(*((const float *)ptr));
			else if (_datapoints[i].data_type == LOGGER_DATA_TYPE_INT)
				_data[_myIdx + i] = (float)(*((const int *)ptr));
			else if (_datapoints[i].data_type == LOGGER_DATA_TYPE_LONG)
				_data[_myIdx + i] = (float)(*((const long *)ptr));
			else if (_datapoints[i].data_type == LOGGER_DATA_TYPE_BOOL) {
				if (*((bool *)ptr))
					_data[_myIdx + i]	= 1;
				else
					_data[_myIdx + i]	= 0;
			}
		}
		_myIdx += _nChannels;
		_nPoints++;
		_ctr++;

		if (_myIdx / _nChannels >= LOGGER_MAX_N_POINTS) {
			//if (_nPoints >= LOGGER_MAX_N_POINTS) {
			fprintf(stdout, "logger loop back.\n");
			//_nPoints = 0;
			_myIdx = 0;
		}
		clamp(_nPoints, 0, LOGGER_MAX_N_POINTS);
	}

	void writeToMRDPLOT(const char *pathname, const char *filename)
	{
		if (!_inited || _nPoints == 0)
			return;
		_recorded = true;
		MRDPLOT_DATA *d;

		d = malloc_mrdplot_data( 0, 0 );
		d->filename = generate_file_name(pathname, filename);
		d->n_channels = _nChannels;
		d->n_points = _nPoints;
		d->total_n_numbers = d->n_channels * d->n_points;
		d->frequency = _frequency;
		d->data = _data;

		fprintf(stdout, "%s SAVING DATA ..... \n", d->filename);

		d->names = new char*[_nChannels];
		d->units = new char*[_nChannels];

		for (int i = 0; i < _nChannels; i++) {
			d->names[i] = new char[LOGGER_MAX_CHARS];
			d->units[i] = new char[LOGGER_MAX_CHARS];

			strcpy(d->names[i], _datapoints[i].names);
			strcpy(d->units[i], _datapoints[i].units);
		}

		write_mrdplot_file( d );

		for (int i = 0; i < _nChannels; i++) {
			delete []d->names[i];
			delete []d->units[i];
		}
		delete []d->names;
		delete []d->units;

		fprintf(stdout, "%s SAVED DATA\n", d->filename);
		free(d);
	}

};

