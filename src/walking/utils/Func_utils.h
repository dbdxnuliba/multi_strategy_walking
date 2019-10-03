/*****************************************************************************
Func_utils.h

Description:	Header file of Func_utils

*****************************************************************************/
#pragma once

#include <iostream>
#include <cmath>

#ifdef __XENO__
    #include <rtdk.h>
    #define DPRINTF rt_printf
#else
    #include <stdio.h>
    #define DPRINTF printf
#endif

inline double RADTODEG(double x) { return x * 180.0 / M_PI;};
inline double DEGTORAD(double x) { return x * M_PI / 180.0;};

template <typename T1>
inline void COUT(T1 sth1) {std::cout << sth1 << std::endl;};

template <typename T1, typename T2>
inline void COUT(T1 sth1, T2 sth2) {std::cout << sth1 << "\t" << sth2 << std::endl;};

template <typename T1, typename T2, typename T3>
inline void COUT(T1 sth1, T2 sth2, T3 sth3) {std::cout << sth1 << "\t" << sth2 << "\t" << sth3 << std::endl;};

template <typename T1, typename T2, typename T3, typename T4>
inline void COUT(T1 sth1, T2 sth2, T3 sth3, T4 sth4) {std::cout << sth1 << "\t" << sth2 << "\t" << sth3 << "\t" << sth4 << std::endl;};

template <typename T1, typename T2, typename T3, typename T4, typename T5>
inline void COUT(T1 sth1, T2 sth2, T3 sth3, T4 sth4, T5 sth5) {std::cout << sth1 << "\t" << sth2 << "\t" << sth3 << "\t" << sth4 << "\t" << sth5 << std::endl;};

template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
inline void COUT(T1 sth1, T2 sth2, T3 sth3, T4 sth4, T5 sth5, T6 sth6) {std::cout << sth1 << "\t" << sth2 << "\t" << sth3 << "\t" << sth4 << "\t" << sth5 << "\t" << sth6 << std::endl;};

template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7>
inline void COUT(T1 sth1, T2 sth2, T3 sth3, T4 sth4, T5 sth5, T6 sth6, T7 sth7) {std::cout << sth1 << "\t" << sth2 << "\t" << sth3 << "\t" << sth4 << "\t" << sth5 << "\t" << sth6  << "\t" << sth7 << std::endl;};

template <typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7, typename T8>
inline void COUT(T1 sth1, T2 sth2, T3 sth3, T4 sth4, T5 sth5, T6 sth6, T7 sth7, T8 sth8) {std::cout << sth1 << "\t" << sth2 << "\t" << sth3 << "\t" << sth4 << "\t" << sth5 << "\t" << sth6  << "\t" << sth7 << "\t" << sth8 << std::endl;};

template <typename T>
inline void COUT(T* sth, int length, int start = 0)
{
	for (int i = 0; i < length; ++i){
		std::cout << sth[i] << "\t";
	}
	std::cout << std::endl;
};

inline double get_time() {
	struct timespec the_tp;
	clock_gettime( CLOCK_MONOTONIC, &the_tp );
	return ((double) (the_tp.tv_sec)) + 1.0e-9 * the_tp.tv_nsec;
}

template <typename T>
inline void clamp(T &num, const T& min, const T& max)
{
	num = (num > max) ? max : num;
	num = (num < min) ? min : num;
};

template <typename T> 
inline T sign(T val) {
	if(val == T(0)) return T(1);
	else return (T(0) < val) - (val < T(0));
}