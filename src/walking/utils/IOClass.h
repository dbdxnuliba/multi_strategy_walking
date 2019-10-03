/***************************************************************************** 
IOClass.h

Description:	Header file of customized input/output functions
*****************************************************************************/ 


#ifndef IOClass_H
#define IOClass_H

#include <iostream>
#include <vector>


class IOClass
{    
public:
	template <class T> 
	static void vout(std::vector<T> v, std::string prefix="")
	{
		std::cout<<prefix<<": ";
		for (auto it = v.begin(); it < v.end(); ++it){
			std::cout<<(*it)<<'\t';
		}
		std::cout<<std::endl;
	};



};
#endif