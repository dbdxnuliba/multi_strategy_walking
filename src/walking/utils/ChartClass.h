/*****************************************************************************
ChartClass.h

Description:	Header file of ChartClass

@Version:	1.0
*****************************************************************************/
#ifndef CHART_CLASS_H
#define CHART_CLASS_H

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>

#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Wrench.h"


#include <boost/shared_ptr.hpp>

// namespace Eigen
// {
// 	    unsigned long operator [](int i) const    {return registers[i];}
//     unsigned long & operator [](int i) {return registers[i];}
// }


class ChartClass
{
public:
	ChartClass() {ChartClass("ChartClass");};

	ChartClass(const std::string& topic_namespace)
		// : _nh(ros::NodeHandle(topic_namespace))
		: counter(0)
	{
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, topic_namespace);
		_nh = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(topic_namespace));
	};

	void addPublisher(const std::string& topic_name, const std::string& msgs_type)
	{
		if (msgs_type == "Point") {
			_pub.push_back(_nh->advertise<geometry_msgs::Point>(topic_name, 1));
			_pub_map.insert(std::make_pair(topic_name, std::make_pair(counter, msgs_type)));
			counter++;
		}
		else if (msgs_type == "Wrench") {
			_pub.push_back(_nh->advertise<geometry_msgs::Wrench>(topic_name, 1));
			_pub_map.insert(std::make_pair(topic_name, std::make_pair(counter, msgs_type)));
			counter++;
		}
	};

	template <typename T>
	void send(const std::string& topic_name, const T& content)
	// void send(const std::string& topic_name, const std::vector<double>& content)
	{
		// // while (_nh.ok()) {
		if (_pub_map.find(topic_name) != _pub_map.end()) {
			if (_pub_map[topic_name].second == "Point" && content.size() == 3) {
				geometry_msgs::Point temp;
				temp.x = content[0];
				temp.y = content[1];
				temp.z = content[2];

				_pub[_pub_map[topic_name].first].publish(temp);
			}
			else if (_pub_map[topic_name].second == "Wrench" && content.size() == 6) {
				geometry_msgs::Wrench temp;
				temp.force.x = content[0];
				temp.force.y = content[1];
				temp.force.z = content[2];
				temp.torque.x = content[3];
				temp.torque.y = content[4];
				temp.torque.z = content[5];

				_pub[_pub_map[topic_name].first].publish(temp);
			}
			ros::spinOnce();
		}
		else {
			std::cout << "Could not find the topic name of \"" << topic_name << "\", please check again." << std::endl;
		}
		// // }
	};

private:
	// ros::NodeHandle _nh;
	boost::shared_ptr<ros::NodeHandle> _nh;
	std::vector<ros::Publisher> _pub;
	std::map<std::string, std::pair<int, std::string> > _pub_map;
	int counter;


};

#endif