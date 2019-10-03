/*****************************************************************************
rt_vector.h

Description:    Header file of rt_vector
				This class is used to replace the std::vector in my code to be real time safe
*****************************************************************************/
#pragma once

#include "Func_utils.h"
#include <vector>

namespace std {

#define DEFAULT_RT_VECTOR_CAPACITY 100


template <class T> class rt_vector
{
public:
	rt_vector()
		: element_n(0)
	{
		reserve(DEFAULT_RT_VECTOR_CAPACITY); //default size
		empty_element = data.back();
	};
	~rt_vector() {};

	// void resize(const int& size) {data.resize(size);}; // not using resize() to avoid mistaken usage with std::vector
	void resize(const unsigned int& size) {
		clear();
		element_n = size;
	};

	void resize(const unsigned int& size, const T& val) {
		assign(size, val);
	};

	void reserve(const unsigned int& size) {data.resize(size);};
	// unsigned int size() {return data.size();};
	const unsigned int& size() const {return element_n;};
	void clear() {erase_all(); element_n = 0;};

	const auto begin() {return data.begin();};
	const auto end() {return data.begin() + element_n;};
	const auto rbegin() {return data.rbegin() + data.size() - element_n;};
	const auto rend() {return data.rend();};
	const T& at(const unsigned int& idx) const
	{
		assert(idx < element_n);
		return data[idx];
	};

	bool empty() const {return element_n == 0;};

	const T& back() const
	{
		if (element_n > 0)
			return data.at(element_n - 1);
		else
			return data.at(0);
	};

	void push_back(const T& element)
	{
		data.at(element_n) = element;
		element_n++;
	};

	void pop_back()
	{
		data.at(element_n) = empty_element;
		element_n--;
	};

	void pop_front()
	{
		for (unsigned int i = 0; i < empty_element - 1; i++) {
			data.at(i) = data.at(i + 1);
		}
		pop_back();
	};

	template <class InputIterator, class iterator>
	void insert(iterator position, InputIterator first, InputIterator last)
	{
		// check the appended vector size
		int added_n = last - first;
		assert(added_n > 0);
		assert((element_n + added_n) <= data.size());
		unsigned int idx = last_index();

		// shift elements back
		if (position != end()) {
			int shift_n = end() - position;
			for (int i = 0; i < shift_n; i++) {
				data.at(idx - i + added_n) = data.at(idx - i);
			}
		}

		// insert
		int k = 0;
		for (auto it = first; it != last; it++) {
			position[k] = *it;
			k++;
		}

		element_n += added_n;
	};

	template <class InputIterator>
	void insert(InputIterator position, T element)
	{
		data.back() = element;
		insert(position, data.end() - 1, data.end());
	};

	template <class InputIterator>
	void erase(InputIterator first, InputIterator last)
	{
		int del_n = last - first;
		assert(del_n >= 0 && del_n <= data.size());

		// shift elements front
		int i = 0;
		if (last != end()) {
			int shift_n = end() - last;
			for (; i < shift_n; i++) {
				first[i] = rbegin()[shift_n - i - 1];
			}
		}

		// erase
		for (int k = 0; k < del_n; k++) {
			first[i + k] = empty_element;
		}

		element_n -= del_n;
	}

	template <class InputIterator>
	void assign(InputIterator first, InputIterator last)
	{
		clear();
		insert(begin(), first, last);
	}

	void assign(unsigned int n, const T &val)
	{
		clear();
		for (unsigned int i = 0; i < n; i++) {
			push_back(val);
		}
	}

	template <class V>
	auto& operator=(const V& from) {
		clear();
		insert(begin(), from.begin(), from.end());
		return *this;
	}

	T& operator[](const unsigned int& idx)
	{
		assert(idx < element_n);
		return data[idx];
	}

	// for testing only, it is not real time safe
	std::vector<T> getData()
	{
		std::vector<T> tmp;
		tmp.insert(tmp.end(), begin(), end());
		return tmp;
	};

	void printSelf(const std::string prefix = " ")
	{
		DPRINTF("%s has %d elements: ", prefix.c_str(), element_n);
		for (int i = 0; i < element_n; i++) {
			DPRINTF("%f ", data[i]);
		}
		DPRINTF("\n");
	};

private:
	std::vector<T> data;
	unsigned int element_n; // how many elements with data
	T empty_element;

	unsigned int last_index() {return element_n - 1;};
	void erase_all() {erase(begin(), end());};
};










}