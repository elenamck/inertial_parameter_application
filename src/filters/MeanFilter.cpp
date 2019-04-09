#include "MeanFilter.h"

#include <Eigen/Dense>
#include <iostream>

namespace Filters
{

MeanFilter::MeanFilter() {}

MeanFilter::MeanFilter(
		const int window_size,
		const int data_size
		)
{
	_window_size = window_size;
	_data_size = data_size;
	_Data_history.setZero(_data_size, _window_size);
	_counter = 0;
}

void MeanFilter::init()
{
	_Data_history.setZero(_data_size, _window_size);
	_counter = 0;
}

void MeanFilter::process(Eigen::VectorXd& output, const Eigen::VectorXd& input)
{	
	if(output.size() != input.size())
	{
		throw std::invalid_argument("Input and output size do not match!\n");
	}
	else if(_data_size != input.size())
	{
		throw std::invalid_argument("Input size does not match data size specified in constructor!\n");
	}
	else if(_counter<_window_size)
	{
		output = getMeanEdge(input);
		updateHistory(input);
	}

	if(_counter >= _window_size)
	{
		output = getMean(input);
		updateHistory(input);
	}
	_counter++;

}

Eigen::VectorXd MeanFilter::getMean(const Eigen::VectorXd& current_data)
{
	Eigen::VectorXd mean = Eigen::VectorXd::Zero(_data_size);

	mean = current_data;
	for (int i = 1; i<(_window_size); i++)
	{
		mean += _Data_history.col(i);
	}

	mean /= _window_size;


	return mean;
}

Eigen::VectorXd MeanFilter::getMeanEdge(const Eigen::VectorXd& current_data)
{
	Eigen::VectorXd mean = Eigen::VectorXd::Zero(_data_size);

	mean = current_data;
	for (int i = 0; i<(_window_size); i++)
	{
		mean += _Data_history.col(i);

	}
	mean /= (_counter+1);


	return mean;
}

void MeanFilter::updateHistory(const Eigen::VectorXd& new_history)
{
	for(int i = 0; i < (_window_size-1); i++)
	{
		_Data_history.col(i) = _Data_history.col(i+1);
	}
	_Data_history.col(_window_size-1) = new_history;


}
} /* namespace Filters */


