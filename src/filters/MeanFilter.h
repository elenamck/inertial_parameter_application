#ifndef FILTERS_MEAN_FILTER_H_
#define FILTERS_MEAN_FILTER_H_


#include <Eigen/Dense>

namespace Filters
{
class MeanFilter {
public: 

	/** 
	* Empty default constructor
	*/
	MeanFilter();

	/**
	Constructor Mean filter with:
	* 	A - dynamics matrix
	*	C - output matrix
	*	Q - process noise covariance matrix
	*	R - measurement noise covariance matrix
	*	P - error covariance matrix
	*/
	MeanFilter(
		const int window_size,
		const int data_size
		);


	/** 
	* Initialize filter data history as zero
	*/
	void init();

	/**
	* @brief takes the mean of a signal 
	* @param output, Vector where the result will be written to
	* @param input, Vector of the input signal which is to be processed
	*/
	void process(Eigen::VectorXd& output, const Eigen::VectorXd& input);



private:

	/**
	* @brief updates data history
	*/
	void updateHistory(const Eigen::VectorXd& new_history);

	/**
	* @brief computed mean
	*/
	Eigen::VectorXd getMean(const Eigen::VectorXd& current_data);


	/**
	* @brief computed mean in the beginning
	*/
	Eigen::VectorXd getMeanEdge(const Eigen::VectorXd& current_data);


	//Matrix storing the data history
	Eigen::MatrixXd _Data_history;

	//window size
	int _window_size;

	//Data dimension
	int _data_size; 

	//counter
	int _counter;




};
} /* namespace Filters */


#endif //FILTERS_MEAN_FILTER_H_