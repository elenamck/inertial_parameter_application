
#include <iostream>
#include "filters/MeanFilter.h"




int main() {

	Eigen::VectorXd current = Eigen::VectorXd::Zero(2); 
	Eigen::VectorXd output = Eigen::VectorXd::Zero(2); 
	const int data_size = 2;
	const int window_size = 4; 
	auto mean = new Filters::MeanFilter(window_size, data_size);

	for(int i = 0 ; i < 20 ; i++)
	{
		current << 2*i+1, 2*i+2;
		mean->process(output, current);
		std::cout << "input is: " << current.transpose() << std::endl;
		std::cout << "mean is: " << output.transpose() << std::endl;
		
	}

    return 0;

}
