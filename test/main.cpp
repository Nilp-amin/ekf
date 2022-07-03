#include <iostream>

#include "../include/ekf.hpp"

int main(void)
{
    Eigen::Matrix<double, 3, 3> sysModel;
    Eigen::Matrix<double, 3, 2> ctrlModel;
    Eigen::Matrix<double, 3, 1> noiseModel;

    sysModel << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;
    ctrlModel << 0.1, 0.2,
                 0, 2,
                 0.5, 1;
    noiseModel << 0.1, 0.7, 0.7;

    StateSpaceModel<double, 3, 2> ssmodel(sysModel, ctrlModel, noiseModel);

    Eigen::Matrix<double, 3, 1> prevState;
    Eigen::Matrix<double, 2, 1> prevInput;

    prevState << 1, 2, 3;
    prevInput << 3, 0.1;

    Eigen::Matrix<double, 3, 1> output = ssmodel.estimate_state(prevState, prevInput);

    std::cout << output(0, 0) << " " << output(1, 0) << " " << output(2, 0) << std::endl;
    return 0;
}