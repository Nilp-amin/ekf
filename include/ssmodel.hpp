/**
 * @file ssmodel.hpp
 * @author nilp amin (nilpamin2@gmail.com)
 * @brief State Space Model template class 
 * @version 0.1
 * @date 2022-07-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include <Eigen/Dense>

/**
 * @brief State space model of a robot.
 * 
 * @tparam Tp Data type of member variables
 * @tparam Ns Number of state variables 
 * @tparam Nc Number of control variables
 */
template<typename Tp, std::size_t Ns, std::size_t Nc>
class StateSpaceModel 
{

/* Type vector */
using Tv = typename Eigen::Matrix<Tp, Ns, 1>;
/* Linearised system model of the robot */
using Tm_s = typename Eigen::Matrix<Tp, Ns, Ns>;
/* Linearised control model of the robot */
using Tm_c = typename Eigen::Matrix<Tp, Ns, Nc>;
/* Control input vector */
using Tv_c = typename Eigen::Matrix<Tp, Nc, 1>;


public:
    /**
     * @brief Construct a new State Space Model object
     * 
     * @param stateModel Linearised system model of robot
     * @param ctrlModel Linearised input control model of robot
     * @param noiseModel Process noise model of robot
     */
    StateSpaceModel(Tm_s sysModel, Tm_c ctrlModel, Tv noiseModel) : 
        sysModel_(sysModel), ctrlModel_(ctrlModel), noiseModel_(noiseModel)
    {
        ;
    }

    /**
     * @brief Estimates the robots current state.
     * 
     * @param prevState Previous robot state 
     * @param prevInput Previous robot control input 
     * @return Tv Estimated current robot state
     */
    Tv estimate_state(const Tv &prevState, const Tv_c &prevInput)
    {
        Tv est_state;

        est_state = sysModel_ * prevState + ctrlModel_ * prevInput + noiseModel_;

        return est_state;
    }

private:
    Tm_s sysModel_;
    Tm_c ctrlModel_;
    Tv noiseModel_;


};