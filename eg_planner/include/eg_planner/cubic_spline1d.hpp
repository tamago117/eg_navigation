#pragma once
#include <vector>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>


class CppCubicSpline1D
{
    public:
        CppCubicSpline1D(std::vector<double> x, std::vector<double> y);
        template <class T> void updateParameter(const T &y);
        double calc_pos(double x);
        double calc_first_derivative(double x);
        double calc_second_derivative(double x);
    private:
        std::vector<double> a_;
        std::vector<double> b_;
        std::vector<double> c_;
        std::vector<double> d_;
        std::vector<double> w_;
        int ndata;
        std::vector<double> x_vec;

        Eigen::MatrixXd calcA(std::vector<double> h);
        Eigen::VectorXd calc_b(std::vector<double> h);
        int search_index(double value);
};
