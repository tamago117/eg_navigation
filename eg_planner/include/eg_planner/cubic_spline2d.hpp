#pragma once
#include <vector>
#include <math.h>
#include <iostream>
#include "eg_planner/cubic_spline1d.hpp"

class CppCubicSpline2D
{
    public:
        CppCubicSpline2D(const std::vector<double> &x, const std::vector<double> &y);
        void calc_spline_course(std::vector<std::vector<double>>& output_path, float ds = 0.1);
    private:
        std::vector<double> x_vec;
        std::vector<double> y_vec;
        std::vector<double> distances;
        CppCubicSpline1D sx;
        CppCubicSpline1D sy;

        std::vector<double> calc_s(const std::vector<double>& x, const std::vector<double>& y);
        double calc_curvature(double s);
        double calc_yaw(double s);
};

