#include "eg_planner/cubic_spline2d.hpp"
#include <iostream>

CppCubicSpline2D::CppCubicSpline2D(const std::vector<double> &x, const std::vector<double> &y) : sx(calc_s(x, y), x), sy(calc_s(x, y), y)
{
    //error checking
    if(x.size() != y.size()){
        throw std::runtime_error("vector size mismatch!");
    }
    if(x.size() <= 2){
        throw std::runtime_error("vector size must be more than 2!");
    }

    x_vec = x;
    y_vec = y;
    distances = calc_s(x, y);
}

void CppCubicSpline2D::calc_spline_course(std::vector<std::vector<double>>& output_path, float ds)
{
    output_path.push_back({0, 0, 0, 0});
    output_path.clear();

    double s = 0;
    int length = distances.back();
    while(true)
    {
        std::vector<double> point(4);
        point[0] = sx.calc_pos(s);    //x
        point[1] = sy.calc_pos(s);    //y
        point[2] = calc_yaw(s);       //angle
        point[3] = calc_curvature(s); //curvature
        output_path.push_back(point);

        s += ds;


        if(s > length+ds){

            double distance = sqrt(pow(sx.calc_pos(length) - x_vec.back(), 2) + pow(sy.calc_pos(length) - y_vec.back(), 2));

            // last point fix
            output_path[output_path.size()-1][0] = sx.calc_pos(length+distance);
            output_path[output_path.size()-1][1] = sy.calc_pos(length+distance);
            output_path[output_path.size()-1][2] = output_path[output_path.size()-2][2];
            output_path[output_path.size()-1][3] = output_path[output_path.size()-2][3];
            break;
        }
    }

}

std::vector<double> CppCubicSpline2D::calc_s(const std::vector<double>& x, const std::vector<double>& y)
{
    std::vector<double> distance_vec{0};
    double s = 0;

    int length = x.size();
    for(int i=0; i<length-1; ++i)
    {
        double ds = sqrt(pow(x[i+1]-x[i], 2) + pow(y[i+1]-y[i], 2));
        s += ds;
        distance_vec.push_back(s);
    }

    return distance_vec;
}

double CppCubicSpline2D::calc_curvature(double s)
{
    double dx = sx.calc_first_derivative(s);
    double ddx = sx.calc_second_derivative(s);
    double dy = sy.calc_first_derivative(s);
    double ddy = sy.calc_second_derivative(s);

    return (ddy*dx - ddx*dy) / pow(pow(dx, 2) + pow(dy, 2), 1.5);
}

double CppCubicSpline2D::calc_yaw(double s)
{
    double dx = sx.calc_first_derivative(s);
    double dy = sy.calc_first_derivative(s);

    return atan2(dy, dx);
}