#include "eg_planner/cubic_spline1d.hpp"
#include <algorithm>
#include <numeric>

CppCubicSpline1D::CppCubicSpline1D(std::vector<double> x, std::vector<double> y)
{
    this->x_vec = x;
    updateParameter(y);
}

template <class T> void CppCubicSpline1D::updateParameter(const T &y)
{
    std::vector<double> h(x_vec.size());
    std::adjacent_difference(x_vec.cbegin(), x_vec.cend(), h.begin());
    h.erase(h.begin());

    // h cordinates must be positive
    for(int i = 0; i < h.size(); ++i){
        if(h[i] <= 0.0){
            throw std::runtime_error("x coordinates must be positive!");
        }
    }


    a_.clear();
    b_.clear();
    c_.clear();
    d_.clear();
    w_.clear();

    ndata = y.size()-1;
    for(int i = 0; i <= ndata; ++i){
        a_.push_back(y[i]);
    }

    //MatrixXd A(x_vec.size(), x_vec.size());
    //calcA(h);
    Eigen::VectorXd x = calcA(h).colPivHouseholderQr().solve(calc_b(h));

    for(int i = 0; i < x_vec.size(); ++i){
        c_.push_back(x(i));
    }

    for(int i = 0; i <= ndata; ++i){
        if(i == ndata){
            d_.push_back(0.0);
            b_.push_back(0.0);
        }else{
            d_.push_back((c_[i+1] - c_[i])/(3.0*h[i]));
            b_.push_back(1.0/h[i] * (a_[i+1] - a_[i]) - h[i]/3.0 * (2.0*c_[i] + c_[i+1]));
        }
    }
}

Eigen::MatrixXd CppCubicSpline1D::calcA(std::vector<double> h)
{
    Eigen::MatrixXd A(x_vec.size(), x_vec.size());
    A.setZero();
    A(0, 0) = 1.0;
    A(x_vec.size()-1, x_vec.size()-1) = 1.0;
    for(int i = 1; i < x_vec.size()-1; ++i){
        A(i, i-1) = h[i-1];
        A(i, i) = 2.0*(h[i-1] + h[i]);
        A(i, i+1) = h[i];
    }

    return A;
}

Eigen::VectorXd CppCubicSpline1D::calc_b(std::vector<double> h)
{
    Eigen::VectorXd B(x_vec.size());
    B.setZero();
    for(int i = 1; i < x_vec.size()-1; ++i){
        B(i, 0) = 3.0*(a_[i+1] - a_[i])/h[i] - 3.0*(a_[i] - a_[i-1])/h[i-1];
    }

    return B;
}

double CppCubicSpline1D::calc_pos(double x)
{
    if(ndata == -1){
        return 0;
    }

    if(x < x_vec.front()){
        return 0;
    }

    int j = search_index(x);
    if(j < 0){
        j = 0;
    }else if(j >= a_.size()){
        j = (a_.size()-1);
    }
    double dt = x - x_vec[j];

    return a_[j] + (b_[j] + (c_[j] + d_[j]*dt)*dt)*dt;
}

double CppCubicSpline1D::calc_first_derivative(double x)
{
    if(ndata == -1){
        return 0;
    }

    if(x < x_vec.front()){
        return 0;
    }else if(x > x_vec.back()){
        return 0;
    }

    int j = search_index(x);
    if(j < 0){
        j = 0;
    }else if(j >= a_.size()){
        j = (a_.size()-1);
    }
    double dt = x - x_vec[j];

    return b_[j] + (2.0*c_[j] + 3.0*d_[j]*dt)*dt;
}

double CppCubicSpline1D::calc_second_derivative(double x)
{
    if(ndata == -1){
        return 0;
    }

    if(x < x_vec.front()){
        return 0;
    }else if(x > x_vec.back()){
        return 0;
    }

    int j = search_index(x);
    if(j < 0){
        j = 0;
    }else if(j >= a_.size()){
        j = (a_.size()-1);
    }
    double dt = x - x_vec[j];

    return 2.0*c_[j] + 6.0*d_[j]*dt;
}

int CppCubicSpline1D::search_index(double value)
{
    // upper_bound
    std::vector<double>::iterator iter_upper = upper_bound(this->x_vec.begin(), this->x_vec.end(), value);
    int idx = distance(this->x_vec.begin(), iter_upper);

    return idx - 1;
}