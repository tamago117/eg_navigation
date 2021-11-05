/**
* @file bezier_curve.h
* @brief bezier curve class
* @author Michikuni Eguchi
* @date 2021.9.14
* @details aproximate bezier curve
*/

#pragma once
#include <math.h>
#include <vector>
#include <array>

class bezier_curve
{
public:
    bezier_curve();
    std::array<double, 2> getPos(float t, std::vector<double> px, std::vector<double> py);
private:
    int comb(int n, int r);
    double bernstein(int n, int i, double t);
};

bezier_curve::bezier_curve()
{

}

// 二項係数nCrの計算
int bezier_curve::comb(int n, int r)
{
    int num = n - r + 1, x = 1;

    if (n == r)
        return 1;
    else if (n < r)
        std::cout << "Error : please set n >= r" << std::endl;

    for (int i = 1; i <= r; i++)
        x = x*num++ / i;

    return x;
}

// Bernstein係数
double bezier_curve::bernstein(int n, int i, double t)
{
    int k = 0;
    double a = 1.0, b = 1.0;

    for (k = 0; k < i; k++)
        a *= t;

    for (k = 0; k < n-i; k++)
        b *= 1 - t;

    return double(comb(n, i)) * a * b;
}

// Bezier curve の算出
std::array<double, 2> bezier_curve::getPos(float t, std::vector<double> px, std::vector<double> py)
{
    int n = px.size()-1;
    double x = 0.0, y = 0.0, B = 0.0;

    for (int i = 0; i <= n; i++) {
        B  = bernstein(n, i, t);
        x += B * px[i];
        y += B * py[i];
    }

    return {{x, y}};
}